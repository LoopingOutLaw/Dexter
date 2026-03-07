#!/usr/bin/env python3
"""
Dexter Web Interface - A Flask-based web control panel for the Dexter robotic arm.
Replaces the Alexa interface with a browser-based UI that supports both
button controls and voice commands via the Web Speech API.
"""
import os
import threading
from flask import Flask, render_template, jsonify, request as flask_request

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from dexter_msgs.action import DexterTask


class WebInterface(Node):
    """ROS 2 node that bridges HTTP requests to the DexterTask action server."""

    TASK_NAMES = {0: "Home/Wake", 1: "Pick", 2: "Sleep"}

    def __init__(self):
        super().__init__("web_interface")
        self.action_client = ActionClient(self, DexterTask, "task_server")
        self._lock = threading.Lock()
        self.get_logger().info("Web interface node initialized, waiting for task_server...")

    def send_task(self, task_number):
        """Send a task goal to the action server. Returns (success, message).

        Uses callbacks + threading.Event so we never call spin from the Flask
        thread (the node is already being spun in a background thread).
        """
        if not self._lock.acquire(blocking=False):
            return False, "Another task is already in progress"

        try:
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Task server not available")
                return False, "Task server not available"

            goal = DexterTask.Goal()
            goal.task_number = task_number

            task_name = self.TASK_NAMES.get(task_number, f"Unknown({task_number})")
            self.get_logger().info(f"Sending task: {task_name} (id={task_number})")

            # Shared state between Flask thread and ROS callback thread
            result_holder = {"accepted": False, "message": ""}
            done_event = threading.Event()

            def goal_response_cb(future):
                goal_handle = future.result()
                if goal_handle is None or not goal_handle.accepted:
                    result_holder["accepted"] = False
                    result_holder["message"] = f"Task {task_name} was rejected by the server"
                    done_event.set()
                    return
                self.get_logger().info(f"Task {task_name} accepted, waiting for result...")
                result_holder["accepted"] = True
                # Now wait for the actual execution result
                get_result_future = goal_handle.get_result_async()
                get_result_future.add_done_callback(result_cb)

            def result_cb(future):
                try:
                    result = future.result()
                    if result.result.success:
                        result_holder["message"] = f"Task {task_name} completed successfully"
                    else:
                        result_holder["message"] = f"Task {task_name} execution failed"
                except Exception as e:
                    result_holder["accepted"] = False
                    result_holder["message"] = f"Task {task_name} error: {str(e)}"
                finally:
                    done_event.set()

            send_goal_future = self.action_client.send_goal_async(goal)
            send_goal_future.add_done_callback(goal_response_cb)

            # Wait for the full round-trip (goal accepted + result returned)
            # Timeout after 30s to avoid hanging forever
            if not done_event.wait(timeout=30.0):
                return False, f"Task {task_name} timed out after 30 seconds"

            return result_holder["accepted"], result_holder["message"]
        finally:
            self._lock.release()


# ---------------------------------------------------------------------------
# Flask application
# ---------------------------------------------------------------------------
def _find_template_dir():
    """Locate the templates directory - works both in dev and installed layouts."""
    # Try the installed ROS 2 share directory first
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("dexter_remote")
        installed = os.path.join(share_dir, "templates")
        if os.path.isdir(installed):
            return installed
    except Exception:
        pass
    # Fallback: relative to this source file (development workspace)
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "templates")


app = Flask(__name__, template_folder=_find_template_dir())

ros_node = None


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/task", methods=["POST"])
def handle_task():
    data = flask_request.get_json(silent=True)
    if data is None or "task_number" not in data:
        return jsonify({"success": False, "message": "Missing task_number"}), 400

    task_number = int(data["task_number"])
    if task_number not in (0, 1, 2):
        return jsonify({"success": False, "message": f"Invalid task number: {task_number}"}), 400

    success, message = ros_node.send_task(task_number)
    status_code = 200 if success else 503
    return jsonify({"success": success, "message": message}), status_code


@app.route("/health")
def health():
    server_ready = ros_node.action_client.server_is_ready() if ros_node else False
    return jsonify({"status": "ok", "task_server_connected": server_ready})


def main():
    global ros_node
    rclpy.init()
    ros_node = WebInterface()

    # Run the ROS 2 spinner in a background thread so Flask can run in the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    ros_node.get_logger().info("Starting web server on http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
