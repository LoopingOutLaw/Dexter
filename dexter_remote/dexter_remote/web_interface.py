#!/usr/bin/env python3
"""
Dexter Web Dashboard - A professional Flask-based control panel for the Dexter robotic arm.

Features:
- Live joint state monitoring via SSE (Server-Sent Events)
- Direct joint control via trajectory publishers
- Preset task execution via DexterTask action server
- Pose save/load/delete with JSON persistence
- Task metrics tracking (success rate, execution time)
- Connection status monitoring
"""
import json
import math
import os
import threading
import time
from pathlib import Path

from flask import (
    Flask,
    Response,
    jsonify,
    render_template,
    request as flask_request,
)

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from dexter_msgs.action import DexterTask
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# Joint configuration
# ---------------------------------------------------------------------------
JOINT_CONFIG = [
    {"name": "joint_1", "label": "Base",     "min": -1.5708, "max":  1.5708, "group": "arm"},
    {"name": "joint_2", "label": "Shoulder", "min": -1.5708, "max":  1.5708, "group": "arm"},
    {"name": "joint_3", "label": "Elbow",    "min": -1.5708, "max":  1.5708, "group": "arm"},
    {"name": "joint_4", "label": "Gripper",  "min": -1.5708, "max":  0.0,    "group": "gripper"},
]

POSES_FILE = Path.home() / ".dexter_poses.json"


class WebInterface(Node):
    """ROS 2 node that bridges HTTP requests to robot control."""

    TASK_NAMES = {0: "Home", 1: "Pick", 2: "Sleep"}

    def __init__(self):
        super().__init__("web_interface")

        # --- Action client (preset tasks) ---
        self.action_client = ActionClient(self, DexterTask, "task_server")
        self._task_lock = threading.Lock()
        self._task_active = False

        # --- Joint state subscriber ---
        self._joint_positions = {}  # name -> position (rad)
        self._joint_lock = threading.RLock()
        self.create_subscription(JointState, "joint_states", self._joint_state_cb, 10)

        # --- Trajectory publishers (direct joint control) ---
        self.arm_pub = self.create_publisher(
            JointTrajectory, "arm_controller/joint_trajectory", 10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory, "gripper_controller/joint_trajectory", 10
        )

        # --- Metrics ---
        self._metrics_lock = threading.Lock()
        self._metrics = {
            "tasks_sent": 0,
            "tasks_succeeded": 0,
            "tasks_failed": 0,
            "total_execution_time": 0.0,
        }

        # --- Saved poses ---
        self._poses = self._load_poses()

        self.get_logger().info("Web interface node initialized, waiting for task_server...")

    # ------------------------------------------------------------------
    # Joint state handling
    # ------------------------------------------------------------------
    def _joint_state_cb(self, msg: JointState):
        with self._joint_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    def get_joint_states(self):
        """Return current joint positions as a dict."""
        with self._joint_lock:
            return dict(self._joint_positions)

    # ------------------------------------------------------------------
    # Direct joint control
    # ------------------------------------------------------------------
    def move_to_joints(self, positions, duration_sec=0.5):
        """Publish trajectory messages to move joints to *positions*.

        *positions* is a list of 4 floats: [j1, j2, j3, j4].
        """
        arm_msg = JointTrajectory()
        arm_msg.joint_names = ["joint_1", "joint_2", "joint_3"]
        arm_point = JointTrajectoryPoint()
        arm_point.positions = [float(positions[0]), float(positions[1]), float(positions[2])]
        arm_point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        arm_msg.points.append(arm_point)

        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = ["joint_4"]
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [float(positions[3])]
        gripper_point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        gripper_msg.points.append(gripper_point)

        self.arm_pub.publish(arm_msg)
        self.gripper_pub.publish(gripper_msg)
        self.get_logger().info(
            f"Published trajectory: [{positions[0]:.2f}, {positions[1]:.2f}, "
            f"{positions[2]:.2f}, {positions[3]:.2f}] duration={duration_sec}s"
        )

    # ------------------------------------------------------------------
    # Preset task execution (action server)
    # ------------------------------------------------------------------
    @property
    def task_active(self):
        return self._task_active

    def send_task(self, task_number):
        """Send a task goal to the action server. Returns (success, message)."""
        if not self._task_lock.acquire(blocking=False):
            return False, "Another task is already in progress"

        self._task_active = True
        start_time = time.monotonic()

        try:
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Task server not available")
                with self._metrics_lock:
                    self._metrics["tasks_sent"] += 1
                    self._metrics["tasks_failed"] += 1
                return False, "Task server not available"

            goal = DexterTask.Goal()
            goal.task_number = task_number

            task_name = self.TASK_NAMES.get(task_number, f"Unknown({task_number})")
            self.get_logger().info(f"Sending task: {task_name} (id={task_number})")

            with self._metrics_lock:
                self._metrics["tasks_sent"] += 1

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

            if not done_event.wait(timeout=30.0):
                with self._metrics_lock:
                    self._metrics["tasks_failed"] += 1
                return False, f"Task {task_name} timed out after 30 seconds"

            elapsed = time.monotonic() - start_time
            with self._metrics_lock:
                self._metrics["total_execution_time"] += elapsed
                if result_holder["accepted"] and "successfully" in result_holder["message"]:
                    self._metrics["tasks_succeeded"] += 1
                else:
                    self._metrics["tasks_failed"] += 1

            return result_holder["accepted"], result_holder["message"]
        finally:
            self._task_active = False
            self._task_lock.release()

    # ------------------------------------------------------------------
    # Metrics
    # ------------------------------------------------------------------
    def get_metrics(self):
        with self._metrics_lock:
            m = dict(self._metrics)
        sent = m["tasks_sent"] or 1  # avoid div-by-zero
        m["success_rate"] = round(m["tasks_succeeded"] / sent * 100, 1) if m["tasks_sent"] else 0.0
        m["avg_execution_time"] = round(m["total_execution_time"] / sent, 2) if m["tasks_sent"] else 0.0
        return m

    # ------------------------------------------------------------------
    # Saved poses (JSON persistence)
    # ------------------------------------------------------------------
    def _load_poses(self):
        try:
            if POSES_FILE.exists():
                data = json.loads(POSES_FILE.read_text())
                self.get_logger().info(f"Loaded {len(data)} saved poses from {POSES_FILE}")
                return data
        except Exception as e:
            self.get_logger().warn(f"Failed to load poses: {e}")
        return {}

    def _save_poses_to_disk(self):
        try:
            POSES_FILE.write_text(json.dumps(self._poses, indent=2))
        except Exception as e:
            self.get_logger().error(f"Failed to save poses: {e}")

    def save_pose(self, name, positions):
        self._poses[name] = positions
        self._save_poses_to_disk()
        self.get_logger().info(f"Saved pose '{name}': {positions}")

    def delete_pose(self, name):
        if name in self._poses:
            del self._poses[name]
            self._save_poses_to_disk()
            self.get_logger().info(f"Deleted pose '{name}'")
            return True
        return False

    def get_saved_poses(self):
        return dict(self._poses)


# ---------------------------------------------------------------------------
# Flask application
# ---------------------------------------------------------------------------
def _find_template_dir():
    """Locate the templates directory - works both in dev and installed layouts."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("dexter_remote")
        installed = os.path.join(share_dir, "templates")
        if os.path.isdir(installed):
            return installed
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "templates")


app = Flask(__name__, template_folder=_find_template_dir())
ros_node: WebInterface = None  # type: ignore[assignment]


# --- Page ---
@app.route("/")
def index():
    return render_template("index.html")


# --- Preset task execution ---
@app.route("/task", methods=["POST"])
def handle_task():
    data = flask_request.get_json(silent=True)
    if data is None or "task_number" not in data:
        return jsonify({"success": False, "message": "Missing task_number"}), 400

    task_number = int(data["task_number"])
    if task_number not in (0, 1, 2):
        return jsonify({"success": False, "message": f"Invalid task number: {task_number}"}), 400

    success, message = ros_node.send_task(task_number)
    return jsonify({"success": success, "message": message}), (200 if success else 503)


# --- Direct joint control ---
@app.route("/move", methods=["POST"])
def handle_move():
    """Move joints to specified positions directly via trajectory publishers."""
    if ros_node.task_active:
        return jsonify({"success": False, "message": "A preset task is executing"}), 409

    data = flask_request.get_json(silent=True)
    if data is None or "positions" not in data:
        return jsonify({"success": False, "message": "Missing positions array"}), 400

    positions = data["positions"]
    if not isinstance(positions, list) or len(positions) != 4:
        return jsonify({"success": False, "message": "positions must be array of 4 floats"}), 400

    duration = float(data.get("duration", 0.5))
    try:
        positions = [float(p) for p in positions]
    except (TypeError, ValueError):
        return jsonify({"success": False, "message": "Invalid position values"}), 400

    # Clamp to joint limits
    for i, jc in enumerate(JOINT_CONFIG):
        positions[i] = max(jc["min"], min(jc["max"], positions[i]))

    ros_node.move_to_joints(positions, duration)
    return jsonify({"success": True, "message": "Trajectory published", "positions": positions})


# --- Joint states (snapshot) ---
@app.route("/joints")
def get_joints():
    return jsonify(ros_node.get_joint_states())


# --- SSE stream ---
@app.route("/stream")
def stream():
    """Server-Sent Events endpoint streaming joint states, metrics, and status at ~10 Hz."""
    def generate():
        while True:
            joints = ros_node.get_joint_states()
            metrics = ros_node.get_metrics()
            server_ready = False
            try:
                server_ready = ros_node.action_client.server_is_ready()
            except Exception:
                pass

            payload = json.dumps({
                "joints": {k: round(v, 5) for k, v in joints.items()},
                "metrics": metrics,
                "connected": server_ready,
                "task_active": ros_node.task_active,
                "timestamp": time.time(),
            })
            yield f"data: {payload}\n\n"
            time.sleep(0.1)  # ~10 Hz

    return Response(generate(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


# --- Saved poses ---
@app.route("/poses", methods=["GET"])
def get_poses():
    return jsonify(ros_node.get_saved_poses())


@app.route("/poses", methods=["POST"])
def save_pose():
    data = flask_request.get_json(silent=True)
    if not data or "name" not in data or "positions" not in data:
        return jsonify({"success": False, "message": "Missing name or positions"}), 400
    name = str(data["name"]).strip()
    if not name:
        return jsonify({"success": False, "message": "Pose name cannot be empty"}), 400
    positions = data["positions"]
    if not isinstance(positions, list) or len(positions) != 4:
        return jsonify({"success": False, "message": "positions must be array of 4 floats"}), 400
    try:
        positions = [float(p) for p in positions]
    except (TypeError, ValueError):
        return jsonify({"success": False, "message": "Invalid position values"}), 400

    ros_node.save_pose(name, positions)
    return jsonify({"success": True, "message": f"Pose '{name}' saved"})


@app.route("/poses/<name>", methods=["DELETE"])
def delete_pose(name):
    if ros_node.delete_pose(name):
        return jsonify({"success": True, "message": f"Pose '{name}' deleted"})
    return jsonify({"success": False, "message": f"Pose '{name}' not found"}), 404


# --- Metrics ---
@app.route("/metrics")
def get_metrics():
    return jsonify(ros_node.get_metrics())


# --- Config (joint limits etc. for frontend) ---
@app.route("/config")
def get_config():
    return jsonify({"joints": JOINT_CONFIG})


# --- Health check ---
@app.route("/health")
def health():
    server_ready = ros_node.action_client.server_is_ready() if ros_node else False
    return jsonify({"status": "ok", "task_server_connected": server_ready})


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    global ros_node
    rclpy.init()
    ros_node = WebInterface()

    # Run the ROS 2 spinner in a background thread so Flask can run in the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    ros_node.get_logger().info("Starting Dexter Dashboard on http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
