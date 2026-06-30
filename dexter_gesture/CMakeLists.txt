cmake_minimum_required(VERSION 3.8)
project(dexter_gesture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install the Python package (dexter_gesture/)
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ${PROJECT_NAME}/gesture_receiver.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Pure-Python unit tests for motion_smoother and gesture_receiver.
  # Selected by `colcon test --packages-select dexter_gesture`.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME dexter_gesture_pytest
    COMMAND "${Python3_EXECUTABLE}" -m pytest -v
            "${CMAKE_CURRENT_SOURCE_DIR}/test"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_package()
