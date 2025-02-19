cmake_minimum_required(VERSION 3.22)
project(leg_v2_moveit_config)

# Required packages
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rclcpp REQUIRED)  # ROS 2 core package
find_package(ros2_control REQUIRED)  # ROS 2 control
find_package(controller_manager REQUIRED)  # For controller manager


# Install launch, config, and world files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# Install .setup_assistant file
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

# Dependencies for your controllers
ament_package()
