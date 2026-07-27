cmake_minimum_required(VERSION 3.22)

project(ffw_moveit_config)

################################################################################
# Dependencies
################################################################################

find_package(ament_cmake REQUIRED)

################################################################################
# Install MoveIt configuration and launch files
################################################################################

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Install the hidden MoveIt Setup Assistant metadata file
################################################################################

install(
  FILES
    .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Package
################################################################################

ament_package()