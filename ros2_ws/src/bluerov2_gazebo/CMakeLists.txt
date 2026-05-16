cmake_minimum_required(VERSION 3.8)
project(bluerov2_gazebo)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*~" EXCLUDE
)

# Install Gazebo world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install configuration files (disturbances, etc.)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install texture files for underwater environment
install(DIRECTORY textures/
  DESTINATION share/${PROJECT_NAME}/textures
)

ament_package()
