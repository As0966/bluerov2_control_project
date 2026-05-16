cmake_minimum_required(VERSION 3.8)
project(bluerov2_description)

find_package(ament_cmake REQUIRED)

# Install SDF / URDF model files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install 3-D mesh assets (DAE, STL)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install Gazebo model directories (subsea_pipeline, etc.)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
