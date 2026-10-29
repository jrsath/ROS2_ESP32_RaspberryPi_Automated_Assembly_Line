cmake_minimum_required(VERSION 3.8)
project(group4_assembly_line)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/joint_control.py
  scripts/ik_offline.py
  scripts/rig_markers.py
  scripts/start_controllers.py
  scripts/wait_for_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  urdf
  models
  rviz
  cad_reference
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
