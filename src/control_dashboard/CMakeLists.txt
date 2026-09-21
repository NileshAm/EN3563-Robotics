cmake_minimum_required(VERSION 3.8)
project(control_dashboard)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToPose.srv"
  "msg/MoveProgress.msg"
  DEPENDENCIES geometry_msgs
)

install(PROGRAMS scripts/dashboard.py
  DESTINATION lib/${PROJECT_NAME} RENAME dashboard)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
