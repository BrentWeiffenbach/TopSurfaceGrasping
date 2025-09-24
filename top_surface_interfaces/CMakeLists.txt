cmake_minimum_required(VERSION 3.5)
project(top_surface_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectObjects.srv"
  "srv/SegmentObjects.srv"
  "srv/ExtractObjectGeometry.srv"
  DEPENDENCIES sensor_msgs geometry_msgs vision_msgs
)

ament_package()