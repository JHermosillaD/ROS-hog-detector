cmake_minimum_required(VERSION 3.0.2)
project(hog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs  
  image_transport
  message_generation
)

add_message_files(
  FILES
  ImageBoundingBoxListStamped.msg
  ImageBoundingBox.msg
  ImagePoint.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(body_detector src/main.cpp)

target_link_libraries(body_detector
  ${catkin_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
)
