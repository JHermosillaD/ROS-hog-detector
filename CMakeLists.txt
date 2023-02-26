cmake_minimum_required(VERSION 3.0.2)
project(hog)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs  
  image_transport
  message_generation
)

#OPEN CV libraries
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
set (LIBS ${LIBS} ${OpenCV_LIBS})

#POINT CLOUD libraries
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
set (LIBS ${LIBS} ${PCL_LIBS})

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   ImagePoint.msg
   ImageBoundingBox.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hog_node src/main.cpp)

target_link_libraries(hog_node
  ${catkin_LIBRARIES} ${LIBS}
)
