cmake_minimum_required(VERSION 3.0.2)
project(qr_code_reader)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
sensor_msgs
std_msgs
cv_bridge
image_transport
message_generation
)

find_package(OpenCV  REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  qr_msg.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 sensor_msgs
                 std_msgs
                 cv_bridge
                 image_transport
                 message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(camera_subscriber src/camera_subscriber.cpp )
target_link_libraries(camera_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(qr_reader src/main.cpp src/reader.cpp)
target_link_libraries(qr_reader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ZBAR_LIBRARIES} zbar)

add_dependencies(qr_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})