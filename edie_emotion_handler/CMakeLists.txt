cmake_minimum_required(VERSION 3.0.2)
project(edie_emotion_handler)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES edie_emotion_handler
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(edie_emotion_node src/edie_emotion_node.cpp)
add_dependencies(edie_emotion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(edie_emotion_node ${catkin_LIBRARIES})
