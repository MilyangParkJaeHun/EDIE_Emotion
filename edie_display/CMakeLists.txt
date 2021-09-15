cmake_minimum_required(VERSION 2.8.3)
project(edie_display)

add_compile_options(-std=c++11)
add_compile_options(-ljsoncpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
    ${PROJECT_NAME}_lib
   CATKIN_DEPENDS roscpp roslib rospy std_msgs 
   DEPENDS system_lib
)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
    src/Emotions.cpp
    src/Emotion.cpp
    src/Parser.cpp
)

add_library(${PROJECT_NAME}_lib
    ${PROJECT_LIB_FILES}
)

add_dependencies(${PROJECT_NAME}_lib
   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)


target_link_libraries(${PROJECT_NAME}_lib 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}
    ${JSONCPP_LIBRARIES}
)

add_executable(edie_display_node src/edie_display_node.cpp)
add_dependencies(edie_display_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(edie_display_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_NAME}_lib)
