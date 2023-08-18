cmake_minimum_required(VERSION 3.0.2)
project(crawler_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
  nav_msgs
  tf
)

add_message_files(
  FILES
  Status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime nav_msgs tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(crawler_controller src/crawler_controller.cpp)

add_dependencies(crawler_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(crawler_controller
  ${catkin_LIBRARIES}
)

