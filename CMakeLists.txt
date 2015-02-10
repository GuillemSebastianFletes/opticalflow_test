cmake_minimum_required(VERSION 2.8.3)
project(opticalflow_test)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  roscpp
  sensor_msgs
  std_msgs
)

find_package( OpenCV REQUIRED )

catkin_package()

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(trajectory_mono  src/trajectory_mono.cpp)
# )
target_link_libraries(trajectory_mono
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${BOOST_LIBRARIES}
)
## Declare a cpp executable
add_executable(opticalflow_test_node src/opticalflow_test_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(opticalflow_test_node opticalflow_test_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(opticalflow_test_node
  trajectory_mono
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${BOOST_LIBRARIES}
)

