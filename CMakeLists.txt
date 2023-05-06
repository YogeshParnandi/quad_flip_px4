cmake_minimum_required(VERSION 3.0.2)
project(geom_flip)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    tf
    dynamic_reconfigure
    message_generation
    mavros
    mavros_extras
    mavros_msgs
    mavlink
    std_msgs
    geometry_msgs
    sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  flatTargetmsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  config/flipConfig.cfg
)

catkin_package(
  INCLUDE_DIRS include  
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  mav_msgs 
  tf
  tf2
  message_runtime
  geometry_msgs
  sensor_msgs
  msg
)


include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${Eigen_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_publisher src/trajectoryPublisher.cpp)
add_dependencies(trajectory_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_publisher ${catkin_LIBRARIES})

add_executable(geom_tracking_node src/flip.cpp)
target_link_libraries(geom_tracking_node ${catkin_LIBRARIES})
add_dependencies(geom_tracking_node ${catkin_EXPORTED_TARGETS})



