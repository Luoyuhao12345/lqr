cmake_minimum_required(VERSION 3.0.2)
project(my_lqr)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  tf
)

add_message_files(
          FILES
          way_point.msg
          car_info.msg
          my_marker.msg
          way_points.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS  roscpp rospy std_msgs tf message_runtime 
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)


 add_executable(car_info_pub_node src/car_info_pub.cpp)
 target_link_libraries(car_info_pub_node
   ${catkin_LIBRARIES}
 )

 add_executable(visual_point_node src/visual_point.cpp)
 target_link_libraries(visual_point_node
   ${catkin_LIBRARIES}
 )