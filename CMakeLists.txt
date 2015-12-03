cmake_minimum_required(VERSION 2.8.3)
project(localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
PCL REQUIRED
)

#find_package( PCL REQUIRED )

add_message_files(
	FILES
	Map_message.msg
	Distance_message.msg
	Position.msg
	Depth_Ranges.msg
)

generate_messages(
DEPENDENCIES
std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES localization
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(particle_filter src/particle_filter.cpp)
target_link_libraries(particle_filter ${catkin_LIBRARIES})

add_executable(distance_converter src/distance_converter.cpp)
target_link_libraries(distance_converter ${catkin_LIBRARIES})

add_executable(depth_readings_publisher_node src/depth_readings_publisher.cpp)
target_link_libraries(depth_readings_publisher_node ${PCL_DEFINITIONS} ${catkin_LIBRARIES})


add_executable(particle_filter_with_depth_sensor src/particle_filter_with_depth_sensor.cpp)
target_link_libraries(particle_filter_with_depth_sensor ${catkin_LIBRARIES})
