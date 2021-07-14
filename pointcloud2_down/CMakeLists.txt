cmake_minimum_required(VERSION 3.0.2)
project(pointcloud2_down)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  pcl_conversions
)
find_package(PCL REQUIRED QUIET)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES depth2cloud
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	# include
	${PCL_LIBRARY_DIRS}
)

add_executable(pointcloud_down_node src/pointcloud_down.cpp)
add_dependencies(pointcloud_down_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_down_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})