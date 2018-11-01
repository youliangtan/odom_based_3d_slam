cmake_minimum_required(VERSION 2.8.3)
project(odom_based_3d_slam)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  nav_msgs
  tf2
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OCTOMAP REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL #OCTOMAP
)


link_directories(${PCL_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS} ${OCTOMAP_DEFINITIONS})
add_definitions( -march=native )


add_executable(pointcloudRegistration src/pointcloudRegistration.cpp)
target_link_libraries(pointcloudRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} )

