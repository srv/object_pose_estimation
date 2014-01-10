cmake_minimum_required(VERSION 2.8.3)
project(object_pose_estimation)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	image_transport
	bag_tools
	vision_msgs
	pcl_ros
	cv_bridge)

find_package(OpenCV REQUIRED)
find_package(pcl REQUIRED)

catkin_package()

include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} include)

add_executable(object_feature_matching_detector_node src/feature_matching_detector_node.cpp)
add_dependencies(object_feature_matching_detector_node vision_msgs_gencpp)
target_link_libraries(object_feature_matching_detector_node ${OpenCV_LIBS} ${pcl_LIBRARIES} ${catkin_LIBRARIES})

add_executable(object_features_to_pcd src/features_to_pcd.cpp)
target_link_libraries(object_features_to_pcd ${OpenCV_LIBS} ${pcl_LIBRARIES} ${catkin_LIBRARIES})
