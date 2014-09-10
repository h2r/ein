cmake_minimum_required(VERSION 2.8.3)

project(oberlin_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  genmsg
  OpenCV
  eigen
  cv_bridge
  image_transport
  message_generation
  #bing
  pcl_ros
)

catkin_package(CATKIN_DEPENDS message_runtime)

link_directories(/opt/ros/hydro/lib ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


include_directories(include
	${catkin_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(oberlin_detector src/oberlin_detection.cpp)
add_executable(capture_object src/capture_object.cpp)
add_executable(train_classifier src/train_classifier.cpp)
add_executable(publish_detections src/publish_detections.cpp)

target_link_libraries(oberlin_detector
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	BINGLIB ###
)
target_link_libraries(capture_object
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	BINGLIB ###
)
target_link_libraries(train_classifier
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	BINGLIB ###
)
target_link_libraries(publish_detections
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	BINGLIB ###
)
