cmake_minimum_required(VERSION 2.8.3)

project(node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  genmsg
  cv_bridge
  image_transport
  message_generation
  #bing
  pcl_ros
  baxter_core_msgs
  control_msgs
)

catkin_package(CATKIN_DEPENDS 
  eigen
  message_runtime)

link_directories(/opt/ros/hydro/lib ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})


include_directories(include
	${catkin_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


set(OPENMP_FLAG "-fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPENMP_FLAG} -O3")

set(CMAKE_BUILD_TYPE Debug)

add_executable(capture_object src/capture_object.cpp)
add_executable(train_classifier src/train_classifier.cpp)
add_executable(publish_detections src/publish_detections.cpp)
add_executable(capture_annotated_objects src/capture_annotated_objects.cpp)
add_executable(capture_hard_class src/capture_hard_class.cpp)
add_executable(filter_time src/filter_time.cpp)
add_executable(pilot_baxter src/pilot_baxter.cpp)
add_executable(ein src/ein.cpp)

target_link_libraries(capture_object
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	opencv_nonfree
	BINGLIB ###
)
target_link_libraries(train_classifier
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	opencv_nonfree
	BINGLIB ###
)
target_link_libraries(publish_detections
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	opencv_nonfree
	BINGLIB ###
)
target_link_libraries(capture_annotated_objects
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	opencv_nonfree
	BINGLIB ###
)
target_link_libraries(capture_hard_class
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	opencv_nonfree
	BINGLIB ###
)
target_link_libraries(filter_time
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	BINGLIB ###
)
target_link_libraries(pilot_baxter
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	BINGLIB ###
)
target_link_libraries(ein
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	opencv_nonfree
	BINGLIB ###
)
