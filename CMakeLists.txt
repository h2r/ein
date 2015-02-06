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
add_executable(ein src/ein.cpp src/distributions.c src/randomkit.c src/cephes/airy.c    src/cephes/fdtr.c    src/cephes/kn.c          src/cephes/scipy_iv.c
src/cephes/bdtr.c    src/cephes/fresnl.c  src/cephes/kolmogorov.c  src/cephes/setprec.c
src/cephes/beta.c    src/cephes/gamma.c   src/cephes/mmmpy.c       src/cephes/shichi.c
src/cephes/btdtr.c   src/cephes/gdtr.c    src/cephes/mtherr.c      src/cephes/sici.c
src/cephes/cbrt.c    src/cephes/gels.c    src/cephes/mtransp.c     src/cephes/simpsn.c
src/cephes/chbevl.c  src/cephes/hyp2f1.c  src/cephes/mvmpy.c       src/cephes/simq.c
src/cephes/chdtr.c   src/cephes/hyperg.c  src/cephes/nbdtr.c       src/cephes/sincos.c
src/cephes/const.c   src/cephes/i0.c      src/cephes/ndtr.c        src/cephes/sindg.c
src/cephes/cpmul.c   src/cephes/i1.c      src/cephes/ndtri.c       src/cephes/spence.c
src/cephes/dawsn.c   src/cephes/igam.c    src/cephes/pdtr.c        src/cephes/stdtr.c
src/cephes/ellie.c   src/cephes/igami.c   src/cephes/polevl.c      src/cephes/struve.c
src/cephes/ellik.c   src/cephes/incbet.c  src/cephes/polmisc.c     src/cephes/tandg.c
src/cephes/ellpe.c   src/cephes/incbi.c   src/cephes/polrt.c       src/cephes/tukey.c
src/cephes/ellpj.c   src/cephes/j0.c      src/cephes/polyn.c       src/cephes/unity.c
src/cephes/ellpk.c   src/cephes/j1.c      src/cephes/powi.c        src/cephes/yn.c
src/cephes/exp10.c   src/cephes/jv.c      src/cephes/psi.c         src/cephes/zeta.c
src/cephes/exp2.c    src/cephes/k0.c      src/cephes/rgamma.c      src/cephes/zetac.c
src/cephes/expn.c    src/cephes/k1.c      src/cephes/round.c
)

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
