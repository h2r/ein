cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(ein)







#SET (CMAKE_CXX_COMPILER             "/home/stefie10/dev/pidrone/src/ein/timecpp.sh")
#SET (CMAKE_CXX_COMPILER             "/usr/bin/clang++-5.0")
#SET (CMAKE_C_COMPILER             "/usr/bin/clang-5.0")
#SET (CMAKE_AR      "/usr/bin/llvm-ar-5.0")
#SET (CMAKE_LINKER  "/usr/bin/llvm-ld-5.0")
#SET (CMAKE_NM      "/usr/bin/llvm-nm-5.0")
#SET (CMAKE_OBJDUMP "/usr/bin/llvm-objdump-5.0")
#SET (CMAKE_RANLIB  "/usr/bin/llvm-ranlib-5.0")

find_package(Boost REQUIRED COMPONENTS filesystem regex)

find_package(rosidl_default_generators REQUIRED)

find_package(ament_cmake REQUIRED)



LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/ein/cmake") 
MESSAGE(CMAKE_MODULE_PATH)
MESSAGE(${CMAKE_MODULE_PATH})

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(OpenCV)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(object_recognition_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(LAPACK REQUIRED)
find_package(GSL REQUIRED)
find_package(ZLIB REQUIRED)

set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL)
find_package(GLUT)
find_package(GLEW)
find_package(OpenCL)
find_package(tf2)
find_package(tf2_ros)







rosidl_generate_interfaces(einmsgs
  "msg/EinState.msg"
  "msg/EinConsole.msg"
  LIBRARY_NAME ${PROJECT_NAME}
  DEPENDENCIES object_recognition_msgs geometry_msgs
  )

rosidl_get_typesupport_target(cpp_typesupport_target einmsgs "rosidl_typesupport_cpp")
ament_export_dependencies(rosidl_default_runtime)









include_directories(include
	${ament_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
)

set(OPENMP_FLAG "-fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPENMP_FLAG} -g -std=c++11 -fpermissive")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPENMP_FLAG} -O3 -std=c++11 -fpermissive")
set(CMAKE_BUILD_TYPE Debug)


find_package (Qt5Widgets)

set (EinQtLib_src ${PROJECT_SOURCE_DIR}/src/qtgui/mainwindow.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/einwindow.cpp 
${PROJECT_SOURCE_DIR}/src/qtgui/window_QT.cpp  ${PROJECT_SOURCE_DIR}/src/qtgui/stackmodel.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/windowmanager.cpp
${PROJECT_SOURCE_DIR}/src/qtgui/wordaction.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/armwidget.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/gaussianmapwidget.cpp  
${PROJECT_SOURCE_DIR}/src/qtgui/gaussianmapwindow.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/streamviewerwindow.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/discrepancywindow.cpp  
${PROJECT_SOURCE_DIR}/src/qtgui/capturelineedit.cpp
${PROJECT_SOURCE_DIR}/src/qtgui/qt_util.cpp  
)
set (EinQtLib_hdr ${PROJECT_SOURCE_DIR}/src/qtgui/mainwindow.h ${PROJECT_SOURCE_DIR}/src/qtgui/einwindow.h ${PROJECT_SOURCE_DIR}/src/qtgui/window_QT.h ${PROJECT_SOURCE_DIR}/src/qtgui/stackmodel.h  ${PROJECT_SOURCE_DIR}/src/qtgui/windowmanager.h   ${PROJECT_SOURCE_DIR}/src/qtgui/wordaction.h ${PROJECT_SOURCE_DIR}/src/qtgui/armwidget.h
${PROJECT_SOURCE_DIR}/src/qtgui/wordaction.h ${PROJECT_SOURCE_DIR}/src/qtgui/gaussianmapwidget.h  ${PROJECT_SOURCE_DIR}/src/qtgui/streamviewerwindow.h 
${PROJECT_SOURCE_DIR}/src/qtgui/gaussianmapwindow.h
${PROJECT_SOURCE_DIR}/src/qtgui/discrepancywindow.h
${PROJECT_SOURCE_DIR}/src/qtgui/capturelineedit.h)
set (EinQtLib_ui  ${PROJECT_SOURCE_DIR}/src/qtgui/mainwindow.ui ${PROJECT_SOURCE_DIR}/src/qtgui/einwindow.ui ${PROJECT_SOURCE_DIR}/src/qtgui/armwidget.ui
                  ${PROJECT_SOURCE_DIR}/src/qtgui/gaussianmapwidget.ui   ${PROJECT_SOURCE_DIR}/src/qtgui/gaussianmapwindow.ui ${PROJECT_SOURCE_DIR}/src/qtgui/streamviewerwindow.ui ${PROJECT_SOURCE_DIR}/src/qtgui/discrepancywindow.ui)

qt5_wrap_cpp(EinQtLib_hdr_moc ${EinQtLib_hdr})
qt5_wrap_ui (EinQtLib_ui_moc  ${EinQtLib_ui})

include_directories (${PROJECT_SOURCE_DIR}/src)
include_directories (${PROJECT_BINARY_DIR})

add_library (EinQtLib SHARED 
    ${EinQtLib_src}
    ${EinQtLib_hdr_moc}
    ${EinQtLib_ui_moc}
    )

qt5_use_modules(EinQtLib Core Gui Widgets Test OpenGL)
  
ament_target_dependencies(EinQtLib OpenCV geometry_msgs rclcpp sensor_msgs tf2 tf2_ros rosgraph_msgs Eigen3 image_transport)
target_link_libraries(EinQtLib ${cpp_typesupport_target})



add_executable(einx src/ein.cpp src/ein_words.cpp src/config.cpp
src/ein_render.cpp src/ein_movement.cpp src/ein_vision_cycle.cpp  src/ein_misc.cpp 
src/ein_stream.cpp src/camera.cpp  
src/word.cpp src/gaussian_map.cpp src/gaussian_map_words.cpp src/eePose.cpp src/eigen_util.cpp  src/ein_util.cpp 
src/opencl/ein_opencl.cpp 
src/slu/math2d.cpp src/slu/gsl_utilities.cpp   src/base64.cpp src/compress.cpp
src/distributions.c src/randomkit.c src/cephes/airy.c    src/cephes/fdtr.c    src/cephes/kn.c          src/cephes/scipy_iv.c
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

target_link_libraries(einx
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
        ${GSL_LIBRARIES}
        ${LAPACK_LIBRARIES}
	${ZLIB_LIBRARIES}
        EinQtLib
	${cpp_typesupport_target}
)
ament_target_dependencies(einx rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV object_recognition_msgs visualization_msgs geometry_msgs tf2 tf2_ros Eigen3)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)



#qt5_use_modules(EinQtLib Core Gui Widgets Test OpenGL)

install(TARGETS einx DESTINATION lib/${PROJECT_NAME})
find_package(spot_msgs REQUIRED)

add_definitions(-DUSE_ROBOT_SPOT)
#add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)
add_library(EinSpot SHARED
  src/spot/ein_spot.cpp
)
ament_target_dependencies(EinSpot OpenCV geometry_msgs rclcpp sensor_msgs tf2 tf2_ros rosgraph_msgs Eigen3 image_transport spot_msgs std_srvs)
target_link_libraries(einx EinSpot Boost::filesystem Boost::regex)
target_link_libraries(EinSpot ${cpp_typesupport_target})


ament_package()
