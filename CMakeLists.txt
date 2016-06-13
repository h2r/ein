cmake_minimum_required(VERSION 2.8.3)

project(ein)



LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/ein/cmake") 
MESSAGE(CMAKE_MODULE_PATH)
MESSAGE(${CMAKE_MODULE_PATH})



find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  std_msgs
  object_recognition_msgs
  geometry_msgs
  visualization_msgs
  tf_conversions
  genmsg
  cv_bridge
  image_transport
  message_generation
  pcl_ros
  baxter_core_msgs
  control_msgs
  kdl_parser
)



find_package(LAPACK REQUIRED)
find_package(GSL REQUIRED)
find_package(ZLIB REQUIRED)

find_package(OpenGL)
find_package(GLUT)
find_package(GLEW)
find_package(OpenCL)






add_message_files(
  FILES
  EinState.msg
  EinConsole.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  object_recognition_msgs
)

catkin_package(CATKIN_DEPENDS 
  eigen
  message_runtime)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

string(REPLACE "." "_" happy_version ${baxter_core_msgs_VERSION}     )
add_definitions(-DRETHINK_SDK_${happy_version})


include_directories(include
	${catkin_INCLUDE_DIRS}
	${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(OPENMP_FLAG "-fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPENMP_FLAG} -g -std=c++11 -fpermissive -Wno-deprecated-register -Wno-parentheses")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OPENMP_FLAG} -O3 -std=c++11 -fpermissive")
set(CMAKE_BUILD_TYPE Debug)


find_package (Qt5Widgets)

set (EinQtLib_src ${PROJECT_SOURCE_DIR}/src/qtgui/mainwindow.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/einwindow.cpp 
${PROJECT_SOURCE_DIR}/src/qtgui/window_QT.cpp  ${PROJECT_SOURCE_DIR}/src/qtgui/stackmodel.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/windowmanager.cpp
${PROJECT_SOURCE_DIR}/src/qtgui/wordaction.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/armwidget.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/gaussianmapwidget.cpp ${PROJECT_SOURCE_DIR}/src/qtgui/capturelineedit.cpp
)
set (EinQtLib_hdr ${PROJECT_SOURCE_DIR}/src/qtgui/mainwindow.h ${PROJECT_SOURCE_DIR}/src/qtgui/einwindow.h ${PROJECT_SOURCE_DIR}/src/qtgui/window_QT.h ${PROJECT_SOURCE_DIR}/src/qtgui/stackmodel.h  ${PROJECT_SOURCE_DIR}/src/qtgui/windowmanager.h   ${PROJECT_SOURCE_DIR}/src/qtgui/wordaction.h ${PROJECT_SOURCE_DIR}/src/qtgui/armwidget.h
${PROJECT_SOURCE_DIR}/src/qtgui/wordaction.h ${PROJECT_SOURCE_DIR}/src/qtgui/gaussianmapwidget.h
${PROJECT_SOURCE_DIR}/src/qtgui/capturelineedit.h)
set (EinQtLib_ui  ${PROJECT_SOURCE_DIR}/src/qtgui/mainwindow.ui ${PROJECT_SOURCE_DIR}/src/qtgui/einwindow.ui ${PROJECT_SOURCE_DIR}/src/qtgui/armwidget.ui
                  ${PROJECT_SOURCE_DIR}/src/qtgui/gaussianmapwidget.ui)

qt5_wrap_cpp(EinQtLib_hdr_moc ${EinQtLib_hdr})
qt5_wrap_ui (EinQtLib_ui_moc  ${EinQtLib_ui})

include_directories (${PROJECT_SOURCE_DIR}/src)
include_directories (${PROJECT_SOURCE_DIR}/src/opencl/oclNbody/inc/)
include_directories (${PROJECT_BINARY_DIR})

add_library (EinQtLib SHARED 
    ${EinQtLib_src}
    ${EinQtLib_hdr_moc}
    ${EinQtLib_ui_moc}
)


add_executable(ein src/ein.cpp src/ein_words.cpp src/config.cpp
src/ein_bandit.cpp src/ein_render.cpp src/ein_movement.cpp src/ein_servo.cpp src/ein_vision_cycle.cpp src/ein_scanning.cpp src/ein_misc.cpp src/ein_pick_place.cpp src/ein_ik.cpp src/ein_stream.cpp src/ikfast/ikfast_wrapper_left.cpp src/ikfast/ikfast_wrapper_right.cpp  src/ein_collision.cpp
src/word.cpp src/gaussian_map.cpp src/faces.cpp src/eePose.cpp src/eigen_util.cpp  src/ein_util.cpp src/ein_polyhedron.cpp src/ein_aibo.cpp src/ein_face_animation.cpp
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


target_link_libraries(ein
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	opencv_nonfree
        ${GSL_LIBRARIES}
        ${LAPACK_LIBRARIES}
	${ZLIB_LIBRARIES}
        EinQtLib
)


qt5_use_modules(EinQtLib Core Gui Widgets Test OpenGL)
message("OpenGL" ${OpenGL})
message("OpenGL_FOUND" ${OpenGL_FOUND})
message("OpenGL" ${OPENGL})
message("OpenGL_FOUND" ${OPENGL_FOUND})
if (OPENGL_FOUND AND GLEW_FOUND AND GLUT_FOUND AND OpenCL_FOUND)
MESSAGE("Building openCL with hardware acceleration.")
add_definitions(-DUSE_OPENCL)
add_library(EinOpenCLLib SHARED
src/opencl/oclNbody/src/cmd_arg_reader.cpp
src/opencl/oclNbody/src/oclBodySystemCpu.cpp
src/opencl/oclNbody/src/oclBodySystemOpencl.cpp
src/opencl/oclNbody/src/oclBodySystemOpenclLaunch.cpp
src/opencl/oclNbody/src/oclNbody.cpp
src/opencl/oclNbody/src/oclNbodyGold.cpp
src/opencl/oclNbody/src/oclRenderParticles.cpp
src/opencl/oclNbody/src/oclUtils.cpp
src/opencl/oclNbody/src/param.cpp
src/opencl/oclNbody/src/paramgl.cpp
src/opencl/oclNbody/src/shrUtils.cpp)


target_link_libraries(EinOpenCLLib
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${OpenCL_LIBRARY}
)




add_executable(nbody
src/opencl/oclNbody/src/cmd_arg_reader.cpp
src/opencl/oclNbody/src/oclBodySystemCpu.cpp
src/opencl/oclNbody/src/oclBodySystemOpencl.cpp
src/opencl/oclNbody/src/oclBodySystemOpenclLaunch.cpp
src/opencl/oclNbody/src/oclNbody.cpp
src/opencl/oclNbody/src/oclNbodyMain.cpp
src/opencl/oclNbody/src/oclNbodyGold.cpp
src/opencl/oclNbody/src/oclRenderParticles.cpp
src/opencl/oclNbody/src/oclUtils.cpp
src/opencl/oclNbody/src/param.cpp
src/opencl/oclNbody/src/paramgl.cpp
src/opencl/oclNbody/src/shrUtils.cpp)

target_link_libraries(nbody
        ${OPENGL_LIBRARIES}
        ${GLEW_LIBRARIES}
        ${GLUT_LIBRARIES}
        ${OpenCL_LIBRARY}
)
target_link_libraries(ein EinOpenCLLib)

else()
MESSAGE("Not building openCL with hardware accelleration.")
endif()



# Add a top-level "tags" target which includes all files in both
# the build and source versions of src/*.
#set_source_files_properties(tags PROPERTIES GENERATED true)
#add_custom_target(tags
#    COMMAND ctags -R --c++-kinds=+p --fields=+iaS --extra=+q  -e --exclude='.\#*'
#        ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
#    COMMAND ln -sf ${CMAKE_CURRENT_BINARY_DIR}/tags ${CMAKE_BINARY_DIR}
#    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# ...but only make it a dependency of the project if the ctags program
# is available, else it will fail to build on Windows.
#find_program(CTAGS_PATH ctags)
#if(CTAGS_PATH)
#    message(STATUS "Found ctags: ${CTAGS_PATH}")
#    add_dependencies(ein tags)
#endif(CTAGS_PATH)







# # Find includes in corresponding build directories
# set(CMAKE_INCLUDE_CURRENT_DIR ON)
# # Instruct CMake to run moc automatically when needed.

# # Find the QtWidgets library
# find_package(Qt5Widgets)

# qt5_wrap_ui(Cmake_form_hdr mainwindow.ui)


# qt5_wrap_cpp(helloworld_hdr_moc src/qtgui/mainwindow.h)
# qt5_wrap_ui (helloworld_ui_moc  src/qtgui/mainwindow.ui)
                                

# include_directories (${PROJECT_SOURCE_DIR})
# include_directories (${PROJECT_BINARY_DIR})


# # Tell CMake to create the helloworld executable
# add_executable(helloworld src/qtgui/main.cpp src/qtgui/mainwindow.cpp)

# # Use the Widgets module from Qt 5.
# target_link_libraries(helloworld Qt5::Widgets)
