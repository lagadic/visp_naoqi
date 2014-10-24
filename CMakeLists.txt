cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(visp_naoqi)

set(CMAKE_VERBOSE_MAKEFILE TRUE)

# this lets you find the qibuild cmake frameworl
find_package(qibuild)

find_package(VISP REQUIRED)

option(USE_METAPOD "Enable Metapod usage" ON)

# include metapod library
if(USE_METAPOD)
  include(metapod.cmake)
  #define and search for metapod library
  #set(WITH_METAPODFROMURDF TRUE)
  set(model_name "romeo")

  ADD_SAMPLEURDFMODEL(${model_name})

  include_directories("${CMAKE_BINARY_DIR}/include/metapod/models/romeo")
  add_definitions("-DUSE_METAPOD")
endif()


include_directories("include")
qi_create_lib(visp_naoqi 
  SRC
    include/visp_naoqi/vpNaoqiGrabber.h 
    include/visp_naoqi/vpNaoqiRobot.h 
    src/grabber/vpNaoqiGrabber.cpp
    src/robot/vpNaoqiRobot.cpp
)

qi_use_lib(visp_naoqi VISP ALCOMMON ALPROXIES ALVISION)
if(USE_METAPOD)
  target_link_libraries(visp_naoqi metapod_${model_name})
endif()
qi_stage_lib(visp_naoqi)

# examples
qi_create_bin(image_viewer_visp example/grabber/image_viewer_visp.cpp)
qi_use_lib(image_viewer_visp visp_naoqi)

qi_create_bin(image_viewer_opencv example/grabber/image_viewer_opencv.cpp)
qi_use_lib(image_viewer_opencv visp_naoqi)

qi_create_bin(motion example/robot/motion.cpp)
qi_use_lib(motion visp_naoqi)

qi_create_bin(head_control example/robot/head_control.cpp)
qi_use_lib(head_control visp_naoqi)

qi_create_bin(test_jacobian example/robot/test_jacobian.cpp)
qi_use_lib(test_jacobian visp_naoqi)


# doxygen
find_package(Doxygen)
if (DOXYGEN_FOUND)
  ## check if dot available
  if(DOXYGEN_DOT_EXECUTABLE)
   set(PROJECT_HAVE_DOT "yes")
  else()
    set(PROJECT_HAVE_DOT "no")
  endif()
  ## we need latex for doxygen because of the formulas
  find_package(LATEX)
   
  configure_file(${PROJECT_SOURCE_DIR}/doc/config-doxygen.cmake
    ${PROJECT_BINARY_DIR}/doc/config-doxygen
    @ONLY )

  add_custom_target(html-doc ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/doc/config-doxygen) 
endif()

