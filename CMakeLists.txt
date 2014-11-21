cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(visp_naoqi)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# this lets you find the qibuild cmake frameworl
find_package(qibuild)

find_package(VISP REQUIRED)

find_package(Metapod)

if(METAPOD_FOUND)
  option(USE_METAPOD "Enable Metapod usage" ON)
else()
  set(USE_METAPOD OFF)
endif()

# include metapod library
if(USE_METAPOD)
  find_package(Eigen3)
  if(EIGEN3_FOUND)
    include_directories(${EIGEN3_INCLUDE_DIR})
  endif()

  #set(WITH_METAPODFROMURDF TRUE)
  set(model_name "romeo")
  include(cmake/metapod.cmake)

  ADD_SAMPLEURDFMODEL(${model_name})

  include_directories("${CMAKE_BINARY_DIR}/include/metapod/models/romeo")

  set(VISP_NAOQI_HAVE_MATAPOD "yes")  # for header vpNaoqiConfig.h
endif()


include_directories("include")
include_directories("${CMAKE_BINARY_DIR}/include")

set(VISP_NAOQI_INTRINSIC_CAMERA_FILE ${PROJECT_SOURCE_DIR}/include/visp_naoqi/camera.xml) # for header vpNaoqiConfig.h
set(VISP_NAOQI_EXTRINSIC_CAMERA_FILE ${PROJECT_SOURCE_DIR}/include/visp_naoqi/eMc.xml)
set(VISP_NAOQI_GENERAL_M_FILE ${PROJECT_SOURCE_DIR}/include/visp_naoqi/learned_oMh.xml)

configure_file(${PROJECT_SOURCE_DIR}/include/visp_naoqi/vpNaoqiConfig.h.cmake
  ${CMAKE_BINARY_DIR}/include/visp_naoqi/vpNaoqiConfig.h
)

qi_create_lib(visp_naoqi 
  SRC
    include/visp_naoqi/vpNaoqiConfig.h
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

qi_create_bin(camera_calibration tools/calibration/camera_calibration.cpp)
qi_use_lib(camera_calibration visp_naoqi)

qi_create_bin(camera_calibration_ext tools/calibration/camera_calibration_ext.cpp)
qi_use_lib(camera_calibration_ext visp_naoqi)

qi_create_bin(LoadM example/parserHomogeneousMatrix/loadM.cpp)
qi_use_lib(LoadM visp_naoqi )

qi_create_bin(writeM example/parserHomogeneousMatrix/writeM.cpp)
qi_use_lib(writeM visp_naoqi )

qi_create_bin(dat_to_xml example/parserHomogeneousMatrix/dat_to_xml.cpp)
qi_use_lib(dat_to_xml visp_naoqi )

# Propagate in subdirs
subdirs(example/robot)

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

