cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(visp_naoqi)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

#if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=deprecated-declarations -std=c++11 -std=gnu++11")
#endif()

find_package(VISP REQUIRED)
find_package(Metapod)

find_package(Boost  COMPONENTS filesystem system REQUIRED)
find_package(naoqi_libqi)
find_package(naoqi_libqicore)


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

  include(cmake/metapod.cmake)

# Include Romeo Robot
  #set(WITH_METAPODFROMURDF TRUE)
  set(model_name "romeo")
  ADD_SAMPLEURDFMODEL(${model_name})
  include_directories("${CMAKE_BINARY_DIR}/include/metapod/models/romeo")

# Include Pepper Robot
  set(model_name_pepper "pepper")
  ADD_SAMPLEURDFMODEL(${model_name_pepper})
  include_directories("${CMAKE_BINARY_DIR}/include/metapod/models/pepper")

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


include_directories(
  ${VISP_INCLUDE_DIRS}
  ${naoqi_libqi_INCLUDE_DIRS}
  ${naoqi_libqicore_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


add_library(visp_naoqi
    include/visp_naoqi/vpNaoqiConfig.h
    include/visp_naoqi/vpNaoqiGrabber.h
    include/al/alvisiondefinitions.h
    include/al/from_any_value.cpp
    include/al/naoqi_image.hpp
    include/visp_naoqi/vpNaoqiRobot.h
    src/grabber/vpNaoqiGrabber.cpp
    src/robot/vpNaoqiRobot.cpp
)

target_link_libraries(visp_naoqi
    ${VISP_LIBRARIES}
    ${naoqi_libqi_LIBRARIES}
    ${naoqi_libqicore_LIBRARIES}
    ${Boost_LIBRARIES}
    )

if(USE_METAPOD)
  target_link_libraries(visp_naoqi metapod_${model_name})
  target_link_libraries(visp_naoqi metapod_${model_name_pepper})
endif()

subdirs(example)
#subdirs(tools/calibration)

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

