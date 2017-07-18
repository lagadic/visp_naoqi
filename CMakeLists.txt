cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(visp_naoqi)

# By default set release configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE String "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

set(visp_naoqi_MAJOR_VERSION 1)
set(visp_naoqi_MINOR_VERSION 0)
set(visp_naoqi_PATCH_VERSION 0)
set(visp_naoqi_VERSION ${visp_naoqi_MAJOR_VERSION}.${visp_naoqi_MINOR_VERSION}.${visp_naoqi_PATCH_VERSION})

find_file(GNU_INSTALL_DIRS_FROM_CMAKE NAMES GNUInstallDirs.cmake PATHS ${CMAKE_ROOT}/Modules)
mark_as_advanced(GNU_INSTALL_DIRS_FROM_CMAKE)
if(GNU_INSTALL_DIRS_FROM_CMAKE)
  include(${CMAKE_ROOT}/Modules/GNUInstallDirs.cmake)
else()
  #include(cmake/GNUInstallDirs.cmake)
  message("Cannot find GNUInstallDirs.cmake")
endif()

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

#if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=deprecated-declarations -std=c++11 -std=gnu++11")
#endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

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
  set(model_name_romeo "romeo")
  ADD_SAMPLEURDFMODEL(${model_name_romeo})
  include_directories("${CMAKE_BINARY_DIR}/include/metapod/models/romeo")

# Include Pepper Robot
  set(model_name_pepper "pepper")
  ADD_SAMPLEURDFMODEL(${model_name_pepper})
  include_directories("${CMAKE_BINARY_DIR}/include/metapod/models/pepper")

  set(VISP_NAOQI_HAVE_MATAPOD "yes")  # for header vpNaoqiConfig.h
endif()

#include_directories("include")
include_directories(
        "${PROJECT_SOURCE_DIR}/src/common"
        "${PROJECT_SOURCE_DIR}/include"
)
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

set(SOURCES
  src/common/vpFaceTrackerOkao.cpp
  src/common/vpPepperFollowPeople.cpp
  src/common/vpServoArm.cpp
  src/common/vpServoHead.cpp
  src/grabber/vpNaoqiGrabber.cpp
  src/robot/vpNaoqiRobot.cpp
  include/al/from_any_value.cpp
)

set(HEADERS_VISP_NAOQI
  include/visp_naoqi/common/vpFaceTrackerOkao.h
  include/visp_naoqi/common/vpPepperFollowPeople.h
  include/visp_naoqi/common/vpServoArm.h
  include/visp_naoqi/common/vpServoHead.h
  include/visp_naoqi/vpNaoqiGrabber.h
  include/visp_naoqi/vpNaoqiRobot.h
)
set(HEADERS_AL
  include/al/alvisiondefinitions.h
  include/al/naoqi_image.hpp
)

add_library(visp_naoqi SHARED
  ${SOURCES}
  ${HEADERS_VISP_NAOQI}
  ${HEADERS_AL}
  include/visp_naoqi/vpNaoqiConfig.h
)

target_link_libraries(visp_naoqi
    ${VISP_LIBRARIES}
    ${naoqi_libqi_LIBRARIES}
    ${naoqi_libqicore_LIBRARIES}
    ${Boost_LIBRARIES}
    )

if(USE_METAPOD)
  target_link_libraries(visp_naoqi metapod_${model_name_romeo})
  target_link_libraries(visp_naoqi metapod_${model_name_pepper})
endif()

subdirs(example)
#subdirs(tools/calibration)
subdirs(demos/gaze)
subdirs(demos/follow-me)

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


install(TARGETS visp_naoqi metapod_${model_name_romeo} metapod_${model_name_pepper}
  # IMPORTANT: Add the visp_naoqi library to the "export-set"
  EXPORT visp_naoqiTargets
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT libs
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT libs
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT dev
  PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/visp_naoqi" COMPONENT dev)

# headers installation
install(FILES ${HEADERS_VISP_NAOQI} ${CMAKE_BINARY_DIR}/include/visp_naoqi/vpNaoqiConfig.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/visp_naoqi
  COMPONENT dev  )
install(FILES ${HEADERS_AL}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/al
  COMPONENT dev
  )

install(FILES include/visp_naoqi/camera.xml
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/visp_naoqi
  COMPONENT dev
  )


# Add all targets to the build-tree export set
export(TARGETS visp_naoqi metapod_${model_name_romeo} metapod_${model_name_pepper} FILE "${PROJECT_BINARY_DIR}/visp_naoqiTargets.cmake")
#export(TARGETS visp_naoqi FILE "${PROJECT_BINARY_DIR}/visp_naoqiTargets.cmake")
#export(TARGETS metapod_${model_name_romeo} FILE "${PROJECT_BINARY_DIR}/metapod_${model_name_romeo}Targets.cmake")
#export(TARGETS metapod_${model_name_pepper} FILE "${PROJECT_BINARY_DIR}/metapod_${model_name_pepper}Targets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE visp_naoqi)

# --------------------------------------------------------------------------------------------
#  Part 1/3: ${BIN_DIR}/visp_naoqiConfig.cmake -> For use *with* "make"
# -------------------------------------------------------------------------------------------

set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include" "${PROJECT_BINARY_DIR}/include" )

configure_file(cmake/visp_naoqiConfig.cmake.in "${PROJECT_BINARY_DIR}/visp_naoqiConfig.cmake" @ONLY)
# ... for both
configure_file(cmake/visp_naoqiConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/visp_naoqiConfigVersion.cmake" @ONLY)

# --------------------------------------------------------------------------------------------
#  Part 2/3: ${BIN_DIR}/unix-install/VISPConfig.cmake -> For use *with* "make install"
# -------------------------------------------------------------------------------------------

set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}")

configure_file(cmake/visp_naoqiConfig.cmake.in "${PROJECT_BINARY_DIR}/tmp-install/visp_naoqiConfig.cmake" @ONLY)
# ... for both
configure_file(cmake/visp_naoqiConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/tmp-install/visp_naoqiConfigVersion.cmake" @ONLY)

# Install the visp_naoqiConfig.cmake and visp_naoqiConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}/tmp-install/visp_naoqiConfig.cmake"
  "${PROJECT_BINARY_DIR}/tmp-install/visp_naoqiConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/visp_naoqi" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT visp_naoqiTargets DESTINATION
  "${CMAKE_INSTALL_LIBDIR}/cmake/visp_naoqi" COMPONENT dev)
