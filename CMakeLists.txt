
cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

project(visp_naoqi)

# this lets you find the qibuild cmake frameworl
find_package(qibuild)

find_package(VISP REQUIRED)

include_directories("include")
qi_create_lib(visp_naoqi 
  SRC
    include/visp_naoqi/vpNaoqiGrabber.h 
    include/visp_naoqi/vpNaoqiRobot.h 
    src/grabber/vpNaoqiGrabber.cpp
    src/robot/vpNaoqiRobot.cpp
)

qi_use_lib(visp_naoqi VISP ALCOMMON ALPROXIES ALVISION)
qi_stage_lib(visp_naoqi)

# examples
qi_create_bin(image_viewer_visp example/grabber/image_viewer_visp.cpp)
qi_use_lib(image_viewer_visp visp_naoqi)

qi_create_bin(image_viewer_opencv example/grabber/image_viewer_opencv.cpp)
qi_use_lib(image_viewer_opencv visp_naoqi)

qi_create_bin(motion example/robot/motion.cpp)
qi_use_lib(motion visp_naoqi)
