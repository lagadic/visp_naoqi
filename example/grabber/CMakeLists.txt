set(source 
image_viewer_visp.cpp
#image_viewer_visp_2cameras.cpp
image_viewer_opencv.cpp
#image_viewer_visp_color.cpp
#image_viewer_opencv_2cameras.cpp
)



foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  add_executable(${binary} ${src})

  target_link_libraries(${binary}
    ${naoqi_libqi_LIBRARIES}
    ${naoqi_libqicore_LIBRARIES}
    ${Boost_LIBRARIES}
    visp_naoqi
    )
endforeach()
