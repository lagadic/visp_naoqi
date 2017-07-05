set(source
  pepper_gaze_head.cpp
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

