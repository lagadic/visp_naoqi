set(source 
  head_control.cpp
  motion_romeo.cpp
  motion_nao.cpp
  test_jacobian.cpp
  eye_control.cpp
  test_manipulability.cpp
  ) 

foreach(component ${VISP_LIB_COMPONENTS})
  if(component STREQUAL "visp_sinatrack")
    list(APPEND source hand_tracking.cpp)
  endif()
endforeach()


foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi)
endforeach()


  qi_create_bin(motion_pepper motion_pepper.cpp)
  qi_use_lib(motion_pepper visp_naoqi ALCOMMON QI)

  qi_create_bin(test_jacobian_pepper test_jacobian_pepper.cpp)
  qi_use_lib(test_jacobian_pepper visp_naoqi ALCOMMON QI)
