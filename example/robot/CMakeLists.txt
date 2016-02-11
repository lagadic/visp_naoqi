set(source 
  head_control.cpp
  motion_romeo.cpp
  motion_nao.cpp
  test_jacobian.cpp
  eye_control.cpp
  test_manipulability.cpp
  hand_tracking.cpp
  ) 

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi)
endforeach()


