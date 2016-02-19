set(source 
  camera_calibration.cpp
  camera_calibration_ext.cpp
  ) 

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi)
endforeach()


