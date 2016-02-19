set(source 
image_viewer_visp.cpp
image_viewer_visp_2cameras.cpp
image_viewer_opencv.cpp
image_viewer_visp_color.cpp
image_viewer_opencv_2cameras.cpp)

foreach(src ${source})
  get_filename_component(binary ${src} NAME_WE)
  qi_create_bin(${binary} ${src})
  qi_use_lib(${binary} visp_naoqi)
endforeach()


