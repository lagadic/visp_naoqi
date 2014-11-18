# Try to find Metapod
#
# This file define
#
#  METAPOD_FOUND - Metapod is found

find_program(Metapod_EXECUTABLE metapodfromurdf HINTS ${PROJECT_BINARY_DIR}/metapodfromurdf)

if(Metapod_EXECUTABLE)
  set(METAPOD_FOUND TRUE)
  message("Metapod found")
else()
  set(METAPOD_FOUND FALSE)
  message("Metapod not found")
endif()

