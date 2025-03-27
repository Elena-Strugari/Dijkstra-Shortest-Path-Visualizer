# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles\\Dijkstra_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\Dijkstra_autogen.dir\\ParseCache.txt"
  "Dijkstra_autogen"
  )
endif()
