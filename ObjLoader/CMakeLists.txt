add_library(objloader objloader.cpp objloader.h)
target_link_libraries(objloader ysclass polygonalmesh)	
target_include_directories(objloader PUBLIC .)

