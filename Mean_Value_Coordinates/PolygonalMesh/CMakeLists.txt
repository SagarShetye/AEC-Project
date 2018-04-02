add_library(polygonalmesh polygonalmesh.cpp polygonalmesh.h polygonalmeshio.cpp)
	
target_include_directories(polygonalmesh PUBLIC .)
target_link_libraries(polygonalmesh ysclass)
