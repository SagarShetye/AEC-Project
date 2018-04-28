add_library(utility glutil.cpp glutil.h meshDeform.cpp meshDeform.h meshColour.h meshColour.cpp)	
target_include_directories(utility PUBLIC .)
target_link_libraries(utility ysclass geblkernel)
