add_library(MeanValueVisualizer meanValueVisualizer.cpp meanValueVisualizer.h)
target_link_libraries(MeanValueVisualizer objloader ysclass fslazywindow polygonalmesh)	
target_include_directories(MeanValueVisualizer PUBLIC .)

