set (CMAKE_CXX_STANDARD 14)

add_subdirectory(VertexAndEdge)

## Declare a C++ library
add_library(auto_complete_graph_lib
  OptimizableAutoCompleteGraph.cpp
  PriorLoaderInterface.cpp
        ACGPriorSE2.cpp
)


target_link_libraries(auto_complete_graph_lib auto_complete_graph_vertexedges_lib ${catkin_LIBRARIES} ${vodigrex_LIBRARIES})

add_subdirectory(Localization)
