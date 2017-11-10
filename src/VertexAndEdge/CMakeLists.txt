## Declare a C++ library
add_library(auto_complete_graph_vertexedges_lib
  EdgeSE2Prior.cpp
)

target_link_libraries(auto_complete_graph_vertexedges_lib ${G2O_LIBS})