## Declare a C++ library
add_library(auto_complete_graph_vertexedges_lib
  EdgeSE2Prior.cpp
        VertexSE2ACG.cpp
        VertexPointXYACG.cpp
        EdgeSE2ACG.cpp
        EdgeSE2PointXYACG.cpp
        EdgeXYPrior.cpp
        EdgePointXYACG.cpp
        EdgeNDTCell.cpp
 registration.cpp
)

target_link_libraries(auto_complete_graph_vertexedges_lib ${G2O_LIBS})