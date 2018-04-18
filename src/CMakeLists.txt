add_subdirectory(VertexAndEdge)

## Declare a C++ library
add_library(auto_complete_graph_lib
        ACG.cpp
  OptimizableAutoCompleteGraph.cpp
  PriorLoaderInterface.cpp
        ACGPriorSE2.cpp

#   VisuACG.cpp
)


target_link_libraries(auto_complete_graph_lib auto_complete_graph_vertexedges_lib ${catkin_LIBRARIES} ${vodigrex_LIBRARIES})

add_subdirectory(Localization)


## Execs


add_executable(acg_node_review acg_node_review.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(acg_node_review ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} auto_complete_graph_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(acg_node_review
  ${catkin_LIBRARIES} auto_complete_graph_lib ${G2O_LIBS} ${vodigrex_LIBRARIES}
)

#add_executable(acg_node_logger acg_node_logger.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
#add_dependencies(acg_node_logger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} auto_complete_graph_generate_messages_cpp)

# Specify libraries to link a library or executable target against
#target_link_libraries(acg_node_logger
#  ${catkin_LIBRARIES} auto_complete_graph_lib ${G2O_LIBS} ${vodigrex_LIBRARIES}
#)

# add_executable(acg_node_feature acg_node_feature.cpp)
#
# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(acg_node_feature ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} auto_complete_graph_generate_messages_cpp)
#
# # Specify libraries to link a library or executable target against
# target_link_libraries(acg_node_feature
#   ${catkin_LIBRARIES} auto_complete_graph_lib ${G2O_LIBS} ${vodigrex_LIBRARIES}
# )


#add_executable(acg_node_rviz_full_fast acg_node_rviz_full_fast.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
#add_dependencies(acg_node_rviz_full_fast ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} auto_complete_graph_generate_messages_cpp)

# Specify libraries to link a library or executable target against
#target_link_libraries(acg_node_rviz_full_fast
#  ${catkin_LIBRARIES} auto_complete_graph_lib ${G2O_LIBS} ${vodigrex_LIBRARIES}
#)

