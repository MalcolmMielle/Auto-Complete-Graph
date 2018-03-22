add_library(auto_complete_graph_localization_lib
    ACG_localization.cpp
    )


add_executable(acg_node_localization acg_node_localization.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(acg_node_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} auto_complete_graph_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(acg_node_localization
        ${catkin_LIBRARIES} auto_complete_graph_lib auto_complete_graph_localization_lib ${G2O_LIBS} ${vodigrex_LIBRARIES}
        )

add_executable(graph_map_publisher_localization_ac graph_map_publisher_localization.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(graph_map_publisher_localization_ac ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} auto_complete_graph_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(graph_map_publisher_localization
        ${catkin_LIBRARIES}
        )


add_executable(graph_map_fuser_node graph_map_fuser_node.cpp)
# Add cmake target dependencies of the executable
# same as for the library above
#add_dependencies(graph_map_fuser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} auto_complete_graph_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(graph_map_fuser_node
        ${catkin_LIBRARIES}
        )


add_executable(ndtmap_pub_testing ndtmap_pub_testing.cpp)

target_link_libraries(ndtmap_pub_testing ${catkin_LIBRARIES}  ${Boost_LIBRARIES})
