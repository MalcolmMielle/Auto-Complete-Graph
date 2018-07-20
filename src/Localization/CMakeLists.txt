add_library(auto_complete_graph_localization_lib
    ACG_localization.cpp
        ACGPriorXY.cpp
    )

target_link_libraries(auto_complete_graph_localization_lib auto_complete_graph_vertexedges_lib ${catkin_LIBRARIES} ${vodigrex_LIBRARIES})

add_dependencies(auto_complete_graph_localization_lib auto_complete_graph_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


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
target_link_libraries(graph_map_publisher_localization_ac
        ${catkin_LIBRARIES}
        )


add_executable(graph_map_fuser_node_ac graph_map_fuser_node.cpp)
# Add cmake target dependencies of the executable
# same as for the library above
#add_dependencies(graph_map_fuser_node_ac ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} auto_complete_graph_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(graph_map_fuser_node_ac
        ${catkin_LIBRARIES}
        )


add_executable(ndtmap_pub_testing ndtmap_pub_testing.cpp)

target_link_libraries(ndtmap_pub_testing ${catkin_LIBRARIES}  ${Boost_LIBRARIES})


add_executable(test_prior_localization test_prior_localization.cpp)
target_link_libraries(test_prior_localization ${catkin_LIBRARIES} auto_complete_graph_lib auto_complete_graph_localization_lib ${G2O_LIBS} ${vodigrex_LIBRARIES})
add_dependencies(test_prior_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} auto_complete_graph_generate_messages_cpp)


add_executable(publish_odom_to_tf publish_odometry_to_tf.cpp)
target_link_libraries(publish_odom_to_tf ${catkin_LIBRARIES} )
add_dependencies(publish_odom_to_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )