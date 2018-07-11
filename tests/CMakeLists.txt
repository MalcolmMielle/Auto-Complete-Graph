## add_executable(test_graph test_graph.cpp)
##
## # Add cmake target dependencies of the executable
## # same as for the library above
## add_dependencies(test_graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
##
## # Specify libraries to link a library or executable target against
## target_link_libraries(test_graph
##   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib
## )
#
## add_executable(test_graph_other test_other_graph.cpp)
#
### Add cmake target dependencies of the executable
### same as for the library above
## add_dependencies(test_graph_other ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
### Specify libraries to link a library or executable target against
## target_link_libraries(test_graph_other
##   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib
## )
#
#add_executable(test_martin test_martin.cpp)
#
## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(test_martin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
## Specify libraries to link a library or executable target against
#target_link_libraries(test_martin
#  ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib ${G2O_LIBS}
#)
#
#add_executable(test_graph_noise test_graph_noise.cpp)
#
## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(test_graph_noise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
#
## Specify libraries to link a library or executable target against
#target_link_libraries(test_graph_noise
#  ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib ${G2O_LIBS}
#)
#
## # # # # # # add_executable(test_links test_links.cpp)
## # # # # # #
## # # # # # # # Add cmake target dependencies of the executable
## # # # # # # # same as for the library above
## # # # # # # add_dependencies(test_links ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## # # # # # #
## # # # # # # # Specify libraries to link a library or executable target against
## # # # # # # target_link_libraries(test_links
## # # # # # #   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib  ${G2O_LIBS}
## # # # # # # )
#
#
#
#add_executable(test_grid_map test_grid_map.cpp)
#
## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(test_grid_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
## Specify libraries to link a library or executable target against
#target_link_libraries(test_grid_map
#  ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib -lboost_unit_test_framework ${G2O_LIBS}
#)
#
add_executable(test_transfo test_transfo.cpp)
#
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(test_transfo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
## Specify libraries to link a library or executable target against
target_link_libraries(test_transfo
  ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib -lboost_unit_test_framework ${G2O_LIBS}
)


add_executable(test_graph_map test_graph_map.cpp)

# Add cmake target dependencies of the executable
# same as for the library above

# Specify libraries to link a library or executable target against
target_link_libraries(test_graph_map
        ${catkin_LIBRARIES}
        )


add_executable(test_graph_localization test_graph_localization.cpp)
#
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(test_graph_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
## Specify libraries to link a library or executable target against
target_link_libraries(test_graph_localization
        ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib auto_complete_graph_localization_lib -lboost_unit_test_framework ${G2O_LIBS}
        )


