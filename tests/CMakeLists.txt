add_executable(test_graph test_graph.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(test_graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(test_graph
  ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib
)