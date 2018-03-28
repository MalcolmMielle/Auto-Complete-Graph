cmake_minimum_required(VERSION 2.8.3)
project(auto_complete_graph)



set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

#set(CMAKE_BUILD_TYPE debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ndt_feature
  ndt_feature_finder
  ndt_localization
  flirtlib_ros 
  ndt_map
  roscpp
  rospy
  std_msgs
  message_generation
  occupancy_grid_utils
  grid_map_core
  grid_map_ros
  grid_map_msgs
  grid_map_cv
  
  graph_map
  
  message_generation
  geometry_msgs
   nav_msgs
  std_msgs
)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} include
)






# System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED )
find_package(vodigrex REQUIRED )


add_subdirectory(cmake_modules)

find_package(G2O REQUIRED)
find_package(SUITESPARSE REQUIRED)
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})
include_directories(${vodigrex_INCLUDE_DIRS})

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

##DEBUG MODE
#set(G2O_LIBS ${G2O_CORE_LIBRARY_DEBUG} ${G2O_SOLVER_CSPARSE_DEBUG}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY_DEBUG} ${CHOLMOD_LIBRARY_DEBUG}  ${G2O_SOLVER_CSPARSE_EXTENSION_DEBUG} ${G2O_SOLVER_SLAM2D_LINEAR_DEBUG} ${G2O_TYPES_SLAM2D_DEBUG})
##NON DEBUG MODE
set(G2O_LIBS ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE}  ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY}  ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ACGMaps.msg
  ACGMapsOM.msg
  LocalizationMsg.msg
  GraphMapLocalizationMsg.msg
#   ACG.msg
#   VertexSE2RobotPoseMsg.msg
#   VertexSE2Msg.msg
#   VertexLandmarkNDTMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs ndt_map grid_map_msgs graph_map auto_complete_graph
  # nav_msgs
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


#This is only for QtCreator to see the .hpp files in prject
#FILE(GLOB files "${CMAKE_CURRENT_SOURCE_DIR}/include/auto_complete_graph/*.hpp")
#add_custom_target(headers SOURCES ${files})




## Declare a C++ library
# add_library(auto_complete_graph_lib
#   src/ACG.cpp
#   src/OptimizableAutoCompleteGraph.cpp
#   src/PriorLoaderInterface.cpp
#   src/ACG_feature.cpp
# #   src/VisuACG.cpp
# )
# 
# target_link_libraries(auto_complete_graph_lib ${catkin_LIBRARIES} ${G2O_LIBS} ${vodigrex_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(auto_complete_graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## TODO : find a way to remove this by linking it where it should be in ndt_feature
# include_directories(/usr/local/include/flirtlib)


## Declare a C++ executable
# # # # # # add_executable(acg_node src/acg_node.cpp)
# # # # # # 
# # # # # # ## Add cmake target dependencies of the executable
# # # # # # ## same as for the library above
# # # # # # add_dependencies(acg_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
# # # # # # 
# # # # # # ## Specify libraries to link a library or executable target against
# # # # # # target_link_libraries(acg_node
# # # # # #   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib ${G2O_LIBS}
# # # # # # )
# # # # # # 
# # # # # # add_executable(acg_node_rviz src/acg_node_rviz.cpp)
# # # # # # 
# # # # # # ## Add cmake target dependencies of the executable
# # # # # # ## same as for the library above
# # # # # # add_dependencies(acg_node_rviz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${G2O_LIBS})
# # # # # # 
# # # # # # ## Specify libraries to link a library or executable target against
# # # # # # target_link_libraries(acg_node_rviz
# # # # # #   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib ${G2O_LIBS}
# # # # # # )
# # # # # # 
# # # # # # add_executable(acg_node_rviz_noise src/acg_node_rviz_noise.cpp)
# # # # # # 
# # # # # # ## Add cmake target dependencies of the executable
# # # # # # ## same as for the library above
# # # # # # add_dependencies(acg_node_rviz_noise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# # # # # # 
# # # # # # ## Specify libraries to link a library or executable target against
# # # # # # target_link_libraries(acg_node_rviz_noise
# # # # # #   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib ${G2O_LIBS}
# # # # # # )
# # # # # # 
# # # # # # add_executable(acg_node_rviz_noptimi src/acg_node_rviz_noptimi.cpp)
# # # # # # 
# # # # # # ## Add cmake target dependencies of the executable
# # # # # # ## same as for the library above
# # # # # # add_dependencies(acg_node_rviz_noptimi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# # # # # # 
# # # # # # ## Specify libraries to link a library or executable target against
# # # # # # target_link_libraries(acg_node_rviz_noptimi
# # # # # #   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib ${G2O_LIBS}
# # # # # # )

# add_executable(acg_node_rviz_noptimi_full src/acg_node_rviz_nooptimi_full.cpp)
# 
# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(acg_node_rviz_noptimi_full ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 
# # Specify libraries to link a library or executable target against
# target_link_libraries(acg_node_rviz_noptimi_full
#   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib ${G2O_LIBS}
# )



# add_executable(test_match tests/test_match.cpp)
# 
# # Add cmake target dependencies of the executable
# # same as for the library above
# add_dependencies(test_match ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 
# # Specify libraries to link a library or executable target against
# target_link_libraries(test_match
#   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib ${G2O_LIBS}
# )

# add_executable(acg_node_rviz_lab_fast src/acg_node_rviz_lab_fast.cpp)

# Add cmake target dependencies of the executable
# same as for the library above
# add_dependencies(acg_node_rviz_lab_fast ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link a library or executable target against
# target_link_libraries(acg_node_rviz_lab_fast
#   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib ${G2O_LIBS}
# )


# add_executable(acg_node_rviz_full src/acg_node_rviz_full.cpp)
# 
# ## Add cmake target dependencies of the executable
# ## same as for the library above
# add_dependencies(acg_node_rviz_full ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# 
# target_link_libraries(acg_node_rviz_full
#   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib ${G2O_LIBS}
# )
# # # # # # # # # # # 
# # # # # # # # # # # 
# # # # # # # # # # # add_executable(acg_node_rviz_full_noise src/acg_node_rviz_full_noise.cpp)
# # # # # # # # # # # 
# # # # # # # # # # # ## Add cmake target dependencies of the executable
# # # # # # # # # # # ## same as for the library above
# # # # # # # # # # # add_dependencies(acg_node_rviz_full_noise ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# # # # # # # # # # # 
# # # # # # # # # # # ## Specify libraries to link a library or executable target against
# # # # # # # # # # # target_link_libraries(acg_node_rviz_full_noise
# # # # # # # # # # #   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib
# # # # # # # # # # # )
# # # # # # # # # # # 
# # # # # # # # # # # add_executable(test_affine_transfo tests/test_affine_transfo.cpp)
# # # # # # # # # # # 
# # # # # # # # # # # ## Add cmake target dependencies of the executable
# # # # # # # # # # # ## same as for the library above
# # # # # # # # # # # add_dependencies(test_affine_transfo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# # # # # # # # # # # 
# # # # # # # # # # # ## Specify libraries to link a library or executable target against
# # # # # # # # # # # target_link_libraries(test_affine_transfo
# # # # # # # # # # #   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} auto_complete_graph_lib -lboost_unit_test_framework
# # # # # # # # # # # )
# # # # # # # # # # # 
# # # # # # # # # # # add_executable(test_covvalue tests/test_covvalue.cpp)
# # # # # # # # # # # 
# # # # # # # # # # # ## Add cmake target dependencies of the executable
# # # # # # # # # # # ## same as for the library above
# # # # # # # # # # # add_dependencies(test_covvalue ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# # # # # # # # # # # 
# # # # # # # # # # # ## Specify libraries to link a library or executable target against
# # # # # # # # # # # target_link_libraries(test_covvalue
# # # # # # # # # # #   ${catkin_LIBRARIES} ${NDT_FEATURE_LIBS} -lboost_unit_test_framework
# # # # # # # # # # # )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS auto_complete_graph auto_complete_graph_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_auto_complete_graph.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#useless commment

#
#
#add_executable(graph_map_publisher_localization src/Localization/graph_map_publisher_localization.cpp)
#
## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(graph_map_publisher_localization auto_complete_graph_generate_messages_cpp)
#
## Specify libraries to link a library or executable target against
#target_link_libraries(graph_map_publisher_localization
#        ${catkin_LIBRARIES}
#        )







add_subdirectory(src)
add_subdirectory(tests)
#add_subdirectory(include/auto_complete_graph/Localization)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

        INCLUDE_DIRS include
        LIBRARIES ${G2O_LIBS} ${vodigrex_LIBRARIES} auto_complete_graph_lib auto_complete_graph_vertexedges_lib auto_complete_graph_localization_lib
        #  LIBRARIES auto_complete_graph
        CATKIN_DEPENDS message_runtime ndt_feature ndt_feature_finder flirtlib_ros ndt_map roscpp rospy std_msgs message_generation occupancy_grid_utils grid_map_core grid_map_ros grid_map_msgs grid_map_cv graph_map message_generation geometry_msgs std_msgs ndt_localization
        DEPENDS EIGEN OpenCV Boost vodigrex
)


