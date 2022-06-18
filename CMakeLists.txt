cmake_minimum_required(VERSION 2.8.12)
project(auto_complete_graph)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")

# set(CMAKE_BUILD_TYPE debug)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ndt_feature_finder
  ndt_localization
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
  graph_map_custom_msgs

  message_generation
  geometry_msgs
  nav_msgs
  std_msgs
)

set(CMAKE_CXX_STANDARD 14)

find_package(glog REQUIRED)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS} include
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(vodigrex REQUIRED)

add_subdirectory(cmake_modules)

find_package(G2O REQUIRED)
find_package(SUITESPARSE REQUIRED)
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})
include_directories(${vodigrex_INCLUDE_DIRS})

set(G2O_LIBS ${G2O_CORE_LIBRARY} ${G2O_SOLVER_CSPARSE} ${CSPARSE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CHOLMOD_LIBRARY} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_SOLVER_SLAM2D_LINEAR} ${G2O_TYPES_SLAM2D})

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ACGMaps.msg
  ACGMapsOM.msg
  LocalizationMsg.msg
  GraphMapLocalizationMsg.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs ndt_map grid_map_msgs auto_complete_graph graph_map_custom_msgs
)

add_subdirectory(src)
add_subdirectory(tests)

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if you package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${G2O_LIBS} ${vodigrex_LIBRARIES} auto_complete_graph_lib auto_complete_graph_vertexedges_lib auto_complete_graph_localization_lib

  # LIBRARIES auto_complete_graph
  CATKIN_DEPENDS message_runtime ndt_feature_finder ndt_map roscpp rospy std_msgs message_generation occupancy_grid_utils grid_map_core grid_map_ros grid_map_msgs grid_map_cv graph_map graph_map_custom_msgs message_generation geometry_msgs std_msgs ndt_localization
  DEPENDS EIGEN OpenCV Boost vodigrex
)
