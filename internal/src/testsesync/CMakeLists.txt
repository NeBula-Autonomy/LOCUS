cmake_minimum_required(VERSION 2.8.3)
project(testsesync)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_utils
  parameter_utils
  point_cloud_filter
  pose_graph_msgs
  visualization_msgs
  message_generation
  pcl_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

find_package(GTSAM REQUIRED)
find_package(PCL 1.7 REQUIRED)

# find_package(Eigen3 REQUIRED
#    NO_MODULE #Should be optional, tells CMake to use config mode
#    PATHS /home/yunchang/code/gtsam/gtsam/3rdparty/Eigen/cmake # Tells CMake to look here
#    NO_DEFAULT_PATH #and don't look anywhere else
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    message_runtime
    geometry_utils
    parameter_utils
    point_cloud_filter
    pose_graph_msgs
    visualization_msgs
    std_msgs
    pcl_ros
)

include_directories(${GTSAM_INCLUDE_DIR})

link_directories(${GTSAM_LIBRARY_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

find_package(SESync REQUIRED COMPONENTS)
include_directories(${SESync_INCLUDE_DIR})
link_directories(${SESync_LIBRARY_DIRS})

add_executable(sesync src/main.cpp)
target_link_libraries(sesync ${catkin_LIBRARIES} SESync)