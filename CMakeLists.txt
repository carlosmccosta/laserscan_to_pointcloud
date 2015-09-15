cmake_minimum_required(VERSION 2.8.3)
project(laserscan_to_pointcloud)


##############
## Packages ##
##############

set(${PROJECT_NAME}_CATKIN_COMPONENTS 
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    rosconsole
    dynamic_reconfigure
    cmake_modules
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)



###################################
## catkin specific configuration ##
###################################

generate_dynamic_reconfigure_options(
    cfg/LaserScanToPointcloudAssembler.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES tf_rosmsg_eigen_conversions tf_collector laserscan_to_pointcloud
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS
        Eigen
        Boost
)



###########
## Build ##
###########

include_directories(
    include
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(tf_rosmsg_eigen_conversions src/tf_rosmsg_eigen_conversions.cpp)
add_library(tf_collector src/tf_collector.cpp)
add_library(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)
add_library(polar_to_cartesian_matrix_cache src/polar_to_cartesian_matrix_cache.cpp)

add_executable(laserscan_to_pointcloud_assembler
    src/laserscan_to_ros_pointcloud.cpp
    src/laserscan_to_pointcloud_assembler.cpp
    src/laserscan_to_pointcloud_assembler_node.cpp
)

add_dependencies(laserscan_to_pointcloud_assembler ${PROJECT_NAME}_gencfg)

target_link_libraries(tf_collector tf_rosmsg_eigen_conversions ${catkin_LIBRARIES})
target_link_libraries(laserscan_to_pointcloud tf_collector polar_to_cartesian_matrix_cache ${catkin_LIBRARIES})
target_link_libraries(laserscan_to_pointcloud_assembler laserscan_to_pointcloud ${catkin_LIBRARIES})

