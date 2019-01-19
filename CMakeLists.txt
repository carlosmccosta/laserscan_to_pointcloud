cmake_minimum_required(VERSION 2.8.3)
project(laserscan_to_pointcloud)


##############
## Packages ##
##############

set(${PROJECT_NAME}_CATKIN_COMPONENTS 
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    rosconsole
    roscpp
    sensor_msgs
    tf2
    tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)



###################################
## catkin specific configuration ##
###################################

generate_dynamic_reconfigure_options(
    cfg/LaserScanToPointcloudAssembler.cfg
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        tf_rosmsg_eigen_conversions
        tf_collector laserscan_to_pointcloud
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS
        Boost
        EIGEN3
)



###########
## Build ##
###########

#==================
# includes
#==================

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)


#==================
# libraries
#==================

add_library(tf_rosmsg_eigen_conversions
    src/tf_rosmsg_eigen_conversions.cpp
)

add_library(tf_collector
    src/tf_collector.cpp
)

add_library(laserscan_to_pointcloud
    src/laserscan_to_pointcloud.cpp
)

add_library(polar_to_cartesian_matrix_cache
    src/polar_to_cartesian_matrix_cache.cpp
)


#==================
# executables
#==================

add_executable(laserscan_to_pointcloud_assembler
    src/laserscan_to_ros_pointcloud.cpp
    src/laserscan_to_pointcloud_assembler.cpp
    src/laserscan_to_pointcloud_assembler_node.cpp
)


#==================
# dependencies
#==================

add_dependencies(tf_rosmsg_eigen_conversions
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(tf_collector
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(laserscan_to_pointcloud
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(polar_to_cartesian_matrix_cache
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(laserscan_to_pointcloud_assembler
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)


#==================
# libraries link
#==================

target_link_libraries(tf_collector
    tf_rosmsg_eigen_conversions
    ${catkin_LIBRARIES}
)

target_link_libraries(laserscan_to_pointcloud
    tf_collector
    polar_to_cartesian_matrix_cache
    ${catkin_LIBRARIES}
)


#==================
# executables link
#==================

target_link_libraries(laserscan_to_pointcloud_assembler
    laserscan_to_pointcloud
    ${catkin_LIBRARIES}
)
