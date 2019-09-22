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


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})



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
        polar_to_cartesian_matrix_cache
        tf_rosmsg_eigen_conversions
        tf_collector
        laserscan_to_pointcloud
    CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_COMPONENTS}
    DEPENDS
        Boost
        EIGEN3
)



###########
## Build ##
###########

#===========
# includes =
#===========

include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)


#============
# libraries =
#============

add_library(polar_to_cartesian_matrix_cache
    src/polar_to_cartesian_matrix_cache.cpp
)

add_library(tf_rosmsg_eigen_conversions
    src/tf_rosmsg_eigen_conversions.cpp
)

add_library(tf_collector
    src/tf_collector.cpp
)

add_library(laserscan_to_pointcloud
    src/laserscan_to_pointcloud.cpp
)


#==============
# executables =
#==============

add_executable(laserscan_to_pointcloud_assembler
    src/laserscan_to_ros_pointcloud.cpp
    src/laserscan_to_pointcloud_assembler.cpp
    src/laserscan_to_pointcloud_assembler_node.cpp
)


#===============
# dependencies =
#===============

add_dependencies(polar_to_cartesian_matrix_cache
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(tf_rosmsg_eigen_conversions
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(tf_collector
    tf_rosmsg_eigen_conversions
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(laserscan_to_pointcloud
    tf_collector
    polar_to_cartesian_matrix_cache
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(laserscan_to_pointcloud_assembler
    laserscan_to_pointcloud
    ${PROJECT_NAME}_gencfg
    ${catkin_EXPORTED_TARGETS}
)


#=================
# libraries link =
#=================

target_link_libraries(polar_to_cartesian_matrix_cache
    ${catkin_LIBRARIES}
)

target_link_libraries(tf_rosmsg_eigen_conversions
    ${catkin_LIBRARIES}
)

target_link_libraries(tf_collector
    tf_rosmsg_eigen_conversions
    ${catkin_LIBRARIES}
)

target_link_libraries(laserscan_to_pointcloud
    tf_collector
    polar_to_cartesian_matrix_cache
    ${catkin_LIBRARIES}
)


#===================
# executables link =
#===================

target_link_libraries(laserscan_to_pointcloud_assembler
    laserscan_to_pointcloud
    ${catkin_LIBRARIES}
)



#############
## Install ##
#############

#============================
# Libraries and executables =
#============================

install(
    TARGETS
        polar_to_cartesian_matrix_cache
        tf_rosmsg_eigen_conversions
        tf_collector
        laserscan_to_pointcloud
        laserscan_to_pointcloud_assembler
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#================
# Header files =
#================

install(
    DIRECTORY
        include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#===============
# Launch files =
#===============

install(
    DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
