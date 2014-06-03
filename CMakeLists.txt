cmake_minimum_required(VERSION 2.8.3)
project(laserscan_to_pointcloud)


##############
## Packages ##
##############

set(${PROJECT_NAME}_CATKIN_COMPONENTS 
    roscpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    rosconsole
    dynamic_reconfigure
    cmake_modules
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_COMPONENTS})
find_package(Eigen REQUIRED)



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
    DEPENDS Eigen
)



###########
## Build ##
###########

include_directories(
    include
    ${Eigen_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_library(tf_rosmsg_eigen_conversions src/tf_rosmsg_eigen_conversions.cpp)
add_library(tf_collector src/tf_collector.cpp)
add_library(laserscan_to_pointcloud src/laserscan_to_pointcloud.cpp)

add_executable(laserscan_to_pointcloud_assembler
    src/laserscan_to_ros_pointcloud.cpp
    src/laserscan_to_pointcloud_assembler.cpp
    src/laserscan_to_pointcloud_assembler_node.cpp
)

add_dependencies(laserscan_to_pointcloud_assembler ${PROJECT_NAME}_gencfg)

target_link_libraries(tf_collector tf_rosmsg_eigen_conversions ${catkin_LIBRARIES})
target_link_libraries(laserscan_to_pointcloud tf_collector ${catkin_LIBRARIES})
target_link_libraries(laserscan_to_pointcloud_assembler laserscan_to_pointcloud ${catkin_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS laserscan_to_pointcloud laserscan_to_pointcloud_assembler
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



#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_laserscan_to_pointcloud.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
