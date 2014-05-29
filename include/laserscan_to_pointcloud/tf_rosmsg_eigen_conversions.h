#pragma once

/**\file tf_rosmsg_eigen_conversions.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes

// ROS includes
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/Transform.h>

// external libs includes
#include <Eigen/Core>

// project includes

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace laserscan_to_pointcloud {

// ##############################################################################   tf2_convertions   #############################################################################
namespace tf_rosmsg_eigen_conversions {

void transformMsgToTF2(const geometry_msgs::Transform& msg, tf2::Transform& tf2);
void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::Transform& msg);

void transformMsgToTF2(const geometry_msgs::Quaternion& msg, tf2::Quaternion& tf2);
void transformTF2ToMsg(const tf2::Quaternion& tf2, geometry_msgs::Quaternion& msg);

void transformMsgToTF2(const geometry_msgs::Vector3& msg, tf2::Vector3& tf2);
void transformTF2ToMsg(const tf2::Vector3& tf2, geometry_msgs::Vector3& msg);

void transformMatrixToTF2(const Eigen::Matrix4f& matrix, tf2::Transform& tf2);

} /* namespace tf_rosmsg_eigen_conversions */
} /* namespace laserscan_to_pointcloud */
