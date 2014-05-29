/**\file tf2_conversions.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "laserscan_to_pointcloud/tf_rosmsg_eigen_conversions.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace laserscan_to_pointcloud {
namespace tf_rosmsg_eigen_conversions {

void transformMsgToTF2(const geometry_msgs::Transform& msg, tf2::Transform& tf2) {
	tf2.setRotation(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w));
	tf2.setOrigin(tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
}

void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::Transform& msg) {
	msg.rotation.x = tf2.getRotation().x();
	msg.rotation.y = tf2.getRotation().y();
	msg.rotation.z = tf2.getRotation().z();
	msg.rotation.w = tf2.getRotation().w();
	msg.translation.x = tf2.getOrigin().x();
	msg.translation.y = tf2.getOrigin().y();
	msg.translation.z = tf2.getOrigin().z();
}


void transformMsgToTF2(const geometry_msgs::Quaternion& msg, tf2::Quaternion& tf2) {
	tf2.setX(msg.x);
	tf2.setY(msg.y);
	tf2.setZ(msg.z);
	tf2.setW(msg.w);
}

void transformTF2ToMsg(const tf2::Quaternion& tf2, geometry_msgs::Quaternion& msg) {
	msg.x = tf2.getX();
	msg.y = tf2.getY();
	msg.z = tf2.getX();
	msg.w = tf2.getW();
}


void transformMsgToTF2(const geometry_msgs::Vector3& msg, tf2::Vector3& tf2) {
	tf2.setX(msg.x);
	tf2.setY(msg.y);
	tf2.setZ(msg.z);
}

void transformTF2ToMsg(const tf2::Vector3& tf2, geometry_msgs::Vector3& msg) {
	msg.x = tf2.getX();
	msg.y = tf2.getY();
	msg.z = tf2.getZ();
}



void transformMatrixToTF2(const Eigen::Matrix4f& matrix, tf2::Transform& tf2) {
//	Eigen::Matrix4d doubleMatrix(matrix.cast());
//	tf2.setFromOpenGLMatrix(doubleMatrix.data());
}

} /* namespace tf_rosmsg_eigen_conversions */
} /* namespace laserscan_to_pointcloud */


