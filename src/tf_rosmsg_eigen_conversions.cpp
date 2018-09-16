/**\file tf_rosmsg_eigen_conversions.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <laserscan_to_pointcloud/tf_rosmsg_eigen_conversions.h>
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
	tf2::Quaternion rotation = tf2.getRotation().normalize();
	msg.rotation.x = rotation.x();
	msg.rotation.y = rotation.y();
	msg.rotation.z = rotation.z();
	msg.rotation.w = rotation.w();
	msg.translation.x = tf2.getOrigin().x();
	msg.translation.y = tf2.getOrigin().y();
	msg.translation.z = tf2.getOrigin().z();
}

void transformMsgToTF2(const geometry_msgs::Pose& msg, tf2::Transform& tf2) {
	transformMsgToTF2(msg.position, tf2.getOrigin());
	tf2.setRotation(tf2::Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w));
}

void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::Pose& msg) {
	transformTF2ToMsg(tf2.getOrigin(), msg.position);
	tf2::Quaternion rotation = tf2.getRotation().normalize();
	msg.orientation.x = rotation.getX();
	msg.orientation.y = rotation.getY();
	msg.orientation.z = rotation.getZ();
	msg.orientation.w = rotation.getW();
}


void transformMsgToTF2(const geometry_msgs::Quaternion& msg, tf2::Quaternion& tf2) {
	tf2.setX(msg.x);
	tf2.setY(msg.y);
	tf2.setZ(msg.z);
	tf2.setW(msg.w);
	tf2.normalize();
}

void transformTF2ToMsg(const tf2::Quaternion& tf2, geometry_msgs::Quaternion& msg) {
	tf2::Quaternion tf2_n = tf2.normalized();
	msg.x = tf2_n.getX();
	msg.y = tf2_n.getY();
	msg.z = tf2_n.getX();
	msg.w = tf2_n.getW();
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

void transformMsgToTF2(const geometry_msgs::Point& msg, tf2::Vector3& tf2) {
	tf2.setX(msg.x);
	tf2.setY(msg.y);
	tf2.setZ(msg.z);
}

void transformTF2ToMsg(const tf2::Vector3& tf2, geometry_msgs::Point& msg) {
	msg.x = tf2.getX();
	msg.y = tf2.getY();
	msg.z = tf2.getZ();
}

void transformMatrixToTF2(const Eigen::Matrix4f& matrix, tf2::Transform& tf2) {
	Eigen::Matrix4d doubleMatrix(matrix.cast<double>());
//	Eigen::Matrix<double, 4, 4, Eigen::ColMajor> doubleMatrix(matrix.cast<double>());
	tf2.setFromOpenGLMatrix(doubleMatrix.data());
}

void transformMatrixToMsg(const Eigen::Matrix4f& matrix, geometry_msgs::Pose& msg) {
	tf2::Transform tf2;
	transformMatrixToTF2(matrix, tf2);
	transformTF2ToMsg(tf2, msg);
}

} /* namespace tf_rosmsg_eigen_conversions */
} /* namespace laserscan_to_pointcloud */
