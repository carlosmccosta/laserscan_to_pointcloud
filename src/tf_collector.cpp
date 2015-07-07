/**\file tf_collector.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <laserscan_to_pointcloud/tf_collector.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace laserscan_to_pointcloud {
// =============================================================================  <public-section>   ===========================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
TFCollector::TFCollector(ros::Duration buffer_duration) :
		tf2_buffer_(buffer_duration), tf2_transform_listener_(tf2_buffer_) {
}

TFCollector::~TFCollector() {
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <TFCollector-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
bool TFCollector::collectTFs(const std::string& target_frame, const std::string& source_frame, const ros::Time& start_time, const ros::Time& endtime, size_t number_tfs, std::vector<tf2::Transform>& collected_tfs_out, const ros::Duration& tf_timeout) {
	collected_tfs_out.clear();

	ros::Time current_tf_time = start_time;
	ros::Duration next_tf_time_increment((endtime - start_time).toSec() / (number_tfs - 1));
	for (size_t tf_number = 0; tf_number < number_tfs; ++tf_number) {
		tf2::Transform tf2;
		if (lookForTransform(tf2, target_frame, source_frame, current_tf_time, tf_timeout)) {
			collected_tfs_out.push_back(tf2);
		}
		current_tf_time += next_tf_time_increment;
	}

	return !collected_tfs_out.empty();
}

bool TFCollector::lookForLatestTransform(tf2::Transform& tf2_transformOut, const std::string& target_frame, const std::string& source_frame, const ros::Duration& timeout, size_t number_of_queries) {
	ros::Time start_time = ros::Time::now();
	ros::Time end_time = start_time + timeout;
	ros::Duration lookup_timeout(timeout.toSec() / ((double) number_of_queries));

	while (ros::Time::now() < end_time) {
		if (lookForTransform(tf2_transformOut, target_frame, source_frame, ros::Time(0.0), lookup_timeout)) {
			return true;
		}
	}

	return false;
}

bool TFCollector::lookForTransform(tf2::Vector3& translation_out, tf2::Quaternion& rotation_out, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration& timeout) {
	try {
		std::string source_frame_stripped = source_frame;
		std::string target_frame_stripped = target_frame;
		stripSlash(source_frame_stripped);
		stripSlash(target_frame_stripped);
		geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform(target_frame_stripped, source_frame_stripped, time, timeout);
		tf_rosmsg_eigen_conversions::transformMsgToTF2(tf.transform.translation, translation_out);
		tf_rosmsg_eigen_conversions::transformMsgToTF2(tf.transform.rotation, rotation_out);
		return true;
	} catch (...) { // no transform available
		return false;
	}
}

bool TFCollector::lookForTransform(tf2::Transform& tf2_transform_out, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration& timeout) {
	try {
		std::string source_frame_stripped = source_frame;
		std::string target_frame_stripped = target_frame;
		stripSlash(source_frame_stripped);
		stripSlash(target_frame_stripped);
		geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform(target_frame_stripped, source_frame_stripped, time, timeout);
		tf_rosmsg_eigen_conversions::transformMsgToTF2(tf.transform, tf2_transform_out);
		return true;
	} catch (...) { // no transform available
		return false;
	}
}

bool TFCollector::lookForTransform(tf2::Transform& tf2_transform_out, const std::string& target_frame, const ros::Time& target_time, const std::string& source_frame, const ros::Time& source_time, const std::string& fixed_frame, const ros::Duration& timeout) {
	try {
		std::string source_frame_stripped = source_frame;
		std::string target_frame_stripped = target_frame;
		stripSlash(source_frame_stripped);
		stripSlash(target_frame_stripped);
		geometry_msgs::TransformStamped tf = tf2_buffer_.lookupTransform(target_frame_stripped, target_time, source_frame_stripped, source_time, fixed_frame, timeout);
		tf_rosmsg_eigen_conversions::transformMsgToTF2(tf.transform, tf2_transform_out);
		return true;
	} catch (...) { // no transform available
		return false;
	}
}

bool TFCollector::startsWithSlash(const std::string& frame_id) {
	if (frame_id.size() > 0) if (frame_id[0] == '/') return true;
	return false;
}

void TFCollector::stripSlash(std::string& frame_id) {
	if (startsWithSlash(frame_id)) frame_id.erase(0, 1);
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </TFCollector-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>   ==========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================
} /* namespace laserscan_to_pointcloud */

