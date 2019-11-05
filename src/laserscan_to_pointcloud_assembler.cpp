/**\file laser_scan_to_pointcloud_assembler.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <laserscan_to_pointcloud/laserscan_to_pointcloud_assembler.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace laserscan_to_pointcloud {
// =============================================================================  <public-section>   ===========================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
LaserScanToPointcloudAssembler::LaserScanToPointcloudAssembler(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) :
		number_droped_laserscans_(0), timeout_for_cloud_assembly_reached_(false), imu_last_message_stamp_(0),
		node_handle_(node_handle), private_node_handle_(private_node_handle) {

	double timeout_for_cloud_assembly = 5.0;
	private_node_handle_->param("laser_scan_topics", laser_scan_topics_, std::string("laser_scan"));
	private_node_handle_->param("pointcloud_publish_topic", pointcloud_publish_topic_, std::string("ambient_pointcloud"));

	private_node_handle_->param("enforce_reception_of_laser_scans_in_all_topics", enforce_reception_of_laser_scans_in_all_topics_, true);

	private_node_handle_->param("include_laser_intensity", include_laser_intensity_, true);
	laserscan_to_pointcloud_.setIncludeLaserIntensity(include_laser_intensity_);

	private_node_handle_->param("number_of_scans_to_assemble_per_cloud", number_of_scans_to_assemble_per_cloud_, 1);
	private_node_handle_->param("timeout_for_cloud_assembly", timeout_for_cloud_assembly, 1.0);
	timeout_for_cloud_assembly_.fromSec(timeout_for_cloud_assembly);

	std::string dynamic_update_of_assembly_configuration_from_twist_topic, dynamic_update_of_assembly_configuration_from_odometry_topic, dynamic_update_of_assembly_configuration_from_imu_topic;
	private_node_handle_->param("dynamic_update_of_assembly_configuration_from_twist_topic", dynamic_update_of_assembly_configuration_from_twist_topic, std::string(""));
	private_node_handle_->param("dynamic_update_of_assembly_configuration_from_odometry_topic", dynamic_update_of_assembly_configuration_from_odometry_topic, std::string("odom"));
	private_node_handle_->param("dynamic_update_of_assembly_configuration_from_imu_topic", dynamic_update_of_assembly_configuration_from_imu_topic, std::string(""));

	if (!dynamic_update_of_assembly_configuration_from_twist_topic.empty()) {
		ROS_INFO_STREAM("Updating laser assembly configurations from twist topic [" << dynamic_update_of_assembly_configuration_from_twist_topic << "]");
		twist_subscriber_ = node_handle_->subscribe(dynamic_update_of_assembly_configuration_from_twist_topic, 5, &laserscan_to_pointcloud::LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromTwist, this);
	}

	if (!dynamic_update_of_assembly_configuration_from_odometry_topic.empty()) {
		ROS_INFO_STREAM("Updating laser assembly configurations from odometry topic [" << dynamic_update_of_assembly_configuration_from_odometry_topic << "]");
		odometry_subscriber_ = node_handle_->subscribe(dynamic_update_of_assembly_configuration_from_odometry_topic, 5, &laserscan_to_pointcloud::LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromOdometry, this);
	}

	if (!dynamic_update_of_assembly_configuration_from_imu_topic.empty()) {
		ROS_INFO_STREAM("Updating laser assembly configurations from imu topic [" << dynamic_update_of_assembly_configuration_from_imu_topic << "]");
		imu_subscriber_ = node_handle_->subscribe(dynamic_update_of_assembly_configuration_from_imu_topic, 5, &laserscan_to_pointcloud::LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromIMU, this);
	}

	private_node_handle_->param("min_number_of_scans_to_assemble_per_cloud", min_number_of_scans_to_assemble_per_cloud_, 1);
	private_node_handle_->param("max_number_of_scans_to_assemble_per_cloud", max_number_of_scans_to_assemble_per_cloud_, 10);
	private_node_handle_->param("min_timeout_seconds_for_cloud_assembly", min_timeout_seconds_for_cloud_assembly_, 0.3);
	private_node_handle_->param("max_timeout_seconds_for_cloud_assembly", max_timeout_seconds_for_cloud_assembly_, 1.3);
	private_node_handle_->param("max_linear_velocity", max_linear_velocity_, 0.5);
	private_node_handle_->param("max_angular_velocity", max_angular_velocity_, 0.174532925);

	std::string target_frame_id, laser_frame_id, motion_estimation_source_frame_id, motion_estimation_target_frame_id;
	double number;
	bool boolean;
	private_node_handle_->param("target_frame", target_frame_id, std::string("odom"));
	laserscan_to_pointcloud_.setTargetFrame(target_frame_id);

	private_node_handle_->param("motion_estimation_source_frame_id", motion_estimation_source_frame_id, std::string(""));
	laserscan_to_pointcloud_.setMotionEstimationSourceFrame(motion_estimation_source_frame_id);
	private_node_handle_->param("motion_estimation_target_frame_id", motion_estimation_target_frame_id, std::string(""));
	laserscan_to_pointcloud_.setMotionEstimationTargetFrame(motion_estimation_target_frame_id);

	private_node_handle_->param("laser_frame", laser_frame_id, std::string(""));
	laserscan_to_pointcloud_.setLaserFrame(laser_frame_id);

	int integer;
	private_node_handle_->param("number_of_tf_queries_for_spherical_interpolation", integer, 4);
	if (integer > 1) { ROS_INFO_STREAM("Laser assembler is using " << integer << " TFs inside laser scan time to perform spherical interpolation"); }

	laserscan_to_pointcloud_.setNumberOfTfQueriesForSphericalInterpolation(integer);
	private_node_handle_->param("tf_lookup_timeout", number, 0.15);
	laserscan_to_pointcloud_.setTFLookupTimeout(number);

	private_node_handle_->param("min_range_cutoff_percentage_offset", number, 1.05);
	laserscan_to_pointcloud_.setMinRangeCutoffPercentageOffset(number);
	private_node_handle_->param("max_range_cutoff_percentage_offset", number, 0.95);
	laserscan_to_pointcloud_.setMaxRangeCutoffPercentageOffset(number);

	private_node_handle_->param("remove_invalid_measurements", boolean, true);
	laserscan_to_pointcloud_.setRemoveInvalidMeasurements(boolean);

	dynamic_reconfigure::Server<laserscan_to_pointcloud::LaserScanToPointcloudAssemblerConfig>::CallbackType callback_dynamic_reconfigure =
			boost::bind(&laserscan_to_pointcloud::LaserScanToPointcloudAssembler::dynamicReconfigureCallback, this, _1, _2);
	dynamic_reconfigure_server_.setCallback(callback_dynamic_reconfigure);
}

LaserScanToPointcloudAssembler::~LaserScanToPointcloudAssembler() {	}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LaserScanToPointcloudAssembler-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void LaserScanToPointcloudAssembler::setupLaserScansSubscribers(std::string laser_scan_topics) {
	std::replace(laser_scan_topics.begin(), laser_scan_topics.end(), '+', ' ');

	std::stringstream ss(laser_scan_topics);
	std::string topic_name;

	while (ss >> topic_name && !topic_name.empty()) {
		ros::Subscriber laserscan_subscriber = node_handle_->subscribe(topic_name, 5, &laserscan_to_pointcloud::LaserScanToPointcloudAssembler::processLaserScan, this);
		laserscan_subscribers_.push_back(laserscan_subscriber);
		ROS_INFO_STREAM("Adding " << topic_name << " to the list of LaserScan topics to assemble");
	}

	if (number_of_scans_to_assemble_per_cloud_ < (int)laserscan_subscribers_.size()) {
		number_of_scans_to_assemble_per_cloud_ = (int)laserscan_subscribers_.size();
	}

	if (min_number_of_scans_to_assemble_per_cloud_ < (int)laserscan_subscribers_.size()) {
		min_number_of_scans_to_assemble_per_cloud_ = (int)laserscan_subscribers_.size();
	}

	if (max_number_of_scans_to_assemble_per_cloud_ <= (int)min_number_of_scans_to_assemble_per_cloud_) {
		max_number_of_scans_to_assemble_per_cloud_ = (int)(min_number_of_scans_to_assemble_per_cloud_ * 2);
	}
}


void LaserScanToPointcloudAssembler::setupRecoveryInitialPose() {
	double x, y, z, roll, pitch ,yaw;
	bool initial_recovery_transform_in_base_link_to_target;
	std::string base_link_frame_id, recovery_frame_id;
	private_node_handle_->param("recovery_frame", recovery_frame_id, std::string("odom"));
	private_node_handle_->param("initial_recovery_transform_in_base_link_to_target", initial_recovery_transform_in_base_link_to_target, true);
	private_node_handle_->param("base_link_frame_id", base_link_frame_id, std::string("base_footprint"));
	private_node_handle_->param("recovery_to_target_frame_transform_initial_x", x, 0.0);
	private_node_handle_->param("recovery_to_target_frame_transform_initial_y", y, 0.0);
	private_node_handle_->param("recovery_to_target_frame_transform_initial_z", z, 0.0);
	private_node_handle_->param("recovery_to_target_frame_transform_initial_roll", roll, 0.0);
	private_node_handle_->param("recovery_to_target_frame_transform_initial_pitch", pitch, 0.0);
	private_node_handle_->param("recovery_to_target_frame_transform_initial_yaw", yaw, 0.0);
	tf2::Quaternion orientation;
	orientation.setRPY(roll, pitch, yaw);
	orientation.normalize();
	tf2::Transform recovery_to_target_frame_transform(orientation, tf2::Vector3(x, y, z));

	ros::Time::waitForValid();

	if (initial_recovery_transform_in_base_link_to_target && !recovery_frame_id.empty() && !base_link_frame_id.empty()) {
		ros::Time start_time = ros::Time::now();
		ros::Time end_time = start_time + ros::Duration(10);
		ros::Duration wait_duration(0.005);

		bool success = false;
		while (ros::Time::now() < end_time) {
			tf2::Transform transform_recovery_to_base_link;
			if (laserscan_to_pointcloud_.getTfCollector().lookForTransform(transform_recovery_to_base_link, base_link_frame_id, recovery_frame_id, ros::Time::now(), laserscan_to_pointcloud_.getTfLookupTimeout())) {
				recovery_to_target_frame_transform = recovery_to_target_frame_transform * transform_recovery_to_base_link;
				success = true;
				break;
			}
			wait_duration.sleep();
		}
		if (!success) {
			ROS_WARN("Failed to correct assembler recovery initial pose");
		}
	}

	if (!recovery_frame_id.empty()) {
		tf2::Quaternion recovery_to_target_frame_transform_q = recovery_to_target_frame_transform.getRotation().normalize();
		ROS_INFO_STREAM("Setting assembler recovery initial pose [ x: " << recovery_to_target_frame_transform.getOrigin().getX()
				<< " y: " << recovery_to_target_frame_transform.getOrigin().getY()
				<< " z: " << recovery_to_target_frame_transform.getOrigin().getZ()
				<< " | qx: " << recovery_to_target_frame_transform_q.getX()
				<< " qy: " << recovery_to_target_frame_transform_q.getY()
				<< " qz: " << recovery_to_target_frame_transform_q.getZ()
				<< " qw: " << recovery_to_target_frame_transform_q.getW()
				<< " ]");
		laserscan_to_pointcloud_.setRecoveryFrame(recovery_frame_id, recovery_to_target_frame_transform);
	}
}


void LaserScanToPointcloudAssembler::startAssemblingLaserScans() {
	setupRecoveryInitialPose();
	pointcloud_publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>(pointcloud_publish_topic_, 10, true);
	setupLaserScansSubscribers(laser_scan_topics_);
}


void LaserScanToPointcloudAssembler::stopAssemblingLaserScans() {
	for (size_t i = 0; i < laserscan_subscribers_.size(); ++i) {
		laserscan_subscribers_[i].shutdown();
	}

	pointcloud_publisher_.shutdown();
}


void LaserScanToPointcloudAssembler::processLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan) {
	int number_of_scans_in_current_pointcloud = (int)laserscan_to_pointcloud_.getNumberOfScansAssembledInCurrentPointcloud();
	if ((number_of_scans_in_current_pointcloud == 0 && laserscan_to_pointcloud_.getNumberOfPointcloudsCreated() == 0)
			|| number_of_scans_in_current_pointcloud >= number_of_scans_to_assemble_per_cloud_
			|| timeout_for_cloud_assembly_reached_) {
		laserscan_to_pointcloud_.setIncludeLaserIntensity(include_laser_intensity_);
		laserscan_to_pointcloud_.initNewPointCloud(laser_scan->ranges.size() * number_of_scans_to_assemble_per_cloud_);
		laser_scans_for_each_topic_frame_id_.clear();
		timeout_for_cloud_assembly_reached_ = false;

		ROS_DEBUG_STREAM("Initializing new point cloud");
	}

	number_of_scans_in_current_pointcloud = (int)laserscan_to_pointcloud_.getNumberOfScansAssembledInCurrentPointcloud();

	std::string laser_frame = laserscan_to_pointcloud_.getLaserFrame().empty() ? laser_scan->header.frame_id : laserscan_to_pointcloud_.getLaserFrame();

	ROS_DEBUG_STREAM((enforce_reception_of_laser_scans_in_all_topics_ ? "Caching" : "Adding") << " laser scan " << number_of_scans_in_current_pointcloud + laser_scans_for_each_topic_frame_id_.size() << " in frame " << laser_frame << " with " << laser_scan->ranges.size() << " points to a point cloud with " << laserscan_to_pointcloud_.getNumberOfPointsInCloud() << " points in frame " << laserscan_to_pointcloud_.getTargetFrame());

	if (enforce_reception_of_laser_scans_in_all_topics_) {
		if (laser_scans_for_each_topic_frame_id_.find(laser_scan->header.frame_id) != laser_scans_for_each_topic_frame_id_.end())
			ROS_WARN_STREAM("Discarding previously cached laser scan for frame " << laser_scan->header.frame_id);

		laser_scans_for_each_topic_frame_id_[laser_scan->header.frame_id] = laser_scan;

		if (laser_scans_for_each_topic_frame_id_.size() >= laserscan_subscribers_.size()) {
			for (std::map<std::string, sensor_msgs::LaserScanConstPtr>::iterator it = laser_scans_for_each_topic_frame_id_.begin(); it != laser_scans_for_each_topic_frame_id_.end(); ++it) {
				if (!laserscan_to_pointcloud_.integrateLaserScanWithShpericalLinearInterpolation(it->second)) {
					ROS_WARN_STREAM("Dropped LaserScan with " << it->second->ranges.size() << " points because of missing TFs between [" << laser_frame << "] and [" << laserscan_to_pointcloud_.getTargetFrame() << "]" << " (dropped " << ++number_droped_laserscans_ << " LaserScans so far)");
				}
			}

			laser_scans_for_each_topic_frame_id_.clear();
		}
	} else if (!laserscan_to_pointcloud_.integrateLaserScanWithShpericalLinearInterpolation(laser_scan)) {
		ROS_WARN_STREAM("Dropped LaserScan with " << laser_scan->ranges.size() << " points because of missing TFs between [" << laser_frame << "] and [" << laserscan_to_pointcloud_.getTargetFrame() << "]" << " (dropped " << ++number_droped_laserscans_ << " LaserScans so far)");
	}

	timeout_for_cloud_assembly_reached_ = (ros::Time::now() - laserscan_to_pointcloud_.getPointcloud()->header.stamp) > timeout_for_cloud_assembly_;
	number_of_scans_in_current_pointcloud = (int)laserscan_to_pointcloud_.getNumberOfScansAssembledInCurrentPointcloud();
	if ((number_of_scans_in_current_pointcloud >= number_of_scans_to_assemble_per_cloud_ || timeout_for_cloud_assembly_reached_) && laserscan_to_pointcloud_.getNumberOfPointsInCloud() > 0) {
		ros::Duration scan_duration((laser_scan->ranges.size() - 1) * laser_scan->time_increment);
		laserscan_to_pointcloud_.getPointcloud()->header.stamp = ros::Time(laser_scan->header.stamp) + scan_duration;
		pointcloud_publisher_.publish(laserscan_to_pointcloud_.getPointcloud());

		ROS_DEBUG_STREAM("Publishing cloud with " << (laserscan_to_pointcloud_.getPointcloud()->width * laserscan_to_pointcloud_.getPointcloud()->height) << " points assembled from " << number_of_scans_in_current_pointcloud << " LaserScans" \
				<< (timeout_for_cloud_assembly_reached_ ? " (timeout reached)" : ""));
	}
}


void LaserScanToPointcloudAssembler::adjustAssemblyConfiguration(const geometry_msgs::Vector3& linear_velocity, const geometry_msgs::Vector3& angular_velocity) {
	double inverse_linear_velocity = max_linear_velocity_ - std::min(std::sqrt(linear_velocity.x * linear_velocity.x + linear_velocity.y * linear_velocity.y + linear_velocity.z * linear_velocity.z), max_linear_velocity_);
	double inverse_angular_velocity = max_angular_velocity_ - std::min(std::sqrt(angular_velocity.x * angular_velocity.x + angular_velocity.y * angular_velocity.y + angular_velocity.z * angular_velocity.z), max_angular_velocity_);

	double linear_velocity_to_number_scans_ratio = (max_number_of_scans_to_assemble_per_cloud_ - min_number_of_scans_to_assemble_per_cloud_) / max_linear_velocity_;
	double angular_velocity_to_number_scans_ratio = (max_number_of_scans_to_assemble_per_cloud_ - min_number_of_scans_to_assemble_per_cloud_) / max_angular_velocity_;
	double number_scans_linear_velocity = min_number_of_scans_to_assemble_per_cloud_ + linear_velocity_to_number_scans_ratio * inverse_linear_velocity;
	double number_scans_angular_velocity = min_number_of_scans_to_assemble_per_cloud_ + angular_velocity_to_number_scans_ratio * inverse_angular_velocity;
	number_of_scans_to_assemble_per_cloud_ = std::ceil(std::min(number_scans_linear_velocity, number_scans_angular_velocity));

	double linear_velocity_to_timeout_ratio = (max_timeout_seconds_for_cloud_assembly_ - min_timeout_seconds_for_cloud_assembly_) / max_linear_velocity_;
	double angular_velocity_to_timeout_ratio = (max_timeout_seconds_for_cloud_assembly_ - min_timeout_seconds_for_cloud_assembly_) / max_angular_velocity_;
	double timeout_linear_velocity = min_timeout_seconds_for_cloud_assembly_ + linear_velocity_to_timeout_ratio * inverse_linear_velocity;
	double timeout_angular_velocity = min_timeout_seconds_for_cloud_assembly_ + angular_velocity_to_timeout_ratio * inverse_angular_velocity;
	double timeout = std::min(timeout_linear_velocity, timeout_angular_velocity);
	timeout_for_cloud_assembly_.fromSec(timeout);

	ROS_DEBUG_STREAM_THROTTLE(0.1, "Laser scan assembly configuration: [ number_of_scans_to_assemble_per_cloud: " << number_of_scans_to_assemble_per_cloud_ << " ] | [ timeout_for_cloud_assembly: " << timeout_for_cloud_assembly_ << " ]");
}


void LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromTwist(const geometry_msgs::TwistConstPtr& twist) {
	adjustAssemblyConfiguration(twist->linear, twist->angular);
}


void LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromOdometry(const nav_msgs::OdometryConstPtr& odometry) {
	adjustAssemblyConfiguration(odometry->twist.twist.linear, odometry->twist.twist.angular);
}


void LaserScanToPointcloudAssembler::adjustAssemblyConfigurationFromIMU(const sensor_msgs::ImuConstPtr& imu) {
	if (imu_last_message_stamp_.toSec() > 0.1) {
		double imu_dt = imu->header.stamp.toSec() - imu_last_message_stamp_.toSec();
		if (imu_dt > 0) {
			imu_linear_velocity.x += imu->linear_acceleration.x * imu_dt;
			imu_linear_velocity.y += imu->linear_acceleration.y * imu_dt;
			imu_linear_velocity.z += imu->linear_acceleration.z * imu_dt;
			adjustAssemblyConfiguration(imu_linear_velocity, imu->angular_velocity);
		}
	}

	imu_last_message_stamp_ = imu->header.stamp;
}


void LaserScanToPointcloudAssembler::dynamicReconfigureCallback(laserscan_to_pointcloud::LaserScanToPointcloudAssemblerConfig& config, uint32_t level) {
	if (level == 1) {
		ROS_INFO_STREAM("LaserScanToPointcloudAssembler dynamic reconfigure (level=" << level << ") -> " \
				<< "\n\t[laser_scan_topics]: " 						<< laser_scan_topics_ 							<< " -> " << config.laser_scan_topics \
				<< "\n\t[pointcloud_publish_topic]: " 				<< pointcloud_publish_topic_					<< " -> " << config.pointcloud_publish_topic \
				<< "\n\t[number_of_scans_to_assemble_per_cloud]: "	<< number_of_scans_to_assemble_per_cloud_ 		<< " -> " << config.number_of_scans_to_assemble_per_cloud \
				<< "\n\t[timeout_for_cloud_assembly]: "				<< timeout_for_cloud_assembly_.toSec() 			<< " -> " << config.timeout_for_cloud_assembly \
				<< "\n\t[target_frame]: " 							<< laserscan_to_pointcloud_.getTargetFrame() 	<< " -> " << config.target_frame \
				<< "\n\t[recovery_frame]: " 						<< laserscan_to_pointcloud_.getRecoveryFrame() 	<< " -> " << config.recovery_frame \
				<< "\n\t[min_range_cutoff_percentage_offset]: " 	<< laserscan_to_pointcloud_.getMinRangeCutoffPercentageOffset() << " -> " << config.min_range_cutoff_percentage_offset \
				<< "\n\t[max_range_cutoff_percentage_offset]: " 	<< laserscan_to_pointcloud_.getMaxRangeCutoffPercentageOffset()	<< " -> " << config.max_range_cutoff_percentage_offset \
				<< "\n\t[include_laser_intensity]: " 				<< include_laser_intensity_						<< " -> " << (config.include_laser_intensity ? "True" : "False") \
				<< "\n\t[interpolate_scans]: " 						<< laserscan_to_pointcloud_.getNumberOfTfQueriesForSphericalInterpolation() << " -> " << config.number_of_tf_queries_for_spherical_interpolation);

		if (!config.laser_scan_topics.empty() && laser_scan_topics_ != config.laser_scan_topics) {
			laser_scan_topics_ = config.laser_scan_topics;
			for (size_t i = 0; i < laserscan_subscribers_.size(); ++i) {
				laserscan_subscribers_[i].shutdown();
			}

			setupLaserScansSubscribers(laser_scan_topics_);
		}

		if (!config.pointcloud_publish_topic.empty() && pointcloud_publish_topic_ != config.pointcloud_publish_topic) {
			pointcloud_publish_topic_ = config.pointcloud_publish_topic;
			pointcloud_publisher_.shutdown();
			pointcloud_publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>(pointcloud_publish_topic_, 10, true);
		}

		number_of_scans_to_assemble_per_cloud_ = config.number_of_scans_to_assemble_per_cloud;
		timeout_for_cloud_assembly_.fromSec(config.timeout_for_cloud_assembly);
		include_laser_intensity_ = config.include_laser_intensity;

		laserscan_to_pointcloud_.setTargetFrame(config.target_frame);
		laserscan_to_pointcloud_.setMinRangeCutoffPercentageOffset(config.min_range_cutoff_percentage_offset);
		laserscan_to_pointcloud_.setMaxRangeCutoffPercentageOffset(config.max_range_cutoff_percentage_offset);
		laserscan_to_pointcloud_.setNumberOfTfQueriesForSphericalInterpolation(config.number_of_tf_queries_for_spherical_interpolation);
		laserscan_to_pointcloud_.setRecoveryFrame(config.recovery_frame);
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LaserScanToPointcloudAssembler-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>   ==========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================



} /* namespace laserscan_to_pointcloud */
