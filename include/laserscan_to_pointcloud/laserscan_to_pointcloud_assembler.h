#pragma once

/**\file laser_scan_to_pointcloud_assembler.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <map>
#include <cmath>

// ROS includes
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <dynamic_reconfigure/server.h>

// external libs includes

// project includes
#include <laserscan_to_pointcloud/laserscan_to_ros_pointcloud.h>
#include <laserscan_to_pointcloud/LaserScanToPointcloudAssemblerConfig.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace laserscan_to_pointcloud {
// ###################################################################   laser_scan_to_pointcloud_assembler   ##################################################################
/**
 * \brief Description...
 */
class LaserScanToPointcloudAssembler {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <enums>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </enums>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		LaserScanToPointcloudAssembler(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle);
		virtual ~LaserScanToPointcloudAssembler();

		void setupLaserScansSubscribers(std::string laser_scan_topics);
		void setupRecoveryInitialPose();
		void startAssemblingLaserScans();
		void stopAssemblingLaserScans();
		void processLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan);
		void adjustAssemblyConfiguration(const geometry_msgs::Vector3& linear_velocity, const geometry_msgs::Vector3& angular_velocity);
		void adjustAssemblyConfigurationFromTwist(const geometry_msgs::TwistConstPtr& twist);
		void adjustAssemblyConfigurationFromOdometry(const nav_msgs::OdometryConstPtr& odometry);
		void adjustAssemblyConfigurationFromIMU(const sensor_msgs::ImuConstPtr& imu);

		void dynamicReconfigureCallback(laserscan_to_pointcloud::LaserScanToPointcloudAssemblerConfig& config, uint32_t level);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LaserScanToPointcloudAssembler-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LaserScanToPointcloudAssembler-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>   ==========================================================================

	// ========================================================================   <protected-section>   ========================================================================
	protected:
	// ========================================================================   </protected-section>  ========================================================================

	// ========================================================================   <private-section>   ==========================================================================
	private:
		// assembler config fields
		std::string laser_scan_topics_;
		std::string pointcloud_publish_topic_;
		int number_of_scans_to_assemble_per_cloud_;
		ros::Duration timeout_for_cloud_assembly_;

		// dynamic reconfiguration based on twist messages
		int min_number_of_scans_to_assemble_per_cloud_;
		int max_number_of_scans_to_assemble_per_cloud_;
		double min_timeout_seconds_for_cloud_assembly_;
		double max_timeout_seconds_for_cloud_assembly_;

		double max_linear_velocity_;
		double max_angular_velocity_;

		// laserscan_to_pointcloud_ config fields
		LaserScanToROSPointcloud laserscan_to_pointcloud_;
		bool include_laser_intensity_;
		bool enforce_reception_of_laser_scans_in_all_topics_;
		std::map<std::string, sensor_msgs::LaserScanConstPtr> laser_scans_for_each_topic_frame_id_;

		// state fieds
		size_t number_droped_laserscans_;
		bool timeout_for_cloud_assembly_reached_;
		ros::Time imu_last_message_stamp_;
		geometry_msgs::Vector3 imu_linear_velocity;

		// communication fields
		ros::NodeHandlePtr node_handle_;
		ros::NodeHandlePtr private_node_handle_;
		std::vector<ros::Subscriber> laserscan_subscribers_;
		ros::Publisher pointcloud_publisher_;
		ros::Subscriber twist_subscriber_;
		ros::Subscriber odometry_subscriber_;
		ros::Subscriber imu_subscriber_;

		dynamic_reconfigure::Server<laserscan_to_pointcloud::LaserScanToPointcloudAssemblerConfig> dynamic_reconfigure_server_;
	// ========================================================================   </private-section>  ==========================================================================
};
} /* namespace laserscan_to_pointcloud */
