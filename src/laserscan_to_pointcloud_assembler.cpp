/**\file laser_scan_to_pointcloud_assembler.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "laserscan_to_pointcloud/laserscan_to_pointcloud_assembler.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace laserscan_to_pointcloud {
// =============================================================================  <public-section>   ===========================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
LaserScanToPointcloudAssembler::LaserScanToPointcloudAssembler(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) :
		number_droped_laserscans_(0), timeout_for_cloud_assembly_reached_(false),
		node_handle_(node_handle), private_node_handle_(private_node_handle) {

	double timeout_for_cloud_assembly = 5.0;
	private_node_handle_->param("laser_scan_topics", laser_scan_topics_, std::string("tilt_scan"));
	private_node_handle_->param("pointcloud_publish_topic", pointcloud_publish_topic_, std::string("ambient_pointcloud"));
	private_node_handle_->param("number_of_scans_to_assemble_per_cloud", number_of_scans_to_assemble_per_cloud_, 10);
	private_node_handle_->param("timeout_for_cloud_assembly", timeout_for_cloud_assembly, 5.0);
	private_node_handle_->param("target_frame", target_frame_, std::string("map"));
	private_node_handle_->param("min_range_cutoff_percentage_offset", min_range_cutoff_percentage_offset_, 1.05);
	private_node_handle_->param("max_range_cutoff_percentage_offset", max_range_cutoff_percentage_offset_, 0.95);
	private_node_handle_->param("include_laser_intensity", include_laser_intensity_, false);
	private_node_handle_->param("interpolate_scans", interpolate_scans_, true);
	private_node_handle_->param("tf_lookup_timeout", tf_lookup_timeout_, 0.2);
	timeout_for_cloud_assembly_.fromSec(timeout_for_cloud_assembly);

	propagatePointCloudAssemblerConfigs();

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
		ros::Subscriber laserscan_subscriber = node_handle_->subscribe(topic_name, 100, &laserscan_to_pointcloud::LaserScanToPointcloudAssembler::processLaserScan, this);
		laserscan_subscribers_.push_back(laserscan_subscriber);
		ROS_INFO_STREAM("Adding " << topic_name << " to the list of LaserScan topics to assemble");
	}
}

void LaserScanToPointcloudAssembler::propagatePointCloudAssemblerConfigs() {
	laserscan_to_pointcloud_.setTargetFrame(target_frame_);
	laserscan_to_pointcloud_.setIncludeLaserIntensity(include_laser_intensity_);
	laserscan_to_pointcloud_.setMinRangeCutoffPercentageOffset(min_range_cutoff_percentage_offset_);
	laserscan_to_pointcloud_.setMaxRangeCutoffPercentageOffset(max_range_cutoff_percentage_offset_);
	laserscan_to_pointcloud_.setInterpolateScans(interpolate_scans_);
	laserscan_to_pointcloud_.setTFLookupTimeout(tf_lookup_timeout_);
}


void LaserScanToPointcloudAssembler::startAssemblingLaserScans() {
	setupLaserScansSubscribers(laser_scan_topics_);
	pointcloud_publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>(pointcloud_publish_topic_, 10, true);
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
		timeout_for_cloud_assembly_reached_ = false;
	}

	if (!laserscan_to_pointcloud_.integrateLaserScanWithShpericalLinearInterpolation(laser_scan)) {
		ROS_WARN_STREAM("Dropped LaserScan with " << laser_scan->ranges.size() << " points because of missing TFs between [" << laser_scan->header.frame_id << "] and [" << target_frame_ << "]" << " (dropped " << ++number_droped_laserscans_ << " LaserScans so far)");
	}

	timeout_for_cloud_assembly_reached_ = (ros::Time::now() - laserscan_to_pointcloud_.getPointcloud()->header.stamp) > timeout_for_cloud_assembly_;
	number_of_scans_in_current_pointcloud = (int)laserscan_to_pointcloud_.getNumberOfScansAssembledInCurrentPointcloud();
	if ((number_of_scans_in_current_pointcloud >= number_of_scans_to_assemble_per_cloud_ || timeout_for_cloud_assembly_reached_) && laserscan_to_pointcloud_.getNumberOfPointsInCloud() > 0) {
		laserscan_to_pointcloud_.getPointcloud()->header.stamp = laser_scan->header.stamp;
		pointcloud_publisher_.publish(laserscan_to_pointcloud_.getPointcloud());

		ROS_DEBUG_STREAM("Publishing cloud with " << laserscan_to_pointcloud_.getPointcloud()->width << " points assembled from " << number_of_scans_in_current_pointcloud << " LaserScans" \
				<< (timeout_for_cloud_assembly_reached_ ? " (timeout reached)" : ""));
	}
}


void LaserScanToPointcloudAssembler::dynamicReconfigureCallback(laserscan_to_pointcloud::LaserScanToPointcloudAssemblerConfig& config, uint32_t level) {
	if (level == 1) {
		ROS_INFO_STREAM("LaserScanToPointcloudAssembler dynamic reconfigure (level=" << level << ") -> " \
				<< "\n\t[laser_scan_topic]: " 						<< laser_scan_topics_ 						<< " -> " << config.laser_scan_topic \
				<< "\n\t[pointcloud_publish_topic]: " 				<< pointcloud_publish_topic_				<< " -> " << config.pointcloud_publish_topic \
				<< "\n\t[number_of_scans_to_assemble_per_cloud]: "	<< number_of_scans_to_assemble_per_cloud_ 	<< " -> " << config.number_of_scans_to_assemble_per_cloud \
				<< "\n\t[timeout_for_cloud_assembly]: "				<< timeout_for_cloud_assembly_.toSec() 		<< " -> " << config.timeout_for_cloud_assembly \
				<< "\n\t[target_frame]: " 							<< target_frame_ 							<< " -> " << config.target_frame \
				<< "\n\t[min_range_cutoff_percentage_offset]: " 	<< min_range_cutoff_percentage_offset_ 		<< " -> " << config.min_range_cutoff_percentage_offset \
				<< "\n\t[max_range_cutoff_percentage_offset]: " 	<< max_range_cutoff_percentage_offset_ 		<< " -> " << config.max_range_cutoff_percentage_offset \
				<< "\n\t[include_laser_intensity]: " 				<< include_laser_intensity_					<< " -> " << (config.include_laser_intensity ? "True" : "False") \
				<< "\n\t[interpolate_scans]: " 						<< interpolate_scans_						<< " -> " << (config.interpolate_scans ? "True" : "False"));

		if (!config.laser_scan_topic.empty() && laser_scan_topics_ != config.laser_scan_topic) {
			laser_scan_topics_ = config.laser_scan_topic;
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
		target_frame_ = config.target_frame;
		min_range_cutoff_percentage_offset_ = config.min_range_cutoff_percentage_offset;
		max_range_cutoff_percentage_offset_ = config.max_range_cutoff_percentage_offset;
		include_laser_intensity_ = config.include_laser_intensity;
		interpolate_scans_ = config.interpolate_scans;

		propagatePointCloudAssemblerConfigs();
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LaserScanToPointcloudAssembler-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>   ==========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================



} /* namespace laserscan_to_pointcloud */

