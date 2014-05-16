/**\file laser_scan_to_pointcloud_assembler.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "laserscan_to_pointcloud/laserscan_to_pointcloud_assembler.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  </includes> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <imports> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  </imports> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace laserscan_to_pointcloud {
// =============================================================================  <public-section>   ===========================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
LaserScanToPointcloudAssembler::LaserScanToPointcloudAssembler(ros::NodeHandlePtr& node_handle, ros::NodeHandlePtr& private_node_handle) :
			node_handle_(node_handle), private_node_handle_(private_node_handle) {

	private_node_handle_->param("target_frame", target_frame_, std::string("map"));
	private_node_handle_->param("laser_scan_topic", laser_scan_topic_, std::string("laser_scan"));
	private_node_handle_->param("min_range_cutoff_percentage_offset", min_range_cutoff_percentage_offset_, 1.05);
	private_node_handle_->param("max_range_cutoff_percentage_offset", max_range_cutoff_percentage_offset_, 0.95);
	private_node_handle_->param("number_of_scans_to_assemble_per_cloud", number_of_scans_to_assemble_per_cloud_, 10);

	propagatePointCloudAssemblerConfigs();

	dynamic_reconfigure::Server<laserscan_to_pointcloud::LaserScanToPointcloudAssemblerConfig>::CallbackType callback_dynamic_reconfigure =
			boost::bind(&laserscan_to_pointcloud::LaserScanToPointcloudAssembler::dynamicReconfigureCallback, this, _1, _2);
	dynamic_reconfigure_server_.setCallback(callback_dynamic_reconfigure);
}

LaserScanToPointcloudAssembler::~LaserScanToPointcloudAssembler() {	}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LaserScanToPointcloudAssembler-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void LaserScanToPointcloudAssembler::propagatePointCloudAssemblerConfigs() {
	laserscan_to_pointcloud_.setTargetFrame(target_frame_);
	laserscan_to_pointcloud_.setMinRangeCutoffPercentageOffset(min_range_cutoff_percentage_offset_);
	laserscan_to_pointcloud_.setMaxRangeCutoffPercentageOffset(max_range_cutoff_percentage_offset_);
}


void LaserScanToPointcloudAssembler::startAssemblingLaserScans() {
	laserscan_subscriber_ = node_handle_->subscribe(laser_scan_topic_, 100, &laserscan_to_pointcloud::LaserScanToPointcloudAssembler::processLaserScan, this);
	pointcloud_publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>("assembled_pointcloud", 10);
}


void LaserScanToPointcloudAssembler::stopAssemblingLaserScans() {
	laserscan_subscriber_.shutdown();
	pointcloud_publisher_.shutdown();
}


void LaserScanToPointcloudAssembler::processLaserScan(const sensor_msgs::LaserScanConstPtr& laser_scan) {
	int number_of_scans_in_current_pointcloud = (int)laserscan_to_pointcloud_.getNumberOfScansAssembledInCurrentPointcloud();

	if ((number_of_scans_in_current_pointcloud == 0 && laserscan_to_pointcloud_.getNumberOfPointcloudsCreated() == 0) ||
			number_of_scans_in_current_pointcloud == number_of_scans_to_assemble_per_cloud_) {
		laserscan_to_pointcloud_.initNewPointCloud(laser_scan->ranges.size() * number_of_scans_to_assemble_per_cloud_);
	}

	laserscan_to_pointcloud_.integrateLaserScanWithShpericalLinearInterpolation(laser_scan);
	if (number_of_scans_in_current_pointcloud == number_of_scans_to_assemble_per_cloud_) {
		pointcloud_publisher_.publish(laserscan_to_pointcloud_.getPointcloud());

		ROS_INFO_STREAM("Publishing cloud with " << laserscan_to_pointcloud_.getPointcloud()->width << " points assembled from " << number_of_scans_in_current_pointcloud << " LaserScan");
	}
}


void LaserScanToPointcloudAssembler::dynamicReconfigureCallback(laserscan_to_pointcloud::LaserScanToPointcloudAssemblerConfig& config, uint32_t level) {
	target_frame_ = config.target_frame;
	laser_scan_topic_ = config.laser_scan_topic;
	min_range_cutoff_percentage_offset_ = config.min_range_cutoff_percentage_offset;
	max_range_cutoff_percentage_offset_ = config.max_range_cutoff_percentage_offset;
	number_of_scans_to_assemble_per_cloud_ = config.number_of_scans_to_assemble_per_cloud;

	propagatePointCloudAssemblerConfigs();
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LaserScanToPointcloudAssembler-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>   ==========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>   ======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>   ========================================================================



} /* namespace laserscan_to_pointcloud */
