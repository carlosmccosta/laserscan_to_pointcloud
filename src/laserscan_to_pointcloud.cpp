/**\file LaserScanToPointcloud.cpp
 * \brief Implementation of a PointCloud2 builder from LaserScans.
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include "laserscan_to_pointcloud/laserscan_to_pointcloud.h"
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  </includes> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <imports> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <imports> <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// =============================================================================  <public-section>   ===========================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
LaserScanToPointcloud::LaserScanToPointcloud(std::string target_frame, size_t number_of_scans_to_assemble) :
		target_frame_(target_frame),
		number_of_scans_to_assemble_(number_of_scans_to_assemble),
		min_range_cutoff_percentage_(1.05), max_range_cutoff_percentage_(0.95),
		number_of_pointclouds_created_(0),
		number_of_points_in_cloud_(0),
		number_of_scans_assembled_(0),
		polar_to_cartesian_matrix_angle_min_(0), polar_to_cartesian_matrix_angle_max_(0), polar_to_cartesian_matrix_angle_increment_(0) {

	polar_to_cartesian_matrix_.resize(Eigen::NoChange, 0);
}

LaserScanToPointcloud::~LaserScanToPointcloud() {}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LaserScanToPointcloud-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void LaserScanToPointcloud::initNewPointCloud(size_t number_of_reserved_points) {
	pointcloud_ = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());

	pointcloud_->header.seq = number_of_pointclouds_created_++;
	pointcloud_->header.stamp = ros::Time::now();
	pointcloud_->header.frame_id = target_frame_;
	pointcloud_->height = 1;
	pointcloud_->width = 0;
	pointcloud_->fields.clear();
	pointcloud_->fields.resize(3);
	pointcloud_->fields[0].name = "x";
	pointcloud_->fields[0].offset = 0;
	pointcloud_->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
	pointcloud_->fields[0].count = 1;
	pointcloud_->fields[1].name = "y";
	pointcloud_->fields[1].offset = 4;
	pointcloud_->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
	pointcloud_->fields[1].count = 1;
	pointcloud_->fields[2].name = "z";
	pointcloud_->fields[2].offset = 8;
	pointcloud_->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
	pointcloud_->fields[2].count = 1;
	pointcloud_->point_step = 12;
	pointcloud_->row_step = 0;
	pointcloud_->data.reserve(number_of_reserved_points * pointcloud_->point_step);
	pointcloud_->is_dense = false;
}


bool LaserScanToPointcloud::updatePolarToCartesianProjectionMatrix(const sensor_msgs::LaserScanConstPtr& laser_scan) {
	size_t number_of_scan_points = laser_scan->ranges.size();
	if (polar_to_cartesian_matrix_.cols() != number_of_scan_points
			|| polar_to_cartesian_matrix_angle_min_ != laser_scan->angle_min
			|| polar_to_cartesian_matrix_angle_max_ != laser_scan->angle_max
			|| polar_to_cartesian_matrix_angle_increment_ != laser_scan->angle_increment) {

		ROS_DEBUG_STREAM("Updating polar to cartesian projection matrix with ->" \
				<< " [ranges.size()]:" << laser_scan->ranges.size() \
				<< " [angle_min]:" << laser_scan->angle_min \
				<< " [angle_max]:" << laser_scan->angle_max
				<< " [increment]:" << laser_scan->angle_increment);

		// recompute sin and cos values
		polar_to_cartesian_matrix_.resize(Eigen::NoChange, laser_scan->ranges.size());
		polar_to_cartesian_matrix_angle_min_ = laser_scan->range_min;
		polar_to_cartesian_matrix_angle_max_ = laser_scan->range_max;
		polar_to_cartesian_matrix_angle_increment_ = laser_scan->angle_increment;

		double current_angle = laser_scan->angle_min;
		for (size_t pointPos = 0; pointPos < number_of_scan_points; ++pointPos) {
			polar_to_cartesian_matrix_(0, pointPos) = std::cos(current_angle);
			polar_to_cartesian_matrix_(1, pointPos) = std::sin(current_angle);
			current_angle += laser_scan->angle_increment;
		}

		return true;
	}

	return false;
}


bool LaserScanToPointcloud::integrateLaserScanWithShpericalLinearInterpolation(const sensor_msgs::LaserScanConstPtr& laser_scan) {
	ros::Duration scan_duration((float)(laser_scan->ranges.size() - 1) * laser_scan->time_increment);
	ros::Time scan_start_time = laser_scan->header.stamp;
	ros::Time scan_end_time = scan_start_time + scan_duration;

	updatePolarToCartesianProjectionMatrix(laser_scan);

	double min_range_cutoff = laser_scan->range_min * min_range_cutoff_percentage_;
	double max_range_cutoff = laser_scan->range_max * max_range_cutoff_percentage_;

	size_t number_of_scan_points = laser_scan->ranges.size();
	for (size_t pointPos = 0; pointPos < number_of_scan_points; ++pointPos) {
		float point_range_value = laser_scan->ranges[pointPos];
		if (point_range_value > min_range_cutoff && point_range_value < max_range_cutoff) {
			tf2::Vector3 projectedPoint(point_range_value * polar_to_cartesian_matrix_(0, pointPos), point_range_value * polar_to_cartesian_matrix_(1, pointPos), 0);
			++number_of_points_in_cloud_;
		}
	}

	return false;
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LaserScanToPointcloud-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>   ==========================================================================

// =============================================================================   <protected-section>   =======================================================================

// =============================================================================   </protected-section>   ======================================================================

// =============================================================================   <private-section>   =========================================================================

// =============================================================================   </private-section>   ========================================================================
