/**\file laserscan_to_ros_pointcloud.cpp
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <laserscan_to_pointcloud/laserscan_to_ros_pointcloud.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <imports>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </imports>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

namespace laserscan_to_pointcloud {

// =============================================================================  <public-section>  ============================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
LaserScanToROSPointcloud::LaserScanToROSPointcloud(std::string target_frame, bool include_laser_intensity, double min_range_cutoff_percentage, double max_range_cutoff_percentage) :
		LaserScanToPointcloud(target_frame, min_range_cutoff_percentage, max_range_cutoff_percentage),
		include_laser_intensity_(include_laser_intensity),
		pointcloud_data_position_(NULL) {
}

LaserScanToROSPointcloud::~LaserScanToROSPointcloud() {	}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LaserScanToROSPointcloud-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void LaserScanToROSPointcloud::initNewPointCloud(size_t number_of_reserved_points) {
	pointcloud_ = sensor_msgs::PointCloud2Ptr(new sensor_msgs::PointCloud2());
	resetNumberOfPointsInCloud();
	resetNumberOfScansAsembledInCurrentCloud();

	pointcloud_->header.seq = getNumberOfPointcloudsCreated();
	pointcloud_->header.stamp = ros::Time::now();
	pointcloud_->header.frame_id = getTargetFrame();
	pointcloud_->height = 1;
	pointcloud_->width = 0;
	pointcloud_->fields.clear();
	pointcloud_->fields.resize(include_laser_intensity_ ? 4 : 3);
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

	if (include_laser_intensity_) {
		pointcloud_->fields[3].name = "intensity";
		pointcloud_->fields[3].offset = 12;
		pointcloud_->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
		pointcloud_->fields[3].count = 1;
		pointcloud_->point_step += 4;
	}

	pointcloud_->row_step = 0;
	pointcloud_->data.reserve(number_of_reserved_points * pointcloud_->point_step);
	pointcloud_->is_dense = true;
	incrementNumberOfPointCloudsCreated();
}

void LaserScanToROSPointcloud::addMeasureToPointCloud(const tf2::Vector3& point, float intensity) {
	*pointcloud_data_position_++ = (float)point.getX();
	*pointcloud_data_position_++ = (float)point.getY();
	*pointcloud_data_position_++ = (float)point.getZ();

	if (include_laser_intensity_) {
		*pointcloud_data_position_++ = intensity;
	}
}

void LaserScanToROSPointcloud::setupPointCloudForNewLaserScan(size_t number_laser_scan_points) {
	pointcloud_->data.resize((getNumberOfPointsInCloud() + number_laser_scan_points) * pointcloud_->point_step); // resize to fit all points in the LaserScan
	pointcloud_data_position_ = (float*)(&pointcloud_->data[getNumberOfPointsInCloud() * pointcloud_->point_step]);
}

void LaserScanToROSPointcloud::finishLaserScanIntegration() {
	pointcloud_->width = getNumberOfPointsInCloud();
	pointcloud_->row_step = pointcloud_->width * pointcloud_->point_step;
	pointcloud_->data.resize(pointcloud_->height * pointcloud_->row_step); // resize to shrink the vector size to the real number of points inserted
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LaserScanToROSPointcloud-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void LaserScanToROSPointcloud::setIncludeLaserIntensity(bool include_laser_intensity) {
	if (include_laser_intensity != include_laser_intensity_) {
		include_laser_intensity_ = include_laser_intensity;
		initNewPointCloud();
	}
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// =============================================================================  </public-section>  ===========================================================================

// =============================================================================   <protected-section>   =======================================================================
// =============================================================================   </protected-section>  =======================================================================

// =============================================================================   <private-section>   =========================================================================
// =============================================================================   </private-section>  =========================================================================

} /* namespace laserscan_to_pointcloud */
