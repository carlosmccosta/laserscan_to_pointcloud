#pragma once

/**\file LaserScanToPointcloud.h
 * \brief Description...
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <cmath>
#include <string>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

// external includes
#include <boost/math/special_functions/fpclassify.hpp>
#include <Eigen/Core>

// project includes
#include <laserscan_to_pointcloud/tf_collector.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace laserscan_to_pointcloud {
// ########################################################################   LaserScanToPointcloud   ##########################################################################
/**
 * \brief PointCloud2 builder from LaserScans
 */
class LaserScanToPointcloud {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <typedefs>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </typedefs>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <enums>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </enums>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constants>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constants>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		LaserScanToPointcloud(std::string target_frame = "", double min_range_cutoff_percentage = 1.05, double max_range_cutoff_percentage = 0.95, int number_of_tf_queries_for_spherical_interpolation = 4, double tf_lookup_timeout = 0.2);
		virtual ~LaserScanToPointcloud();
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LaserScanToPointcloud-virtual-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		virtual void initNewPointCloud(size_t number_of_reserved_points = 684) = 0;
		virtual void addMeasureToPointCloud(const tf2::Vector3& point, float intensity) = 0;
		virtual void setupPointCloudForNewLaserScan(size_t number_laser_scan_points) = 0;
		virtual void finishLaserScanIntegration() = 0;
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LaserScanToPointcloud-virtual-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <LaserScanToPointcloud-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		bool updatePolarToCartesianProjectionMatrix(const sensor_msgs::LaserScanConstPtr& laser_scan);
		bool integrateLaserScanWithShpericalLinearInterpolation(const sensor_msgs::LaserScanConstPtr& laser_scan);
		bool lookForTransformWithRecovery(tf2::Vector3& translation_out, tf2::Quaternion& rotation_out, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration& timeout = ros::Duration(0.2));
		bool lookForTransformWithRecovery(tf2::Transform& point_transform_out, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration& timeout = ros::Duration(0.2));
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </LaserScanToPointcloud-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		inline const std::string& getTargetFrame() const { return target_frame_; }
		inline const std::string& getRecoveryFrame() const { return recovery_frame_; }
		inline const std::string& getLaserFrame() const { return laser_frame_; }
		inline double getMaxRangeCutoffPercentageOffset() const { return max_range_cutoff_percentage_offset_;}
		inline double getMinRangeCutoffPercentageOffset() const { return min_range_cutoff_percentage_offset_; }

		inline size_t getNumberOfPointcloudsCreated() const { return number_of_pointclouds_created_; }
		inline size_t getNumberOfPointsInCloud() const { return number_of_points_in_cloud_; }
		inline size_t getNumberOfScansAssembledInCurrentPointcloud() const { return number_of_scans_assembled_in_current_pointcloud_; }
		inline ros::Duration getTfLookupTimeout() const { return tf_lookup_timeout_; }
		inline int getNumberOfTfQueriesForSphericalInterpolation() const { return number_of_tf_queries_for_spherical_interpolation_; }
		inline bool isRemoveInvalidMeasurements() const { return remove_invalid_measurements_; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <sets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		inline void setTargetFrame(const std::string& target_frame) { target_frame_ = target_frame; }
		inline void setLaserFrame(const std::string& laser_frame) { laser_frame_ = laser_frame; }
		void setRecoveryFrame(const std::string& recovery_frame, const tf2::Transform& recovery_to_target_frame_transform = tf2::Transform::getIdentity());
		inline void setMaxRangeCutoffPercentageOffset(double max_range_cutoff_percentage_offset) { max_range_cutoff_percentage_offset_ = max_range_cutoff_percentage_offset; }
		inline void setMinRangeCutoffPercentageOffset(double min_range_cutoff_percentage_offset) { min_range_cutoff_percentage_offset_ = min_range_cutoff_percentage_offset; }
		inline void incrementNumberOfPointCloudsCreated() { ++number_of_pointclouds_created_; }
		inline void resetNumberOfPointsInCloud() { number_of_points_in_cloud_ = 0; }
		inline void resetNumberOfScansAsembledInCurrentCloud() { number_of_scans_assembled_in_current_pointcloud_ = 0; }
		inline void setTFLookupTimeout(double tf_lookup_timeout) { tf_lookup_timeout_.fromSec(tf_lookup_timeout); }
		inline TFCollector& getTfCollector() { return tf_collector_; }
		inline void setNumberOfTfQueriesForSphericalInterpolation(int number_of_tf_queries_for_spherical_interpolation) { number_of_tf_queries_for_spherical_interpolation_ = number_of_tf_queries_for_spherical_interpolation; }
		inline void setRemoveInvalidMeasurements(bool removeInvalidMeasurements) { remove_invalid_measurements_ = removeInvalidMeasurements; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </sets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>   ==========================================================================


	// ========================================================================   <protected-section>   ========================================================================
	protected:
	// ========================================================================   </protected-section>  ========================================================================

	// ========================================================================   <private-section>   ==========================================================================
	private:
		// configuration fields
		std::string target_frame_;
		std::string recovery_frame_;
		std::string laser_frame_;
		tf2::Transform recovery_to_target_frame_transform_;
		double min_range_cutoff_percentage_offset_;
		double max_range_cutoff_percentage_offset_;
		int number_of_tf_queries_for_spherical_interpolation_;
		ros::Duration tf_lookup_timeout_;
		bool remove_invalid_measurements_;

		// state fields
		size_t number_of_pointclouds_created_;
		size_t number_of_points_in_cloud_;
		size_t number_of_scans_assembled_in_current_pointcloud_;
		Eigen::Array2Xf polar_to_cartesian_matrix_; ///> matrix with sin(theta) and cos(theta) for each laser scan ray
		float polar_to_cartesian_matrix_angle_min_;
		float polar_to_cartesian_matrix_angle_max_;
		float polar_to_cartesian_matrix_angle_increment_;

		// communication fields
		TFCollector tf_collector_;
	// ========================================================================   </private-section>  ==========================================================================
};
} /* namespace laserscan_to_pointcloud */
