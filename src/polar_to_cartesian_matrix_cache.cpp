/**\file polar_to_cartesian_matrix_cache.cpp
 * \brief Implementation of a cache of matrices to convert polar coordinates to Cartesian coordinates.
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#include <laserscan_to_pointcloud/polar_to_cartesian_matrix_cache.h>
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace laserscan_to_pointcloud {
// =============================================================================  <public-section>   ===========================================================================
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <PolarToCartesianCache-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
const Eigen::Array2Xf& PolarToCartesianCache::getPolarToCartesianMatrix(size_t polar_to_cartesian_matrix_number_measurements, float polar_to_cartesian_matrix_angle_min, float polar_to_cartesian_matrix_angle_increment) {
	for (size_t i = 0; i < matrices_cache_.size(); ++i) {
		if (matrices_cache_[i].polar_to_cartesian_matrix_number_measurements_ 	== polar_to_cartesian_matrix_number_measurements &&
			matrices_cache_[i].polar_to_cartesian_matrix_angle_min_ 			== polar_to_cartesian_matrix_angle_min &&
			matrices_cache_[i].polar_to_cartesian_matrix_angle_increment_ 		== polar_to_cartesian_matrix_angle_increment) {
			return matrices_cache_[i].polar_to_cartesian_matrix_;
		}
	}

	ROS_INFO_STREAM("Adding new polar to Cartesian projection matrix with ->" \
				<< "\n\t[number_measuremnts]: " << polar_to_cartesian_matrix_number_measurements \
				<< "\n\t         [angle_min]: " << polar_to_cartesian_matrix_angle_min \
				<< "\n\t   [angle_increment]: " << polar_to_cartesian_matrix_angle_increment);

	matrices_cache_.push_back(PolarToCartesianMatrix(polar_to_cartesian_matrix_number_measurements, polar_to_cartesian_matrix_angle_min, polar_to_cartesian_matrix_angle_increment));
	Eigen::Array2Xf& polar_to_cartesian_matrix = matrices_cache_.back().polar_to_cartesian_matrix_;

	polar_to_cartesian_matrix.resize(Eigen::NoChange, polar_to_cartesian_matrix_number_measurements);

	double current_angle = polar_to_cartesian_matrix_angle_min;
	for (size_t point_pos = 0; point_pos < polar_to_cartesian_matrix_number_measurements; ++point_pos) {
		polar_to_cartesian_matrix(0, point_pos) = std::cos(current_angle);
		polar_to_cartesian_matrix(1, point_pos) = std::sin(current_angle);
		current_angle += polar_to_cartesian_matrix_angle_increment;
	}

	return polar_to_cartesian_matrix;
}
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </PolarToCartesianCache-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// =============================================================================  </public-section>   ==========================================================================
} /* namespace laserscan_to_pointcloud */

