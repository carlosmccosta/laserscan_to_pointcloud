#pragma once

/**\file polar_to_cartesian_matrix_cache.h
 * \brief Cache of matrices to convert polar coordinates to Cartesian coordinates
 *
 * @version 1.0
 * @author Carlos Miguel Correia da Costa
 */


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <vector>

// ROS includes
#include <ros/ros.h>

// external includes
#include <boost/smart_ptr/shared_ptr.hpp>
#include <Eigen/Core>

// project includes
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </includes>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


namespace laserscan_to_pointcloud {
// ########################################################################   PolarToCartesianCache   ##########################################################################

struct PolarToCartesianMatrix {
		PolarToCartesianMatrix(size_t polar_to_cartesian_matrix_number_measurements, float polar_to_cartesian_matrix_angle_min, float polar_to_cartesian_matrix_angle_increment) :
			polar_to_cartesian_matrix_number_measurements_(polar_to_cartesian_matrix_number_measurements),
			polar_to_cartesian_matrix_angle_min_(polar_to_cartesian_matrix_angle_min),
			polar_to_cartesian_matrix_angle_increment_(polar_to_cartesian_matrix_angle_increment) {}
		virtual ~PolarToCartesianMatrix() {}

		size_t polar_to_cartesian_matrix_number_measurements_;
		float polar_to_cartesian_matrix_angle_min_;
		float polar_to_cartesian_matrix_angle_increment_;

		Eigen::Array2Xf polar_to_cartesian_matrix_; ///> matrix with sin(theta) and cos(theta) for each laser scan ray
};


class PolarToCartesianCache {
	// ========================================================================   <public-section>   ===========================================================================
	public:
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <constructors-destructor>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		PolarToCartesianCache() {}
		virtual ~PolarToCartesianCache() {}

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </constructors-destructor>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <PolarToCartesianCache-functions>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		const Eigen::Array2Xf& getPolarToCartesianMatrix(size_t polar_to_cartesian_matrix_number_measurements, float polar_to_cartesian_matrix_angle_min, float polar_to_cartesian_matrix_angle_increment);
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </PolarToCartesianCache-functions>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <gets>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		inline const std::vector<PolarToCartesianMatrix>& getMatricesCache() const { return matrices_cache_; }
		// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </gets>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// ========================================================================   </public-section>   ==========================================================================


	// ========================================================================   <protected-section>   ========================================================================
	protected:
		std::vector<PolarToCartesianMatrix> matrices_cache_;
	// ========================================================================   </protected-section>  ========================================================================
};
} /* namespace laserscan_to_pointcloud */
