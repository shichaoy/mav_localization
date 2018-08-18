/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */


#ifndef MAVLOCALIZATION_MAVLOCALIZATION_DEFS_H_
#define MAVLOCALIZATION_MAVLOCALIZATION_DEFS_H_

#include <vector>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>

#include <tf/transform_datatypes.h>

#include <Eigen/Core>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace MAV_localization
{

/// Particle consists of a pose and a weight
struct Particle
{
    double weight;
    tf::Pose pose;
};

typedef std::vector<Particle> Particles;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


/// Boost RNG engine:
typedef boost::mt19937                 	EngineT;
/// Boost RNG distribution:
typedef boost::normal_distribution<>   	NormalDistributionT;
// Ugh! boost uniform_01 noise sucks: http://www.bnikolic.co.uk/blog/cpp-boost-uniform01.html
// => using uniform_real instead
typedef boost::uniform_real<>     		UniformDistributionT;
/// standard normal-distributed noise
typedef boost::variate_generator<EngineT&, NormalDistributionT>   NormalGeneratorT;
/// uniform noise in range [0,1)
typedef boost::variate_generator<EngineT&, UniformDistributionT>   UniformGeneratorT;


typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

}
#endif
