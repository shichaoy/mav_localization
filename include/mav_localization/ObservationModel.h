/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */


#ifndef MAVLOCALIZATION_OBSERVATIONMODEL_H_
#define MAVLOCALIZATION_OBSERVATIONMODEL_H_

#include <limits>
#include<cmath>

// #include <omp.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/transforms.h>

#include <mav_localization/MAVLocalization_defs.h>
#include <mav_localization/MapModel.h>
#include <octomap/octomap.h>

#include <sensor_msgs/PointCloud2.h>


namespace MAV_localization
{

/// sqrt(2*pi)
  const static double SQRT_2_PI = 2.506628274;

/// log(sqrt(2*pi))
  const static double LOG_SQRT_2_PI = 0.91893853320467274178;

  class ObservationModel
  {
    public:
      ObservationModel ( ros::NodeHandle* nh, boost::shared_ptr<MapModel> mapModel, EngineT * rngEngine );
      virtual ~ObservationModel();

      /// Helper function to compute the log likelihood
      static inline double logLikelihood ( double x, double sigma )
      {
        assert ( !isnan ( x ) );
        return -1.0* ( LOG_SQRT_2_PI ) - log ( sigma ) - ( ( x * x ) / ( 2* sigma * sigma ) );
      }

      /// Helper function to compute the log likelihood based on sqared distances
      static inline double logLikelihoodSq ( double x_sq, double sigma )
      {
        assert ( !isnan ( x_sq ) );
        return -1.0* ( LOG_SQRT_2_PI ) - log ( sigma ) - ( ( x_sq ) / ( 2* sigma * sigma ) );
      }


      //static double logLikelihoodSimple(double x, double sigma){
      //	return  -((x * x) / (2* sigma * sigma));
      //}

      /**
       * Integrate a measurement in particle set, update weights accordingly
       * Particle weights should be in log scale before, weights are added.
       */
      virtual void integrateMeasurement ( Particles& particles, const PointCloud& pc, 
					  const std::vector<float>& ranges, float max_range, 
					  const tf::Transform& baseToSensor, int bestParticleIdx ) = 0;

      virtual void integratePoseMeasurement ( Particles& particles, double roll, double pitch, 
					      const tf::StampedTransform& odomToBase );
      
      virtual void integratePoseMeasurement ( Particles& particles, double roll, double pitch, double groundHeight );

      virtual void setMap ( boost::shared_ptr<octomap::OcTree> map );
    protected:
      virtual bool getHeightError ( const Particle& p, const tf::StampedTransform& odomToBase, double& heightError ) const = 0;
      boost::shared_ptr<MapModel> m_mapModel;
      EngineT m_rngEngine;
      NormalGeneratorT m_rngNormal;
      UniformGeneratorT m_rngUniform;
      boost::shared_ptr<octomap::OcTree> m_map;
      ros::Publisher m_pc_pub;

      double m_weightRoll;
      double m_weightPitch;
      double m_weightZ;

      double m_sigmaZ;
      double m_sigmaRoll;
      double m_sigmaPitch;

      bool m_use_squared_error;

  };
}

#endif /* OBSERVATIONMODEL_H_ */
