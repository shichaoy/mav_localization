/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */

#include <mav_localization/ObservationModel.h>

using namespace std;
using namespace tf;

namespace MAV_localization
{

  ObservationModel::ObservationModel ( ros::NodeHandle* nh, boost::shared_ptr<MapModel> mapModel, EngineT* rngEngine ) :
    m_mapModel ( mapModel ), m_rngNormal ( *rngEngine, NormalDistributionT ( 0.0, 1.0 ) ), m_rngUniform ( *rngEngine,
        UniformDistributionT ( 0.0, 1.0 ) ), m_weightRoll ( 1.0 ), m_weightPitch ( 1.0 ), m_weightZ ( 1.0 ), m_sigmaZ ( 0.02 ), m_sigmaRoll (
          0.05 ), m_sigmaPitch ( 0.05 )
  {

    m_map = m_mapModel->getMap();

    nh->param ( "weight_factor_roll", m_weightRoll, m_weightRoll );
    nh->param ( "weight_factor_pitch", m_weightPitch, m_weightPitch );
    nh->param ( "weight_factor_z", m_weightZ, m_weightZ );
    nh->param ( "motion_sigma_z", m_sigmaZ, m_sigmaZ );
    nh->param ( "motion_sigma_roll", m_sigmaRoll, m_sigmaRoll );
    nh->param ( "motion_sigma_pitch", m_sigmaPitch, m_sigmaPitch );

    if ( m_sigmaZ <= 0.0 || m_sigmaRoll <= 0.0 || m_sigmaPitch <= 0.0 )
    {
      ROS_ERROR ( "Sigma (std.dev) needs to be > 0 in ObservationModel" );
    }
  }

  ObservationModel::~ObservationModel()
  {
  }

  void ObservationModel::integratePoseMeasurement ( Particles& particles, double poseRoll, double posePitch,
      const tf::StampedTransform& odomToBase )
  {
    double poseHeight = odomToBase.getOrigin().getZ();
    ROS_INFO ( "Pose measurement z=%f R=%f P=%f", poseHeight, poseRoll, posePitch );

    //#pragma omp parallel for
    for ( unsigned i = 0; i < particles.size(); ++i )
    {
      // integrate IMU meas.:
      double roll, pitch, yaw;
      particles[i].pose.getBasis().getRPY ( roll, pitch, yaw );
// 	std::cout << "Predicted roll and pitch are: " <<  roll << "  " << pitch << std::endl;

      particles[i].weight += m_weightRoll * logLikelihood ( poseRoll - roll, m_sigmaRoll );
      particles[i].weight += m_weightPitch * logLikelihood ( posePitch - pitch, m_sigmaPitch );

      // integrate height measurement (z)
      double heightError;
      if ( getHeightError ( particles[i], odomToBase, heightError ) )
      {
        particles[i].weight += m_weightZ * logLikelihood ( heightError, m_sigmaZ );
        ROS_DEBUG ( "heightError of number %d particle is:  %f", i, heightError );
      }
      else
      {
        ROS_DEBUG ( "Cannot get height from the map" );
      }

    }
//         if ( i == 100 )
//             std::cout << "z error is: " << heightError << std::endl;

  }

  void ObservationModel::integratePoseMeasurement ( Particles& particles, double poseRoll, double posePitch,
      double groundHeight )
  {
    ROS_DEBUG ( "Pose measurement z=%f R=%f P=%f", groundHeight, poseRoll, posePitch );

    //#pragma omp parallel for
    for ( unsigned i = 0; i < particles.size(); ++i )
    {
      // integrate IMU meas.:
      double roll, pitch, yaw;
      particles[i].pose.getBasis().getRPY ( roll, pitch, yaw );
      double height;
      height = particles[i].pose.getOrigin().getZ();
      if ( i % 50 == 0)
	  ROS_DEBUG ("Predicted Pose z=%f R=%f P=%f", height, roll, pitch);

      particles[i].weight += m_weightRoll  * logLikelihood ( poseRoll  - roll, m_sigmaRoll );
      particles[i].weight += m_weightPitch * logLikelihood ( posePitch - pitch, m_sigmaPitch );

      // integrate height measurement (z)
      particles[i].weight += m_weightZ     * logLikelihood ( groundHeight - height, m_sigmaZ );
    }
  }

  void ObservationModel::setMap ( boost::shared_ptr<octomap::OcTree> map )
  {
    m_map = map;
  }

}
