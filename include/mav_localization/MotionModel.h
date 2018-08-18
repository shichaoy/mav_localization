/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */


#ifndef MAVLOCALIZATION_MOTIONMODEL_H_
#define MAVLOCALIZATION_MOTIONMODEL_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Cholesky>

#include <mav_localization/MAVLocalization_defs.h>

namespace MAV_localization
{
  class MotionModel
  {
    public:
      MotionModel ( ros::NodeHandle* nh, EngineT* rngEngine, tf::TransformListener* tf, const std::string& odomFrameId, const std::string& baseFrameId );
      virtual ~MotionModel();
      void reset();
      /// look up the odom pose at a certain time through tf
      bool lookupOdomPose ( const ros::Time& t, tf::Stamped<tf::Pose>& odomPose ) const;

      /// looks up the odometry pose at time t and then calls computeOdomTransform()
      bool lookupOdomTransform ( const ros::Time& t, tf::Transform& odomTransform ) const;

      /// computes the odometry transform from m_lastOdomPose to currentPose as relative
      /// 6D rigid body transform (=identity if m_lastOdomPose not available)
      tf::Transform computeOdomTransform ( const tf::Transform currentPose ) const;

      /// lookup the local target frame in the base frame (local transform)
      bool lookupLocalTransform ( const std::string& targetFrame, const ros::Time& t, tf::StampedTransform& localTransform ) const;

      /// lookup the height of the torso, based on tf between the base and footprint frames
      bool lookupPoseHeight ( const ros::Time& t, double& poseHeight ) const;

      /// apply odomTransform to all particles (with random noise and calibration)
      void applyOdomTransform ( Particles& particles, const tf::Transform& odomTransform );

      /// add big random noise to all particles if the visual odometry fails
      void addRandomNoise ( Particles& particles );

      /// apply odomTransform to all particles (noisy), with temporal sampling over the range of dt.
      /// Times are sampled in an interval +-dt/2 around t, iff dt > 0.0.
      bool applyOdomTransformTemporal ( Particles& particles, const ros::Time& t, double dt );

      /// store odomPose as m_lastOdomPose to compute future odom transforms
      void storeOdomPose ( const tf::Stamped<tf::Pose>& odomPose );

      /// get the last stored odomPose
      /// returns false when there is no valid previous pose stored
      bool getLastOdomPose ( tf::Stamped<tf::Pose>& lastOdomPose ) const;


      // aligns fixed-size members in memory
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW


      std::string m_odomFrameId;
      std::string m_baseFrameId;
      std::string m_footprintFrameId;

    protected:
      /// apply odomTransform to one particle pose (with random noise and calibration)
      void applyOdomTransform ( tf::Pose& particlePose, const tf::Transform& odomTransform );

      /// Transform a particle's pose with the relative odomTransform with added random noise
      void transformPose ( tf::Pose& particlePose, const tf::Transform& odomTransform );

      /// Generates motion noise corresponding to odomTransform
      /// May not be called in parallel, accesses the random generator m_rngNormal
      tf::Transform odomTransformNoise ( const tf::Transform& odomTransform );

      /// @return calibrated odometry transform w.r.t. 2D drift (pos. + orientation)
      tf::Transform calibrateOdometry ( const tf::Transform& odomTransform ) const;

      tf::TransformListener* m_tfListener;

      NormalGeneratorT m_rngNormal; // standard normal-distributed noise
      UniformGeneratorT m_rngUniform;
      // parameters:
      /// variance parameters for calibrated odometry noise
      Eigen::Matrix3d m_odomNoise2D;
      /// systematic calibration parameters for odometry drift
      Eigen::Matrix3d m_odomCalibration2D;
      /// std.dev param of noise in Z (depending on distance traveled)
      double m_odomNoiseZ;
      /// std.dev param of noise in roll (depending on distance traveled)
      double m_odomNoiseRoll;
      /// std.dev param of noise in pitch (depending on distance traveled)
      double m_odomNoisePitch;

      bool m_firstOdometryReceived;
      tf::Stamped<tf::Pose> m_lastOdomPose;


  };
}

#endif
