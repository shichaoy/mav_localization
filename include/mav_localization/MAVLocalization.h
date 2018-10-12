/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */

#ifndef MAVLOCALIZATION_MAVLOCALIZATION_H_
#define MAVLOCALIZATION_MAVLOCALIZATION_H_

#include <ctime>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mav_localization/MAVLocalization_defs.h>
#include <mav_localization/MotionModel.h>
#include <mav_localization/ObservationModel.h>
#include <mav_localization/RaycastingModel.h>
#ifndef SKIP_ENDPOINT_MODEL
#include <mav_localization/EndpointModel.h>
#endif

#include <octomap/octomap.h>
#include <sensor_msgs/Imu.h>
#include <boost/circular_buffer.hpp>

#include <nav_msgs/Odometry.h>

#include "rangeflow_odom/vo_state.h"

//#include <planesegmentation/PlaneSegmenter.h>

//using namespace CA;

namespace MAV_localization
{

  static inline void getRP ( const geometry_msgs::Quaternion& msg_q, double& roll, double& pitch )
  {
    tf::Quaternion bt_q;
    tf::quaternionMsgToTF ( msg_q, bt_q );
    double useless_yaw;
    tf::Matrix3x3 ( bt_q ).getRPY ( roll, pitch, useless_yaw );

    if ( std::abs ( useless_yaw ) > 0.00001 )
      ROS_DEBUG ( "Non-zero yaw in IMU quaterion is ignored" );
  }

  class MAVLocalization
  {
    public:
      // some typedefs
      //typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
      
      tf::Pose prevPose;

    public:
      MAVLocalization ( unsigned randomSeed );
      virtual ~MAVLocalization();

      virtual void pointCloudCallback ( const sensor_msgs::PointCloud2::ConstPtr& msg, const rangeflow_odom::vo_state::ConstPtr& odom);
      void initPoseCallback ( const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg );

      bool globalLocalizationCallback ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res );
      void imuCallback ( const sensor_msgs::ImuConstPtr& msg );

      /**
       * Importance sampling from m_particles according to weights,
       * resets weight to 1/numParticles. Uses low variance sampling
       *
       * @param numParticles how many particles to sample, 0 (default): keep size of particle distribution
       */
      void resample ( unsigned numParticles = 0 );
      
      void random_resample ( unsigned numParticles = 0 );
      
      void augmented_resample ( unsigned numParticles = 0 );
      
      /// Returns index of particle with highest weight (log or normal scale)
      unsigned getBestParticleIdx() const;
      /// Returns the 6D pose of a particle
      tf::Pose getParticlePose ( unsigned particleIdx ) const;
      /// Returns the 6D pose of the best particle (highest weight)
      tf::Pose getBestParticlePose() const;
      /// Returns the 6D pose of the weighted mean particle
      tf::Pose getMeanParticlePose() const;

      /// function call for global initialization (called by globalLocalizationCallback)
      void initGlobal();

    protected:
      /**
       * General reset of the filter:
       * sets pose around initial pose (or truepose, if requested) and resets
       * the internal state. Calls initPoseCallback().
       */
      void reset();

      // converts particles into PoseArray and publishes them for visualization
      void publishPoseEstimate ( const ros::Time& time, bool publish_eval );

      /**
       * Normalizes the weights and transforms from log to normal scale
       * m_minWeight gives the lower bound for weight (normal scale).
       * No adjustment will be done for minWeight = 0 (default)
       */
      void normalizeWeights();

      /// cumulative weight of all particles (=1 when normalized)
      double getCumParticleWeight() const;

      /**
       * nEff - returns the number of effective particles = 1/sum(w_i^2)
       *
       * Needed for selective resampling (Doucet 98, Arulampalam 01), when nEff < n/2
       **/
      double nEff() const;

      /**
       * Converts particles into log scale
       */
      void toLogForm();

      /**
       * Returns the IMU message with stamp closest to a given stamp.
       * @param[in] stamp Timestamp to search.
       * @param[out] imuStamp Stamp of closest IMU message (or interpolation of two IMU messages).
       * @param[out] angleX Interpolated roll angle.
       * @param[out] angleY Interpolated pitch angle.
       * @return Success.
       */
      bool getImuMsg ( const ros::Time& stamp, ros::Time& imuStamp, double& angleX, double& angleY ) const;
      
      /**
       * Prepares a PointCloud msg to be integrated into the observations model. Filters
       * near range, floor and subsamples a sparse point cloud (out of m_numSensorBeams points)
       *
       */
      void preparePointCloud ( const sensor_msgs::PointCloud2::ConstPtr& msg, PointCloud& pc,
                               std::vector<float>& ranges );

      void detectGroundPlane ( const PointCloud& pc, PointCloud& ground, PointCloud& nonground,
                               double distanceThreshold, double angleThreshold,
                               double groundFilterPlaneDistance, double groundHeight );

      void voxelGridSampling ( const PointCloud & pc, pcl::PointCloud<int> & sampledIndices,
                               double searchRadius ) const;

      bool checkAboveMotionThreshold ( const tf::Pose& odomTransform );

      bool localizeWithMeasurement ( const PointCloud& pc_filtered, const std::vector<float>& ranges,
                                     double max_range );

      void constrainMotion ( const tf::Pose& odomPose );

      /**
       * Initializes z, roll and pitch values either from parameters
       * or fromo real (odom) values
       */
      void initZRP ( double& z, double& roll, double& pitch );

      bool lookupPoseHeight ( const ros::Time& t, double& poseHeight ) const;

      EngineT m_rngEngine;
      /// standard normal distribution
      NormalGeneratorT m_rngNormal;
      /// uniform distribution [0:1]
      UniformGeneratorT m_rngUniform;
      boost::shared_ptr<MotionModel> m_motionModel;
      boost::shared_ptr<ObservationModel> m_observationModel;
      boost::shared_ptr<MapModel> m_mapModel;

      ros::NodeHandle m_nh, m_privateNh;
      ros::Subscriber m_pauseIntegrationSub;

            
      message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub;
      message_filters::Subscriber<rangeflow_odom::vo_state> odomSub;
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,rangeflow_odom::vo_state> MySyncPolicy;
      message_filters::Synchronizer<MySyncPolicy>* sync1;
      

      message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
      message_filters::Subscriber<rangeflow_odom::vo_state>* m_voSub;
      
      tf::MessageFilter<sensor_msgs::PointCloud2>* m_pointCloudFilter;

      message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseSub;
      tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseFilter;

      ros::Publisher m_posePub,m_poseCovPub, m_poseArrayPub, m_filteredPointCloudPub, m_groundCloudPub, m_finalcloudPub, m_imuOdomPub;

      ros::Subscriber m_imuSub;
      ros::ServiceServer m_globalLocSrv, m_pauseLocSrv, m_resumeLocSrv;
      tf::TransformListener m_tfListener;
      tf::TransformBroadcaster m_tfBroadcaster;
      ros::Timer m_timer;

      std::string m_odomFrameId;
      std::string m_targetFrameId;
      std::string m_baseFrameId;
      std::string m_baseFootprintId;
      std::string m_globalFrameId;

      bool m_useRaycasting;
      int m_numParticles;
      double m_sensorSampleDist;

      double m_nEffFactor;
      double m_minParticleWeight;
      Vector6d m_initPose;	// fixed init. pose (from params)
      Vector6d m_initNoiseStd; // Std.dev for init. pose
      bool m_initPoseRealZRP; // override z, roll, pitch with real values from robot

      double m_filterMaxRange;
      double m_filterMinRange;

      Particles m_particles;
      int m_bestParticleIdx;
      tf::Pose m_odomPose; // incrementally added odometry pose (=dead reckoning)
      nav_msgs::Odometry m_imu_odom; //integrated pose from IMU 
      bool firstIMU;
      ros::Time current_imu_ts, last_imu_ts;
      double current_acc_x, last_acc_x;
      double current_acc_y, last_acc_y;
      double current_acc_z, last_acc_z;
      
      double vel_x, vel_y, vel_z;
      double pos_x, pos_y, pos_z;
      
      geometry_msgs::PoseArray m_poseArray; // particles as PoseArray (preallocated)
      boost::circular_buffer<sensor_msgs::Imu> m_lastIMUMsgBuffer;

      bool m_bestParticleAsMean;
      bool m_receivedSensorData;
      bool m_initialized;
      bool m_initGlobal;
      bool m_paused;
      bool m_syncedTruepose;
      double w_long;
      double w_short;
      double w_avg;
      double a_long;
      double a_short;

      double m_observationThresholdTrans;
      double m_observationThresholdRot;
      double m_observationThresholdHeadYawRot;
      double m_observationThresholdHeadPitchRot;
      double m_temporalSamplingRange;
      double m_transformTolerance;
      ros::Time m_lastLaserTime;
      ros::Time m_lastPointCloudTime;


      bool m_detectGroundPlane;
      double m_distanceThreshold;
      double m_angleThreshold;
      double m_groundFilterPlaneDistance;
      double m_sensorSampleDistGroundFactor;
      int m_numFloorPoints;
      int m_numNonFloorPoints;
      int m_select_number;
      
      int vo_threshold; 
      int minimum_update_threshold;


      /// sensor data last integrated at this odom pose, to check if moved enough since then
      tf::Pose m_lastLocalizedPose;
      tf::StampedTransform m_latest_transform;

      /// absolute, summed yaw angle since last measurement integraton
      double m_headYawRotationLastScan;
      /// absolute, summed pitch angle since last measurement integraton
      double m_headPitchRotationLastScan;

      bool m_useIMU; ///< True = use IMU for initialization and observation models, false = use orientation from odometry
      bool m_constrainMotionZ; /// < True = do not estimate height, directly use odometry pose
      bool m_constrainMotionRP; /// < True = do not estimate roll and pitch, directly use odometry pose

      // timer stuff
      bool m_useTimer;
      double m_timerPeriod;

      //Motion model test
      bool m_motionTest;

      FILE* fileVar;
      //PlaneSegmenter planeSegmenter;
      tf::Pose lastOdomPose;
      tf::Transform lastrelativeTrans;

      double totalTime;
      int FramNum;
      double m_heightEstGround;
      bool m_logIntofile;
  };
}
#endif
