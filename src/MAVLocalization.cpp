/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */

#include <mav_localization/MAVLocalization.h>
#include <iostream>
#include <ctime>
#include <pcl/keypoints/uniform_sampling.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/sampling_surface_normal.h>

#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/graph/graph_concepts.hpp>


namespace MAV_localization
{
   MAVLocalization::MAVLocalization ( unsigned randomSeed ) :
      m_rngEngine ( randomSeed ), m_rngNormal ( m_rngEngine, NormalDistributionT ( 0.0, 1.0 ) ), m_rngUniform ( m_rngEngine,
            UniformDistributionT ( 0.0, 1.0 ) ), m_nh(), m_privateNh ( "~" ), m_odomFrameId ( "/odom_frame" ), m_targetFrameId (
               "/odom_frame" ), m_baseFrameId ( "/base_loc_frame" ), m_baseFootprintId ( "base_loc_footprint" ), m_globalFrameId ( "/map" ), m_useRaycasting (
                  true ), m_numParticles ( 500 ), m_sensorSampleDist ( 0.2 ), m_nEffFactor ( 1.0 ), m_minParticleWeight (
                     0.0 ), m_bestParticleIdx ( -1 ), m_lastIMUMsgBuffer ( 100 ), m_bestParticleAsMean ( true ), m_receivedSensorData (
                        false ), m_initialized ( false ), m_initGlobal ( false ), m_paused ( false ), m_syncedTruepose ( false ), m_observationThresholdTrans (
                           0.1 ), m_observationThresholdRot ( M_PI / 6 ), m_observationThresholdHeadYawRot ( 0.1 ), m_observationThresholdHeadPitchRot (
                              0.1 ), m_temporalSamplingRange ( 0.1 ), m_transformTolerance ( 0.1 ), m_headYawRotationLastScan ( 0.0 ), m_headPitchRotationLastScan ( 0.0 ), m_useIMU (
                                 false ), m_constrainMotionZ ( false ), m_constrainMotionRP ( false ), m_useTimer ( false ), m_timerPeriod ( 0.1 ), m_motionTest (
                                    false ), m_detectGroundPlane ( true ), m_distanceThreshold (
                                       0.04 ), m_angleThreshold ( 0.15 ), m_groundFilterPlaneDistance ( 0.07 ), m_sensorSampleDistGroundFactor ( 3 ), m_numFloorPoints (
                                          20 ), m_numNonFloorPoints ( 80 ), m_logIntofile ( false ), m_select_number ( 1000 ), vo_threshold ( 1000 ), minimum_update_threshold (50)
   {

      m_latest_transform.setData ( tf::Transform ( tf::createIdentityQuaternion() ) );
      m_privateNh.param ( "use_raycasting", 	m_useRaycasting, 	m_useRaycasting );
      m_privateNh.param ( "odom_frame_id", 	m_odomFrameId, 		m_odomFrameId );
      m_privateNh.param ( "target_frame_id", 	m_targetFrameId, 	m_targetFrameId );
      m_privateNh.param ( "base_frame_id", 	m_baseFrameId, 		m_baseFrameId );
      m_privateNh.param ( "base_footprint_id", 	m_baseFootprintId, 	m_baseFootprintId );
      m_privateNh.param ( "global_frame_id", 	m_globalFrameId, 	m_globalFrameId );
      m_privateNh.param ( "init_global", 		m_initGlobal, 		m_initGlobal );
      m_privateNh.param ( "best_particle_as_mean",m_bestParticleAsMean, 	m_bestParticleAsMean );
      m_privateNh.param ( "num_particles", 	m_numParticles, 	m_numParticles );
      m_privateNh.param ( "neff_factor", 		m_nEffFactor, 		m_nEffFactor );
      m_privateNh.param ( "min_particle_weight", 	m_minParticleWeight, 	m_minParticleWeight );

      m_privateNh.param ( "initial_pose/x",     	m_initPose ( 0 ), 	0.0 );
      m_privateNh.param ( "initial_pose/y", 	m_initPose ( 1 ), 	0.0 );
      m_privateNh.param ( "initial_pose/z", 	m_initPose ( 2 ),      -0.3 );
      m_privateNh.param ( "initial_pose/roll", 	m_initPose ( 3 ), 	0.0 );
      m_privateNh.param ( "initial_pose/pitch", 	m_initPose ( 4 ), 	0.0 );
      m_privateNh.param ( "initial_pose/yaw", 	m_initPose ( 5 ), 	0.0 );

      m_privateNh.param ( "initial_std/x",     	m_initNoiseStd ( 0 ), 	0.1 ); // 0.1
      m_privateNh.param ( "initial_std/y",     	m_initNoiseStd ( 1 ), 	0.1 ); // 0.1
      m_privateNh.param ( "initial_std/z",     	m_initNoiseStd ( 2 ), 	0.02 ); // 0.02
      m_privateNh.param ( "initial_std/roll",  	m_initNoiseStd ( 3 ), 	0.04 ); // 0.04
      m_privateNh.param ( "initial_std/pitch", 	m_initNoiseStd ( 4 ), 	0.04 ); // 0.04
      m_privateNh.param ( "initial_std/yaw",   	m_initNoiseStd ( 5 ), 	0.1 ); // M_PI/12

      // depth observation model parameters:
      m_privateNh.param ( "sensor_sampling_dist", m_sensorSampleDist, 	m_sensorSampleDist );
      m_privateNh.param ( "max_range", 		m_filterMaxRange, 	5.0 );
      m_privateNh.param ( "min_range", 		m_filterMinRange, 	0.5 );
      ROS_DEBUG ( "Using a range filter of %f to %f", m_filterMinRange, m_filterMaxRange );

      m_privateNh.param ( "update_min_trans", 	m_observationThresholdTrans, 		m_observationThresholdTrans );
      m_privateNh.param ( "update_min_rot", 	m_observationThresholdRot, 		m_observationThresholdRot );
      m_privateNh.param ( "update_min_head_yaw", 	m_observationThresholdHeadYawRot, 	m_observationThresholdHeadYawRot );
      m_privateNh.param ( "update_min_head_pitch",m_observationThresholdHeadPitchRot, 	m_observationThresholdHeadPitchRot );
      m_privateNh.param ( "temporal_sampling_range", m_temporalSamplingRange, m_temporalSamplingRange );
      m_privateNh.param ( "transform_tolerance", m_transformTolerance, m_transformTolerance );

      m_privateNh.param ( "use_imu", m_useIMU, m_useIMU );
      m_privateNh.param ( "constrain_motion_z", m_constrainMotionZ, m_constrainMotionZ );
      m_privateNh.param ( "constrain_motion_rp", m_constrainMotionRP, m_constrainMotionRP );

      m_privateNh.param ( "use_timer", m_useTimer, m_useTimer );
      m_privateNh.param ( "timer_period", m_timerPeriod, m_timerPeriod );
      m_privateNh.param ( "motion_test", m_motionTest, m_motionTest );
      m_privateNh.param ( "logIntofile", m_logIntofile, m_logIntofile );


      // point cloud observation model parameters
      m_privateNh.param ( "detect_ground_plane", m_detectGroundPlane, m_detectGroundPlane );
      m_privateNh.param ( "plane_fitting_distance_threshold", m_distanceThreshold, m_distanceThreshold );
      m_privateNh.param ( "plane_fitting_angle_threshold", m_angleThreshold, m_angleThreshold );
      m_privateNh.param ( "ground_height_threshold", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance );
      m_privateNh.param ( "sensor_sampling_dist_ground_factor", m_sensorSampleDistGroundFactor,
                          m_sensorSampleDistGroundFactor );
      m_privateNh.param ( "num_floor_points", m_numFloorPoints, m_numFloorPoints );
      m_privateNh.param ( "num_non_floor_points", m_numNonFloorPoints, m_numNonFloorPoints );
      m_privateNh.param ( "select_number",m_select_number, m_select_number );
      m_privateNh.param ( "vo_threshold",vo_threshold, vo_threshold );
      m_privateNh.param ("minimum_update_threshold", minimum_update_threshold, minimum_update_threshold);


      // motion model parameters
      m_motionModel = boost::shared_ptr<MotionModel> (
                         new MotionModel ( &m_privateNh, &m_rngEngine, &m_tfListener, m_odomFrameId, m_baseFrameId ) );

      if ( m_useRaycasting )
      {
         m_mapModel = boost::shared_ptr<MapModel> ( new OccupancyMap ( &m_privateNh ) );
         m_observationModel = boost::shared_ptr<ObservationModel> (
                                 new RaycastingModel ( &m_privateNh, m_mapModel, &m_rngEngine ) );
      }
      else
      {
#ifndef SKIP_ENDPOINT_MODEL
         //m_mapModel = boost::shared_ptr<MapModel>(new DistanceMap(&m_privateNh));
         m_mapModel = boost::shared_ptr<MapModel> ( new OccupancyMap ( &m_privateNh ) );
         m_observationModel = boost::shared_ptr<ObservationModel> (
                                 new EndpointModel ( &m_privateNh, m_mapModel, &m_rngEngine ) );
#else
         ROS_FATAL ( "EndpointModel not compiled due to missing dynamicEDT3D" );
         exit ( -1 );
#endif
      }

      m_particles.resize ( m_numParticles );
      m_poseArray.poses.resize ( m_numParticles );
      m_poseArray.header.frame_id = m_globalFrameId;
      m_tfListener.clear();

      // publish best
//     m_posePub               = m_nh.advertise<geometry_msgs::PoseStamped> ( "pf_pose", 10 );
      m_posePub               = m_nh.advertise<nav_msgs::Odometry> ( "pf_pose", 10 );
      m_poseCovPub  	      = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ( "pf_pose_cov", 10 );
      m_poseArrayPub          = m_nh.advertise<geometry_msgs::PoseArray> ( "particlecloud", 10 );
      m_filteredPointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2> ( "filtered_cloud", 10 );
      m_groundCloudPub        = m_nh.advertise<sensor_msgs::PointCloud2> ( "ground_cloud", 10 );
      m_finalcloudPub         = m_nh.advertise<sensor_msgs::PointCloud2> ( "final_cloud", 10 );
      m_imuOdomPub            = m_nh.advertise<nav_msgs::Odometry> ( "imu_odom", 10 );

      //TODO Propagate particles independent of sensor callback
      reset();

      // ROS subscriptions last:
      m_globalLocSrv = m_nh.advertiseService ( "global_localization", &MAVLocalization::globalLocalizationCallback, this );


//       // subscription on point cloud, tf message filter
//       m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> ( m_nh, "point_cloud", 100 );
//       m_voSub = new message_filters::Subscriber<ca_rangeflow::vo_state> ( m_nh, "rangflow_state", 100 );
//       m_pointCloudFilter = new tf::MessageFilter<sensor_msgs::PointCloud2> ( *m_pointCloudSub, m_tfListener, m_odomFrameId,
//             100 );
//       m_pointCloudFilter->registerCallback ( boost::bind ( &MAVLocalization::pointCloudCallback, this, _1, _2 ) );

      cloudSub.subscribe ( m_nh, "point_cloud", 10 );
      odomSub.subscribe ( m_nh, "rangeflow_state", 10 );
      sync1 = new message_filters::Synchronizer<MySyncPolicy> ( 100 );
      sync1->connectInput ( cloudSub, odomSub );
      sync1->registerCallback ( boost::bind ( &MAVLocalization::pointCloudCallback, this, _1, _2 ) );


      // subscription on init pose, tf message filter
      m_initPoseSub = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> ( m_nh, "initialpose", 2 );
      m_initPoseFilter = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped> ( *m_initPoseSub, m_tfListener,
            m_globalFrameId, 2 );
      m_initPoseFilter->registerCallback ( boost::bind ( &MAVLocalization::initPoseCallback, this, _1 ) );

      if ( m_useIMU )
      {
         m_imuSub = m_nh.subscribe ( "imu", 5, &MAVLocalization::imuCallback, this );
      }

      ROS_INFO ( "6DLocalization initialized with %d particles.", m_numParticles );

      if ( m_logIntofile == true )
      {
         fileVar = fopen ( "localization.txt", "w" );

         if ( fileVar == NULL )
         {
            ROS_ERROR_STREAM ( "Error creating localization.txt file" );
            perror ( "Error opening file" );
         }
         else
         {
            ROS_INFO ( "Creating localization.txt file successfully." );
         }
      }
      totalTime = 0.0;
      FramNum = 0;

      m_imu_odom.pose.pose.position.x = 0;
      m_imu_odom.pose.pose.position.y = 0;
      m_imu_odom.pose.pose.position.z = 0;

      vel_x = vel_y = vel_z = 0;
      pos_x = pos_y = pos_z = 0;

      firstIMU = true;
   }

   MAVLocalization::~MAVLocalization()
   {

      delete m_pointCloudFilter;
      delete m_pointCloudSub;

      delete m_initPoseFilter;
      delete m_initPoseSub;

   }

   void MAVLocalization::reset()
   {
      if ( m_initGlobal )
      {
         this->initGlobal();
      }
      else
      {
         geometry_msgs::PoseWithCovarianceStampedPtr posePtr ( new geometry_msgs::PoseWithCovarianceStamped() );
         posePtr.reset ( new geometry_msgs::PoseWithCovarianceStamped() );

         for ( int i = 0; i < 6; ++i )
         {
            posePtr->pose.covariance.at ( i * 6 + i ) = m_initNoiseStd ( i ) * m_initNoiseStd ( i );
         }

         posePtr->pose.pose.position.x = m_initPose ( 0 );
         posePtr->pose.pose.position.y = m_initPose ( 1 );
         posePtr->pose.pose.position.z = m_initPose ( 2 );
         posePtr->header.frame_id = m_globalFrameId;
         tf::Quaternion quat;
         quat.setRPY ( m_initPose ( 3 ), m_initPose ( 4 ), m_initPose ( 5 ) );
         tf::quaternionTFToMsg ( quat, posePtr->pose.pose.orientation );

         ROS_DEBUG_STREAM ( "Initial Pose: " << std::endl << posePtr->pose );
         this->initPoseCallback ( posePtr );
      }
   }

   void MAVLocalization::constrainMotion ( const tf::Pose& odomPose )
   {
      // skip if nothing to do:
      if ( !m_constrainMotionZ && !m_constrainMotionRP )
      {
         return;
      }

      // reset z according to current odomPose:
      double z = odomPose.getOrigin().getZ();
      double odomRoll, odomPitch, uselessYaw;
      odomPose.getBasis().getRPY ( odomRoll, odomPitch, uselessYaw );

      ////#pragma omp parallel for
      for ( unsigned i = 0; i < m_particles.size(); ++i )
      {
         if ( m_constrainMotionZ )
         {
            tf::Vector3 pos = m_particles[i].pose.getOrigin();
            double floor_z = m_mapModel->getFloorHeight ( m_particles[i].pose );
            //double floor_z = 0.0;
            pos.setZ ( z + floor_z );
            m_particles[i].pose.setOrigin ( pos );
         }

         if ( m_constrainMotionRP )
         {
            double yaw = tf::getYaw ( m_particles[i].pose.getRotation() );
            m_particles[i].pose.setRotation ( tf::createQuaternionFromRPY ( odomRoll, odomPitch, yaw ) );

         }
      }
   }

   bool MAVLocalization::checkAboveMotionThreshold ( const tf::Pose& odomPose )
   {
      tf::Transform odomTransform = m_lastLocalizedPose.inverse() * odomPose;

      double yaw, pitch, roll;
      odomTransform.getBasis().getRPY ( roll, pitch, yaw );

      return ( odomTransform.getOrigin().length() >= m_observationThresholdTrans
               || std::abs ( yaw ) >= m_observationThresholdRot );
   }

   bool MAVLocalization::localizeWithMeasurement ( const PointCloud& pc_filtered, const std::vector<float>& ranges,
         double max_range )
   {
      ros::WallTime startTime = ros::WallTime::now();
      ros::Time t = pcl_conversions::fromPCL ( pc_filtered.header ).stamp;

//     for ( int i = 0; i < m_particles.size(); i ++ )
//     {
//       double roll, pitch, yaw;
//       m_particles[i].pose.getBasis().getRPY ( roll, pitch, yaw );
//       double height;
//       height = m_particles[i].pose.getOrigin().getZ();
//       ROS_DEBUG ( "First Particle Pose z=%f R=%f P=%f", height, roll, pitch );
//     }

      // apply motion model with temporal sampling:
      //m_motionModel->applyOdomTransformTemporal ( m_particles, t, m_temporalSamplingRange );

      // constrain to ground plane, if desired:
      tf::Stamped<tf::Transform> odomPose;
      assert ( m_motionModel->lookupOdomPose ( t, odomPose ) );
      //constrainMotion ( odomPose );

//          double tmp_roll, tmp_pitch, tmp_yaw;
//          odomPose.getBasis().getRPY ( tmp_roll,tmp_pitch,tmp_yaw );
//          ROS_DEBUG ( "Odometry is stamp:%f x:%f y:%f z:%f roll:%f pitch:%f yaw:%f: odom:%s base:%s" ,
//                            t.toSec(),
//                            odomPose.getOrigin().getX(),
//                            odomPose.getOrigin().getY(),
//                            odomPose.getOrigin().getZ(),
//                            tmp_roll,
//                            tmp_pitch,
//                            tmp_yaw,
//                            m_motionModel->m_odomFrameId.c_str(),
//                            m_motionModel->m_baseFrameId.c_str()
//                         );

//          for ( int i = 0; i < m_particles.size(); i ++ )
//          {
//              double roll, pitch, yaw;
//              m_particles[i].pose.getBasis().getRPY ( roll, pitch, yaw );
//              double height;
//              height = m_particles[i].pose.getOrigin().getZ();
//              if ( i % 50 == 0 )
//                   ROS_DEBUG ( "Particle Pose z=%f R=%f P=%f", height, roll, pitch );
//          }

      // transformation from base frame to sensor:
      //this takes the latest tf, assumes that base to sensor did not change over temp. sampling!
      tf::StampedTransform localSensorFrame;
      if ( !m_motionModel->lookupLocalTransform ( pc_filtered.header.frame_id, t, localSensorFrame ) )
      {
         return false;
      }

      tf::Transform baseToSensor ( localSensorFrame.inverse() );

      //### Particles in log-form from here...
      toLogForm();

      // skip pose integration if z, roll and pitch constrained to floor by odometry
      if ( ! ( m_constrainMotionRP && m_constrainMotionZ ) )
      {
         ROS_INFO ( "Integrate z, roll and pitch using map and IMU." );
         bool imuMsgOk = false;
         double angleX, angleY;
         if ( m_useIMU )
         {
            ros::Time imuStamp;
            imuMsgOk = getImuMsg ( t, imuStamp, angleX, angleY );
         }
         else
         {
            tf::Stamped<tf::Pose> lastOdomPose;
            if ( m_motionModel->lookupOdomPose ( t, lastOdomPose ) )
            {
               double dropyaw;
               lastOdomPose.getBasis().getRPY ( angleX, angleY, dropyaw );
               imuMsgOk = true;
            }
         }

         tf::StampedTransform odomToBase;
         // integrated pose (z, roll, pitch) meas. only if data OK:
         if ( imuMsgOk )
         {
            if ( !m_motionModel->lookupLocalTransform ( m_odomFrameId, t, odomToBase ) )
            {
               ROS_WARN ( "Could not obtain pose height in localization, skipping Pose integration" );
            }
            else
            {
//           m_observationModel->integratePoseMeasurement ( m_particles, angleX, angleY, odomToBase );
               m_observationModel->integratePoseMeasurement ( m_particles, angleX, angleY, m_heightEstGround );
            }
         }
         else
         {
            ROS_WARN ( "Could not obtain roll and pitch measurement, skipping Pose integration" );
         }
      }

      ros::WallTime t0 = ros::WallTime::now();
      m_filteredPointCloudPub.publish ( pc_filtered );
      m_observationModel->integrateMeasurement ( m_particles, pc_filtered, ranges, max_range, baseToSensor,
            m_bestParticleIdx );
      ROS_INFO_STREAM ( "Observation update took: " << ( ros::WallTime::now() - t0 ).toSec() << "s" );

      ros::WallTime t1 = ros::WallTime::now();
      m_mapModel->verifyPoses ( m_particles );

      // normalize weights and transform back from log:
      normalizeWeights();
      //### Particles back in regular form now
      ROS_INFO_STREAM ( "Normalization took: " << ( ros::WallTime::now() - t1 ).toSec() << "s" );
//       for ( int i = 0; i < m_particles.size(); i = i + 10 )
//       {
//          fprintf ( fileVar, "%f %f %f %f %f %f %f %f %f %f %f\n", t.toSec(),
//                    m_particles[i].weight,
//                    m_particles[i+1].weight,
//                    m_particles[i+2].weight,
//                    m_particles[i+3].weight,
//                    m_particles[i+4].weight,
//                    m_particles[i+5].weight,
//                    m_particles[i+6].weight,
//                    m_particles[i+7].weight,
//                    m_particles[i+8].weight,
//                    m_particles[i+9].weight );
//       }
//       fprintf ( fileVar, "%s\n", " " );


      double nEffParticles = nEff();

      ros::WallTime t2 = ros::WallTime::now();
      if ( nEffParticles <= m_nEffFactor * m_particles.size() )
      {
         // selective resampling
         ROS_INFO ( "Resampling, nEff=%f, numParticles=%zd", nEffParticles, m_particles.size() );
         resample();
//          random_resample ( 400 );
//          augmented_resample();
      }
      else
      {
         ROS_INFO ( "Skipped resampling, nEff=%f, numParticles=%zd", nEffParticles, m_particles.size() );
      }
      ROS_INFO_STREAM ( "Resampling took: " << ( ros::WallTime::now() - t2 ).toSec() << "s" );

      m_receivedSensorData = true;

      double dt = ( ros::WallTime::now() - startTime ).toSec();
      ROS_INFO_STREAM (
         "Observations for " << m_numParticles << " particles took " << dt << "s (=" << dt / m_numParticles << "s/particle)" );

      return true;
   }

   void MAVLocalization::preparePointCloud ( const sensor_msgs::PointCloud2::ConstPtr& msg, PointCloud& pc,
         std::vector<float>& ranges )
   {
      ros::WallTime startTime = ros::WallTime::now();
      pc.clear();

      // pass-through filter to get rid of near and far ranges
      pcl::PassThrough<pcl::PointXYZ> pass;
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_tmp ( new pcl::PointCloud<pcl::PointXYZ>() );
      pcl::PCLPointCloud2 pcd2_tmp;
      pcl_conversions::toPCL ( *msg, pcd2_tmp );
      pcl::fromPCLPointCloud2 ( pcd2_tmp, *pcd_tmp );

      // cloud is in camera coordinate: z-axis forward
      pass.setInputCloud ( pcd_tmp );
      pass.setFilterFieldName ( "z" );
      pass.setFilterLimits ( m_filterMinRange, m_filterMaxRange );
      pass.filter ( pc );

      // lookup Transfrom Sensor to BaseFootprint
      tf::StampedTransform sensorToMap;
      tf::StampedTransform sensorToBase;
      try
      {
         m_tfListener.waitForTransform ( m_baseFrameId, msg->header.frame_id, msg->header.stamp, ros::Duration ( 0.5 ) );
         m_tfListener.lookupTransform ( m_baseFrameId, msg->header.frame_id, msg->header.stamp, sensorToBase );
      }
      catch ( tf::TransformException& ex )
      {
         ROS_ERROR_STREAM ( "Transform error for pointCloudCallback: " << ex.what() << ", quitting callback.\n" );
         return;
      }

      PointCloud ground, nonground;
      if ( m_detectGroundPlane )
      {

//         double height = sensorToMap.getOrigin().getZ();
         double height = m_lastLocalizedPose.getOrigin().getZ();
//       std::cout  << "heigt is: " << height << std::endl;

         Eigen::Matrix4f matSensorToBase, matBaseToSensor;
         pcl_ros::transformAsMatrix ( sensorToBase, matSensorToBase );
         pcl_ros::transformAsMatrix ( sensorToBase.inverse(), matBaseToSensor );

         //cloud is in base_frame coordinate: z-axis is downward, x-axis is forward
         pcl::transformPointCloud ( pc, pc, matSensorToBase );

         detectGroundPlane ( pc, ground, nonground, m_distanceThreshold, m_angleThreshold,
                             m_groundFilterPlaneDistance, height );

         sensor_msgs::PointCloud2 groundMsg;
         pcl::toROSMsg ( ground, groundMsg );
         groundMsg.header.frame_id = m_baseFrameId;
         m_groundCloudPub.publish ( groundMsg );

         // clear pc again and refill it based on classification
         pc.clear();
         pcl::PointCloud<int> sampledIndices;

         int numFloorPoints = 0;
         if ( ground.size() > 0 )
         {
            // check for 0 size, otherwise PCL crashes
            // transform clouds back to sensor for integration
            pcl::transformPointCloud ( ground, ground, matBaseToSensor );
            voxelGridSampling ( ground, sampledIndices, m_sensorSampleDist * m_sensorSampleDistGroundFactor );
            pcl::copyPointCloud ( ground, sampledIndices.points, pc );
            numFloorPoints = sampledIndices.size();
         }

         int numNonFloorPoints = 0;
         if ( nonground.size() > 0 )
         {
            // check for 0 size, otherwise PCL crashes
            // transform clouds back to sensor for integration
            pcl::transformPointCloud ( nonground, nonground, matBaseToSensor );
//         voxelGridSampling ( nonground, sampledIndices, m_sensorSampleDist );

            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal ( new pcl::PointCloud<pcl::PointNormal> );
            pcl::PointCloud<pcl::PointNormal>::Ptr select_nonground ( new pcl::PointCloud<pcl::PointNormal> );
            pcl::PointCloud<pcl::PointNormal>::Ptr cloudPtr ( new pcl::PointCloud<pcl::PointNormal> );
            pcl::copyPointCloud ( nonground, *cloudPtr );

            //Step2: Calculate the Normals of the point cloud using KD-tree
            pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
            norm_est.setSearchMethod ( pcl::search::KdTree<pcl::PointNormal>::Ptr
                                       ( new pcl::search::KdTree<pcl::PointNormal> ) );
            norm_est.setKSearch ( 5 );
            norm_est.setInputCloud ( cloudPtr );
            norm_est.compute ( *cloud_normal );

            //Step3: Select points with most constraints using Normal Space Sampling method
            pcl::NormalSpaceSampling<pcl::PointNormal, pcl::PointNormal> norm_sample;
            norm_sample.setInputCloud ( cloudPtr );
            norm_sample.setNormals ( cloud_normal );
            norm_sample.setBins ( 10, 10, 10 );
            if ( cloudPtr->size() > m_select_number )
            {
               norm_sample.setSample ( m_select_number );
            }
            else
            {
               norm_sample.setSample ( 0.6 * cloudPtr->size() );
            }
            norm_sample.setSeed ( rand() );
            norm_sample.filter ( *select_nonground );

//         pcl::copyPointCloud ( nonground, sampledIndices.points, nonground );
//         numNonFloorPoints = sampledIndices.size();
            pcl::copyPointCloud ( *select_nonground, nonground );
            numNonFloorPoints = select_nonground->size();

            pc += nonground;
         }

         ROS_INFO (
            "PointCloudGroundFiltering done. Added %d non-ground points and %d ground points (from %zu). Cloud size is %zu",
            numNonFloorPoints, numFloorPoints, ground.size(), pc.size() );
         // create sparse ranges..
         ranges.resize ( pc.size() );
         for ( unsigned int i = 0; i < pc.size(); ++i )
         {
            pcl::PointXYZ p = pc.at ( i );
            ranges[i] = sqrt ( p.x * p.x + p.y * p.y + p.z * p.z );
         }

      }
      else
      {
         ROS_INFO ( "Starting uniform sampling" );
         pcl::PointCloud<int> sampledIndices;
         voxelGridSampling ( pc, sampledIndices, m_sensorSampleDist );
         pcl::copyPointCloud ( pc, sampledIndices.points, pc );

         ranges.resize ( sampledIndices.size() );
         for ( size_t i = 0; i < ranges.size(); ++i )
         {
            pcl::PointXYZ p = pc[i];
            ranges[i] = sqrt ( p.x * p.x + p.y * p.y + p.z * p.z );
         }
         ROS_DEBUG_STREAM ( "Point cloud size after downsampling is: " << sampledIndices.size() );
      }


      double dwalltime = ( ros::WallTime::now() - startTime ).toSec();
      ROS_DEBUG_STREAM ( "Prepare point cloud took: " << dwalltime << "s " );

      return;
   }

   void MAVLocalization::detectGroundPlane ( const PointCloud& pc, PointCloud& ground, PointCloud& nonground,
         double distanceThreshold, double angleThreshold, double groundFilterPlaneDistance, double groundHeight )
   {
      ros::WallTime t0 = ros::WallTime::now();

      ground.header = pc.header;
      nonground.header = pc.header;

      if ( pc.size() < 50 )
      {
         ROS_WARN ( "Pointcloud is too small, skipping ground plane extraction" );
         nonground = pc;
      }
      else
      {
         // plane detection for ground plane removal:
//          pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients );
//          pcl::PointIndices::Ptr inliers ( new pcl::PointIndices );

         pcl::ModelCoefficients coefficients;
         pcl::PointIndices  inliers;

         // Create the segmentation object and set up:
         pcl::SACSegmentation<pcl::PointXYZ> seg;
         seg.setOptimizeCoefficients ( true );

         seg.setModelType ( pcl::SACMODEL_PERPENDICULAR_PLANE );
         seg.setMethodType ( pcl::SAC_RANSAC );
         seg.setMaxIterations ( 200 );
         seg.setDistanceThreshold ( distanceThreshold );
         seg.setAxis ( Eigen::Vector3f ( 0, 0, 1 ) );
         seg.setEpsAngle ( angleThreshold );


         PointCloud cloud_filtered ( pc );

         // Create the filtering object
         pcl::ExtractIndices<pcl::PointXYZ> extract;
         bool groundPlaneFound = false;

         int i = 0;
         while ( i < 1 && !groundPlaneFound )
         {
            seg.setInputCloud ( cloud_filtered.makeShared() );
            seg.segment ( inliers, coefficients );
            if ( inliers.indices.size() == 0 )
            {
               ROS_WARN ( "PCL segmentation did not find any plane." );
               break;
            }

            pcl::PointIndices::Ptr ins ( new pcl::PointIndices() );
            ins->header = inliers.header;
            ins->indices = inliers.indices;

            extract.setInputCloud ( cloud_filtered.makeShared() );
            extract.setIndices ( ins );

            m_heightEstGround = coefficients.values.at ( 3 );
            ROS_DEBUG_STREAM ( "Height estimated from ground plane is: " << m_heightEstGround );
// 	    std::cout << "d is: " << coefficients->values.at ( 3 ) << std::endl;
//
//             if ( std::abs ( coefficients->values.at ( 3 ) - groundHeight) < groundFilterPlaneDistance )
//             {
//             ROS_INFO ( "Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(),
//                        cloud_filtered.size(), coefficients->values.at ( 0 ), coefficients->values.at ( 1 ),
//                        coefficients->values.at ( 2 ), coefficients->values.at ( 3 ) );
            extract.setNegative ( false );
            extract.filter ( ground );

            // remove ground points from full pointcloud:
            // workaround for PCL bug:
            if ( inliers.indices.size() != cloud_filtered.size() )
            {
               extract.setNegative ( true );
               PointCloud cloud_out;
               extract.filter ( cloud_out );
               nonground += cloud_out;
               cloud_filtered = cloud_out;
            }

            groundPlaneFound = true;
//             }
//             else
//             {
//                 ROS_INFO ( "Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f",
//                             inliers->indices.size(), cloud_filtered.size(), coefficients->values.at ( 0 ),
//                             coefficients->values.at ( 1 ), coefficients->values.at ( 2 ), coefficients->values.at ( 3 ) );
//                 pcl::PointCloud<pcl::PointXYZ> cloud_out;
//                 extract.setNegative ( false );
//                 extract.filter ( cloud_out );
//                 nonground += cloud_out;
//
//                 // remove current plane from scan for next iteration:
//                 // workaround for PCL bug:
//                 if ( inliers->indices.size() != cloud_filtered.size() )
//                 {
//                     extract.setNegative ( true );
//                     cloud_out.points.clear();
//                     extract.filter ( cloud_out );
//                     cloud_filtered = cloud_out;
//                 }
//                 else
//                 {
//                     cloud_filtered.points.clear();
//                 }
//             }

            i++;
         }

         // TODO: also do this if overall starting pointcloud too small?
         if ( !groundPlaneFound )
         {
            // no plane found or remaining points too small
            ROS_WARN ( "No ground plane found in scan" );

            // do a rough fitlering on height to prevent spurious obstacles
            pcl::PassThrough<pcl::PointXYZ> second_pass;
            second_pass.setFilterFieldName ( "z" );
            second_pass.setFilterLimits ( -groundFilterPlaneDistance, groundFilterPlaneDistance );
            second_pass.setInputCloud ( pc.makeShared() );
            second_pass.filter ( ground );

            second_pass.setFilterLimitsNegative ( true );
            second_pass.filter ( nonground );
         }
      }

      ROS_DEBUG_STREAM ( "Ground plane detection took: " << ( ros::WallTime::now() - t0 ).toSec() << "s" );
   }

   void MAVLocalization::voxelGridSampling ( const PointCloud & pc, pcl::PointCloud<int> & sampledIndices,
         double search_radius ) const
   {
      pcl::UniformSampling<pcl::PointXYZ> uniformSampling;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
      cloudPtr.reset ( new pcl::PointCloud<pcl::PointXYZ> ( pc ) );
      uniformSampling.setInputCloud ( cloudPtr );
      uniformSampling.setRadiusSearch ( search_radius );
      uniformSampling.compute ( sampledIndices );
   }

   void MAVLocalization::pointCloudCallback ( const sensor_msgs::PointCloud2::ConstPtr& msg, const ca_rangeflow::vo_state::ConstPtr& odom )
   {
      ROS_DEBUG ( "PointCloud received (time: %f)", msg->header.stamp.toSec() );
      ros::WallTime startTime = ros::WallTime::now();
      ros::WallTime midTime;
      double dwalltime;

      if ( !m_initialized )
      {
         ROS_WARN ( "Loclization not initialized yet, skipping PointCloud callback." );
         return;
      }

      double timediff = ( msg->header.stamp - m_lastPointCloudTime ).toSec();
      if ( m_receivedSensorData && timediff < 0 )
      {
         ROS_WARN ( "Ignoring received PointCloud data that is %f s older than previous data!", timediff );
         return;
      }

      /// absolute, current odom pose
      tf::Stamped<tf::Pose> odomPose;
      // check if odometry available, skip scan if not.
      if ( !m_motionModel->lookupOdomPose ( msg->header.stamp, odomPose ) )
      {
         ROS_WARN ( "Skip current scan since no corresponding odometry transform available!" );
         return;
      }


      bool sensor_integrated = false;

      // update particles by using observation model after traveling for a certain distance
      if ( checkAboveMotionThreshold ( odomPose ) && ( !m_motionTest ) )
      {
         // downsample the input point cloud
         PointCloud pc_filtered;
         std::vector<float> rangesSparse;
         
         preparePointCloud ( msg, pc_filtered, rangesSparse );

         sensor_msgs::PointCloud2 finaCloudMsg;
         pcl::toROSMsg ( pc_filtered, finaCloudMsg );
         finaCloudMsg.header.frame_id = msg->header.frame_id;
         m_finalcloudPub.publish ( finaCloudMsg );
	 
	 if ( pc_filtered.size() < minimum_update_threshold)
         {
            ROS_WARN_STREAM ( "Input point cloud is too small, ignore the update!" );
	    publishPoseEstimate ( msg->header.stamp, sensor_integrated );
            return;
         }

         double maxRange = m_filterMaxRange;
         ROS_INFO ( "Updating Pose Estimate from a PointCloud with %zu points and %zu ranges", pc_filtered.size(),
                    rangesSparse.size() );
         sensor_integrated = localizeWithMeasurement ( pc_filtered, rangesSparse, maxRange );

         midTime = ros::WallTime::now();
         dwalltime = ( midTime - startTime ).toSec();
         totalTime += dwalltime;
         FramNum ++;
         ROS_WARN_STREAM ( "Mean update time is: " << totalTime / FramNum << "s " );
      }


      // Propagate particles forward by using motion model
      if ( !sensor_integrated )
      {
         tf::Transform odomTransform;
         odomTransform = m_motionModel->computeOdomTransform ( odomPose );
         double roll, pitch, yaw;
         odomTransform.getBasis().getRPY ( roll, pitch, yaw );

         ROS_DEBUG ( "Odometry z %f, r %f, p %f\n", odomTransform.getOrigin().getX(), roll, pitch );
         ROS_DEBUG_STREAM ( "Odometry Translation is: " << odomTransform.getOrigin().length() );
         ROS_DEBUG_STREAM ( "Odometry Yaw is: " << yaw );

         //if ( odomTransform.getOrigin().length() < 0.05 || std::abs ( yaw ) < 0.03 )
         if ( odom->conditionNumber < vo_threshold )
         {
            ROS_INFO ( "Predicting Pose using motion model." );
            m_motionModel->applyOdomTransform ( m_particles, odomTransform );
            constrainMotion ( odomPose );
            lastrelativeTrans = odomTransform;
         }
         else
         {
            ROS_WARN ( "Predicting pose using random noise since visual odometry fails!" );
            m_motionModel->addRandomNoise ( m_particles );
         }
      }
      else
      {
         m_lastLocalizedPose = odomPose;
      }

      m_motionModel->storeOdomPose ( odomPose );
      publishPoseEstimate ( msg->header.stamp, sensor_integrated );
      m_lastPointCloudTime = msg->header.stamp;
      ROS_INFO_STREAM ( "PointCloud callback complete." << std::endl );
   }

   void MAVLocalization::initPoseCallback ( const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg )
   {
      tf::Pose pose;
      tf::poseMsgToTF ( msg->pose.pose, pose );

      if ( msg->header.frame_id != m_globalFrameId )
      {
         ROS_WARN ( "Frame ID of \"initialpose\" (%s) is different from the global frame %s", msg->header.frame_id.c_str(),
                    m_globalFrameId.c_str() );
      }

      std::vector<double> heights;
      double poseHeight = m_initPose ( 2 );
      if ( std::abs ( pose.getOrigin().getZ() ) < 0.01 )
      {
         m_mapModel->getHeightlist ( pose.getOrigin().getX(), pose.getOrigin().getY(), 0.6, heights );
         if ( heights.size() == 0 )
         {
            ROS_WARN ( "No ground found at map position, assuming 0" );
            heights.push_back ( 0.0 );
         }
      }

      //Here, 0.25 and M_PI/12 are the default covariance of initialpose message of RVIZ
      //We replace them with our default parameters
      Matrix6d initCov;
      if ( ( std::abs ( msg->pose.covariance.at ( 6 * 0 + 0 ) - 0.25 ) < 0.1 )
            && ( std::abs ( msg->pose.covariance.at ( 6 * 1 + 1 ) - 0.25 ) < 0.1 )
            && ( std::abs ( msg->pose.covariance.at ( 6 * 5 + 5 ) - M_PI/12 * M_PI/12 ) < 0.1 ) )
      {
         ROS_INFO ( "Covariance originates from RViz, using default parameters instead" );
         initCov = Matrix6d::Zero();
         initCov.diagonal() = m_initNoiseStd.cwiseProduct ( m_initNoiseStd );

         // manually set r&p, rviz values are 0
         const double yaw = tf::getYaw ( pose.getRotation() );
         ROS_INFO ( "roll and pitch not set in initPoseCallback, use init_pose_{roll,pitch} parameters instead" );
         pose.setRotation ( tf::createQuaternionFromRPY ( m_initPose ( 3 ), m_initPose ( 4 ), yaw ) );
      }
      else
      {
         ROS_INFO ( "Covariance originates from configuration file" );
         for ( int j = 0; j < initCov.cols(); ++j )
         {
            for ( int i = 0; i < initCov.rows(); ++i )
            {
               initCov ( i, j ) = msg->pose.covariance.at ( i * initCov.cols() + j );
            }
         }
      }
      ROS_DEBUG_STREAM ( "Pose Covariance intialization: " << std::endl <<initCov );


      // sample from initial pose covariance:
      Matrix6d initCovL = initCov.llt().matrixL();
      tf::Transform transformNoise; // transformation on original pose from noise
      unsigned idx = 0;
      for ( Particles::iterator it = m_particles.begin(); it != m_particles.end(); ++it )
      {
         Vector6d poseNoise;
         for ( unsigned i = 0; i < 6; ++i )
         {
            poseNoise ( i ) = m_rngNormal();
         }
         Vector6d poseCovNoise = initCovL * poseNoise; // is now drawn according to covariance noise
         // if a variance is set to 0 => no noise!
         for ( unsigned i = 0; i < 6; ++i )
         {
            if ( std::abs ( initCov ( i, i ) ) < 0.00001 )
            {
               poseCovNoise ( i ) = 0.0;
            }
         }

         transformNoise.setOrigin ( tf::Vector3 ( poseCovNoise ( 0 ), poseCovNoise ( 1 ), poseCovNoise ( 2 ) ) );
         tf::Quaternion q;
         q.setRPY ( poseCovNoise ( 3 ), poseCovNoise ( 4 ), poseCovNoise ( 5 ) );
         transformNoise.setRotation ( q );

         //ROS_DEBUG_STREAM ("Initial pose noise: " << std::endl << poseCovNoise);

         it->pose = pose;
         it->pose.getOrigin().setZ ( poseHeight );
         it->pose *= transformNoise;

         it->weight = 1.0 / m_particles.size();
         idx++;
      }

      ROS_INFO ( "Pose reset around mean (%f %f %f)",
                 pose.getOrigin().getX(),
                 pose.getOrigin().getY(),
                 pose.getOrigin().getZ() );

      // reset internal state:
      m_motionModel->reset();
      m_receivedSensorData = false;
      m_initialized = true;

      m_lastLocalizedPose = pose;
      publishPoseEstimate ( msg->header.stamp, false );
   }

   void MAVLocalization::imuCallback ( const sensor_msgs::ImuConstPtr& msg )
   {


//       current_imu_ts = msg->header.stamp;
//       current_acc_x  = msg->linear_acceleration.x;
//       current_acc_y  = msg->linear_acceleration.y;
//       current_acc_z  = msg->linear_acceleration.z;
//
//       if ( firstIMU )
//       {
//          last_imu_ts = current_imu_ts;
//          last_acc_x = current_acc_x  ;
//          last_acc_y = current_acc_y  ;
//          last_acc_z = current_acc_z  ;
//
//          firstIMU = false;
//          return;
//       }
//
//       double dt = ( current_imu_ts -last_imu_ts ).toSec();
//
//       m_imu_odom.header.stamp = msg->header.stamp;
//       m_imu_odom.header.frame_id = "/imu_init";
//       m_imu_odom.child_frame_id = "/imu_frame";
//
//       vel_x += current_acc_x * dt;
//       vel_y += current_acc_y * dt;
//       vel_z += ( current_acc_z - 9.81 ) * dt;
//
//       pos_x += vel_x * dt + 0.5 * current_acc_x * dt * dt;
//       pos_y += vel_y * dt + 0.5 * current_acc_y * dt * dt;
//       pos_z += vel_z * dt + 0.5 * current_acc_z * dt * dt;
//
//       m_imu_odom.pose.pose.position.x = pos_x;
//       m_imu_odom.pose.pose.position.y = pos_y;
//       m_imu_odom.pose.pose.position.z = pos_z;
//
//       m_imuOdomPub.publish ( m_imu_odom );

//       std::cout << "deltaT" << ( current_imu_ts -last_imu_ts ).toSec() << std::endl;


      m_lastIMUMsgBuffer.push_back ( *msg );

//       last_imu_ts = current_imu_ts;
//       last_acc_x = current_acc_x  ;
//       last_acc_y = current_acc_y  ;
//       last_acc_z = current_acc_z  ;
   }

   bool MAVLocalization::getImuMsg ( const ros::Time& stamp, ros::Time& imuStamp, double& angleX, double& angleY ) const
   {
      if ( m_lastIMUMsgBuffer.empty() )
      {
         return false;
      }

      typedef boost::circular_buffer<sensor_msgs::Imu>::const_iterator ItT;
      const double maxAge = 0.2;
      double closestOlderStamp = std::numeric_limits<double>::max();
      double closestNewerStamp = std::numeric_limits<double>::max();
      ItT closestOlder = m_lastIMUMsgBuffer.end(), closestNewer = m_lastIMUMsgBuffer.end();
      for ( ItT it = m_lastIMUMsgBuffer.begin(); it != m_lastIMUMsgBuffer.end(); it++ )
      {
         const double age = ( stamp - it->header.stamp ).toSec();
         if ( age >= 0.0 && age < closestOlderStamp )
         {
            closestOlderStamp = age;
            closestOlder = it;
         }
         else if ( age < 0.0 && -age < closestNewerStamp )
         {
            closestNewerStamp = -age;
            closestNewer = it;
         }
      }

      if ( closestOlderStamp < maxAge && closestNewerStamp < maxAge && closestOlderStamp + closestNewerStamp > 0.0 )
      {
         // Linear interpolation
         const double weightOlder = closestNewerStamp / ( closestNewerStamp + closestOlderStamp );
         const double weightNewer = 1.0 - weightOlder;
         imuStamp = ros::Time (
                       weightOlder * closestOlder->header.stamp.toSec() + weightNewer * closestNewer->header.stamp.toSec() );
         double olderX, olderY, newerX, newerY;
         getRP ( closestOlder->orientation, olderX, olderY );
         getRP ( closestNewer->orientation, newerX, newerY );
         angleX = weightOlder * olderX + weightNewer * newerX;
         angleY = weightOlder * olderY + weightNewer * newerY;
         ROS_DEBUG ( "Msg: %.3f, Interpolate [%.3f .. %.3f .. %.3f]\n", stamp.toSec(), closestOlder->header.stamp.toSec(),
                     imuStamp.toSec(), closestNewer->header.stamp.toSec() );
         return true;
      }
      else if ( closestOlderStamp < maxAge || closestNewerStamp < maxAge )
      {
         // Return closer one
         ItT it = ( closestOlderStamp < closestNewerStamp ) ? closestOlder : closestNewer;
         imuStamp = it->header.stamp;
         getRP ( it->orientation, angleX, angleY );
         return true;
      }
      else
      {
         if ( closestOlderStamp < closestNewerStamp )
         {
            ROS_WARN ( "Closest IMU message is %.2f seconds too old, skipping pose integration", closestOlderStamp );
         }
         else
         {
            ROS_WARN ( "Closest IMU message is %.2f seconds too new, skipping pose integration", closestNewerStamp );
         }
         return false;
      }
   }

   bool MAVLocalization::globalLocalizationCallback ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
   {

      initGlobal();

      return true;
   }

   void MAVLocalization::normalizeWeights()
   {

      double wmin = std::numeric_limits<double>::max();
      double wmax = -std::numeric_limits<double>::max();

      for ( unsigned i = 0; i < m_particles.size(); ++i )
      {
         double weight = m_particles[i].weight;
         assert ( !isnan ( weight ) );
         if ( weight < wmin )
         {
            wmin = weight;
         }
         if ( weight > wmax )
         {
            wmax = weight;
            m_bestParticleIdx = i;
         }
      }
      if ( wmin > wmax )
      {
         ROS_ERROR_STREAM (
            "Error in weights: min=" << wmin << ", max=" << wmax << ", 1st particle weight=" << m_particles[1].weight << std::endl );

      }

      double min_normalized_value;
      if ( m_minParticleWeight > 0.0 )
      {
         min_normalized_value = std::max ( log ( m_minParticleWeight ), wmin - wmax );
      }
      else
      {
         min_normalized_value = wmin - wmax;
      }

      double max_normalized_value = 0.0; // = log(1.0);
      double dn = max_normalized_value - min_normalized_value;
      double dw = wmax - wmin;
      if ( dw == 0.0 )
      {
         dw = 1;
      }
      double scale = dn / dw;
      if ( scale < 0.0 )
      {
         ROS_WARN ( "normalizeWeights: scale is %f < 0, dw=%f, dn=%f", scale, dw, dn );
      }
      double offset = -wmax * scale;
      double weights_sum = 0.0;

      //////#pragma omp parallel
      {
         ////#pragma omp for
         for ( unsigned i = 0; i < m_particles.size(); ++i )
         {
            double w = m_particles[i].weight;
            w = exp ( scale * w + offset );
            assert ( !isnan ( w ) );
            m_particles[i].weight = w;
            ////#pragma omp atomic

            weights_sum += w;
         }

         assert ( weights_sum > 0.0 );
         // normalize sum to 1:

         ////#pragma omp for
         for ( unsigned i = 0; i < m_particles.size(); ++i )
         {
            m_particles[i].weight /= weights_sum;
         }

      }
   }

   double MAVLocalization::getCumParticleWeight() const
   {
      double cumWeight = 0.0;

      //compute the cumulative weights
      for ( Particles::const_iterator it = m_particles.begin(); it != m_particles.end(); ++it )
      {
         cumWeight += it->weight;
      }

      return cumWeight;
   }

   void MAVLocalization::resample ( unsigned numParticles )
   {

      if ( numParticles <= 0 )
      {
         numParticles = m_numParticles;
      }

      //compute the interval
      double interval = getCumParticleWeight() / numParticles;

      //compute the initial target weight
      double target = interval * m_rngUniform();

      //compute the resampled indexes
      double cumWeight = 0;
      std::vector<unsigned> indices ( numParticles );

      unsigned n = 0;
      for ( unsigned i = 0; i < m_particles.size(); ++i )
      {
         cumWeight += m_particles[i].weight;
         while ( cumWeight > target && n < numParticles )
         {
            if ( m_bestParticleIdx >= 0 && i == unsigned ( m_bestParticleIdx ) )
            {
               m_bestParticleIdx = n;
            }

            indices[n++] = i;
            target += interval;
         }
      }
      // indices now contains the indices to draw from the particles distribution

      Particles oldParticles = m_particles;
      m_particles.resize ( numParticles );
      m_poseArray.poses.resize ( numParticles );
      double newWeight = 1.0 / numParticles;

      ////#pragma omp parallel for
      for ( unsigned i = 0; i < numParticles; ++i )
      {
         m_particles[i].pose = oldParticles[indices[i]].pose;
         m_particles[i].weight = newWeight;
      }
   }

   void MAVLocalization::random_resample ( unsigned numParticles )
   {

      if ( numParticles <= 0 )
      {
         numParticles = m_numParticles;
      }

      //compute the interval
      double interval = getCumParticleWeight() / numParticles;

      //compute the initial target weight
      double target = interval * m_rngUniform();

      //compute the resampled indexes
      double cumWeight = 0;
      std::vector<unsigned> indices ( numParticles );

      unsigned n = 0;
      for ( unsigned i = 0; i < m_particles.size(); ++i )
      {
         cumWeight += m_particles[i].weight;
         while ( cumWeight > target && n < numParticles )
         {
            if ( m_bestParticleIdx >= 0 && i == unsigned ( m_bestParticleIdx ) )
            {
               m_bestParticleIdx = n;
            }

            indices[n++] = i;
            target += interval;
         }
      }
      // indices now contains the indices to draw from the particles distribution

      Particles oldParticles = m_particles;
      double newWeight = 1.0 / m_particles.size();

      ////#pragma omp parallel for
      for ( unsigned i = 0; i < m_particles.size(); ++i )
      {
         if ( i < numParticles )
         {
            m_particles[i].pose = oldParticles[indices[i]].pose;
            m_particles[i].weight = newWeight;
         }
         else
         {
            /// Generate uniform number between 0 and 1
            typedef boost::mt19937                 	EngineT;
            typedef boost::uniform_real<>     		UniformDistributionT;
            typedef boost::variate_generator<EngineT&, UniformDistributionT>   UniformGeneratorT;
            unsigned seed = static_cast<unsigned int> ( std::time ( 0 ) );

            EngineT m_rngEngine ( seed );
            UniformGeneratorT m_rngUniform ( m_rngEngine, UniformDistributionT ( 0.0, 0.2 ) );

            tf::Pose tmpPose = prevPose;
            double x = prevPose.getOrigin().x() + m_rngUniform();
            double y = prevPose.getOrigin().y() + m_rngUniform();
            double z = prevPose.getOrigin().z();
            tf::Vector3 t ( x,y,z );
            tmpPose.setOrigin ( t );

            m_particles[i].pose = tmpPose;
            m_particles[i].weight = newWeight;
         }
      }

      m_poseArray.poses.resize ( m_particles.size() );
   }

   void MAVLocalization::augmented_resample ( unsigned numParticles )
   {

      if ( numParticles <= 0 )
      {
         numParticles = m_numParticles;
      }

      a_long = 0.2;
      a_short = 0.01;
      double v = 2.0;
      for ( unsigned i = 0; i < m_particles.size(); ++i )
      {
         w_avg = w_avg + m_particles[i].weight / m_particles.size() ;
      }
      w_long  = w_long  + a_long  * ( w_avg - w_long );
      w_short = w_short + a_short * ( w_avg - w_short );


      //compute the interval
      double interval = getCumParticleWeight() / numParticles;

      //compute the initial target weight
      double target = interval * m_rngUniform();

      //compute the resampled indexes
      double cumWeight = 0;
      std::vector<unsigned> indices ( numParticles );

      unsigned n = 0;
      for ( unsigned i = 0; i < m_particles.size(); ++i )
      {
         cumWeight += m_particles[i].weight;
         while ( cumWeight > target && n < numParticles )
         {
            if ( m_bestParticleIdx >= 0 && i == unsigned ( m_bestParticleIdx ) )
            {
               m_bestParticleIdx = n;
            }

            indices[n++] = i;
            target += interval;
         }
      }
      // indices now contains the indices to draw from the particles distribution

      Particles oldParticles = m_particles;
      double newWeight = 1.0 / numParticles;

      int rand_num = 0;
      srand ( ( unsigned ) time ( 0 ) );
      for ( unsigned i = 0; i < m_particles.size(); ++i )
      {
         if ( ( double ) rand() / RAND_MAX <= std::max ( 0.0, 1 - v * w_short / w_long ) )
         {
//             typedef boost::mt19937                     ENG;    // Mersenne Twister
//             typedef boost::normal_distribution<double> DIST;   // Normal Distribution
//             typedef boost::variate_generator<ENG,DIST> GEN;    // Variate generator
//
//             ENG  eng;
//             DIST dist ( 0, 0.3 );
//             GEN  gen ( eng,dist );


            /// Generate uniform number between 0 and 1
            typedef boost::mt19937                 	EngineT;
            typedef boost::uniform_real<>     		UniformDistributionT;
            typedef boost::variate_generator<EngineT&, UniformDistributionT>   UniformGeneratorT;
            unsigned seed = static_cast<unsigned int> ( std::time ( 0 ) );

            EngineT m_rngEngine ( seed );
            UniformGeneratorT m_rngUniform ( m_rngEngine, UniformDistributionT ( 0.0, 0.5 ) );

            tf::Pose tmpPose = prevPose;
            double x = prevPose.getOrigin().x() + m_rngUniform();
            double y = prevPose.getOrigin().y() + m_rngUniform();
            double z = prevPose.getOrigin().z();
            tf::Vector3 t ( x,y,z );
            tmpPose.setOrigin ( t );

            m_particles[i].pose = tmpPose;
            m_particles[i].weight = newWeight;

            rand_num ++;
         }
         else
         {
            m_particles[i].pose = oldParticles[indices[i]].pose;
            m_particles[i].weight = newWeight;
         }
      }
      ROS_WARN ( "w_long: %f, w_short: %f", w_long, w_short );
      ROS_WARN_STREAM ( "Add random pose number is: " << rand_num );

      m_poseArray.poses.resize ( numParticles );
   }

   void MAVLocalization::initGlobal()
   {
      ROS_INFO ( "Initializing with uniform distribution" );

      double roll, pitch, z;
      z = m_initPose ( 2 );
      roll = m_initPose ( 3 );
      pitch = m_initPose ( 4 );

      m_mapModel->initGlobal ( m_particles, z, roll, pitch, m_initNoiseStd, m_rngUniform, m_rngNormal );

      ROS_INFO ( "Global localization done" );
      m_motionModel->reset();
      m_receivedSensorData = false;
      m_initialized = true;

      publishPoseEstimate ( ros::Time::now(), false );

   }

   void MAVLocalization::publishPoseEstimate ( const ros::Time& time, bool publish_eval )
   {
      // send all hypotheses as arrows:
      m_poseArray.header.stamp = time;
      if ( m_poseArray.poses.size() != m_particles.size() )
      {
         m_poseArray.poses.resize ( m_particles.size() );
      }
      ////#pragma omp parallel for
      for ( unsigned i = 0; i < m_particles.size(); ++i )
      {
         tf::poseTFToMsg ( m_particles[i].pose, m_poseArray.poses[i] );
      }
      m_poseArrayPub.publish ( m_poseArray );


      // send best particle as pose and one array:
//     geometry_msgs::PoseStamped p;
//     p.header.stamp = time;
//     p.header.frame_id = m_globalFrameId;
//     tf::Pose bestParticlePose;
//     if ( m_bestParticleAsMean )
//     {
//       bestParticlePose = getMeanParticlePose();
//     }
//     else
//     {
//       bestParticlePose = getBestParticlePose();
//     }
//
//     //tf::poseTFToMsg ( bestParticlePose, p.pose.pose );
//     tf::poseTFToMsg ( bestParticlePose, p.pose );
//     m_posePub.publish ( p );

      nav_msgs::Odometry p;
      p.header.stamp = time;
      p.header.frame_id = m_globalFrameId;
      tf::Pose bestParticlePose;
      if ( m_bestParticleAsMean )
      {
         bestParticlePose = getMeanParticlePose();
      }
      else
      {
         bestParticlePose = getBestParticlePose();
      }

      tf::poseTFToMsg ( bestParticlePose, p.pose.pose );
      m_posePub.publish ( p );

      //Store the last localization pose
      prevPose = bestParticlePose;

      geometry_msgs::PoseWithCovarianceStamped pcov;
      pcov.header.stamp = time;
      pcov.header.frame_id = m_globalFrameId;
      tf::poseTFToMsg ( bestParticlePose,pcov.pose.pose );

      bool compute_covariance=true;

      if ( compute_covariance )
      {
         Eigen::VectorXd	vars ( 6 );
         vars.setZero();
         Eigen::Matrix<double, 6, 6> cov;
         cov.setZero();

         double	std_xy = 0,std_xz = 0,std_xya= 0,std_xp = 0,std_xr = 0;
         double	std_yz = 0,std_yya = 0,std_yp = 0,std_yr = 0;
         double	std_zya = 0,std_zp = 0,std_zr = 0;
         double	std_yap = 0,std_yar = 0;
         double	std_pr = 0;

         // Mean values in [0, 2pi] range:
         double mean_roll, mean_pitch, mean_yaw;
         bestParticlePose.getBasis().getRPY ( mean_roll, mean_pitch, mean_yaw );

//       double M_2PI = 3.1415926;
//       if ( mean_yaw<0 ) mean_yaw += M_2PI;
//       if ( mean_pitch<0 ) mean_pitch += M_2PI;
//       if ( mean_roll<0 ) mean_roll += M_2PI;

         for ( unsigned i = 0; i < m_particles.size(); ++i )
         {
            double w = m_particles[i].weight;

            double x, y, z, roll, pitch, yaw;
            m_particles[i].pose.getBasis().getRPY ( roll, pitch, yaw );
            x = m_particles[i].pose.getOrigin().getX();
            y = m_particles[i].pose.getOrigin().getY();
            z = m_particles[i].pose.getOrigin().getZ();


            // Manage 1 PI range:
            double	err_yaw   =  yaw - mean_yaw ;
            if ( err_yaw>3.1415926 ) err_yaw -= 2*3.1415926;
            if ( err_yaw<-3.1415926 ) err_yaw += 2*3.1415926;


            double	err_pitch =  pitch - mean_pitch;
            double	err_roll  =  roll - mean_roll;


            double	err_x	  = x - bestParticlePose.getOrigin().getX();
            double	err_y	  = y - bestParticlePose.getOrigin().getY();
            double	err_z	  = z - bestParticlePose.getOrigin().getZ();

            vars ( 0 ) 	+=  err_x * err_x *w;
            vars ( 1 ) 	+=  err_y * err_y *w;
            vars ( 2 ) 	+=  err_z * err_z *w;
            vars ( 3 ) 	+=  err_roll * err_roll *w;
            vars ( 4 ) 	+=  err_pitch * err_pitch *w;
            vars ( 5 ) 	+=  err_yaw * err_yaw *w;

            std_xy		+= err_x*err_y * w;
            std_xz		+= err_x*err_z * w;
            std_xya	+= err_x*err_yaw * w;
            std_xp		+= err_x*err_pitch * w;
            std_xr		+= err_x*err_roll * w;

            std_yz		+= err_y*err_z * w;
            std_yya	+= err_y*err_yaw * w;
            std_yp		+= err_y*err_pitch * w;
            std_yr		+= err_y*err_roll * w;

            std_zya	+= err_z*err_yaw * w;
            std_zp		+= err_z*err_pitch * w;
            std_zr		+= err_z*err_roll * w;

            std_yap	+= err_yaw*err_pitch * w;
            std_yar	+= err_yaw*err_roll * w;

            std_pr 	+= err_pitch*err_roll * w;

         }

         // Unbiased estimation of variance:
         cov ( 0,0 ) = vars[0];
         cov ( 1,1 ) = vars[1];
         cov ( 2,2 ) = vars[2];
         cov ( 3,3 ) = vars[3];
         cov ( 4,4 ) = vars[4];
         cov ( 5,5 ) = vars[5];

         cov ( 1,0 ) = cov ( 0,1 ) = std_xy;
         cov ( 2,0 ) = cov ( 0,2 ) = std_xz;
         cov ( 3,0 ) = cov ( 0,3 ) = std_xr;
         cov ( 4,0 ) = cov ( 0,4 ) = std_xp;
         cov ( 5,0 ) = cov ( 0,5 ) = std_xya;

         cov ( 2,1 ) = cov ( 1,2 ) = std_yz;
         cov ( 3,1 ) = cov ( 1,3 ) = std_yr;
         cov ( 4,1 ) = cov ( 1,4 ) = std_yp;
         cov ( 5,1 ) = cov ( 1,5 ) = std_yya;

         cov ( 3,2 ) = cov ( 2,3 ) = std_zr;
         cov ( 4,2 ) = cov ( 2,4 ) = std_zp;
         cov ( 5,2 ) = cov ( 2,5 ) = std_zya;

         cov ( 4,3 ) = cov ( 3,4 ) = std_pr;
         cov ( 5,3 ) = cov ( 3,5 ) = std_yar;

         cov ( 5,4 ) = cov ( 4,5 ) = std_yap;

         for ( int i = 0; i < 6; i ++ )
            for ( int j = 0; j < 6; j ++ )
               pcov.pose.covariance[i * 6 + j] = cov ( i,j );

//       double entropy=0.5*6*(1+log(2*3.1415926)/log(2.718))+0.5*log(fabs(cov.determinant()))/log(2.718);
//       for (int k=0;k<6;k++) ROS_ERROR_STREAM("var at "<<k<<"  is  "<<vars[k]);
//       ROS_ERROR_STREAM(" covariance at time  "<<time<<"  \n"<<cov);
//       double pose_weight=(10-std::max(std::min(entropy+20,(double)10),(double)0))/10;   //normalize weight between 0-1
//       double entropy=vars[0]+vars[1]+vars[2]+vars[3]+vars[4]+vars[5];
//       ROS_ERROR_STREAM("mav_localization entropy is = "<<entropy);   //the higher, the worse
//       ROS_ERROR_STREAM("mav_localization pose_weight is = "<<pose_weight);   //the higher, the worse
//       p.twist.twist.angular.x = entropy;   //manually stored


         //covariance 1*36matrix,cheange it to cov 6*6 matrix, from row to row
         double entropy = 0.5*6* ( 1+log ( 2*3.1415926 ) /log ( 2.718 ) )
                          + 0.5*log ( fabs ( cov.determinant() ) ) /log ( 2.718 );

         //normalize weight between 0-1   raw data is around -20,-10
         double pose_belief= ( 10-std::max ( std::min ( entropy+20, ( double ) 10 ), ( double ) 0 ) ) /10;

         ROS_INFO_STREAM ( "Pose belief: " << pose_belief );
      }

      m_poseCovPub.publish ( pcov );

      // Send tf target->map (where target is typically odom)
      tf::Stamped<tf::Pose> targetToMapTF;
      try
      {
         tf::Stamped<tf::Pose> baseToMapTF ( bestParticlePose.inverse(), time, m_baseFrameId );
         m_tfListener.transformPose ( m_targetFrameId, baseToMapTF, targetToMapTF ); // typically target == odom
      }
      catch ( const tf::TransformException& e )
      {
         ROS_WARN ( "Failed to subtract base to %s transform, will not publish pose estimate: %s", m_targetFrameId.c_str(),
                    e.what() );
         return;
      }

      tf::Transform latestTF ( tf::Quaternion ( targetToMapTF.getRotation() ), tf::Point ( targetToMapTF.getOrigin() ) );
//
//     // We want to send a transform that is good up until a tolerance time so that odom can be used
//     ros::Duration transformTolerance ( m_transformTolerance );
//     ros::Time transformExpiration = ( time + transformTolerance );
//
      tf::StampedTransform tmp_tf_stamped ( latestTF.inverse(), time, m_globalFrameId, m_targetFrameId );
//       m_latest_transform = tmp_tf_stamped;
      m_tfBroadcaster.sendTransform ( tmp_tf_stamped );


      if ( m_logIntofile == true )
      {
         // Print the localization results into a Txt file.
         tf::StampedTransform mapTocamera_link;
         try
         {
            m_tfListener.waitForTransform ( m_globalFrameId, m_baseFrameId, time, ros::Duration ( 0.5 ) );
            m_tfListener.lookupTransform ( m_globalFrameId, m_baseFrameId, time, mapTocamera_link );
//       fprintf ( fileVar, "%f %f %f %f %f %f %f %f\n",
//                 time.toSec(),
//                 mapTocamera_link.getOrigin().getX(),
//                 mapTocamera_link.getOrigin().getY(),
//                 mapTocamera_link.getOrigin().getZ(),
//                 mapTocamera_link.getRotation().getX(),
//                 mapTocamera_link.getRotation().getY(),
//                 mapTocamera_link.getRotation().getZ(),
//                 mapTocamera_link.getRotation().getW() );
         }
         catch ( tf::TransformException& ex )
         {
            ROS_ERROR_STREAM ( "Can't get global transformation: " << ex.what() << " .\n" );
            return;
         }
      }
   }

   unsigned MAVLocalization::getBestParticleIdx() const
   {
      if ( m_bestParticleIdx < 0 || m_bestParticleIdx >= m_numParticles )
      {
         ROS_WARN ( "Index (%d) of best particle not valid, using 0 instead", m_bestParticleIdx );
         return 0;
      }

      return m_bestParticleIdx;
   }

   tf::Pose MAVLocalization::getParticlePose ( unsigned particleIdx ) const
   {
      return m_particles.at ( particleIdx ).pose;
   }

   tf::Pose MAVLocalization::getBestParticlePose() const
   {
      return getParticlePose ( getBestParticleIdx() );
   }

   tf::Pose MAVLocalization::getMeanParticlePose() const
   {
      tf::Pose meanPose = tf::Pose::getIdentity();

      double totalWeight = 0.0;

      meanPose.setBasis ( tf::Matrix3x3 ( 0, 0, 0, 0, 0, 0, 0, 0, 0 ) );
      for ( Particles::const_iterator it = m_particles.begin(); it != m_particles.end(); ++it )
      {
         meanPose.getOrigin() += it->pose.getOrigin() * it->weight;
         meanPose.getBasis() [0] += it->pose.getBasis() [0];
         meanPose.getBasis() [1] += it->pose.getBasis() [1];
         meanPose.getBasis() [2] += it->pose.getBasis() [2];
         totalWeight += it->weight;
      }
      assert ( !isnan ( totalWeight ) );

      //assert(totalWeight == 1.0);

      // just in case weights are not normalized:
      meanPose.getOrigin() /= totalWeight;
      // TODO: only rough estimate of mean rotation, asserts normalized weights!
      meanPose.getBasis() = meanPose.getBasis().scaled (
                               tf::Vector3 ( 1.0 / m_numParticles, 1.0 / m_numParticles, 1.0 / m_numParticles ) );

      // Apparently we need to normalize again
      meanPose.setRotation ( meanPose.getRotation().normalized() );

      return meanPose;
   }

   double MAVLocalization::nEff() const
   {

      double sqrWeights = 0.0;
      for ( Particles::const_iterator it = m_particles.begin(); it != m_particles.end(); ++it )
      {
         sqrWeights += ( it->weight * it->weight );
      }

      if ( sqrWeights > 0.0 )
      {
         return 1. / sqrWeights;
      }
      else
      {
         return 0.0;
      }
   }

   void MAVLocalization::toLogForm()
   {
      // TODO: linear offset needed?
      ////#pragma omp parallel for
      for ( unsigned i = 0; i < m_particles.size(); ++i )
      {
         assert ( m_particles[i].weight > 0.0 );
         m_particles[i].weight = log ( m_particles[i].weight );
      }
   }

   bool MAVLocalization::lookupPoseHeight ( const ros::Time& t, double& poseHeight ) const
   {
      tf::StampedTransform tf;
      if ( m_motionModel->lookupLocalTransform ( m_odomFrameId, t, tf ) )
      {
         poseHeight = tf.getOrigin().getZ();
         return true;
      }
      else
      {
         return false;
      }
   }

}


