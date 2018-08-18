/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */

#include <mav_localization/EndpointModel.h>

namespace MAV_localization
{

  EndpointModel::EndpointModel ( ros::NodeHandle* nh, boost::shared_ptr<MapModel> mapModel, EngineT * rngEngine ) :
    ObservationModel ( nh, mapModel, rngEngine ), m_sigma ( 0.2 ), m_maxObstacleDistance ( 0.5 )
  {
    ROS_INFO ( "Using Endpoint observation model (precomputing...)" );

    nh->param ( "endpoint/sigma", m_sigma, m_sigma );
    nh->param ( "endpoint/max_obstacle_distance", m_maxObstacleDistance, m_maxObstacleDistance );

    if ( m_sigma <= 0.0 )
    {
      ROS_ERROR ( "Sigma (std.dev) needs to be > 0 in EndpointModel" );
    }

    initDistanceMap();

    virtual_cloud_pub = nh->advertise<sensor_msgs::PointCloud2> ( "virtual_cloud", 1 );

    
  }

  EndpointModel::~EndpointModel()
  {

  }

  void EndpointModel::integrateMeasurement ( Particles& particles, const PointCloud& pc, const std::vector<float>& ranges,
      float max_range, const tf::Transform& baseToSensor, int bestParticleIdx )
  {

    // iterate over samples, multithreaded:
      //#pragma omp parallel for
    for ( unsigned i = 0; i < particles.size(); ++i )
    {
      Eigen::Matrix4f globalLaserOrigin;
      pcl_ros::transformAsMatrix ( particles[i].pose * baseToSensor, globalLaserOrigin );
      PointCloud pc_transformed;
      pcl::transformPointCloud ( pc, pc_transformed, globalLaserOrigin );


      std::vector<float>::const_iterator ranges_it = ranges.begin();
      // iterate over beams:
      for ( PointCloud::const_iterator it = pc_transformed.begin(); it != pc_transformed.end(); ++it, ++ranges_it )
      {
        // search only for endpoint in tree
        octomap::point3d endPoint ( it->x, it->y, it->z );
//         float dist = m_distanceMap->getDistance ( endPoint );
	float dist = m_distanceMap->getDistance ( endPoint );
        float sigma_scaled = m_sigma;
        //sigma_scaled = ( *ranges_it ) * ( *ranges_it ) * ( m_sigma );
	sigma_scaled = ( *ranges_it ) * ( m_sigma );
	
        if ( dist > 0.0 )
        {
          // endpoint is inside map:
          particles[i].weight += logLikelihood ( dist, sigma_scaled );
        }
        else
        {
          //assign weight of max.distance:
          particles[i].weight += logLikelihood ( m_maxObstacleDistance, sigma_scaled );
        }
      }
      // TODO: handle max range measurements
      //std::cout << "\n";

      if ( i == bestParticleIdx )
      {
        sensor_msgs::PointCloud2 virtual_cloud;
        pcl::toROSMsg ( pc, virtual_cloud );
        virtual_cloud_pub.publish ( virtual_cloud );
      }
    }

  }

  bool EndpointModel::getHeightError ( const Particle& p, const tf::StampedTransform& odomToBase,
                                       double& heightError ) const
  {
    tf::Vector3 xyz = p.pose.getOrigin();
    double poseHeight = odomToBase.getOrigin().getZ();
    std::vector<double> heights;
    m_mapModel->getHeightlist ( xyz.getX(), xyz.getY(), 0.6, heights );
    if ( heights.size() == 0 )
      return false;

    // TODO: verify this!
    // find nearest z-level:
    heightError = std::numeric_limits<double>::max();
    for ( unsigned i = 0; i < heights.size(); i++ )
    {
      ROS_DEBUG ( "Height list is: %f, odometry height is: %f, particle height is: %f",
                  heights[i], poseHeight, xyz.getZ() );

      double dist = std::abs ( ( heights[i] + poseHeight ) - xyz.getZ() );
      if ( dist < heightError )
        heightError = dist;

    }

    return true;
  }

  void EndpointModel::setMap ( boost::shared_ptr<octomap::OcTree> map )
  {
    m_map = map;
    initDistanceMap();
  }

  void EndpointModel::initDistanceMap()
  {
    double x, y, z;
    m_map->getMetricMin ( x, y, z );
    octomap::point3d min ( x, y, z );
    m_map->getMetricMax ( x, y, z );
    octomap::point3d max ( x, y, z );
    m_distanceMap = boost::shared_ptr<DynamicEDTOctomap> (
                      new DynamicEDTOctomap ( float ( m_maxObstacleDistance ), & ( *m_map ), min, max, false ) );
    m_distanceMap->update();

//     float ***distMap;
//     double treeResolution;
//     int offsetX, offsetY, offsetZ;
//     m_distanceMap->d

    ROS_INFO ( "Distance map for endpoint model completed" );
  }

}

