/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */

#ifndef MAV_LOCALIZATION_ENDPOINTMODEL_H_
#define MAV_LOCALIZATION_ENDPOINTMODEL_H_

#include <limits>
#include<cmath>

// #include <omp.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <mav_localization/ObservationModel.h>
#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <visualization_msgs/Marker.h>

namespace MAV_localization
{
class EndpointModel: public ObservationModel
{
public:
    EndpointModel ( ros::NodeHandle* nh, boost::shared_ptr<MapModel> mapModel, EngineT * rngEngine );
    virtual ~EndpointModel();
    virtual void integrateMeasurement ( Particles& particles, const PointCloud& pc,
                                        const std::vector<float>& ranges, float max_range, const tf::Transform& baseToSensor, int bestParticleIdx );

    virtual void setMap ( boost::shared_ptr<octomap::OcTree> map );

protected:
    bool getHeightError ( const Particle& p, const tf::StampedTransform& footprintToBase,
                          double& heightError ) const;
    void initDistanceMap();
    double m_sigma;
    double m_maxObstacleDistance;
    boost::shared_ptr<DynamicEDTOctomap> m_distanceMap;

    ros::Publisher virtual_cloud_pub;
};

}

#endif
