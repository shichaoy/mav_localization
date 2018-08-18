/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */


#ifndef MAVLOCALIZATION_RAYCASTINGMODEL_H_
#define MAVLOCALIZATION_RAYCASTINGMODEL_H_

#include <limits>
#include<cmath>

// #include <omp.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <mav_localization/ObservationModel.h>

#include <octomap/octomap.h>


namespace MAV_localization
{
class RaycastingModel : public ObservationModel
{
public:
    RaycastingModel ( ros::NodeHandle* nh, boost::shared_ptr<MapModel> mapModel, EngineT * rngEngine );
    virtual ~RaycastingModel();
    virtual void integrateMeasurement ( Particles& particles, const PointCloud& pc, const std::vector<float>& ranges, float max_range, const tf::Transform& baseToSensor, int bestParticleIdx );

protected:
    bool getHeightError ( const Particle& p, const tf::StampedTransform& footprintToBase, double& heightError ) const;
    // laser parameters:
    double m_zHit;
    double m_zRand;
    double m_zShort;
    double m_zMax;
    double m_sigmaHit;
    double m_lambdaShort;

    bool m_filterPointCloudGround;
    double m_groundFilterDistance;
    double m_groundFilterAngle;
    double m_groundFilterPlaneDistance;
    int m_numFloorPoints;
    int m_numNonFloorPoints;

    ros::Publisher virtual_cloud_pub;
};

}

#endif
