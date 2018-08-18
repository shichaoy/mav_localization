/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */


#ifndef MAVLOCALIZATION_MAPMODEL_H_
#define MAVLOCALIZATION_MAPMODEL_H_

// #include <omp.h>    
#include <ros/ros.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_ros/conversions.h>

#include <mav_localization/MAVLocalization_defs.h>

namespace MAV_localization
{

class MapModel
{
public:
    MapModel ( ros::NodeHandle* nh );
    virtual ~MapModel();

    //void setMap(boost::shared_ptr<octomap::OcTree> map);
    boost::shared_ptr<octomap::OcTree> getMap() const;

    /**
     * Check if particles represent valid poses:
     * Must be within map bounding box and not in an occupied area.
     * Otherwise weight is minimized (=> die out at next resampling)
     */
    virtual void verifyPoses ( Particles& particles );

    virtual void initGlobal ( Particles& particles,
                              double z, double roll, double pitch,
                              const Vector6d& initNoise,
                              UniformGeneratorT& rngUniform, NormalGeneratorT& rngNormal );

    /// @return whether a map coordinate is occupied. Will return
    /// "false" if the coordinate does not exist in the map (e.g. out of bounds).
    virtual bool isOccupied ( const octomap::point3d& position ) const;
    virtual bool isOccupied ( octomap::OcTreeNode* node ) const = 0;
    virtual double getFloorHeight ( const tf::Transform& pose ) const = 0;

    /**
     * Get a list of valid z values at a given xy-position
     *
     * @param x
     * @param y
     * @param totalHeight clearance of the robot required to be free
     * @param[out] heights list of valid heights, return by ref.
     */
    void getHeightlist ( double x, double y, double totalHeight, std::vector<double>& heights );


protected:
    boost::shared_ptr<octomap::OcTree> m_map;

    double m_motionMeanZ;
    double m_motionRangeZ;
    double m_motionRangeRoll;
    double m_motionRangePitch;
    double m_motionObstacleDist;
    
    bool inside_map;

};


class DistanceMap : public MapModel
{
public:
    DistanceMap ( ros::NodeHandle* nh );
    virtual ~DistanceMap();

    virtual bool isOccupied ( octomap::OcTreeNode* node ) const;
    virtual double getFloorHeight ( const tf::Transform& pose ) const;

};


class OccupancyMap : public MapModel
{
public:
    OccupancyMap ( ros::NodeHandle* nh );
    virtual ~OccupancyMap();

    virtual bool isOccupied ( octomap::OcTreeNode* node ) const;
    virtual double getFloorHeight ( const tf::Transform& pose ) const;
};

}
#endif
