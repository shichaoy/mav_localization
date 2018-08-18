/*
 * 6D localization for Micro Aerial Vehicle
 *
 * Copyright 20014-2015 Zheng Fang, Carnegie Mellon University
 * Email: fangzheng81@gmail.com
 *
 *
 */

#include <ros/ros.h>
#include <mav_localization/MAVLocalization.h>

int main ( int argc, char** argv )
{
    ros::init ( argc, argv, "MAV_localization" );
    ros::NodeHandle private_nh ( "~" );


    unsigned seed;
    int iseed;
    private_nh.param ( "seed", iseed, -1 );
    
    if ( iseed == -1 )
        seed = static_cast<unsigned int> ( std::time ( 0 ) );
    else
        seed = static_cast<unsigned int> ( iseed );

    MAV_localization::MAVLocalization localization ( seed );

    ros::spin();
    

    return 0;
}
