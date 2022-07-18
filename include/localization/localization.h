#pragma once

#include "covariance.h"
#include "navsat_conversions.h"


#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <novatel_gps_msgs/NovatelVelocities.h>
#include <novatel_gps_msgs/NovatelPosition.h>

#include <tf/transform_broadcaster.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <visualization_msgs/Marker.h>

#include <vector>
#include <iomanip>

class Localization{
private:
    void gps_CB(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void bestvel_CB(const novatel_gps_msgs::NovatelVelocity::ConstPtr& msg);
    void bestpos_CB(const novatel_gps_msgs::NovatelPosition::ConstPtr& msg);

    void imu_CB(const sensor_msgs::Imu::ConstPtr& msg);

    ros::Time time;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber subGPS;
    ros::Subscriber subBestVel;
    ros::Subscriber subBestPos;

    ros::Subscriber subIMU;

    sensor_msgs::NavSatFix GPS;
    sensor_msgs::Imu IMU;
    novatel_gps_msgs::NovatelVelocity bestVel;
    novatel_gps_msgs::NovatelPosition bestPos;

    double utm_x;
    double utm_y;
public:
    Localization();
};