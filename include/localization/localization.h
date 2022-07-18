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

#include <novatel_gps_msgs/NovatelVelocity.h>
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

    int encvel_Scale = 10;
    int encsteer_Scale = 71;
    void encvel_CB(const std_msgs::Int16::ConstPtr&msg);
    void encSteer_CB(const std_msgs::Int16::ConstPtr&msg);



    ros::Time time;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber subGPS;
    ros::Subscriber subBestVel;
    ros::Subscriber subBestPos;

    ros::Subscriber subIMU;

    ros::Subscriber subEncVel;
    ros::Subscriber subEncSteer;

    sensor_msgs::NavSatFix gps;
    sensor_msgs::Imu imu;
    novatel_gps_msgs::NovatelVelocity bestVel;
    novatel_gps_msgs::NovatelPosition bestPos;
    std_msgs::Int16 ENC_VEL;
    std_msgs::Int16 ENC_Steer;

    struct GPS{
        enum IDX{
            X=0,
            Y=1,
            YAW=2,
            V=3
        };
    };
    typedef GPS::IDX GPSIDX;
    Eigen::MatrixXd gpsData;

    double imu_prev;
    double imu_now;
    Eigen::MatrixXd NED = Eigen::MatrixXd(3,1);

    double utm_x;
    double utm_y;

public:
    Localization();
};