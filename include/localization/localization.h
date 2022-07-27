#pragma once

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
#include <string>
#include <cmath>

#include "selfcar_lib/erp42.h"
#include "selfcar_lib/kalman_filter.h"
#include "selfcar_lib/navsat_conversions.h"
#include "selfcar_lib/trigonometric.h"

class Localization{
private:

    void gps_CB(const sensor_msgs::NavSatFix::ConstPtr& msg);

    void bestvel_CB(const novatel_gps_msgs::NovatelVelocity::ConstPtr& msg);
    void bestpos_CB(const novatel_gps_msgs::NovatelPosition::ConstPtr& msg);

    void imu_CB(const sensor_msgs::Imu::ConstPtr& msg);

    void initialpose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void filter();
    void predict();
    void update();




    ros::Time time;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Publisher pubGPS_Marker;
    ros::Publisher pubGPS_LOC;
    ros::Publisher pubGPS_ORI;

    ros::Subscriber subGPS;
    ros::Subscriber subBestVel;
    ros::Subscriber subBestPos;

    ros::Subscriber subIMU;

    ros::Subscriber subInitPose;




    /*
     * GPS BLOCK
     */

    //DATA
    sensor_msgs::NavSatFix gps;

    novatel_gps_msgs::NovatelVelocity bestVel;
    novatel_gps_msgs::NovatelPosition bestPos;

    struct GPS{
        enum IDX{
            X=0,
            Y=1,
            YAW=2,
            V=3
        };
    };
    typedef GPS::IDX GPSIDX;
    Eigen::MatrixXd gpsData ;
    Eigen::MatrixXd gpsSample;
    Eigen::MatrixXd gpsCov;
    int sampleNum;
    double utm_x;
    double utm_y;

    //FALG
    bool InitGPS;
    bool InitGPS_Sample;
    bool GPS_available;
    /*
     * IMU BLOCK
     */
    // DATA
    sensor_msgs::Imu imu;
    double localHeading;
    double heading;
    double wz_dt ;
    Eigen::MatrixXd wzdtSample;
    Eigen::MatrixXd wzdtCov;
    ros::Time IMU_PREV_TIME;
    double IMU_TIME_LAPSE;
    tf2::Quaternion qYawBias;
    //PARAM

    //FLAG
    bool IMU_available;

    /*
     * Kalman Filter BLOCK
     */

    /*
     * ETC
     */
    ERP42 erp;

    /*
     * Visualization
     */
    int markerId;



public:
    Localization();
};