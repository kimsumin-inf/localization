#include "localization/localization.h"

using namespace std;

Localization::Localization()
:nh(""), pnh("")
{
    std::string gps_subscribe_topic_name;
    std::string gps_bestvel_subscribe_topic_name;
    std::string gps_bestpos_subscribe_topic_name;
    std::string imu_subscribe_topic_name;

    //launch file parameters
    pnh.param<std::string>("gps_subscribe_topic_name",gps_subscribe_topic_name,"/fix");
    pnh.param<std::string>("gps_bestvel_subscribe_topic_name",gps_bestvel_subscribe_topic_name,"/bestvel");
    pnh.param<std::string>("gps_bestpos_subscribe_topic_name",gps_bestpos_subscribe_topic_name,"/bestpos");

    pnh.param<std::string>("imu_subscribe_topic_name", imu_subscribe_topic_name, "/vectornav/IMU");

    pnh.param<double>("utm_x",utm_x,302533.174487);
    pnh.param<double>("utm_y",utm_y,4124215.346310);

    subGPS = nh.subscribe(gps_subscribe_topic_name,1,&Localization::gps_CB,this);
    subBestPos = nh.subscribe(gps_bestpos_subscribe_topic_name,1,&Localization::bestpos_CB,this);
    subBestVel = nh.subscribe(gps_bestvel_subscribe_topic_name,1,&Localization::bestvel_CB,this);

    subIMU = nh.subscribe(imu_subscribe_topic_name,1,&Localization::imu_CB,this);
}

void Localization::gps_CB(const sensor_msgs::NavSatFix::ConstPtr &msg) {

}

void Localization::bestvel_CB(const novatel_gps_msgs::NovatelVelocity::ConstPtr& msg){

}
void Localization::bestpos_CB(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg) {

}
void Localization::imu_CB(const sensor_msgs::Imu::ConstPtr &msg) {

}
