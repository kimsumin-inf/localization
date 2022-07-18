#include "localization/localization.h"

using namespace std;

Localization::Localization()
:nh(""), pnh("")
{
    std::string gps_subscribe_topic_name;
    std::string gps_bestvel_subscribe_topic_name;
    std::string gps_bestpos_subscribe_topic_name;
    std::string imu_subscribe_topic_name;
    std::string Enc_Vel_subscribe_topic_name;
    std::string Enc_Steer_subscribe_topic_name;
    //launch file parameters
    pnh.param<std::string>("gps_subscribe_topic_name",gps_subscribe_topic_name,"/fix");
    pnh.param<std::string>("gps_bestvel_subscribe_topic_name",gps_bestvel_subscribe_topic_name,"/bestvel");
    pnh.param<std::string>("gps_bestpos_subscribe_topic_name",gps_bestpos_subscribe_topic_name,"/bestpos");

    pnh.param<std::string>("imu_subscribe_topic_name", imu_subscribe_topic_name, "/vectornav/IMU");

    pnh.param<std::string>("Enc_Vel_subscribe_topic_name", Enc_Vel_subscribe_topic_name, "/MSG_CON/Rx_Vel");
    pnh.param<std::string>("Enc_Steer_subscribe_topic_name", Enc_Steer_subscribe_topic_name, "/MSG_CON/Rx_Steer");

    pnh.param<double>("utm_x",utm_x,302533.174487);
    pnh.param<double>("utm_y",utm_y,4124215.346310);

    subGPS = nh.subscribe(gps_subscribe_topic_name,1,&Localization::gps_CB,this);
    subBestPos = nh.subscribe(gps_bestpos_subscribe_topic_name,1,&Localization::bestpos_CB,this);
    subBestVel = nh.subscribe(gps_bestvel_subscribe_topic_name,1,&Localization::bestvel_CB,this);

    subIMU = nh.subscribe(imu_subscribe_topic_name,1,&Localization::imu_CB,this);

    subEncVel = nh.subscribe(Enc_Vel_subscribe_topic_name,1,&Localization::encvel_CB,this);
    subEncSteer = nh.subscribe(Enc_Steer_subscribe_topic_name,1, &Localization::encSteer_CB,this);

}

void Localization::gps_CB(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps = *msg;

}

void Localization::bestvel_CB(const novatel_gps_msgs::NovatelVelocity::ConstPtr& msg){
    bestVel = *msg;
}
void Localization::bestpos_CB(const novatel_gps_msgs::NovatelPosition::ConstPtr &msg) {
    bestPos = *msg;
}
void Localization::imu_CB(const sensor_msgs::Imu::ConstPtr &msg) {
    imu = *msg;
    imu_now = ros::Time::now().toSec();
    double r,p,y;
    tf::Quaternion rpy;
    tf::quaternionMsgToTF(imu.orientation, rpy);
    tf::Matrix3x3(rpy).getRPY(r,p,y);
    //y-=1.58;

    NED(0,0) = (ENC_VEL.data/encvel_Scale) * cos(r);
    NED(1,0) = (ENC_VEL.data/encvel_Scale) * sin(r);
    NED(2,0) = 0;
    double dt = double(imu_now-imu_prev);

    cout <<NED<<endl;
    cout << r<< " " << p<<" " << y<<endl;
    cout <<"************************************************"<<endl;

    imu_prev = imu_now;

}
void Localization::encvel_CB(const std_msgs::Int16::ConstPtr &msg) {
    ENC_VEL = *msg;
    //cout << ENC_VEL.data/encvel_Scale <<endl;
}
void Localization::encSteer_CB(const std_msgs::Int16::ConstPtr &msg) {
    ENC_Steer = *msg;
    //cout << ENC_Steer.data/encsteer_Scale<<endl;

}
