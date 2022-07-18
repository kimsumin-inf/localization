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
    pnh.param<double>("utm_y",utm_y,4124215.34631);

    pubGPS_Marker = nh.advertise<visualization_msgs::Marker>("/Localization/no_filtering_marker",1);
    pubGPS_ORI = nh.advertise<nav_msgs::Odometry>("/Localization/GPS_ORI",1);
    pubGPS_LOC = nh.advertise<geometry_msgs::PoseWithCovariance>("/Localization/LocalPose",1);


    subIMU = nh.subscribe(imu_subscribe_topic_name,1,&Localization::imu_CB,this);

    subEncVel = nh.subscribe(Enc_Vel_subscribe_topic_name,1,&Localization::encvel_CB,this);
    subEncSteer = nh.subscribe(Enc_Steer_subscribe_topic_name,1, &Localization::encSteer_CB,this);

    InitGPS=true;

    markerId = 0;

    cout.precision(16);
}

void Localization::gps_CB(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps = *msg;
    double utm_x_meas=0.;
    double utm_y_meas=0.;
    string utm_zone;
    RobotLocalization::NavsatConversions::LLtoUTM(gps.latitude,gps.longitude,utm_y_meas,utm_x_meas,utm_zone);

    if(InitGPS){
        gpsData(GPSIDX::X) = utm_x_meas;
        gpsData(GPSIDX::Y) = utm_y_meas;
        gpsData(GPSIDX::YAW) = 0;
        gpsData(GPSIDX::V) = 0;
        InitGPS = false;
    }

    double theta = std::atan2(utm_y_meas - gpsData(GPSIDX::Y), utm_x_meas - gpsData(GPSIDX::X));
    theta  = std::atan2(sin(theta), cos(theta));
    gpsData(GPSIDX::X) = utm_x_meas;
    gpsData(GPSIDX::Y) = utm_y_meas;
    gpsData(GPSIDX::YAW) = theta;

    tf2::Quaternion Q_GPS;
    Q_GPS.setRPY(0,0,gpsData(GPS::YAW));
    Q_GPS.normalize();

    //publish loc marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp  = ros::Time::now();
    marker.ns="raw";
    marker.id = markerId++;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = gpsData(GPS::X)- utm_x;
    marker.pose.position.y =gpsData(GPS::Y)- utm_y;
    marker.pose.position.z = 0;

    marker.pose.orientation = tf2::toMsg(Q_GPS);
    marker.scale.x = 0.2;
    marker.scale.y=0.1;
    marker.scale.z=0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    pubGPS_Marker.publish(marker);

    nav_msgs::Odometry GPS_Axis;
    GPS_Axis.header.frame_id = "map";
    GPS_Axis.header.stamp = ros::Time::now();
    GPS_Axis.pose.pose.position.x = gpsData(GPS::X)- utm_x;
    GPS_Axis.pose.pose.position.y = gpsData(GPS::Y)- utm_y;;
    GPS_Axis.pose.pose.position.z = 0;

    GPS_Axis.pose.pose.orientation = tf2::toMsg(Q_GPS);

    //publish location
    geometry_msgs::PoseWithCovariance gps_pos;
    gps_pos.pose.position.x = gpsData(GPSIDX::X)- utm_x;
    gps_pos.pose.position.y = gpsData(GPSIDX::Y)- utm_y;
    gps_pos.pose.position.z = 0;
    gps_pos.pose.orientation = tf2::toMsg(Q_GPS);
    pubGPS_LOC.publish(gps_pos);




}

void Localization::bestvel_CB(const novatel_gps_msgs::NovatelVelocity::ConstPtr& msg){
    bestVel = *msg;
    gpsData(GPSIDX::V) = sqrt(pow(bestVel.horizontal_speed,2) + pow(bestVel.vertical_speed,2));

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
