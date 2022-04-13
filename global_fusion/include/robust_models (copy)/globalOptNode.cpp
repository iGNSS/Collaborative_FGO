/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of fgo_gnss.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 * 
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 * 
 *******************************************************/



#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "globalOpt.h"

/****GMM related head file****/
#include "ErrorModel.h"
#include "Gaussian.h"
#include "GaussianComponent.h"
#include "GaussianMixture.h"
#include "SumMixture.h"
#include "VectorMath.h"

/* Reference from NovAtel GNSS/INS */
#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include "gnss_tools.h"

#include <geometry_msgs/Point32.h>


GlobalOptimization globalEstimator;
ros::Publisher pub_ref, pub_global_odometry, pub_global_path, pub_car, pub_lowCost_GNSS_odometry,pub_lio_cov;
nav_msgs::Path *global_path;

double lio_time = 0;

bool ref_flag = 0;
Eigen::Matrix <double, 3,1> ref_ini_p;
Eigen::Matrix <double, 3,1> ref_ini_q;

Eigen::Vector3d gnss_p(0,0,0);
Eigen::Vector3d lio_p(0,0,0);
Eigen::Vector3d lio_cov(0,0,0);
Eigen::Vector3d ref_p(0,0,0);
Eigen::Vector3d fusion_p(0,0,0);

/* save result */
FILE* trajectory = fopen("../ion_PLANS_2020_GNSS_LiDAR/src/result/trajectory.csv", "w+");
FILE* error = fopen("../ion_PLANS_2020_GNSS_LiDAR/src/result/error.csv", "w+");
int epoch = 0;

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot, rot2;
    Eigen::AngleAxisd rotation_vector(3.1415926 / 2,Eigen::Vector3d(1,0,0)); // rotate a certain angle
    rot = rotation_vector.matrix();
    Eigen::AngleAxisd rotation_vector2(-1*3.1415926 / 2,Eigen::Vector3d(0,1,0)); // rotate a certain angle
    rot2 = rotation_vector2.matrix();
    // rot << 0, 0, -1, 
    //        0, -1, 0, 
    //        -1, 0, 0;
    Eigen::Quaterniond Q;
    Q = q_w_car * rot * rot2; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 1.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

/**
 * @brief process GNSS measurements
 * @param GNSS_msg
 * @return none
 */
void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GNSS_msg)
{
    //printf("GPS_callback! \n");

    /* get time */
    double t = GNSS_msg->header.stamp.toSec();
    t = lio_time; // replace the gnss time with lio time
    //printf("receive GPS with timestamp %f\n", GNSS_msg->header.stamp.toSec());

    /* get position */
    double latitude = GNSS_msg->latitude;
    double longitude = GNSS_msg->longitude;
    double altitude = GNSS_msg->altitude;

    /* get covariance */
    double pos_accuracy = GNSS_msg->position_covariance[0];
    // double scalingFactor = 0.2; // the batch based trajectory works pretty well
    // double scalingFactor = 0.02;
    double scalingFactor = 1;
    pos_accuracy = pos_accuracy * scalingFactor;
    // printf("receive covariance %lf \n", pos_accuracy);

    /* input GNSS to estimator */
    Eigen::Vector3d GNSSOdom =  globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);

    /* publish GNSS odometry */
    nav_msgs::Odometry odometry;
    odometry.header = GNSS_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = GNSSOdom.x();
    odometry.pose.pose.position.y = GNSSOdom.y();
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation.x = 0.2;
    odometry.pose.pose.orientation.y = 0.3;
    odometry.pose.pose.orientation.z = 0.1;
    odometry.pose.pose.orientation.w = 0.2;
    pub_lowCost_GNSS_odometry.publish(odometry);

    gnss_p = GNSSOdom;
}

/**
 * @brief process LiDAR odometry
 * @param pose_msg
 * @return none
 */
void lio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("lio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    lio_time = t;
    Eigen::Vector3d lio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    lio_p = lio_t;
    lio_cov(0) = pose_msg->pose.covariance[0];
    Eigen::Quaterniond lio_q;
    lio_q.w() = pose_msg->pose.pose.orientation.w;
    lio_q.x() = pose_msg->pose.pose.orientation.x;
    lio_q.y() = pose_msg->pose.pose.orientation.y;
    lio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, lio_t, lio_q, lio_cov); // input odometry from lio

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    publish_car_model(t, global_t, global_q);

    /* get gloabl translation */
    fusion_p = global_t;
}

/**
 * @brief process ref measurements
 * @param GNSS_msg
 * @return none
 */
void ref_callback(const novatel_msgs::INSPVAXConstPtr& ref_msg)
{
    epoch ++;
    /* get time */
    double t = 0;
    t = lio_time; // replace the ref time with lio time
    
    /* get lat lon alt */
    GNSS_Tools gnss_tools;
    Eigen::Matrix <double, 3,1> ref_lla;
    ref_lla<< ref_msg->longitude, ref_msg->latitude, ref_msg->altitude;
    Eigen::Matrix <double, 3,1> ref_ecef;
    ref_ecef = gnss_tools.llh2ecef(ref_lla);

    /* get ref ENU */
    if(!ref_flag)
    {
        ref_ini_p<< ref_msg->longitude, ref_msg->latitude, ref_msg->altitude;
        ref_ini_q<<ref_msg->roll, ref_msg->pitch, ref_msg->azimuth;
        ref_flag = true;
    }
    Eigen::Matrix <double, 3,1> ref_enu;
    ref_enu = gnss_tools.ecef2enu(ref_ini_p, ref_ecef);
    double prex_ = ref_enu(0);
    double prey_ = ref_enu(1);
    double theta = (ref_ini_q(2) +90)*( 3.141592 / 180.0 ); //
    ref_enu(0) = prex_ * cos(theta) - prey_ * sin(theta) ;
    ref_enu(1) = prex_ * sin(theta) + prey_ * cos(theta) ; 
    
    /* pubslih ref ENU */
    nav_msgs::Odometry odometry;
    odometry.header.stamp = ros::Time::now();
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = ref_enu(0);
    odometry.pose.pose.position.y = ref_enu(1);
    odometry.pose.pose.position.z = 0;
    pub_ref.publish(odometry);
    std::cout << "get ref msg..."<<std::endl;
    ref_p<<ref_enu(0),ref_enu(1), ref_enu(2);

    /* save error to csv */
    double gnss_error = gnss_tools.getMSE(gnss_p, ref_p, "2D_Error"); // gnss
    double lio_error = gnss_tools.getMSE(lio_p, ref_p, "2D_Error"); // lio
    double fusion_error = gnss_tools.getMSE(fusion_p, ref_p, "2D_Error"); // fusion
    Eigen::Vector3d xyz_Error = gnss_tools.getXYZ_error(gnss_p, ref_p,"xyz_Error");// Gnss error in xyz
    fprintf(error, "%d,%3.7f,%3.7f,%3.7f,%3.7f,%3.7f,%3.7f \n", epoch, gnss_error, lio_error, fusion_error, xyz_Error(0),xyz_Error(1),xyz_Error(2));//
    fflush(error);

    /* publish lio cov */
    std::cout << "lio_cov-> "<< lio_cov(0)<<std::endl;
    geometry_msgs::Point32 lio_cov_msg;
    lio_cov_msg.x = lio_error;
    lio_cov_msg.y = lio_cov(0);
    pub_lio_cov.publish(lio_cov_msg);

    /* save trajectory to csv */
    fprintf(trajectory, "%d,%3.7f,%3.7f,", epoch, ref_p(0),ref_p(1)); // ref
    fprintf(trajectory, "%3.7f, %3.7f,", gnss_p(0), gnss_p(1)); // gnss
    fprintf(trajectory, "%3.7f, %3.7f,", lio_p(0), lio_p(1)); // lio
    fprintf(trajectory, "%3.7f, %3.7f \n", fusion_p(0), fusion_p(1)); // lio
    fflush(trajectory);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    global_path = &globalEstimator.global_path; // path 

    /* GNSS */
    // ros::Subscriber sub_GPS = n.subscribe("/navsat/fix", 100, GPS_callback); // subscribe gps message
    ros::Subscriber sub_GPS = n.subscribe("/ublox_node/fix", 100, GPS_callback); // subscribe gps message

    /* LiDAR odometry */
    // ros::Subscriber sub_lio = n.subscribe("/vins_estimator/odometry", 100, lio_callback); // odometry from vins
    // ros::Subscriber sub_lio = n.subscribe("/graph_mapping_tools/graph_pose", 100, lio_callback); // odometry from lio
    ros::Subscriber sub_lio = n.subscribe("/ndt_odom", 100, lio_callback); // odometry from lio
    
    /* GNSS odometry */
    pub_lowCost_GNSS_odometry = n.advertise<nav_msgs::Odometry>("lowCost_odometry", 100); // publish global odometry

    /* Estimation */
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100); // publish global path
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100); // publish global odometry
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000); // publish car model 

    /* Reference */
    ros::Subscriber sub_ref = n.subscribe("/novatel_data/inspvax", 100, ref_callback);
    pub_ref = n.advertise<nav_msgs::Odometry>("ref_odometry", 100); // publish global odometry

    /* lio uncertainty estimation and error */
    pub_lio_cov = n.advertise<geometry_msgs::Point32>("lio_cov", 100); // publish global odometry

    ros::spin();
    return 0;
}
