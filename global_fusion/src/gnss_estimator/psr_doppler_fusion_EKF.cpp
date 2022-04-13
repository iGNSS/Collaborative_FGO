/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of GraphGNSSLib.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 * Main fucntions: pseudorange/Doppler velocity fusion using extended Klaman        filter, 
 * input: pseudorange, Doppler velocity from GPS/BeiDou.
 * output: position of the GNSS receiver 
 * Date: 2020/11/24
 * Date: 20201126, successfully past the evaluation of the 
 *******************************************************/

// std inputs and outputs, fstream
#include <iostream>
#include <string>  
#include <fstream>
#include<sstream>
#include <stdlib.h>
#include <iomanip>

// math
#include <math.h>
//time 
#include <time.h>
//algorithm 
#include <algorithm>

// google eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include<Eigen/Core>

// google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>
// ros
#include <ros/ros.h>
/* Reference from NovAtel GNSS/INS */
#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include "gnss_tools.h"
#include <nlosExclusion/GNSS_Raw_Array.h>

#include <geometry_msgs/Point32.h>
#include <stdio.h>
#include <queue>
#include <map>
#include <queue>
#include <mutex>
#include <thread>
#include <nav_msgs/Odometry.h>

#include <nav_msgs/Path.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "../tic_toc.h"

// allign 
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/NavSatFix.h>

// rtklib /
#include <stdarg.h>
#include "../../RTKLIB/src/rtklib.h"

#include "../VectorMath.h"


#define EKFIterNum 5


/* save real time trajectory result */
FILE* trajectory = fopen("trajectory_psr_dop_fusion.csv", "w+");

#define INF 10000
#define D2R 3.1415926/180.0
#define pi 3.1415926
#define stateSize 5
#define controlInputSize 3

class ekf_estimator
{
public: 
    GNSS_Tools m_GNSS_Tools; // utilities
    Eigen::MatrixXd Q_matrix, G_matrix, sigma_matrix, R_matrix, K_matrix, H_matrix;
    VectorXd ekf_state; // state:  px, py, pz, bclo_gps, bclo_BeiDow
    VectorXd ekf_u_t; // control input: vx, vy, vz
    VectorXd ekf_z_t; // observation pseudoranges commonly 6~20 satellites

    double cur_t = 0;
    Eigen::MatrixXd ENULlhRef;

    

public:
    /**
     * @brief covariance estimation
     * @param nlosExclusion::GNSS_Raw_Array GNSS_data
     * @return weight_matrix
     @ 
    */
    MatrixXd cofactorMatrixCal_EKF(nlosExclusion::GNSS_Raw_Array GNSS_data)
    {
        double snr_1 = 50.0; // T = 50
        double snr_A = 30.0; // A = 30
        double snr_a = 30.0;// a = 30
        double snr_0 = 10.0; // F = 10
        VectorXd cofactor_;  // cofactor of satellite
        cofactor_.resize(GNSS_data.GNSS_Raws.size());
        for(int i = 0; i < GNSS_data.GNSS_Raws.size(); i++)
        {
        // if( (m_GNSS_Tools.PRNisGPS(GNSS_data.GNSS_Raws[i].prn_satellites_index)) || (m_GNSS_Tools.PRNisBeidou(GNSS_data.GNSS_Raws[i].prn_satellites_index)) )
            {
                double snr_R = GNSS_data.GNSS_Raws[i].snr;
                double elR = GNSS_data.GNSS_Raws[i].elevation;
                double q_R_1 = 1 / (pow(( sin(elR * pi/180.0 )),2));
                double q_R_2 = pow(10,(-(snr_R - snr_1) / snr_a));
                double q_R_3 = (((snr_A / (pow(10,(-(snr_0 - snr_1) / snr_a))) - 1) / (snr_0 - snr_1)) * (snr_R - snr_1) + 1);
                double q_R = q_R_1* (q_R_2 * q_R_3);
                cofactor_[i]=(float(q_R)); // uncertainty: cofactor_[i] larger, larger uncertainty
            }
        }
        // cout<<"cofactor_ -> "<<cofactor_<<endl;

        MatrixXd weight_matrix;
        weight_matrix.resize(GNSS_data.GNSS_Raws.size(),GNSS_data.GNSS_Raws.size());
        weight_matrix.setIdentity();
        for(int k = 0; k < weight_matrix.rows(); k++)
        {
        weight_matrix.row(k) = weight_matrix.row(k) * cofactor_(k);
        }

        return weight_matrix;
    }


public:
    void initializeEKFEstimator(void)
    {
        //
        setStateSize();
        setUSize();
        setQMatSize();
        setGMatSize();
        setInitializeSigmaSize();
        setENURefLLh();
    }


    /* set the size of the state */
    void setStateSize()
    {
        ekf_state.resize(stateSize);
        ekf_state.setZero();
    }

    /* set the size of the control input */
    void setUSize()
    {
        ekf_u_t.resize(controlInputSize);
    }

    /* set the size of the matrix Q_matrix */
    void setQMatSize()
    {
        Q_matrix.resize(stateSize, stateSize);
        Q_matrix << 0.6 * 0.6, 0, 0, 0, 0,
                    0, 0.6 * 0.6, 0, 0, 0,
                    0, 0, 0.6 * 0.6, 0, 0,
                    0, 0, 0, 10000, 0,
                    0, 0, 0, 0, 10000;
    }

    /* set the size of the matrix G_matrix */
    void setGMatSize()
    {
        G_matrix.resize(stateSize, stateSize);
    }

    /* set the size of the sigma matrix sigma_matrix */
    void setInitializeSigmaSize()
    {
        sigma_matrix.resize(stateSize, stateSize);
        sigma_matrix << 10, 0, 0, 0, 0,
                        0, 10, 0, 0, 0,
                        0, 0, 20, 0, 0,
                        0, 0, 0, 100, 0,
                        0, 0, 0, 0, 100;
    }

    /* setup ENU reference */
    void setENURefLLh()
    {
        /* reference point for ENU calculation */
        ENULlhRef.resize(3,1);
        ENULlhRef<< ref_lon, ref_lat, ref_alt;
    }

    /* setup current time */
    bool setCurrentTime(double Time)
    {
        //
        cur_t = Time;
    }

    /* setup ENU reference */
    void setStateInitialization(Eigen::MatrixXd input)
    {
        /* reference point for ENU calculation */
        ekf_state(0) = input(0);
        ekf_state(1) = input(1);
        ekf_state(2) = input(2);
        ekf_state(3) = input(3);
        ekf_state(4) = input(4);
    }

    /* get latest state in ENU frame */
    Eigen::Vector3d  getLatestEKFSatteENU()
    {
        //
        /** calculate the state in ENU frame */
        Eigen::MatrixXd ekfENU;
        ekfENU.resize(3, 1);
        std::cout << std::setprecision(12);
        ekfENU = m_GNSS_Tools.ecef2enu(ENULlhRef, ekf_state);
        std::cout << std::setprecision(12);
        // std::cout << "observation: ekf_state -> "<< std::endl << ekf_state <<std::endl;
        std::cout << "**********ekfENU -> "<< std::endl << ekfENU << std::endl;
        return ekfENU;
    }

    /* get latest state in LLH frame */
    Eigen::MatrixXd  getLatestEKFSatteLLH()
    {
        /** calculate the state in LLH frame */
        Eigen::Matrix<double,3,1> ekfllh = m_GNSS_Tools.ecef2llh(ekf_state);
    
        return ekfllh;
    }

    /* prediction: based on the Doppler measurement */
    bool prediction(Eigen::Vector3d input, double deltaTime)
    {
        //
        ekf_u_t = input; // 

        /*** position prediction */
        ekf_state(0) = ekf_state(0) + ekf_u_t(0) * deltaTime;
        ekf_state(1) = ekf_state(1) + ekf_u_t(1) * deltaTime;
        ekf_state(2) = ekf_state(2) + ekf_u_t(2) * deltaTime;

        /***receiver clock bias prediction */ 
        ekf_state(3) = ekf_state(3); // GPS
        ekf_state(4) = ekf_state(4); // BeiDou

        /** update the G matrix */
        G_matrix << 1, 0, 0, 0, 0,
                    0, 1, 0, 0, 0,
                    0, 0, 1, 0, 0,
                    0, 0, 0, 1, 0,
                    0, 0, 0, 0, 1;
        sigma_matrix = G_matrix * sigma_matrix * G_matrix.transpose() + Q_matrix;
        std::cout << std::setprecision(12);
        // std::cout << "prediction: ekf_state -> "<< std::endl << ekf_state<< std::endl;
    }

    /* observation: based on the GNSS pseudorange measurement */
    double observation(nlosExclusion::GNSS_Raw_Array GNSS_data, double time)
    {
        /**initialize the predicted observation */
        VectorXd predicted_ekf_z_t; // predicted observation
        predicted_ekf_z_t.resize(GNSS_data.GNSS_Raws.size());
        H_matrix.resize(GNSS_data.GNSS_Raws.size(),stateSize);
        MatrixXd weight_matrix = cofactorMatrixCal_EKF(GNSS_data);

        /** tranverse the observation to form observation */
        ekf_z_t.resize(GNSS_data.GNSS_Raws.size());
        for(int i =0; i < GNSS_data.GNSS_Raws.size(); i++)
        {
            if(m_GNSS_Tools.PRNisGPS(GNSS_data.GNSS_Raws[i].prn_satellites_index))
            {
                ekf_z_t(i) = GNSS_data.GNSS_Raws[i].pseudorange;
            } // gps
            else 
            {
                ekf_z_t(i) = GNSS_data.GNSS_Raws[i].pseudorange;
            } // beidou
        }
        
        std::cout << "observation: ekf_z_t -> "<< std::endl << ekf_z_t << std::endl;
        std::cout << "observation: ekf_state -> "<< std::endl << ekf_state << std::endl;

        /**tranverse the observation to form matrix */
        for(int i =0; i < GNSS_data.GNSS_Raws.size(); i++) // tranverse all the satellites 
        {
            VectorXd H_row; // state:  px, py, pz, bclo_GPS, bclo_BeiDou
            H_row.resize(stateSize);
            // cout<<"construct observation matrxi"<<endl;

            double dis_x = GNSS_data.GNSS_Raws[i].sat_pos_x - ekf_state(0);
            double dis_y = GNSS_data.GNSS_Raws[i].sat_pos_y - ekf_state(1);
            double dis_z = GNSS_data.GNSS_Raws[i].sat_pos_z - ekf_state(2);
            double dis_r_s = sqrt( pow(dis_x,2) + pow(dis_y,2) + pow(dis_z,2));

            // std::cout << "sat_pos_y -> "<< std::endl << GNSS_data.GNSS_Raws[i].sat_pos_x << std::endl;
            // std::cout << "sat_pos_y -> "<< std::endl << GNSS_data.GNSS_Raws[i].sat_pos_y << std::endl;
            // std::cout << "sat_pos_z -> "<< std::endl << GNSS_data.GNSS_Raws[i].sat_pos_z << std::endl;


            double OMGE_ = 7.2921151467E-5;
            double CLIGHT_ = 299792458.0;
            dis_r_s = dis_r_s + OMGE_ * (GNSS_data.GNSS_Raws[i].sat_pos_x*ekf_state[1]-GNSS_data.GNSS_Raws[i].sat_pos_y*ekf_state[0])/CLIGHT_;
            
            H_row(0) = -1 * (dis_x) / dis_r_s;
            H_row(1) = -1 * (dis_y) / dis_r_s;
            H_row(2) = -1 * (dis_z) / dis_r_s;

            if(m_GNSS_Tools.PRNisGPS(GNSS_data.GNSS_Raws[i].prn_satellites_index)) //
            {
                H_row(3) = 1;
                H_row(4) =0;
                predicted_ekf_z_t(i) = dis_r_s + ekf_state(3);
                std::cout << "ekf_state(3) -> "<< std::endl << ekf_state(3) << std::endl;
            }
            else 
            {
                H_row(3) = 0;
                H_row(4) = 1;
                predicted_ekf_z_t(i) = dis_r_s + ekf_state(4);
                std::cout << "ekf_state(4) -> "<< std::endl << ekf_state(4) << std::endl;
            }
            H_matrix.row(i) = H_row;        
        }

        /** perform the observation of the Kalman filter */
        R_matrix.resize(GNSS_data.GNSS_Raws.size(),GNSS_data.GNSS_Raws.size());
        R_matrix.setIdentity();
        R_matrix = weight_matrix ;
        // std::cout<<"weight_matrix-> \n"<< weight_matrix<<std::endl;

        // update K_matrix
        K_matrix.resize(stateSize,GNSS_data.GNSS_Raws.size());
        Eigen::MatrixXd I_matrix;
        I_matrix.resize(stateSize,stateSize);
        I_matrix.setIdentity();
        K_matrix = sigma_matrix * H_matrix.transpose() * (H_matrix * sigma_matrix * H_matrix.transpose() + R_matrix).inverse();
        // std::cout<<"K_matrix-> \n"<< K_matrix<<std::endl;
        
        ekf_state = ekf_state + K_matrix * (ekf_z_t -predicted_ekf_z_t);
        sigma_matrix = (I_matrix - K_matrix * H_matrix) * sigma_matrix;
        
        // std::cout<<"H_matrix-> \n"<< H_matrix<<std::endl;
        std::cout << "observation: ekf_z_t -predicted_ekf_z_t -> "<< std::endl << ekf_z_t -predicted_ekf_z_t << std::endl;
        Eigen::MatrixXd residualNorm = (ekf_z_t -predicted_ekf_z_t).transpose() * (ekf_z_t -predicted_ekf_z_t);
        std::cout<<"residualNorm-> \n"<< sqrt(residualNorm(0,0))<<std::endl;
        return sqrt(residualNorm(0,0));
    }
};


class psr_doppler_fusion_ekf_estimator
{
    ros::NodeHandle nh; // node handle

    // ros subscriber
    ros::Subscriber gnss_raw_sub;
    ros::Publisher pub_WLS, pub_EKF,pub_EKF_path, pub_WLS_path;
    nav_msgs::Path EKF_path, WLS_path;

    // map of measurements set
    std::map<double, nlosExclusion::GNSS_Raw_Array> gnss_raw_map;
    std::map<double, nav_msgs::Odometry> doppler_map;

    std::mutex m_gnss_raw_mux;
    GNSS_Tools m_GNSS_Tools; // utilities
    int gnss_frame = 0;
    Eigen::MatrixXd ENULlhRef;

    /* subscribe the raw pseudorange and Doppler measurements */
    std::unique_ptr<message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>> gnss_raw_array_sub;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> doppler_sub;
    std::unique_ptr<message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>> syncdoppler2GNSSRaw;
    
    double var = 0.6; // doppler var 0.6

    ekf_estimator ekf_Estimator;
    Eigen::Vector3d lastDopplerVel;
    double last_t = 0;

public:
    psr_doppler_fusion_ekf_estimator()
    {        
        ekf_Estimator.initializeEKFEstimator();
        pub_WLS = nh.advertise<nav_msgs::Odometry>("/poseWLS", 100); // 
        pub_EKF = nh.advertise<nav_msgs::Odometry>("/poseEKF", 100); //  
        pub_EKF_path = nh.advertise<nav_msgs::Path>("/EKF_path", 100); //
        pub_WLS_path = nh.advertise<nav_msgs::Path>("/WLS_path", 100); //

        gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov1", 10000));
        doppler_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/gnss_preprocessor_node/GNSSDopVelRov1", 10000));
        syncdoppler2GNSSRaw.reset(new message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>(*gnss_raw_array_sub, *doppler_sub, 10000));

        syncdoppler2GNSSRaw->registerCallback(boost::bind(&psr_doppler_fusion_ekf_estimator::gnssraw_doppler_msg_callback,this, _1, _2));
      
        /* reference point for ENU calculation */
        ENULlhRef.resize(3,1);
        ENULlhRef<< ref_lon, ref_lat, ref_alt;
    }
    ~psr_doppler_fusion_ekf_estimator()
    {
        std::cout << "de-construction function \n";
    }

    
    /**
	 * @brief get distance between two points
	 * @param p1, p2
	 * @return distance
	 */
    double getDistanceFrom2Points(Eigen::Vector3d p1, Eigen::Vector3d p2)
    {
        double xMod = pow((p1.x() - p2.x()), 2);
        double yMod = pow((p1.y() - p2.y()), 2);
        double zMod = pow((p1.z() - p2.z()), 2);
        double mod = sqrt(xMod + yMod + zMod);
        return mod;
    }


    /* check the valid epoch based on gps time span*/
    bool checkValidEpoch(double gps_sec)
    {
        if((gps_sec >= start_gps_sec) && (gps_sec <=end_gps_sec))
        {
            return true;
        }
        else return false;
    }

    /**
   * @brief gnss raw msg and doppler msg callback
   * @param gnss raw msg and doppler msg
   * @return void
   @ 
   */
    void gnssraw_doppler_msg_callback(const nlosExclusion::GNSS_Raw_ArrayConstPtr& gnss_msg, const nav_msgs::OdometryConstPtr& doppler_msg)
    {
        // m_gnss_raw_mux.lock();
        double time_frame = doppler_msg->pose.pose.position.x;
        std::cout << "doppler time -> " << time_frame << std::endl;
        // if (gnss_msg->GNSS_Raws.size()==0){
        //     return;
        // }
        // std::cout<< "psr time size ->" << gnss_msg->GNSS_Raws.size()<<std::endl;
        std::cout << "psr time -> " << gnss_msg->GNSS_Raws[0].GNSS_time << std::endl;

        if(checkValidEpoch(time_frame))
        {
            if(gnss_msg->GNSS_Raws.size()>2 && m_GNSS_Tools.checkRepeating(*gnss_msg))
            {
                gnss_frame++;
                LOG(INFO) << "\n\n\n****************time_frame*************** -> "<< std::endl << time_frame << std::endl;
                doppler_map[time_frame] = *doppler_msg;
                gnss_raw_map[time_frame] = *gnss_msg;
                
                /****** perform WLS for further evaluation **** */
                Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                        m_GNSS_Tools.getAllPositions(*gnss_msg),
                                        m_GNSS_Tools.getAllMeasurements(*gnss_msg),
                                        *gnss_msg, "WLS");
                if(gnss_frame==1)
                {
                    ekf_Estimator.setStateInitialization(eWLSSolutionECEF);
                }
                
                Eigen::Matrix<double ,3,1> WLSENU;
                WLSENU = m_GNSS_Tools.ecef2enu(ENULlhRef, eWLSSolutionECEF);
                LOG(INFO) << "WLSENU -> "<< std::endl << WLSENU;

                /*******perform the EKF estimation for further evaluation *********/
                ekf_Estimator.setCurrentTime(gnss_msg->GNSS_Raws[0].GNSS_time);

                TicToc EKFOptTime;

                //prediction 
                ekf_Estimator.prediction(lastDopplerVel, gnss_msg->GNSS_Raws[0].GNSS_time - last_t);
                std::cout << "***********state presiction*********** -> \n";
                ekf_Estimator.getLatestEKFSatteENU();

                // observation
                int iterNum = 0;
                while(iterNum<EKFIterNum)
                {
                    double resSquareNorm = ekf_Estimator.observation(*gnss_msg, gnss_msg->GNSS_Raws[0].GNSS_time);
                    iterNum++;
                }

                std::cout << "EKFOptTime-> "<< EKFOptTime.toc()<< std::endl;

                /************publish the WLS result*****************/
                nav_msgs::Odometry odometry;
                // odometry.header = pose_msg->header;
                odometry.header.frame_id = "map";
                odometry.child_frame_id = "map";
                odometry.pose.pose.position.x = WLSENU(0);
                odometry.pose.pose.position.y = WLSENU(1);
                odometry.pose.pose.position.z = 10;
                pub_WLS.publish(odometry);
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.header.frame_id = "map";
                pose_stamped.pose.position.x = WLSENU(0);
                pose_stamped.pose.position.y = WLSENU(1);
                pose_stamped.pose.position.z = 10;
                WLS_path.header.frame_id = "map";
                WLS_path.poses.push_back(pose_stamped);
                pub_WLS_path.publish(WLS_path); 

                /************publish the EKF result*****************/
                std::cout << "***********state update*********** -> \n";
                Eigen::Matrix<double ,3,1> EKFENU = ekf_Estimator.getLatestEKFSatteENU();
                odometry.pose.pose.position.x = EKFENU(0);
                odometry.pose.pose.position.y = EKFENU(1);
                odometry.pose.pose.position.z = 10;
                pub_EKF.publish(odometry);
                pose_stamped.header.stamp = ros::Time::now();
                pose_stamped.header.frame_id = "map";
                pose_stamped.pose.position.x = EKFENU(0);
                pose_stamped.pose.position.y = EKFENU(1);
                pose_stamped.pose.position.z = 10;
                EKF_path.header.frame_id = "map";
                EKF_path.poses.push_back(pose_stamped);
                pub_EKF_path.publish(EKF_path); 

                /* save EKF trajectory LLH to csv */
                Eigen::MatrixXd EKFLLH = ekf_Estimator.getLatestEKFSatteLLH();
                std::ofstream foutC("/home/wws/FGO_GNSS_V1.0/src/global_fusion/EKF_trajectoryllh_psr_dop_fusion.csv", std::ios::app);
                foutC.setf(std::ios::fixed, std::ios::floatfield);
                foutC.precision(0);             
                foutC<<time_frame<<",";
                foutC<<time_frame<<",";
                foutC.precision(10);
                foutC<<EKFLLH(1)<<",";
                foutC<<EKFLLH(0)<<",";
                foutC<<EKFLLH(2)<<std::endl;
                foutC.close();

                /********save the last doppler velocity********** */
                lastDopplerVel(0) = doppler_msg->twist.twist.linear.x;
                lastDopplerVel(1) = doppler_msg->twist.twist.linear.y;
                lastDopplerVel(2) = doppler_msg->twist.twist.linear.z;
                last_t = gnss_msg->GNSS_Raws[0].GNSS_time;
            }
        }
        
        // m_gnss_raw_mux.unlock();

    }

};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;  // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags 
    ros::init(argc, argv, "psr_doppler_fusion_ekf_estimator");  
    // std::cout
    ROS_INFO("\033[1;32m----> Psr Doppler fusion with EKF Started.\033[0m");
    // ...
    psr_doppler_fusion_ekf_estimator psr_doppler_fusion_ekf_estimator;
    ros::spin();
    return 0;
}
