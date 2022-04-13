/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of fgo_gnss.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 * 
 * Main fucntions: pseudorange/Doppler velocity fusion using factor graph optimization aided by graduated non-convexity (GNC) with Geman McClure (GM)
 * input: pseudorange, Doppler velocity from GPS/BeiDou.
 * output: position of the GNSS receiver 
 * Date: 2020/11/27
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

// rtklib
#include <stdarg.h>
#include "../../RTKLIB/src/rtklib.h"

// /* factor graph related head file */
#include "../../include/gnss_estimator_robust/psr_doppler_fusion_gnc_gm.h"

class psr_doppler_fusion_gnc_gm
{
    // ros::NodeHandle nh;

    /* ros subscriber */
    ros::Publisher pub_WLSENU, pub_FGOENU, pub_global_path, pub_fgo_llh;
    std::map<double, nlosExclusion::GNSS_Raw_Array> gnss_raw_map;
    std::map<double, nav_msgs::Odometry> doppler_map;

    GNSS_Tools m_GNSS_Tools; // utilities

    /* subscriber */
    std::unique_ptr<message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>> gnss_raw_array_sub;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> doppler_sub;
    std::unique_ptr<message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>> syncdoppler2GNSSRaw;

    /* thread lock for data safe */
    std::mutex m_gnss_raw_mux;

    /* thread for data processing */
    std::thread optimizationThread;

    int gnss_frame = 0;
    Eigen::Matrix<double, 3,1> ENU_ref;
    int slidingWindowSize = 1000000; // ? epoch measurements 150 100000
    bool hasNewData = false;

    /* latest state in ENU */
    Eigen::Matrix<double ,3,1> FGOENULatest;

    /* path in ENU */
    nav_msgs::Path fgo_path;

    /* parameters for GNC infinite: convex; zero: non-convex */
    double GNC_mu = 1000000; 

    /* the maximum error you can accept (0.7) */
    double GNC_c = 2; 

    /* the maximum residual at first solving */
    double GNC_r_max = 0; 

    /* remove large outliers */
    bool removeLargeOurliers = true;

    /* log path */
    std::string logPath;

private:
    // std::unique_ptr<factor_graph> factor_graph_ptr_; // factor graph ptr
    FactorGraphRobust factor_graph_robust;
    
    bool initializeFactorGraph(ceres::Solver::Options& options) 
    {
        /* initialize the factor graph size */
        factor_graph_robust.setWindowSize(slidingWindowSize);

        

        /* set up ceres-solver options */
        factor_graph_robust.setupSolverOptions(options);

        /* set up loss functions (Huber, Cauchy)*/
        factor_graph_robust.setupLossFunction("Huber");
    }


public:
    psr_doppler_fusion_gnc_gm(ros::NodeHandle& nh)
    {
        /* setup GNC-GM parameters */
        nh.param("GNC_mu",     GNC_mu, 1000000.0);
        nh.param("GNC_c",      GNC_c, 2.0);
        nh.param("GNC_r_max",  GNC_r_max, 0.0);
        nh.param("removeLargeOurliers",  removeLargeOurliers, true);
        nh.param("slidingWindowSize",    slidingWindowSize, 100000);
        ros::param::get("logPath", logPath);

        /* thread for factor graph optimization */
        optimizationThread = std::thread(&psr_doppler_fusion_gnc_gm::solveOptimization, this);
        
        pub_WLSENU = nh.advertise<nav_msgs::Odometry>("WLSGoGPS", 100); // 
        pub_FGOENU = nh.advertise<nav_msgs::Odometry>("FGO", 100); //  
        pub_fgo_llh = nh.advertise<sensor_msgs::NavSatFix>("fgo_llh", 100);

        gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov1", 10000));
        doppler_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/gnss_preprocessor_node/GNSSDopVelRov1", 10000));
        syncdoppler2GNSSRaw.reset(new message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>(*gnss_raw_array_sub, *doppler_sub, 10000));

        syncdoppler2GNSSRaw->registerCallback(boost::bind(&psr_doppler_fusion_gnc_gm::gnssraw_doppler_msg_callback,this, _1, _2));

        pub_global_path = nh.advertise<nav_msgs::Path>("/FGOGlobalPath", 100); // 
        
        /* reference point for ENU calculation */
        ENU_ref<< ref_lon, ref_lat, ref_alt;
    }


    /**
	 * @brief perform factor graph optimization
	 * @param none
	 * @return none
	 */
    void solveOptimization()
    {
        /* clear data stream */
        factor_graph_robust.clearDataStream();
        
        while(1)
        {
            m_gnss_raw_mux.lock();

            if(factor_graph_robust.getDataStreamSize()>3 && hasNewData)
            {
                /* Ceres solver object */
                ceres::Problem problem;
                ceres::Solver::Options options;
                ceres::Solver::Summary summary;

                /* start clock for factor graph optimization */
                TicToc OptTime;

                /* initialize factor graph */
                initializeFactorGraph(options);

                /* clear variables */
                factor_graph_robust.clearVariables();

                /* setup GNC-GM paras */
                factor_graph_robust.GNC_Geman_McClure.setGNCGMParas(GNC_mu, GNC_c, GNC_r_max);

                /* get data stream size */
                factor_graph_robust.getDataStreamSize();

                /* setup memory for state */
                factor_graph_robust.setupStateMemory();

                /* initialize factor graph parameters */
                factor_graph_robust.initializeFactorGraphParas();

                /* initialize the previous optimzied states */
                factor_graph_robust.initializeOldGraph();

                /* initialize the newly added states */
                factor_graph_robust.initializeNewlyAddedGraph();

                /* add parameter blocks */
                factor_graph_robust.addParameterBlocksToGraph(problem);

                /* fix the first parameter block */
                factor_graph_robust.fixFirstState(false,problem);

                /* add Doppler FACTORS to factor graph */
                factor_graph_robust.addDopplerFactors(problem);

                /* add pseudorange FACTORS to factor graph */
                factor_graph_robust.addPseudorangeFactors(problem);

                /* solve the factor graph */
                factor_graph_robust.solveFactorGraph(problem, options, summary);

                /* initialize the satNumPerT */
                factor_graph_robust.initializeSatNumPerT();

                /* allocate memory to weights vector */
                factor_graph_robust.allocateMemoryForWeightings();

                /* get the residuals after first FGO */
                factor_graph_robust.getResiudalsOfFirstFGO(problem);

                /* update maximum residual */
                factor_graph_robust.updateMaxResidual();

                // /* update origin GNC_mu */
                factor_graph_robust.updateOriginGNCMu();

                // /* update origin GNC-GM weightings */
                factor_graph_robust.updateFirstGNCGMWeightings();

                // /* update GNC-GM iteratively until the GNC-GM converge */
                factor_graph_robust.updateGNCGMIteratively(problem, options, summary);

                // /* remove large outliers and re-solve */
                factor_graph_robust.removeLargeOutliersAndResolve(problem, options, summary);



                /* save graph state to variables */
                factor_graph_robust.saveGraphStateToVector();

                /* set reference point for ENU calculation */
                factor_graph_robust.setupReferencePoint();

                /* get the path in factor graph */
                FGOENULatest = factor_graph_robust.getLatestStateENU();

                /* publish the lastest state in factor graph */
                factor_graph_robust.printLatestStateENU();

                /* publish the path from FGO */
                fgo_path = factor_graph_robust.getPathENU(fgo_path);
                pub_global_path.publish(fgo_path);

                /* remove the data outside sliding window */
                factor_graph_robust.removeStatesOutsideSlidingWindow();

                /* log result */
                factor_graph_robust.logResults(logPath);

                /* free memory (done when finish all the data) */
                // factor_graph.freeStateMemory();

                hasNewData = false;

                /** */
                std::cout << "OptTime for solving batch data-> "<< OptTime.toc()<< std::endl;
            }
            m_gnss_raw_mux.unlock();
            std::chrono::milliseconds dura(10); // this thread sleep for 10 ms
            std::this_thread::sleep_for(dura);
        }
    }

    /**
   * @brief gnss raw msg and doppler msg callback
   * @param gnss raw msg and doppler msg
   * @return void
   @ 
   */
    void gnssraw_doppler_msg_callback(const nlosExclusion::GNSS_Raw_ArrayConstPtr& gnss_msg, const nav_msgs::OdometryConstPtr& doppler_msg)
    {
        m_gnss_raw_mux.lock();
        hasNewData = true;
        gnss_frame++;
        double time_frame = doppler_msg->pose.pose.position.x;
        /* save the  */
        if(checkValidEpoch(time_frame) && m_GNSS_Tools.checkRepeating(*gnss_msg))
        {
            if(gnss_msg->GNSS_Raws.size())
            {
                doppler_map[time_frame] = *doppler_msg;
                gnss_raw_map[time_frame] = *gnss_msg;

                factor_graph_robust.input_gnss_raw_data(*gnss_msg, time_frame);
                factor_graph_robust.input_doppler_data(*doppler_msg, time_frame);

                Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                        m_GNSS_Tools.getAllPositions(*gnss_msg),
                                        m_GNSS_Tools.getAllMeasurements(*gnss_msg),
                                        *gnss_msg, "WLS");
                Eigen::Matrix<double ,3,1> WLSENU;
                WLSENU = m_GNSS_Tools.ecef2enu(ENU_ref, eWLSSolutionECEF);
                LOG(INFO) << "WLSENU -> "<< std::endl << WLSENU;

                nav_msgs::Odometry odometry;
                odometry.header.frame_id = "map";
                odometry.child_frame_id = "map";
                odometry.pose.pose.position.x = WLSENU(0);
                odometry.pose.pose.position.y = WLSENU(1);
                odometry.pose.pose.position.z = WLSENU(2);
                pub_WLSENU.publish(odometry);
            }
        }
        
        /* release the lock */
        m_gnss_raw_mux.unlock();
    }


    ~psr_doppler_fusion_gnc_gm()
    {
    }
   
};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;  // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags 
    ros::init(argc, argv, "psr_doppler_fusion_gnc_gm_node"); 
    ROS_INFO("\033[1;32m----> psr_doppler_fusion_gnc_gm_node Started.\033[0m"); 
    ros::NodeHandle nh;
    // ...
    psr_doppler_fusion_gnc_gm psr_doppler_fusion_gnc_gm(nh);
    ros::spin();
    return 0;
}
