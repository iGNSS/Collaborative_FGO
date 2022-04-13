/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of fgo_gnss.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 * 
 * Main fucntions: pseudorange/Doppler tight fusion using factor graph optimization 
 * input: pseudorange, Doppler frequency from GPS/BeiDou.
 * output: position of the GNSS receiver 
 * Date: 2021/07/17
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
#include "../../include/gnss_estimator/psr_tcdoppler_sw.h"

// #include "../../include/gnss_estimator/psr_tcdoppler_sw.h"


// std::string  swo_fgo = "../global_fusion/swo_fgo.csv";
FILE* timeConsume  = fopen( "../timeConsume.csv", "w+");

FILE* PseudResi = fopen("../PseudResi.csv", "w+");

class psr_tcdoppler_swopriorfactor_fusion
{
    ros::NodeHandle nh;

    /* ros subscriber */
    ros::Publisher pub_WLSENU_0,pub_WLSENU_1,pub_WLSENU_2,pub_FGOENU_0,pub_FGOENU_1, pub_FGOENU_2,pub_global_path_0,pub_global_path_1,pub_global_path_2,pub_fgo_llh_0,pub_fgo_llh_1,pub_fgo_llh_2,pub_global_path_poly_0,pub_global_path_poly_1,pub_global_path_poly_2;
    std::map<double, nlosExclusion::GNSS_Raw_Array> gnss_raw_map_0,gnss_raw_map_1,gnss_raw_map_2;
    std::map<double, nav_msgs::Odometry> doppler_map_0,doppler_map_1,doppler_map_2;


   /*for data transfer station*/
    std::queue<std::pair<double, nlosExclusion::GNSS_Raw_Array>> gnss_raw_queen_0,gnss_raw_queen_1,gnss_raw_queen_2;
    std::queue<std::pair<double, nav_msgs::Odometry>> doppler_queen_0,doppler_queen_1,doppler_queen_2;

    GNSS_Tools m_GNSS_Tools; // utilities

    /* subscriber */
    std::unique_ptr<message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>> gnss_raw_array_sub_0,gnss_raw_array_sub_1,gnss_raw_array_sub_2;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> doppler_sub_0,doppler_sub_1,doppler_sub_2;
    std::unique_ptr<message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>> syncdoppler2GNSSRaw_0;
    std::unique_ptr<message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>> syncdoppler2GNSSRaw_1;
    std::unique_ptr<message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>> syncdoppler2GNSSRaw_2;
    /* thread lock for data safe */
    std::mutex m_gnss_raw_mux;

    /* thread for data processing */
    std::thread optimizationThread;

    int gnss_frame = 0;
    Eigen::Matrix<double, 3,1> ENU_ref;
    int slidingWindowSize = 100; // ? epoch measurements 150100000
    bool hasNewData = false;
    double prev_time_frame =0;

    /* latest state in ENU */
    Eigen::Matrix<double ,3,1> FGOENULatest_0,FGOENULatest_1,FGOENULatest_2;

    /* path in ENU */
    nav_msgs::Path fgo_path_0,fgo_path_1,fgo_path_2, fgo_path_poly_0,fgo_path_poly_1,fgo_path_poly_2;

    /* log path for FGO result */
    std::string logPath_0,logPath_1,logPath_2;

    /* log path for WLS result */
    std::string WLSLogPath_0,WLSLogPath_1,WLSLogPath_2, SDMPath;

    /* parameters for GNC infinite: convex; zero: non-convex */
    bool enableGNC = false;
    double GNC_mu = 1000000; 

    /* the maximum error you can accept (0.7) */
    double GNC_c = 2; 

    /* the maximum residual at first solving */
    double GNC_r_max = 0; 

    /* remove large outliers in GNC based on outlier weightings */
    bool removeLargeOurliers = true;

    /* apply the TDCP factor or not */
    bool enableTDCP = false;

    /* enable 3DMA */
    bool enable3DMA = false;

    bool enableMarginalPrior = false;

    bool enableResReject = false;

    bool saveBatchResult = false;

    /* velocity limit constraint via hint loss function */
    bool enableVelocityLimit = false;
    double VelocityLimit = 2.0;

    /* setup parameters for the Gaussian Mixture Models */
    bool enableGMM = false;
    int NumberOfComponents = 3;
    double DefaultStdDev = 0.7; // 1
    bool ReduceComponents = true;
    bool removeOffset = true;
    int GMMResidualWindowSize = 1000; // widow size for the resiudal of GMM

    /* enable the polyminal fitting or not */
    bool enablePolynominal = false;

    bool enableWSTuning = false;

    FactorGraph factor_graph;

    double lastOptTime = -1;

    
    

// private:
//     // std::unique_ptr<factor_graph> factor_graph_ptr_; // factor graph ptr
//     FactorGraph factor_graph;
    
    


public:
    psr_tcdoppler_swopriorfactor_fusion()
    {
        
        /* setup logpath */
        ros::param::get("logPath", logPath);
        /* clear the log file */
        // bool result = cleanLogFile();

        std::ofstream gnss_log_output(logPath, std::ios::out);
        gnss_log_output.close();

        std::cout << "here............... " << std::endl;

        ros::param::get("WLSLogPath", WLSLogPath);
        ros::param::get("SDMPath", SDMPath);

        /* setup GNC-GM parameters */
        nh.param("GNC_mu",     GNC_mu, 1000000.0);
        nh.param("GNC_c",      GNC_c, 2.0);
        nh.param("GNC_r_max",  GNC_r_max, 0.0);
        nh.param("removeLargeOurliers",  removeLargeOurliers, true);

        nh.param("enableTDCP",  enableTDCP, false);
        nh.param("enableGNC",   enableGNC, false);
        nh.param("enableGMM",   enableGMM, false);
        nh.param("enableMarginalPrior",   enableMarginalPrior, false);
        nh.param("enableResReject",       enableResReject, false);
        nh.param("saveBatchResult",       saveBatchResult, false);
        nh.param("enableVelocityLimit",   enableVelocityLimit, false);
        nh.param("VelocityLimit",         VelocityLimit, 2.0);

        nh.param("enablePolynominal",     enablePolynominal, false);   

        // if(enableGNC) enablePolynominal = true;     

        nh.param("slidingWindowSize",   slidingWindowSize, 100);

        nh.param("NumberOfComponents",      NumberOfComponents, 3);
        nh.param("DefaultStdDev",           DefaultStdDev, 0.7);
        nh.param("ReduceComponents",        ReduceComponents, true);
        nh.param("removeOffset",            removeOffset, true);
        nh.param("GMMResidualWindowSize",   GMMResidualWindowSize, 1000);

        
        nh.param("enable3DMA",  enable3DMA, false);

        nh.param("enableWSTuning",  enableWSTuning, false);

        

        

        std::ofstream foutWLS0(WLSLogPath_0, std::ios::out); // clean the file for the 1st receiver
        std::ofstream foutWLS1(WLSLogPath_1, std::ios::out); // clean the file for the 1st receiver
        std::ofstream foutWLS2(WLSLogPath_2, std::ios::out); // clean the file for the 1st receiver
        foutWLS0.close();
        foutWLS1.close();
        foutWLS2.close();
        std::cout << "folder path for save WLS result " << WLSLogPath_0<<" WLSLogPath_1:  " <<WLSLogPath_1<<"WLSLogPath_2:   "<<WLSLogPath_2<<std::endl;

        /* thread for factor graph optimization */
        optimizationThread = std::thread(&psr_tcdoppler_swopriorfactor_fusion::solveOptimization, this);
        
        pub_WLSENU_0 = nh.advertise<nav_msgs::Odometry>("WLSGoGPS_0", 100); // 1st receiver WLS result 
        pub_WLSENU_1 = nh.advertise<nav_msgs::Odometry>("WLSGoGPS_1", 100); // 2nd receiver WLS result
        pub_WLSENU_2 = nh.advertise<nav_msgs::Odometry>("WLSGoGPS_2", 100); // 3rd receiver WLS result

        pub_FGOENU_0 = nh.advertise<nav_msgs::Odometry>("FGO_0", 100); // 
        pub_FGOENU_1 = nh.advertise<nav_msgs::Odometry>("FGO_1", 100); // 
        pub_FGOENU_2 = nh.advertise<nav_msgs::Odometry>("FGO_2", 100); //  

        pub_fgo_llh_0 = nh.advertise<sensor_msgs::NavSatFix>("fgo_llh_0", 100);
        pub_fgo_llh_1 = nh.advertise<sensor_msgs::NavSatFix>("fgo_llh_1", 100);
        pub_fgo_llh_2 = nh.advertise<sensor_msgs::NavSatFix>("fgo_llh_2", 100);

        gnss_raw_array_sub_0.reset(new message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov1", 10000));
        gnss_raw_array_sub_1.reset(new message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov2", 10000));
        gnss_raw_array_sub_2.reset(new message_filters::Subscriber<nlosExclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov3", 10000));
        doppler_sub_0.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/gnss_preprocessor_node/GNSSDopVelRov1", 10000));
        doppler_sub_1.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/gnss_preprocessor_node/GNSSDopVelRov1", 10000));
        doppler_sub_2.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/gnss_preprocessor_node/GNSSDopVelRov1", 10000));
        syncdoppler2GNSSRaw_0.reset(new message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>(*gnss_raw_array_sub_0, *doppler_sub_0, 10000));
        syncdoppler2GNSSRaw_1.reset(new message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>(*gnss_raw_array_sub_1, *doppler_sub_1, 10000));
        syncdoppler2GNSSRaw_2.reset(new message_filters::TimeSynchronizer<nlosExclusion::GNSS_Raw_Array, nav_msgs::Odometry>(*gnss_raw_array_sub_2, *doppler_sub_2, 10000));

        syncdoppler2GNSSRaw->registerCallback(boost::bind(&psr_tcdoppler_swopriorfactor_fusion::gnssraw_doppler_msg_callback,this, _1, _2));

        pub_global_path = nh.advertise<nav_msgs::Path>("/FGOGlobalPath", 100); // 

        pub_global_path_poly = nh.advertise<nav_msgs::Path>("/FGOGlobalPathPoly", 100); // 
        
        /* reference point for ENU calculation */
        ENU_ref<< ref_lon, ref_lat, ref_alt;

        /* load results from 3DMA GNSS */
        if(enable3DMA)
            factor_graph.get3DMAData(SDMPath);

    }

    bool initializeFactorGraph(ceres::Solver::Options& options) 
    {
        
        /* initialize the factor graph size */
        factor_graph.setWindowSize(slidingWindowSize);

        /* set up ceres-solver options */
        factor_graph.setupSolverOptions(options);

        /* set up loss functions (Huber, Cauchy, NULL)*/
        factor_graph.setupLossFunction("Huber");

        /* set up loss functions (Huber, Cauchy, NULL) for psr */
        factor_graph.setupDopLossFunction("Huber");
        
        /* set up loss functions (Huber, Cauchy, NULL) for dop */
        factor_graph.setupPsrLossFunction("Huber");

        return true;
    }

    /* to clear the Log file path */
    bool cleanLogFile()
    {
        // clear output file
        std::ofstream gnss_log_output(logPath, std::ios::out);
        gnss_log_output.close();

        return true;
    }


    /**
	 * @brief perform factor graph optimization
	 * @param none
	 * @return none
	 */
    void solveOptimization()
    {
        /* clear data stream (only do once at the begining) */
        factor_graph.clearDataStream();

        factor_graph.initializeMarginalization();

        if(enableGMM)
        {
            /* clear variables of GMM */
            factor_graph.gaussianMixtureModel.clearGMMVariables();

            /* setup the Gaussian component numbers */
            factor_graph.gaussianMixtureModel.setNumberOfComponents(NumberOfComponents);

            /* setup the DefaultStdDev of Gaussian mixture */
            factor_graph.gaussianMixtureModel.setDefaultStdDev(DefaultStdDev);

            /* setup the ReduceComponents of Gaussian mixture */
            factor_graph.gaussianMixtureModel.setReduceComponents(ReduceComponents);

            /* setup the removeOffset of Gaussian mixture */
            factor_graph.gaussianMixtureModel.setremoveOffset(removeOffset);

            /* setup the GMMResidualWindowSize of Gaussian mixture */
            factor_graph.gaussianMixtureModel.setGMMResidualWindowSize(GMMResidualWindowSize);

            /* initialization of Gaussian mixture */
            factor_graph.gaussianMixtureModel.initializeGMM();
        }
        

        while(1)
        {
            m_gnss_raw_mux.lock();
            
            /*to reserve the data from publisher 
            into a vector, like a data transfer station*/
            while(factor_graph.getDataStreamSize()<=3 && gnss_raw_queen.size()>0)
            {
                factor_graph.input_gnss_raw_data(gnss_raw_queen.front().second, gnss_raw_queen.front().first);
                factor_graph.input_doppler_data(doppler_queen.front().second, doppler_queen.front().first);
                gnss_raw_queen.pop();
                doppler_queen.pop();
                std::cout<<"inputs the first three epochs data:-> "<<factor_graph.getDataStreamSize()<<std::endl;
            }
      
            // if(factor_graph.getDataStreamSize()>3 && hasNewData)
            if(factor_graph.getDataStreamSize()>3)
            {
                if(gnss_raw_queen.size()>0)
                {  
                    /* input new data only if last epoch is processed */
                    factor_graph.input_gnss_raw_data(gnss_raw_queen.front().second, gnss_raw_queen.front().first);
                    factor_graph.input_doppler_data(doppler_queen.front().second, doppler_queen.front().first);
                    gnss_raw_queen.pop();
                    doppler_queen.pop();
                    std::cout<<"Input one epoch GNSS data with timestamp at:-> "<<doppler_queen.front().first<<std::endl;
                }
                else
                {
                    std::cout << "No data in the doppler_queen-> \n " << std::endl;
                }

                bool dataReady = true;
                // if((saveBatchResult && factor_graph.gnss_raw_map.size()>1170) || (saveBatchResult==false)) 
                // {
                    
                //     dataReady = true;
                // }
                // else
                // {
                //     dataReady = false;
                //     std::cout<<"waiting for data factor_graph.gnss_raw_map.size():-> "<<factor_graph.gnss_raw_map.size()<<std::endl;
                // }
                

                if(dataReady)
                {
                    /* define the problem */
                    ceres::Problem problem;
                    ceres::Solver::Options options;
                    ceres::Solver::Summary summary;

                    /* start clock for factor graph optimization */
                    TicToc OptTime;

                    // std::cout << "here test ............ " << WLSLogPath << std::endl;

                    /* initialize factor graph */
                    // bool result = initializeFactorGraph(options);

                    /* initialize the factor graph size */
                    // slidingWindowSize++;
                    factor_graph.setWindowSize(slidingWindowSize);

                    /* set up ceres-solver options */
                    factor_graph.setupSolverOptions(options);

                    /* set up loss functions (Huber, Cauchy, NULL)*/
                    factor_graph.setupLossFunction("Huber");

                    /* set up loss functions (Huber, Cauchy, NULL) for psr */
                    factor_graph.setupDopLossFunction("Huber");
                    
                    /* set up loss functions (Huber, Cauchy, NULL) for dop */
                    factor_graph.setupPsrLossFunction("Huber");

                    // std::cout << "here test ............1 " << WLSLogPath << std::endl;

                    /* add the last marginalization */
                    factor_graph.addLastMarginalizationFactor(problem);

                    /* clear variables */
                    factor_graph.clearVariables();

                    

                    /* setup GNC-GM paras */
                    factor_graph.GNC_Geman_McClure.setGNCGMParas(GNC_mu, GNC_c, GNC_r_max);

                    /* get data stream size */
                    factor_graph.getDataStreamSize();

                    /* setup memory for state */
                    factor_graph.setupStateMemory();

                    

                    /* initialize factor graph parameters */
                    bool result  = factor_graph.initializeFactorGraphParas();

                    // std::cout << "here test ............1 "  << std::endl;

                    /* initialize the previous optimzied states */
                    factor_graph.initializeOldGraph();

                    /* initialize the newly added states */
                    factor_graph.initializeNewlyAddedGraph();

                    /* add parameter blocks */
                    factor_graph.addParameterBlocksToGraph(problem);

                    /* fix the first parameter block */
                    factor_graph.fixFirstState(false, problem);

                    

                    /* add Doppler FACTORS to factor graph (loosely) */
                    // factor_graph.addDopplerFactors(problem);

                    /* add TC Doppler FACTORS to factor graph */
                    factor_graph.addTCDopplerFactors(problem);

                    /* add TDCP FACTORS to factor graph */
                    if(enableTDCP)
                        factor_graph.addTDCPFactors(problem);

                    /* add constant receiver clock drift FACTORS to factor graph */
                    factor_graph.addConstantClockDriftFactors(problem);

                    /* add time-correlated receievr clock drift FACTORS to factor graph */
                    // factor_graph.addTimeCorrelatedClockDriftFactors(problem);

                    /* add moditon model factor FACTORS to factor graph */
                    // factor_graph.addMotionModelFactors(problem);
                    factor_graph.addAnaliticalMotionModelFactors(problem);

                    

                    /* add constant velocity factor to factor graph */
                    // factor_graph.addConstantVelFactors(problem);

                    /* add the velocity limit constraint */
                    if(enableVelocityLimit)
                    {
                        //VelocityLimit
                        factor_graph.addVelLimitFactors(problem, VelocityLimit);
                    }
                    

                    
                    

                    /* add acc model factor FACTORS to factor graph */
                    factor_graph.addAccModelFactors(problem);

                    /* add zero acc factor FACTORS to factor graph */
                    factor_graph.addZeroAccFactors(problem);

                    /* add jer model factor FACTORS to factor graph */
                    factor_graph.addJerModelFactors(problem);


                    if(enableGMM) //enableGMM
                    {
                        /* add pseudorange FACTORS to factor graph */
                        factor_graph.addPseudorangeFactorsForInitialization(problem);
                        /* solve the factor graph */
                        factor_graph.solveFactorGraph(problem, options, summary);
                        std::cout << "factor_graph.psrIDsTmp.size() " << factor_graph.psrIDsTmp.size() << std::endl;

                        ceres::Problem::EvaluateOptions EvalOpts;
                        EvalOpts.num_threads = 8;
                        EvalOpts.apply_loss_function = false;
                        EvalOpts.residual_blocks =factor_graph.psrIDsTmp;

                        std::vector<double> Residuals;
                        problem.Evaluate(EvalOpts,nullptr, &Residuals, nullptr, nullptr);

                        std::cout << "max residual is--------------------------------------> "<< *max_element(Residuals.begin(), Residuals.end()) << std::endl;

                        for(int i=0; i<factor_graph.psrIDsTmp.size();i++)
                        {
                            problem.RemoveResidualBlock(factor_graph.psrIDsTmp[i]);
                        }
                    }



                    if(enableGNC)
                    {
                        /* add GNC aided pseudorange FACTORS to factor graph */
                        factor_graph.addPseudorangeFactorsViaGNC(problem);
                    }
                    else if(enableGMM)
                    // else if(0)
                    {
                        /* add pseudorange FACTORS to factor graph */
                        factor_graph.addPseudorangeFactorsGMM(problem);
                    }
                    else 
                    {
                        /* add pseudorange FACTORS to factor graph */
                        factor_graph.addPseudorangeFactors(problem);
                    }

                    if(enableMarginalPrior)
                    {
                        /* add prior positioning factor */
                        factor_graph.addPriorFactors(problem);
                    }

                    // std::cout << "here test ............0 " << WLSLogPath << std::endl;
                    if(enablePolynominal)
                    {
                        /* add prior positioning factor from polynomial estimation */
                        bool result = factor_graph.addPolyPosePriorFactors(problem);
                    }

                    // std::cout << "here test ............1 " << WLSLogPath << std::endl;

                    if(enable3DMA)
                        factor_graph.add3DMAFactors(problem);

                    /* solve the factor graph */
                    factor_graph.solveFactorGraph(problem, options, summary);

                    /* allocate the memory for the weightings first */
                    if(enableGNC) 
                    {
                        /* initialize the satNumPerT */
                        factor_graph.initializeSatNumPerT();

                        /* allocate memory to weights vector */
                        factor_graph.allocateMemoryForWeightings();
                    }

                    /* Remove residual larger than 300, to avoid unnecessary iterations in GNC */
                    if(enableResReject) // outlier removal based on residuals
                    {
                        /*save measurements residuals*/
                        //  pseudorange residuals
                        ceres::Problem::EvaluateOptions EvalOpts;
                        EvalOpts.num_threads = 8;
                        EvalOpts.apply_loss_function = false;
                        EvalOpts.residual_blocks =factor_graph.psrIDs;
                        std::cout << "factor_graph.psrIDs.size()-> " << factor_graph.psrIDs.size() <<"\n";

                        std::vector<double> Residuals;
                        problem.Evaluate(EvalOpts,nullptr, &Residuals, nullptr, nullptr);
                        std::string residualPath;
                        residualPath = residualPath + "../FGO-GNSSV3.0/src/result/residuals/";

                        // std::cout<<"time-> " << factor_graph.state_gps_sec_vec[factor_graph.state_gps_sec_vec.size()-1] << std::endl;
                        double time = factor_graph.state_gps_sec_vec[factor_graph.state_gps_sec_vec.size()-1];

                        residualPath = residualPath + std::to_string(int(time)) + ".csv";
                        std::ofstream foutC(residualPath, std::ios::ate);
                        foutC.setf(std::ios::fixed, std::ios::floatfield);

                        int numberRemoval = 0;
                        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator itR;
                        itR = gnss_raw_map.begin();

                        int length = factor_graph.measSize;
                        int pr_cnt = -1;
                        std::vector<ceres::ResidualBlockId> psrIDsTmp;
                        for(int i = 0;  i < length; i++) // 
                        {
                            nlosExclusion::GNSS_Raw_Array gnssDataTmp;
                            for(int j = 0; j < factor_graph.satNumPerT[i]; j++)
                            {
                                pr_cnt++;

                                /* remove the residual block from the vector */
                                if(Residuals.at(pr_cnt)>300)
                                {
                                    numberRemoval++;
                                    problem.RemoveResidualBlock(factor_graph.psrIDs[pr_cnt]);
                                    // itR->second.GNSS_Raws.erase(itR->second.GNSS_Raws.begin()+j);
                                    factor_graph.satNumPerT[i]--;
                                }
                                else // free of rejection
                                {
                                    psrIDsTmp.push_back(factor_graph.psrIDs[pr_cnt]);
                                    foutC.precision(10);
                                    gnssDataTmp.GNSS_Raws.push_back(itR->second.GNSS_Raws[j]);

                                    foutC<<Residuals.at(pr_cnt)<<std::endl;
                                }
                            }
                            itR->second = gnssDataTmp; // updtae the GNSS data 
                            std::cout << "gnssDataTmp.GNSS_Raws.size()-> " << gnssDataTmp.GNSS_Raws.size() <<"\n";
                            std::cout << "factor_graph.satNumPerT[i]-> " << factor_graph.satNumPerT[i] <<"\n";

                            ++itR;
                        }
                        factor_graph.psrIDs.clear();
                        factor_graph.psrIDs = psrIDsTmp;

                        foutC.close();
                        // std::cout << "numberRemoval-> " << numberRemoval <<"\n";
                        
                        /* solve the factor graph */
                        factor_graph.solveFactorGraph(problem, options, summary);
                    }
                    

                    /* GNC for outlier rejection */
                    if(enableGNC)
                    {

                        /* get the residuals after first FGO */
                        factor_graph.getResiudalsOfFirstFGO(problem);

                        /* update maximum residual */
                        factor_graph.updateMaxResidual();

                        /* update origin GNC_mu */
                        factor_graph.updateOriginGNCMu();

                        /* update origin GNC-GM weightings */
                        // factor_graph.updateFirstGNCGMWeightings();

                        /* update GNC-GM iteratively until the GNC-GM converge */
                        factor_graph.updateGNCGMIteratively(problem, options, summary);

                        /* update weightings to GNSS RAW data until the GNC-GM converge */
                        factor_graph.updateWeightingsInGNSSRawData();
                        
                        /* remove large outliers (based on GNC weightings) and re-solve */
                        if(removeLargeOurliers)
                            factor_graph.removeLargeOutliersAndResolve(problem, options, summary);
                    }

                    /* solve the factor graph again */
                    // factor_graph.solveFactorGraph(problem, options, summary);
                    
                    /* add the new measurements into marginalization factor */
                    // factor_graph.addMeasurementsToMarginalization();

                    /* if the GMM is enabled */
                    if(enableGMM || enableWSTuning)
                    // if(enableGMM)
                    {
                        /* update GMM residuals */
                        // factor_graph.updateResidualsOfGMM(problem);

                        // better in KLT dataset
                        factor_graph.updateResidualsOfGMM_v2(problem);

                        // /* estimate GMM with GM */
                        factor_graph.estimateGMMViaEM();
                    }

                    // enable sliding window tuning
                    if(enableWSTuning)
                    {
                        //lastOptTime
                        factor_graph.updateWS(lastOptTime);
                    }
                    
                
                    /* save graph state to variables */
                    factor_graph.saveGraphStateToVector();

                    /* set reference point for ENU calculation */
                    factor_graph.setupReferencePoint();

                    /* get the path in factor graph */
                    FGOENULatest = factor_graph.getLatestStateENU();

                    /* publish the lastest state in factor graph */
                    factor_graph.printLatestStateENU();

                    /* publish the path from FGO */
                    fgo_path = factor_graph.getPathENU(fgo_path);
                    pub_global_path.publish(fgo_path);

                    /* estimate the polyminal model */
                    if(enablePolynominal)
                    {
                        /* get the ENU trajectory */
                        factor_graph.getENUTrajectory();

                        factor_graph.getNormalizedResiduals(problem);

                        /* assign the trajectory */
                        factor_graph.inputTrajectory();

                        /* optimize the polyminal model paras */
                        factor_graph.polynomial_estimation.optimizePolyParas();

                        fgo_path_poly = factor_graph.getPathENUPoly(fgo_path_poly);
                        pub_global_path_poly.publish(fgo_path_poly);
                    }
                
                    std::cout<<"the GNSS queue size is"<<gnss_raw_queen.size()<<std::endl;
                    std::cout<<"the Doppler queue size is"<<doppler_queen.size()<<std::endl;

                    /* remove the data outside sliding window */
                    factor_graph.removeStatesOutsideSlidingWindow();

                    /* log result */
                    if(!saveBatchResult) // sliding window optimization results
                        factor_graph.logResults(logPath);
                    else // batch results
                    {
                        /* code */
                        factor_graph.logBatchResults(logPath);
                    }
                        

                    /*reserve the solved solution from current sliding window
                    * this is for the prior factor which is an approximation of the marginalization. 
                    */
                    factor_graph.saveStatesInCurrentSlidingWindow();

                    /* free memory (done when finish all the data) */
                    // factor_graph.freeStateMemory();

                    // hasNewData = false;

                    /** */
                    std::cout << "Time for factor graph optimization-> "<< OptTime.toc()/1000<< " seconds\n"<<std::endl;

                    lastOptTime = OptTime.toc();

                    // save the time consumption to csv file
                    fprintf(timeConsume, "%5.2f  \n", OptTime.toc());
                    fflush(timeConsume); // make sure the data is saved
                }

                
            }
            m_gnss_raw_mux.unlock();
            std::chrono::milliseconds dura(1); // this thread sleep for 10 ms
            std::this_thread::sleep_for(dura);

            // may change as data change
        //     if(doppler_queen.front().first >118509) 
        //    {
        //        std::cout<<"finally the queue size is ---->"<<doppler_queen.size()<<std::endl;
        //       while(1);
        //    } 
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
             
            if(gnss_msg->GNSS_Raws.size()&& prev_time_frame <time_frame)
            {
                doppler_map[time_frame] = *doppler_msg;
                gnss_raw_map[time_frame] = *gnss_msg;

               /*push the  data to the queue*/
                gnss_raw_queen.push(std::make_pair(time_frame,*gnss_msg));
                doppler_queen.push(std::make_pair(time_frame,*doppler_msg));
                prev_time_frame= time_frame;
               /*reserve data to map*/
                // factor_graph.input_gnss_raw_data(*gnss_msg, time_frame);
                // factor_graph.input_doppler_data(*doppler_msg, time_frame);

                Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                        m_GNSS_Tools.getAllPositions(*gnss_msg),
                                        m_GNSS_Tools.getAllMeasurements(*gnss_msg),
                                        *gnss_msg, "WLS");
                Eigen::Matrix<double ,3,1> WLSENU;
                WLSENU = m_GNSS_Tools.ecef2enu(ENU_ref, eWLSSolutionECEF);
                // std::cout << "eWLSSolutionECEF-> " << eWLSSolutionECEF << std::endl;
                // LOG(INFO) << "eWLSSolutionECEF -> "<< std::endl << WLSENU;
                // LOG(INFO) << "doppler_map.size() -> "<< std::endl << doppler_map.size();

                
                std::ofstream foutC(WLSLogPath, std::ios::app);
                foutC.setf(std::ios::fixed, std::ios::floatfield);
                
                /* gps time */
                foutC.precision(0);             
                foutC<<time_frame<<",";
                foutC<<time_frame<<",";

                /* longitude, latitude and altitude */
                Eigen::Matrix<double,3,1> llh = m_GNSS_Tools.ecef2llh(eWLSSolutionECEF);
                foutC.precision(10);
                foutC<<llh(1)<<",";
                foutC<<llh(0)<<",";
                foutC<<llh(2)<<std::endl;

                foutC.close();

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


    ~psr_tcdoppler_swopriorfactor_fusion()
    {
    }
   
};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;  // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags 
    ros::init(argc, argv, "psr_tcdoppler_swopriorfactor_fusion_node"); 
    ROS_INFO("\033[1;32m----> psr_tcdoppler_swopriorfactor_fusion_node Started.\033[0m"); 
    // ...
    psr_tcdoppler_swopriorfactor_fusion psr_tcdoppler_swopriorfactor_fusion_;
    ros::spin();
    return 0;
}
