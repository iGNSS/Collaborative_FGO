/*******************************************************
 * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of fgo_gnss.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 *******************************************************/

#define D2R 3.1415926/180.0
#define Doppler2PSRWeight 10 // original is 10 5
#define Car2PSRWeight 50 // original is 20 100
 
#define setUpperLowerBound 0
#include <nlosExclusion/GNSS_Raw_Array.h>
// google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>
#include "../gnss_tools.h"

#include "pseudorange_factor.h"
#include "doppler_factor.hpp"
#include "polynominalEstimation.hpp"

#include "../robust_models/gnc_gm.h"
#include "../gnss_estimator_robust/pseudorange_factor_robust.h"

#include "MarginalizationFactor.h"

/* head files for Gaussian mixture models */
#include "../robust_models/SumMixture.h"
#include "../robust_models/GaussianMixture.h"
#include "../robust_models/GaussianComponent.h"
#include "../robust_models/gmm.h"
#include "../gnss_estimator_robust/pseudorange_factor_gmm.h"


GNSS_Tools m_GNSS_Tools; // utilities

#define stateSize 5 + 1 // x,y,z,clock_gps, clock_BSD, receiver clock drift 

#define velStateSize 3 // vel x, vel y, vel z

#define  enable_save_for_batch_data 1

class FactorGraph{
public:
    /* continuous data stream */
    std::map<double, nlosExclusion::GNSS_Raw_Array> gnss_raw_map_0,gnss_raw_map_1,gnss_raw_map_2;
    std::map<double, nav_msgs::Odometry> doppler_map_0,doppler_map_1,doppler_map_2;
    std::vector<ceres::ResidualBlockId> PseudIDs_0,PseudIDs_1,PseudIDs_2;
    std::vector<double> PseudVars_0,PseudVars_0,PseudVars_2;
    std::vector<ceres::ResidualBlockId>DoppIDs_0,DoppIDs_1,DoppIDs_2; 
    std::vector<ceres::ResidualBlockId>PriorIDs_0,PriorIDs_1,PriorIDs_2; 
    /* Ceres solver object */
    // ceres::Problem problem;
    // ceres::Solver::Options options;
    // ceres::Solver::Summary summary;
    ceres::LossFunction *psr_loss_function_0,*psr_loss_function_1,*psr_loss_function_2;
    ceres::LossFunction *dop_loss_function_0,*dop_loss_function_1,*dop_loss_function_2;
    ceres::LossFunction *loss_function_0,*loss_function_1,*loss_function_2;

    /* size of factor graph */
    int sizeOfFactorGraph = 0;

    struct States_0 
    {  
    /* state saved in vector pair */
        Eigen::Vector3d position;
        Eigen::Vector3d clocks;
        Eigen::Vector3d velocity;
    } states_0;

    struct States_1 
    {  
    /* state saved in vector pair */
        Eigen::Vector3d position;
        Eigen::Vector3d clocks;
        Eigen::Vector3d velocity;
    } states_1;

    struct States_2 
    {  
    /* state saved in vector pair */
        Eigen::Vector3d position;
        Eigen::Vector3d clocks;
        Eigen::Vector3d velocity;
    } states_2;

    /*reserve the solved solution from current sliding window*/
    std::map<double, States> slidingwindow_map_0,slidingwindow_map_1,slidingwindow_map_2;
    std::map<double, States>::iterator iter_0,iter_1,iter_2;
   

    /* state array of factor graph */
    std::vector<double*> state_array_0,state_array_1,state_array_2, last_state_array_0,last_state_array_1,last_state_array_2;

    std::vector<double*> state_array_vel_0,state_array_vel_1,state_array_vel_2;// velocity in ECEF

    std::vector<double*> state_array_acc_0,state_array_acc_1,state_array_acc_2;// acceleration in ECEF

    std::vector<double*> state_array_jer_0,state_array_jer_1,state_array_jer_2;// jerk in ECEF

    std::vector<int> satNumPerT_0,satNumPerT_1,satNumPerT_2;

    int* gps_sec_array_0;
    int* gps_sec_array_1;
    int* gps_sec_array_2;

    std::vector<int> state_gps_sec_vec_0,state_gps_sec_vec_1,state_gps_sec_vec_2;

    /* apply weighting when state is initialized via FGO */
    bool* initialized;

    /* state saved in vector pair */
    std::vector<std::pair<double, Eigen::Vector3d>> Ps_0,Ps_1,Ps_2;
    std::vector<std::pair<double, Eigen::Vector2d>> Clocks_0,Clocks_1,Clocks_2;

    /* size of the last factor graph optimization */
    int lastFactorGraphSize = 0;

    /* fixed variance of doppler measurements */
    double var = 0.6;
    // double var = 1.6;

    /* reference point for ENU calculation */
    Eigen::MatrixXd ENULlhRef;

    /* measurements size */
    int measSize = 0;

    /* parameters */
    int numOfPsrFactors = 0;
    int numOfDopplerFactors = 0;
    int numOfStates =0;

    /* Marginalization */
    MarginalizationInfo *last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;
    bool marg = true;

    /* setup GNC-GM-based robuster */
    GNCGemanMcClure GNC_Geman_McClure;

    /* Gaussian mixture model */
    GaussianMixtureModel gaussianMixtureModel;

    /* vector to save factor ids of pseudorange */
    std::vector<ceres::ResidualBlockId> psrIDs_0,psrIDs_1,psrIDs_2;

    std::vector<ceres::ResidualBlockId> psrIDsTmp_0,psrIDsTmp_1,psrIDsTmp_2;

    double latestRecCloDrift = 0;

    /** save the trajectory for polynomial models*/
    std::vector<Eigen::Matrix<double ,3,1>> ENUTrajectory_0,ENUTrajectory_1,ENUTrajectory_2;
    std::vector<double> normalizedResiduals;
    PolynomialEstimation polynomial_estimation;
    Eigen::Matrix<double ,3,1> posePrediction_0,posePrediction_1,posePrediction_2;
    // posePrediction.setZero();
    // posePrediction(0,0) = 0;
    // posePrediction(1) = 0;
    // posePrediction(2) = 0;
    // posePrediction << 0,0,0;

    struct data3dma_
    {
        double timestamp;
        double lati; // weidu
        double longti; //jingdu
        double alti; //gaodu
    } data3dma;

    std::map<int, data3dma_> data3dma_map;
    bool add3DMA = false;
    double SV=0;
    std::vector<ceres::ResidualBlockId>amd3IDs; 

    double lastfeedback = 0.0;



public:

    /* initialize the marginalization variable */
    bool initializeMarginalization()
    {
        /* marginalization */
        last_marginalization_info = nullptr;
        return true;
    }

    /* add the marginalization factor */
    bool addLastMarginalizationFactor(ceres::Problem& problem)
    {
        if(true) {
            if (last_marginalization_info) {
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                problem.AddResidualBlock(marginalization_factor, NULL,
                                            last_marginalization_parameter_blocks);
            }
        }
        return true;
    }

    /* initialize the marginalization variable */
    bool addMeasurementsToMarginalization()
    {
        if(gnss_raw_map.size() - sizeOfFactorGraph >0) return false;
        
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();

        if (last_marginalization_info) {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                if (last_marginalization_parameter_blocks[i] == state_array[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor

            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);

            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->AddResidualBlockInfo(residual_block_info);
        }

        /* add the marginalization for the pseudorange factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;
        iter_pr = gnss_raw_map.begin();
        int length = measSize;
        // for(int m = 0;  m < length; m++,iter_pr++) // 
        {
            nlosExclusion::GNSS_Raw_Array gnss_data = (iter_pr->second);
            MatrixXd weight_matrix; //goGPS weighting
            weight_matrix = m_GNSS_Tools.cofactorMatrixCal_WLS(gnss_data, "WLS"); //goGPS
            int sv_cnt = gnss_data.GNSS_Raws.size();
            double t = gnss_data.GNSS_Raws[0].GNSS_time;

            int factor_index = -1;

            std::cout<< "sv_cnt-> " << sv_cnt <<std::endl;  
            for(int i =0; i < sv_cnt; i++)
            {
                factor_index++;

                std::string sat_sys;
                double s_g_x = 0, s_g_y = 0,s_g_z = 0, var = 1;
                double pseudorange = 0;
                if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[i].prn_satellites_index)) sat_sys = "GPS";
                else sat_sys = "BeiDou";

                s_g_x = gnss_data.GNSS_Raws[i].sat_pos_x;
                s_g_y = gnss_data.GNSS_Raws[i].sat_pos_y;
                s_g_z = gnss_data.GNSS_Raws[i].sat_pos_z;

                pseudorange = gnss_data.GNSS_Raws[i].pseudorange;

                double ele = gnss_data.GNSS_Raws[i].elevation;
                double snr = gnss_data.GNSS_Raws[i].snr;

                if(snr>15) // simple thresholds to exlude the outlier 28
                { 
                    ceres::CostFunction* ps_function = new ceres::AutoDiffCostFunction<pseudorangeFactor, 1 
                                                                , 6>(new 
                                                                pseudorangeFactor(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1/weight_matrix(i,i))));
                    vector<double*> tmp;
                    tmp.push_back(state_array[0]);

                    vector<int> drop_set;
                    drop_set.push_back(0);

                    ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(ps_function, psr_loss_function,tmp,drop_set);
                    marginalization_info->AddResidualBlockInfo(residual_block_info);
                }
            }            
        }

        if (1) {
            marginalization_info->PreMarginalize();
            marginalization_info->Marginalize();
        }
        

        /* shift for the states to be optimized */
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i < measSize; ++i) {
            addr_shift[reinterpret_cast<long>(state_array[i])] = state_array[i-1];
        }

        vector<double *> parameter_blocks = marginalization_info->GetParameterBlocks(addr_shift);


        if (last_marginalization_info) {
            delete last_marginalization_info;
        }
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;


        /* window size of 10 */
        // if(last_marginalization_parameter_blocks.size() > slide_window_width + 9) {
        //     last_marginalization_parameter_blocks[last_marginalization_parameter_blocks.size() - slide_window_width - 10] = nullptr;
        // }


        return true;
    }

    /* input gnss doppler data */
    bool clearVariables()
    {
        // posePrediction.setZero();

        state_gps_sec_vec.clear();

        psrIDs.clear();
        satNumPerT.clear();

        initialized = new bool[1]; //
        *initialized = false;

        return true;
    }   

    /* input gnss raw (pseudorange/carrier-phase) data  */
    bool input_gnss_raw_data(nlosExclusion::GNSS_Raw_Array GNSS_data, double timestamp)
    {
        if(timestamp<0) return false;
        else 
        {
            gnss_raw_map[timestamp] = GNSS_data;
            return true;
        }
    }

    /* input Doppler data  */
    bool input_doppler_data(nav_msgs::Odometry dopplerData, double timestamp)
    {
        if(timestamp<0) return false;
        else 
        {
            doppler_map[timestamp] = dopplerData;
            return true;
        }
    }

    /* input gnss doppler data */
    bool setWindowSize(int windowSize)
    {
        sizeOfFactorGraph = windowSize;
        return true;
    }

    /* clear data stream */
    bool clearDataStream()
    {
        gnss_raw_map_0.clear();
        gnss_raw_map_1.clear();
        gnss_raw_map_2.clear();
        doppler_map_0.clear();
        doppler_map_1.clear();
        doppler_map_2.clear();
        return true;
    }

    /* input 3DMA data  */   
     void get3DMAData(std::string Path)
     {
         FILE* solutionFile;
         /* the path need change as need */

         solutionFile=std::fopen((Path).c_str(),"r");

         std::cout<<"Path"<<Path<<std::endl;

         if(solutionFile==NULL)
         {
            printf("cannot find the file , please try again \n");
            ROS_BREAK();
         }

        int timestamp;
        double lati; // weidu
        double longti; //jingdu
        double alti; //gaodu
        char line[1024];
        
        if(!data3dma_map.size())
        {
            while((fscanf(solutionFile, "%[^\n]", line)) != EOF)// read data in rows
            {
                fgetc(solutionFile);
                // printf("Line=%s \n",line);
                std::stringstream ss(line); 
                std::vector<string> result; 
                while(ss.good())
                {
                    // atoi(ss.str().c_str());//change to int format  from stringstream
                    string substr;
                    getline(ss, substr, ',');
                    result.push_back(substr);
                    std::cout << std::setprecision(17);
                }

                data3dma.timestamp=strtod((result[1]).c_str(), NULL); //c_str()是将 stringstream 类型转换为 string 类型
                data3dma.lati=strtod((result[2]).c_str(), NULL);
                data3dma.longti=strtod((result[3]).c_str(), NULL);
                data3dma.alti=strtod((result[4]).c_str(), NULL);

                std::cout<<"see it data correctly read"<<data3dma.timestamp<<std::endl;
                std::cout<<"see it data correctly read"<<data3dma.lati<<std::endl;
                std::cout<<"see it data correctly read"<<data3dma.longti<<std::endl;
                std::cout<<"see it data correctly read"<<data3dma.alti<<std::endl;
                
                data3dma_map[int(data3dma.timestamp)] = data3dma;
                std::cout << std::setprecision(17);
            }
        }
          std::fclose(solutionFile);
     }


    /* set up ceres-solver options */
    bool setupSolverOptions(ceres::Solver::Options& options)
    {
        options.use_nonmonotonic_steps = true;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        // options.linear_solver_type = ceres::DENSE_QR;

        options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
        options.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
        options.num_threads = 8;
        options.max_num_iterations = 258;
        return true;
    }

    /* set up Loss functions options for pseudorange factor */
    bool setupPsrLossFunction(std::string loss)
    {
        if(loss=="Huber")
            psr_loss_function = new ceres::HuberLoss(1.0);
        else if(loss=="Cauchy")
        {
            psr_loss_function = new ceres::CauchyLoss(1.0);
        }
        else
        {
            psr_loss_function = NULL;
        }
        
        return true;
    }

    /* set up Loss functions options for dop factor */
    bool setupDopLossFunction(std::string loss)
    {
        if(loss=="Huber")
            dop_loss_function = new ceres::HuberLoss(1.0);
        else if(loss=="Cauchy")
        {
            dop_loss_function = new ceres::CauchyLoss(1.0);
        }
        else
        {
            dop_loss_function = NULL;
        }
        
        return true;
    }

    /* set up Loss functions options for dop factor */
    bool setupLossFunction(std::string loss)
    {
        if(loss=="Huber")
            loss_function = new ceres::HuberLoss(1.0);
        else if(loss=="Cauchy")
        {
            loss_function = new ceres::CauchyLoss(1.0);
        }
        else
        {
            loss_function = NULL;
        }
        
        return true;
    }

    /* get data stream size */  
    int getDataStreamSize()
    {
        measSize = gnss_raw_map_0.size();
        return measSize;
    }

    bool initializeFactorGraphParas()
    {
        numOfPsrFactors = 0;
        numOfDopplerFactors = 0;
        numOfStates =0;

        return true;
    }

    /* setup state size */
    bool setupStateMemory()
    {
        // measSize = 100;
        state_array.reserve(measSize);
        state_array_vel.reserve(measSize);
        state_array_acc.reserve(measSize);
        state_array_jer.reserve(measSize);
        int length = measSize;
        // LOG(INFO) << "length" << length << std::endl;

        for(int i = 0; i < length;i++)
        {
            /* x, y, z, gps_cloc_bias, beidou_cloc_bias */
            state_array[i] = new double[stateSize]; //

            state_array_vel[i] = new double[velStateSize]; //

            state_array_acc[i] = new double[velStateSize]; //

            state_array_jer[i] = new double[velStateSize]; //
        }
        // gps_sec_array = new int[length];

        return true;
    }

    /* initialize the previous optimzied states */
    bool initializeOldGraph()
    {
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;
        iter_pr = gnss_raw_map.begin();
        int length = measSize;

        /* tranverse the stateArray */
        for(int m = 0;  m < length; m++,iter_pr++) // 
        {
            nlosExclusion::GNSS_Raw_Array gnss_data = (iter_pr->second);
            double time = gnss_data.GNSS_Raws[0].GNSS_time;

            /* initialize the state using previously optimized state */
            for(int i = 0; i < Ps.size(); i++)
            {
                if(time == Ps[i].first)
                {
                    state_array[m][0] = Ps[i].second[0];
                    state_array[m][1] = Ps[i].second[1];
                    state_array[m][2] = Ps[i].second[2];
                    state_array[m][3] = Clocks[i].second[0];
                    state_array[m][4] = Clocks[i].second[1];

                    // state_array[m][0] = 0;
                    // state_array[m][1] = 0;
                    // state_array[m][2] = 0;
                    // state_array[m][3] = 0;
                    // state_array[m][4] = 0;

                    state_array[m][5] = 0;

                    state_array_vel[m][0] = 0;
                    state_array_vel[m][1] = 0;
                    state_array_vel[m][2] = 0;

                    state_array_acc[m][0] = 0;
                    state_array_acc[m][1] = 0;
                    state_array_acc[m][2] = 0;

                    
                    state_array_jer[m][0] = 0;
                    state_array_jer[m][1] = 0;
                    state_array_jer[m][2] = 0;
                }
            } 
        }
        return true;
    }

    /* initialize the newly added state using WLS*/
    bool initializeNewlyAddedGraph()
    {
        int length = measSize;
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter;
        iter = gnss_raw_map.begin();
        for(int i = 0; i <length; i++,iter++)
        {
            if(i>=(lastFactorGraphSize-1))
            {
                nlosExclusion::GNSS_Raw_Array gnss_data = (iter->second);
                Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                    m_GNSS_Tools.getAllPositions(gnss_data),
                                    m_GNSS_Tools.getAllMeasurements(gnss_data),
                                    gnss_data, "WLS");
                // state_array[i][0] = -2417129;
                // state_array[i][1] = 5385879;
                // state_array[i][2] = 2406596;
                // state_array[i][3] = 24;
                // state_array[i][4] = 23;

                state_array[i][0] = 0;
                state_array[i][1] = 0;
                state_array[i][2] = 0;
                state_array[i][3] = 0;
                state_array[i][4] = 0;

                // state_array[i][0] = eWLSSolutionECEF(0);
                // state_array[i][1] = eWLSSolutionECEF(1);
                // state_array[i][2] = eWLSSolutionECEF(2);
                // state_array[i][3] = eWLSSolutionECEF(3);
                // state_array[i][4] = eWLSSolutionECEF(4);

                std::cout<< "eWLSSolutionECEF(3)-> " << eWLSSolutionECEF(3) << std::endl;
                std::cout<< "eWLSSolutionECEF(4)-> " << eWLSSolutionECEF(4) << std::endl;

                // state_array[i][0] = 0;
                // state_array[i][1] = 0;
                // state_array[i][2] = 0; 
                // state_array[i][3] = 0;
                // state_array[i][4] = 0;

                state_array[i][5] = 0; // receiver clock drift rate

                state_array_vel[i][0] = 0; 
                state_array_vel[i][1] = 0;
                state_array_vel[i][2] = 0;

                state_array_acc[i][0] = 0;
                state_array_acc[i][1] = 0;
                state_array_acc[i][2] = 0;

                state_array_jer[i][0] = 0;
                state_array_jer[i][1] = 0;
                state_array_jer[i][2] = 0;
            }
        }
        return true;
    }

    /* add parameter blocks */
    bool addParameterBlocksToGraph(ceres::Problem& problem)
    {
        int length = measSize;
        for(int i = 0; i <length; i++)
        {
            problem.AddParameterBlock(state_array[i],stateSize);
            problem.AddParameterBlock(state_array_vel[i],velStateSize);
            problem.AddParameterBlock(state_array_acc[i],velStateSize);
            problem.AddParameterBlock(state_array_jer[i],velStateSize);
            
            #if setUpperLowerBound
            double velBound = 0.8;
            double accBound = 0.3;
            double jerBound = 0.2;

            /* setup upper bound of vel */
            problem.SetParameterUpperBound(state_array_vel[i], 0, velBound);
            problem.SetParameterUpperBound(state_array_vel[i], 1, velBound);
            problem.SetParameterUpperBound(state_array_vel[i], 2, velBound);

            /* setup lower bound of vel */ 
            problem.SetParameterLowerBound(state_array_vel[i], 0, -velBound);
            problem.SetParameterLowerBound(state_array_vel[i], 1, -velBound);
            problem.SetParameterLowerBound(state_array_vel[i], 2, -velBound);

            /* setup upper bound of acc */
            problem.SetParameterUpperBound(state_array_acc[i], 0, accBound);
            problem.SetParameterUpperBound(state_array_acc[i], 1, accBound);
            problem.SetParameterUpperBound(state_array_acc[i], 2, accBound);

            /* setup lower bound of acc */ 
            problem.SetParameterLowerBound(state_array_acc[i], 0, -accBound);
            problem.SetParameterLowerBound(state_array_acc[i], 1, -accBound);
            problem.SetParameterLowerBound(state_array_acc[i], 2, -accBound);

            /* setup upper bound of jer */
            problem.SetParameterUpperBound(state_array_jer[i], 0, jerBound);
            problem.SetParameterUpperBound(state_array_jer[i], 1, jerBound);
            problem.SetParameterUpperBound(state_array_jer[i], 2, jerBound);
            
            /* setup lower bound of jer */
            problem.SetParameterLowerBound(state_array_jer[i], 0, -jerBound);
            problem.SetParameterLowerBound(state_array_jer[i], 1, -jerBound);
            problem.SetParameterLowerBound(state_array_jer[i], 2, -jerBound);
            #endif
        }
        return true;
    }

    /* fix the first parameter block */
    bool fixFirstState(bool flag, ceres::Problem& problem)
    {
        if(flag)
            problem.SetParameterBlockConstant(state_array[0]);
        return true;
    }

    /* add Doppler FACTORS */ 
    bool addDopplerFactors(ceres::Problem& problem)
    {
        /* process doppler measurements */
        std::map<double, nav_msgs::Odometry>::iterator iterdopp, iterdoppNext;
        int i = 0;
        for(iterdopp = doppler_map.begin(); iterdopp != doppler_map.end();iterdopp++, i++)
        {
            /* add doppler measurements */
            iterdoppNext = iterdopp;
            iterdoppNext ++;
            if(iterdoppNext != doppler_map.end())
            {
                double delta_t = iterdoppNext->first - iterdopp->first;
                double v_x_i = iterdopp->second.twist.twist.linear.x;
                double v_y_i = iterdopp->second.twist.twist.linear.y;
                double v_z_i = iterdopp->second.twist.twist.linear.z;
                double dop_var_scal = 0.06 * 5;
                double var_x = dop_var_scal * sqrt(iterdopp->second.twist.covariance[0]);
                double var_y = dop_var_scal * sqrt(iterdopp->second.twist.covariance[1]);
                double var_z = dop_var_scal * sqrt(iterdopp->second.twist.covariance[2]);               
                
                Eigen::Vector3d var_vec(var,var,var);
                ceres::CostFunction* doppler_function = new ceres::AutoDiffCostFunction<dopplerFactor, 3 
                                                        , 6,6>(new 
                                                        dopplerFactor(v_x_i, v_y_i, v_z_i, delta_t,  var_vec));
               auto ID =  problem.AddResidualBlock(doppler_function, dop_loss_function, state_array[i],state_array[i+1]);
                numOfDopplerFactors++;
                DoppIDs.push_back(ID);
            }
        }
        return true;
    }

    /* add pseudorange FACTORS */
    bool addPseudorangeFactors(ceres::Problem& problem)
    {
        PseudIDs.clear();
        PseudVars.clear();
        psrIDsTmp.clear();
        /* add pseudorange factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;
        iter_pr = gnss_raw_map.begin();
        int length = measSize;
        for(int m = 0;  m < length; m++,iter_pr++) // 
        {
            nlosExclusion::GNSS_Raw_Array gnss_data = (iter_pr->second);
            // gps_sec_array[m] = int(gnss_data.GNSS_Raws[0].GNSS_time);
            state_gps_sec_vec.push_back(int(gnss_data.GNSS_Raws[0].GNSS_time));
            MatrixXd weight_matrix; //goGPS weighting
            weight_matrix = m_GNSS_Tools.cofactorMatrixCal_WLS(gnss_data, "WLS"); //goGPS
            // std::cout << "weight_matrix-> "<<weight_matrix<<std::endl;
            int sv_cnt = gnss_data.GNSS_Raws.size();
            double t = gnss_data.GNSS_Raws[0].GNSS_time;

            int factor_index = 0;
            for(int i =0; i < sv_cnt; i++)
            {
                std::string sat_sys;
                double s_g_x = 0, s_g_y = 0,s_g_z = 0, var = 1;
                double pseudorange = 0;
                if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[i].prn_satellites_index)) sat_sys = "GPS";
                else sat_sys = "BeiDou";

                s_g_x = gnss_data.GNSS_Raws[i].sat_pos_x;
                s_g_y = gnss_data.GNSS_Raws[i].sat_pos_y;
                s_g_z = gnss_data.GNSS_Raws[i].sat_pos_z;

                pseudorange = gnss_data.GNSS_Raws[i].pseudorange;

                double ele = gnss_data.GNSS_Raws[i].elevation;
                double snr = gnss_data.GNSS_Raws[i].snr;

                if(snr>15) // simple thresholds to exlude the outlier measurements 28
                { // 15 28 32
                    factor_index++;
                    ceres::CostFunction* ps_function = new ceres::AutoDiffCostFunction<pseudorangeFactor, 1 
                                                                , 6>(new 
                                                                pseudorangeFactor(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1/weight_matrix(i,i))));
                    auto ID = problem.AddResidualBlock(ps_function, new ceres::HuberLoss(1.0), state_array[m]);
                    numOfPsrFactors++;
                    // if(m==(length-1))
                    {
                        // PseudIDs.push_back(ID);
                        // psrIDs.push_back(ID);

                        psrIDsTmp.push_back(ID);
                        
                        // PseudVars.push_back(sqrt(1/weight_matrix(i,i)));
                    }
                    // std::cout<<"sqrt(1/weight_matrix(i,i))-> " << (1/weight_matrix(i,i)) << "\n";
                }
            }  
            satNumPerT.push_back(factor_index);          
        }
       
        return true;
    }


    /* add pseudorange FACTORS */
    bool addPseudorangeFactorsForInitialization(ceres::Problem& problem)
    {
        psrIDsTmp.clear();
        /* add pseudorange factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;
        iter_pr = gnss_raw_map.begin();
        int length = measSize;
        for(int m = 0;  m < length; m++,iter_pr++) // 
        {
            nlosExclusion::GNSS_Raw_Array gnss_data = (iter_pr->second);
            MatrixXd weight_matrix; //goGPS weighting
            weight_matrix = m_GNSS_Tools.cofactorMatrixCal_WLS(gnss_data, "WLS"); //goGPS
            // std::cout << "weight_matrix-> "<<weight_matrix<<std::endl;
            int sv_cnt = gnss_data.GNSS_Raws.size();
            double t = gnss_data.GNSS_Raws[0].GNSS_time;

            int factor_index = 0;
            for(int i =0; i < sv_cnt; i++)
            {
                std::string sat_sys;
                double s_g_x = 0, s_g_y = 0,s_g_z = 0, var = 1;
                double pseudorange = 0;
                if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[i].prn_satellites_index)) sat_sys = "GPS";
                else sat_sys = "BeiDou";

                s_g_x = gnss_data.GNSS_Raws[i].sat_pos_x;
                s_g_y = gnss_data.GNSS_Raws[i].sat_pos_y;
                s_g_z = gnss_data.GNSS_Raws[i].sat_pos_z;

                pseudorange = gnss_data.GNSS_Raws[i].pseudorange;

                double ele = gnss_data.GNSS_Raws[i].elevation;
                double snr = gnss_data.GNSS_Raws[i].snr;

                { // 15 28 32
                    factor_index++;
                    ceres::CostFunction* ps_function_ini = new ceres::AutoDiffCostFunction<pseudorangeFactor, 1 
                                                                , 6>(new 
                                                                pseudorangeFactor(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1/weight_matrix(i,i))));
                    auto ID = problem.AddResidualBlock(ps_function_ini, psr_loss_function, state_array[m]);
                    psrIDsTmp.push_back(ID);
                }
            }  
        }
       
        return true;
    }

        /* add pseudorange FACTORS */
    bool addPseudorangeFactorsGMM(ceres::Problem& problem)
    {
        PseudIDs.clear();
        PseudVars.clear();
        /* add pseudorange factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;
        iter_pr = gnss_raw_map.begin();
        int length = measSize;
        for(int m = 0;  m < length; m++,iter_pr++) // 
        {
            nlosExclusion::GNSS_Raw_Array gnss_data = (iter_pr->second);
            // gps_sec_array[m] = int(gnss_data.GNSS_Raws[0].GNSS_time);
            state_gps_sec_vec.push_back(int(gnss_data.GNSS_Raws[0].GNSS_time));
            MatrixXd weight_matrix; //goGPS weighting
            weight_matrix = m_GNSS_Tools.cofactorMatrixCal_WLS(gnss_data, "WLS"); //goGPS
            // weight_matrix.setIdentity();
            // std::cout << "weight_matrix-> "<<weight_matrix<<std::endl;
            int sv_cnt = gnss_data.GNSS_Raws.size();
            double t = gnss_data.GNSS_Raws[0].GNSS_time;

            int factor_index = 0;
            for(int i =0; i < sv_cnt; i++)
            {
                std::string sat_sys;
                double s_g_x = 0, s_g_y = 0,s_g_z = 0, var = 1;
                double pseudorange = 0;
                if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[i].prn_satellites_index)) sat_sys = "GPS";
                else sat_sys = "BeiDou";

                s_g_x = gnss_data.GNSS_Raws[i].sat_pos_x;
                s_g_y = gnss_data.GNSS_Raws[i].sat_pos_y;
                s_g_z = gnss_data.GNSS_Raws[i].sat_pos_z;

                pseudorange = gnss_data.GNSS_Raws[i].pseudorange;

                double ele = gnss_data.GNSS_Raws[i].elevation;
                double snr = gnss_data.GNSS_Raws[i].snr;

                libRSF::GaussianMixture<1> GMMTmp;
                if(gaussianMixtureModel.GMM_psr.getNumberOfComponents()>=1)
                {
                    GMMTmp = gaussianMixtureModel.GMM_psr;
                }
                else
                {
                    std::cout <<" number of Gaussian Component is less than 1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n";
                    GMMTmp.initSpread(1,gaussianMixtureModel.DefaultStdDev); // 0.1
                }

                // GMMTmp.initSpread(3,10); // 0.1

                // if(snr>15) // simple thresholds to exlude the outlier measurements 28
                { // 15 28 32
                    factor_index++;

                    // ceres::CostFunction* ps_function = new ceres::AutoDiffCostFunction<pseudorangeFactor, 1 
                    //                                             , 6>(new 
                    //                                             pseudorangeFactor(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1/weight_matrix(i,i))));
                    // auto ID = problem.AddResidualBlock(ps_function, psr_loss_function, state_array[m]);

                    ceres::CostFunction* ps_function_GMM = new ceres::AutoDiffCostFunction<pseudorangeFactor_GMM, 1 
                                                                , 6>(new 
                                                                pseudorangeFactor_GMM(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1/weight_matrix(i,i)), GMMTmp));
                    auto ID = problem.AddResidualBlock(ps_function_GMM, new ceres::HuberLoss(1.0), state_array[m]);

                    numOfPsrFactors++;
                    // if(m==(length-1))
                    {
                        PseudIDs.push_back(ID);
                        psrIDs.push_back(ID);
                        
                        PseudVars.push_back(sqrt(1/weight_matrix(i,i)));
                    }
                    // std::cout<<"sqrt(1/weight_matrix(i,i))-> " << (1/weight_matrix(i,i)) << "\n";
                }
            }  
            satNumPerT.push_back(factor_index);          
        }
       
        return true;
    }

    /* add pseudorange FACTORS aided by GNC */
    bool addPseudorangeFactorsViaGNC(ceres::Problem& problem)
    {
        PseudIDs.clear();
        PseudVars.clear();

        /* add pseudorange factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;
        iter_pr = gnss_raw_map.begin();
        int length = measSize;
        for(int m = 0;  m < length; m++,iter_pr++) // 
        {
            nlosExclusion::GNSS_Raw_Array gnss_data = (iter_pr->second);
            // gps_sec_array[m] = int(gnss_data.GNSS_Raws[0].GNSS_time);
            state_gps_sec_vec.push_back(int(gnss_data.GNSS_Raws[0].GNSS_time));
            MatrixXd weight_matrix; //goGPS weighting
            weight_matrix = m_GNSS_Tools.cofactorMatrixCal_WLS(gnss_data, "WLS"); //goGPS
            // std::cout << "weight_matrix-> "<<weight_matrix<<std::endl;
            int sv_cnt = gnss_data.GNSS_Raws.size();
            double t = gnss_data.GNSS_Raws[0].GNSS_time;

            int factor_index = 0;
            for(int i =0; i < sv_cnt; i++)
            {
                std::string sat_sys;
                double s_g_x = 0, s_g_y = 0,s_g_z = 0, var = 1;
                double pseudorange = 0;
                if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[i].prn_satellites_index)) sat_sys = "GPS";
                else sat_sys = "BeiDou";

                s_g_x = gnss_data.GNSS_Raws[i].sat_pos_x;
                s_g_y = gnss_data.GNSS_Raws[i].sat_pos_y;
                s_g_z = gnss_data.GNSS_Raws[i].sat_pos_z;

                pseudorange = gnss_data.GNSS_Raws[i].pseudorange;

                double ele = gnss_data.GNSS_Raws[i].elevation;
                double snr = gnss_data.GNSS_Raws[i].snr;

                // if(snr>15) // simple thresholds to exlude the outlier measurements 28
                {
                    factor_index++;
                    ceres::CostFunction* ps_function = new ceres::AutoDiffCostFunction<pseudorangeFactorRobust, 1 
                                                                , 6>(new 
                                                                pseudorangeFactorRobust(sat_sys, s_g_x, s_g_y, s_g_z, pseudorange, sqrt(1/weight_matrix(i,i)),m, i, &(GNC_Geman_McClure.weighings),initialized));
                    auto ID = problem.AddResidualBlock(ps_function, loss_function, state_array[m]);
                    if(*initialized) //initialized
                    {
                        // LOG(INFO) << "update weightings!!!!-> " << GNC_Geman_McClure.weighings.size() << std::endl;
                        
                        // LOG(INFO) << "m---- = " << m << std::endl;
                        // LOG(INFO) << "i---- = " << i << std::endl;
                        // LOG(INFO) << "update weightings-> " << GNC_Geman_McClure.weighings[m][i] << std::endl;
                        // std::cout << "weightings---- = " << GNC_Geman_McClure.weighings[m][i] << std::endl;
                    }

                    PseudIDs.push_back(ID);
                    PseudVars.push_back(sqrt(1/weight_matrix(i,i)));
                    psrIDs.push_back(ID);
                    numOfPsrFactors++;
                }
            }
            satNumPerT.push_back(factor_index);            
        }
       
        return true;
    }

    /* add sliding window prior  position x,y,z , bias_gps, bias_beidou, bias_drift, and velocity x,y,z  FACTORS */
    bool addPriorFactors(ceres::Problem& problem)
    {
        /* add previous last state of sliding window to next first state of sliding window as a prior factor */
        double position_x =0 ,position_y=0, position_z=0;
        double gps_bias =0, beidou_bias=0, bias_drift =0;
        double velocity_x =0, velocity_y=0, velocity_z=0;
        double var=1.5;

        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr_cur;
        iter_pr_cur = gnss_raw_map.begin();

        double timestamp_ = iter_pr_cur->first;
        int length = measSize;

        //   if (gnss_raw_map.size()>=100)
        //     {
        states = slidingwindow_map[timestamp_];
        position_x = states.position[0];
        position_y = states.position[1] ;
        position_z = states.position[2] ;
        gps_bias = states.clocks[0];
        beidou_bias = states.clocks[1];
        bias_drift = states.clocks[2];
        velocity_x = states.velocity[0];
        velocity_y = states.velocity[1];
        velocity_z= states.velocity[2];

        ceres::CostFunction* priorError_function = new ceres::AutoDiffCostFunction<priorFactor, 3
                                                ,6>(new 
                                                priorFactor(position_x,position_y,position_z,gps_bias,beidou_bias,bias_drift,velocity_x,velocity_y,velocity_z,var)); //known parameters
        auto ID = problem.AddResidualBlock(priorError_function, dop_loss_function, state_array[0]); //9,6,3 parameters to be optimized,state_array_vel[0]
        // }
        PriorIDs.push_back(ID);
                
        return true;
    }


    /* add 3DMA as prior to each frame, latitude, longtitude, altitude */
    bool add3DMAFactors(ceres::Problem& problem)
    {
        // double timestamp=0;
        double latitude=0; // weidu
        double longtitude=0; //jingdu
        double altitude=0; //gaodu
        double ox, oy, oz; // save original reference position in ecef
        // double position_x =0 ,position_y=0, position_z=0;
        // double var=1.5/0.2; //for WG data
        double var=1.5/0.5;
        
       
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr_cur;
        iter_pr_cur = gnss_raw_map.begin();
        // iter_pr_cur--;

        
        int length = measSize;
        for(int m = 0;  m < length; m++,iter_pr_cur++) // 
        {
            double time_stamp = iter_pr_cur->first;

            // std::cout<<"time stamp in doppler"<<time_stamp<<std::endl;

            // std::cout<<"data3dma_map.size()"<< data3dma_map.size()<< std::endl;

            // data3dma_map.count(time_stamp);
            // std::cout<<"time stamp count"<< data3dma_map.count(int(time_stamp))<< std::endl;
            // std::cout<<"time stamp"<<time_stamp<<std::endl;
            nlosExclusion::GNSS_Raw_Array gnss_msg = iter_pr_cur->second;
            SV = gnss_msg.GNSS_Raws.size();

            // std::cout<<"data3dma_map.count"<<data3dma_map.count(int(time_stamp))<<std::endl;
            // std::cout<<"satellite number"<<SV<<std::endl;

            if(data3dma_map.count(int(time_stamp)))
            // if(data3dma_map.count(int(time_stamp)))
            {
                data3dma = data3dma_map[int(time_stamp)];
                latitude = data3dma.lati;
                longtitude = data3dma.longti;
                altitude = data3dma.alti;

                // save the origin lon alt in llh
                Eigen::Matrix3d originllh;
                originllh(0) = longtitude;//longtitude
                originllh(1) = latitude;//latitude
                originllh(2) = altitude;

                GNSS_Tools m_GNSS_Tools;
                Eigen::MatrixXd oxyz; // the original position 
                oxyz.resize(3, 1); // resize to 3X1
                oxyz = m_GNSS_Tools.llh2ecef(originllh);
                ox = oxyz(0); // obtain x in ecef 
                oy = oxyz(1); // obtain y in ecef
                oz = oxyz(2); // obtain z in ecef

                // std::cout<<"oxyz-.>        ------------------------"<<oxyz<<std::endl;

                ceres::CostFunction* Error3dma_function = new ceres::AutoDiffCostFunction<Factor3dma,3
                                                            ,6>(new Factor3dma(ox,oy,oz,var)); //known parameters
                auto ID = problem.AddResidualBlock(Error3dma_function, NULL, state_array[m]);
                amd3IDs.push_back(ID);

                add3DMA = true;                     
            }
            else
                add3DMA = false;
        }

        
        return true;
    }

    /* add polynominal pose priot FACTORS */
    bool addPolyPosePriorFactors(ceres::Problem& problem)
    {
        int length = measSize;
        
        if(normalizedResiduals.size()>0)
        {
            std::cout<<"posePrediction->" << posePrediction << "\n";
            double position_x = posePrediction(0), position_y=posePrediction(1), position_z=0;
            // double position_x = 0, position_y=0, position_z=0;
            double var=normalizedResiduals.back();
            // var = var / 10;

            ceres::CostFunction* priorError_function = new ceres::AutoDiffCostFunction<polyposePriorFactor, 2
                                                    ,6>(new 
                                                    polyposePriorFactor(position_x,position_y,var)); //known parameters
            auto ID = problem.AddResidualBlock(priorError_function, loss_function, state_array[length-1]); 
        }
        
                
        return true;
    }

    /* add tightly coupled Doppler FACTORS !!!!!!!! */
    bool addTCDopplerFactors(ceres::Problem& problem)
    {
        /* add tightly coupled Doppler factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;
        iter_pr = gnss_raw_map.begin();
        int length = measSize;
        for(int m = 0;  m < length; m++,iter_pr++) // 
        {
            iter_prNext = iter_pr;
            iter_prNext++;
            // if(iter_prNext != gnss_raw_map.end())
            {
                double delta_t = iter_prNext->first - iter_pr->first;
                nlosExclusion::GNSS_Raw_Array gnss_data = (iter_pr->second);
                MatrixXd weight_matrix; //goGPS weighting
                weight_matrix = m_GNSS_Tools.cofactorMatrixCal_WLS(gnss_data, "WLS"); //goGPS
                weight_matrix = Doppler2PSRWeight * weight_matrix;
                // std::cout << "delta_t-> "<<delta_t<<std::endl;
                int sv_cnt = gnss_data.GNSS_Raws.size();
                double t = gnss_data.GNSS_Raws[0].GNSS_time;

                int factor_index = -1;
                for(int i =0; i < sv_cnt; i++)
                {
                    factor_index++;

                    std::string sat_sys;
                    double s_g_x = 0, s_g_y = 0,s_g_z = 0; // 
                    double svDdt = gnss_data.GNSS_Raws[i].ddt; // satellite clock drift rate
                    double s_v_x = 0, s_v_y = 0, s_v_z =0;
                    double doppler = 0; 
                    if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[i].prn_satellites_index)) sat_sys = "GPS";
                    else sat_sys = "BeiDou";

                    s_g_x = gnss_data.GNSS_Raws[i].sat_pos_x;
                    s_g_y = gnss_data.GNSS_Raws[i].sat_pos_y;
                    s_g_z = gnss_data.GNSS_Raws[i].sat_pos_z;

                    s_v_x = gnss_data.GNSS_Raws[i].vel_x;
                    s_v_y = gnss_data.GNSS_Raws[i].vel_y;
                    s_v_z = gnss_data.GNSS_Raws[i].vel_z;

                    doppler = gnss_data.GNSS_Raws[i].doppler * gnss_data.GNSS_Raws[i].lamda;

                    ceres::CostFunction* tcdoppler_function = new ceres::AutoDiffCostFunction<tcdopplerFactor, 1 
                                                                    , 6, 3>(new 
                                                                    tcdopplerFactor(sat_sys, s_g_x, s_g_y, s_g_z, s_v_x,s_v_y, s_v_z, svDdt, doppler, delta_t, sqrt(1/weight_matrix(i,i))));
                    auto ID = problem.AddResidualBlock(tcdoppler_function, loss_function, state_array[m],state_array_vel[m]);
                }         
            }
               
        }
       
        return true;
    }


    /* add the TDCP factor */
    bool addTDCPFactors(ceres::Problem& problem)
    {
        /* add tightly coupled Doppler factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;
        iter_pr = gnss_raw_map.begin();
        int length = measSize;
        ceres::LossFunction *tdcp_loss_function = new ceres::CauchyLoss(4.0);
        // ceres::LossFunction *tdcp_loss_function = new ceres::CauchyLoss(4.0);
        for(int m = 0;  m < length; m++,iter_pr++) // 
        {
            iter_prNext = iter_pr;
            iter_prNext++;
            
            if(iter_prNext != gnss_raw_map.end())
            {
                /* gnss data at current epoch */
                nlosExclusion::GNSS_Raw_Array gnss_data = iter_prNext->second;
                nlosExclusion::GNSS_Raw_Array pre_gnss_data = iter_pr->second;

                MatrixXd weight_matrix; //goGPS weighting
                weight_matrix = m_GNSS_Tools.cofactorMatrixCal_WLS(gnss_data, "WLS"); //goGPS
                weight_matrix = Car2PSRWeight * weight_matrix;
                int sv_cnt = gnss_data.GNSS_Raws.size();
                int pre_sv_cnt = pre_gnss_data.GNSS_Raws.size();
                double t = gnss_data.GNSS_Raws[0].GNSS_time;

                int factor_index = -1;
                for(int i =0; i < sv_cnt; i++)
                {
                    
                    bool CycleSlip = true;
                    int carIndex = -1;

                    double car_measure = gnss_data.GNSS_Raws[i].carrier_phase;
                    double pre_car_measure = 0;
                    
                    /* Cycle Slip Detection */
                    int targetPrn = gnss_data.GNSS_Raws[i].prn_satellites_index;
                    double lmbda = gnss_data.GNSS_Raws[i].lamda;
                    double Doppler = gnss_data.GNSS_Raws[i].doppler;
                    
                    for(int j =0; j < pre_sv_cnt; j++)
                    {
                        if(pre_gnss_data.GNSS_Raws[j].prn_satellites_index == targetPrn)
                        {
                            // find a same satellite from previous epoch
                            double carrierDiff = pre_gnss_data.GNSS_Raws[j].carrier_phase - gnss_data.GNSS_Raws[i].carrier_phase;
                            // carrierDiff = carrierDiff + latestRecCloDrift;
                            double car2DopDiff = Doppler - carrierDiff;

                            // std::cout<<"carrierDiff-> " << carrierDiff << "\n";
                            // std::cout<<"Doppler->     " << Doppler << "\n";
                            // std::cout<<"car2DopDiff->->>>>>>>>>>>>>>>>> " << car2DopDiff << "\n";
                            // std::cout<<"latestRecCloDrift->->>>>>>>>>>> " << latestRecCloDrift << "\n";
                            // std::cout<<"snr-> " << gnss_data.GNSS_Raws[i].snr << "\n";

                            // if(fabs(car2DopDiff)>100) // if there is large difference, remove the carrier-phase measurements
                            if(gnss_data.GNSS_Raws[i].slip ==1 || fabs(car2DopDiff)>300)
                            // if(gnss_data.GNSS_Raws[i].slip ==1 )
                            {
                                // gnss_msg.GNSS_Raws[i].carrier_phase = 0.0;
                                CycleSlip = true;
                                // std::cout<<"cycle slip->->>>>>>>>>>> " << CycleSlip << "\n";
                            }
                            else
                            {
                                CycleSlip = false;
                                carIndex = j;
                                pre_car_measure = pre_gnss_data.GNSS_Raws[j].carrier_phase;
                                // factor_index++;
                                
                            }
                        }
                    }

                    car_measure = car_measure * lmbda;
                    pre_car_measure = pre_car_measure * lmbda;
                    
                    /* if the cycle slip is not detected */
                    if(!CycleSlip && car_measure>1000 && pre_car_measure>1000)
                    {
                        std::string sat_sys;
                        double s_g_x = 0, s_g_y = 0,s_g_z = 0; // 
                        double pre_s_g_x = 0, pre_s_g_y = 0,pre_s_g_z = 0; // 
                        if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[i].prn_satellites_index)) sat_sys = "GPS";
                        else sat_sys = "BeiDou";

                        s_g_x = gnss_data.GNSS_Raws[i].sat_pos_x;
                        s_g_y = gnss_data.GNSS_Raws[i].sat_pos_y;
                        s_g_z = gnss_data.GNSS_Raws[i].sat_pos_z;

                        pre_s_g_x = pre_gnss_data.GNSS_Raws[carIndex].sat_pos_x;
                        pre_s_g_y = pre_gnss_data.GNSS_Raws[carIndex].sat_pos_y;
                        pre_s_g_z = pre_gnss_data.GNSS_Raws[carIndex].sat_pos_z;


                        ceres::CostFunction* TDCP_function = new ceres::AutoDiffCostFunction<TDCPFactor, 1 
                                                                        , 6, 6>(new 
                                                                        TDCPFactor(sat_sys, s_g_x, s_g_y, s_g_z, pre_s_g_x,pre_s_g_y, pre_s_g_z, car_measure, pre_car_measure, lmbda, sqrt(1/weight_matrix(i,i))));
                        auto ID = problem.AddResidualBlock(TDCP_function, tdcp_loss_function, state_array[m+1],state_array[m]);
                        factor_index++;
                    }

                    
                }    
                // std::cout<<"number of TDCP constraints->->>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> " << factor_index << "\n";     
            }
               
        }
       
        return true;
    }


    /* add receiver clock drift FACTORS */
    bool addConstantClockDriftFactors(ceres::Problem& problem)
    {
        /* add clock drift factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        int i = 0;
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            /* add doppler measurements */
            iter_prNext = iter_pr;
            iter_prNext ++;
            if(iter_prNext != gnss_raw_map.end())
            {
                double delta_t = iter_prNext->first - iter_pr->first;

                ceres::CostFunction* constantClockDrift_function = new ceres::AutoDiffCostFunction<constantClockDriftFactor, 1 
                                                        , 6,6>(new 
                                                        constantClockDriftFactor(0.1));
                problem.AddResidualBlock(constantClockDrift_function, loss_function, state_array[i],state_array[i+1]);
            }
        }
        return true;
    }


    /* time-correlated receievr clock drift FACTORS */
    bool addTimeCorrelatedClockDriftFactors(ceres::Problem& problem)
    {
        /* add clock drift and motion model factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        int i = 0;
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            /* add doppler measurements */
            iter_prNext = iter_pr;
            iter_prNext ++;
            if(iter_prNext != gnss_raw_map.end())
            {
                double delta_t = iter_prNext->first - iter_pr->first;

                ceres::CostFunction* timeCorrelatedClockDrift_function = new ceres::AutoDiffCostFunction<timeCorrelatedClockDriftFactor, 2 
                                                        , 6,6>(new 
                                                        timeCorrelatedClockDriftFactor(delta_t, 100));
                problem.AddResidualBlock(timeCorrelatedClockDrift_function, loss_function, state_array[i],state_array[i+1]);
            }
        }
        return true;
    }

    /* moditon model factor FACTORS to factor graph */
    bool addMotionModelFactors(ceres::Problem& problem)
    {
        /* add clock drift and motion model factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        int i = 0;
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            /* add doppler measurements */
            iter_prNext = iter_pr;
            iter_prNext ++;
            if(iter_prNext != gnss_raw_map.end())
            {
                double delta_t = iter_prNext->first - iter_pr->first;

                ceres::CostFunction* motionModel_function = new ceres::AutoDiffCostFunction<motionModelFactorSmooth, 3 
                                                        , 6,6,3,3>(new 
                                                        motionModelFactorSmooth(delta_t, 0.6 * 1)); // 0.3
                problem.AddResidualBlock(motionModel_function, loss_function, state_array[i],state_array[i+1],state_array_vel[i],state_array_vel[i+1]);
            }
        }
        return true;
    }


    Eigen::Vector3d ecef2geo(const Eigen::Vector3d &xyz)
    {
        Eigen::Vector3d lla = Eigen::Vector3d::Zero();
        if (xyz.x() == 0 && xyz.y() == 0)
        {
            LOG(ERROR) << "LLA coordinate is not defined if x = 0 and y = 0";
            return lla;
        }

        double EARTH_ECCE_2 = 6.69437999014e-3;
        double EARTH_SEMI_MAJOR = 6378137;

        double e2 = EARTH_ECCE_2;
        double a = EARTH_SEMI_MAJOR;
        double a2 = a * a;
        double b2 = a2 * (1 - e2);
        double b = sqrt(b2);
        double ep2 = (a2 - b2) / b2;
        double p = xyz.head<2>().norm();

        // two sides and hypotenuse of right angle triangle with one angle = theta:
        double s1 = xyz.z() * a;
        double s2 = p * b;
        double h = sqrt(s1 * s1 + s2 * s2);
        double sin_theta = s1 / h;
        double cos_theta = s2 / h;

        // two sides and hypotenuse of right angle triangle with one angle = lat:
        s1 = xyz.z() + ep2 * b * pow(sin_theta, 3);
        s2 = p - a * e2 * pow(cos_theta, 3);
        h = sqrt(s1 * s1 + s2 * s2);
        double tan_lat = s1 / s2;
        double sin_lat = s1 / h;
        double cos_lat = s2 / h;
        double lat = atan(tan_lat);
        double lat_deg = lat * R2D;

        double N = a2 * pow((a2 * cos_lat * cos_lat + b2 * sin_lat * sin_lat), -0.5);
        double altM = p / cos_lat - N;

        double lon = atan2(xyz.y(), xyz.x());
        double lon_deg = lon * R2D;
        lla << lat_deg, lon_deg, altM;
        return lla;
    }


    Eigen::Matrix3d geo2rotation(const Eigen::Vector3d &ref_geo)
    {
        double lat = ref_geo.x() * D2R, lon = ref_geo.y() * D2R;
        double sin_lat = sin(lat), cos_lat = cos(lat);
        double sin_lon = sin(lon), cos_lon = cos(lon);
        Eigen::Matrix3d R_ecef_enu;
        R_ecef_enu << -sin_lon, -sin_lat*cos_lon, cos_lat*cos_lon,
                       cos_lon, -sin_lat*sin_lon, cos_lat*sin_lon,
                       0      ,  cos_lat        , sin_lat;
        return R_ecef_enu;
    }

    Eigen::Matrix3d ecef2rotation(const Eigen::Vector3d &ref_ecef)
    {
        return geo2rotation(ecef2geo(ref_ecef));
    } 

    /* analytical moditon model factor FACTORS to factor graph */
    bool addAnaliticalMotionModelFactors(ceres::Problem& problem)
    {
        /* add clock drift and motion model factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        int i = 0;
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            /* add doppler measurements */
            iter_prNext = iter_pr;
            iter_prNext ++;
            if(iter_prNext != gnss_raw_map.end())
            {
                double delta_t = iter_prNext->first - iter_pr->first;

                // ceres::CostFunction* motionModel_function = new ceres::AutoDiffCostFunction<motionModelFactorSmooth, 3 
                //                                         , 6,6,3,3>(new 
                //                                         motionModelFactorSmooth(delta_t, 0.6 * 1)); // 0.3
                // problem.AddResidualBlock(motionModel_function, loss_function, state_array[i],state_array[i+1],state_array_vel[i],state_array_vel[i+1]);


                double scale = 0.6 * 1; // 0.3
                
                MatrixXd cov;
                cov.resize(3,3);
                cov.setIdentity();
                for(int k = 0; k < cov.rows(); k++)
                {
                    cov.row(k) = cov.row(k) * scale;
                }

                if(1)
                {
                    cov = cov * 0.2;
                    cov.row(2) = cov.row(2) * 10;
                    
                    cov = cov.inverse();
                    cov = cov.cwiseSqrt();
                    ENULlhRef.resize(3,1);
                    ENULlhRef<< ref_lon, ref_lat, ref_alt;
                    Eigen::Vector3d ref_ecef = m_GNSS_Tools.llh2ecef(ENULlhRef);
                    Eigen::Matrix3d R_ecef_enu = ecef2rotation(ref_ecef);
                    cov = R_ecef_enu * cov * R_ecef_enu.transpose();
                    // std::cout<<"cov-> " << cov <<std::endl;
                }

                AnalyticalMotionModelFactor *motionModel_function = new 
                                                        AnalyticalMotionModelFactor(delta_t, cov); 
                problem.AddResidualBlock(motionModel_function, loss_function, state_array[i],state_array[i+1],state_array_vel[i],state_array_vel[i+1]);
            }
        }
        return true;
    }

    /* add constant velocity factor to factor graph */
    bool addConstantVelFactors(ceres::Problem& problem)
    {
        /* add clock drift and motion model factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        int i = 0;
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            /* add doppler measurements */
            iter_prNext = iter_pr;
            iter_prNext ++;
            if(iter_prNext != gnss_raw_map.end())
            {
                double delta_t = iter_prNext->first - iter_pr->first;

                /* for  */
                ceres::CostFunction* constantVel_function = new ceres::AutoDiffCostFunction<constantVelFactor, 3 
                                                        ,3>(new 
                                                        constantVelFactor(delta_t, 9));
                problem.AddResidualBlock(constantVel_function, loss_function, state_array_vel[i]);
            }
        }
        return true;
    }

    /* add velocity limit factor to factor graph */
    bool addVelLimitFactors(ceres::Problem& problem, double velLimit)
    {
        /* add clock drift and motion model factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        int i = 0;
        double alpha = 2000; // 20
        double sigma = 0.2; // 0.5  
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            iter_prNext = iter_pr;
            iter_prNext ++;
            if(iter_prNext != gnss_raw_map.end())
            {
                double delta_t = iter_prNext->first - iter_pr->first;

                /* add velocity limit constraints  */
                ceres::CostFunction* velLimit_function = new ceres::AutoDiffCostFunction<velLimitFactor, 3 
                                                        ,3>(new 
                                                        velLimitFactor(delta_t, 1, alpha, sigma, velLimit));
                problem.AddResidualBlock(velLimit_function, loss_function, state_array_vel[i]);
            }
        }
        return true;
    }


    /* acc model factor FACTORS to factor graph (to estimate the acc) */
    bool addAccModelFactors(ceres::Problem& problem)
    {
        /* add clock drift and motion model factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        int i = 0;
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            /* add doppler measurements */
            iter_prNext = iter_pr;
            iter_prNext ++;
            if(iter_prNext != gnss_raw_map.end())
            {
                double delta_t = iter_prNext->first - iter_pr->first;

                ceres::CostFunction* accModel_function = new ceres::AutoDiffCostFunction<accModelFactor, 3 
                                                        , 3,3,3>(new 
                                                        accModelFactor(delta_t, 0.1));
                problem.AddResidualBlock(accModel_function, loss_function, state_array_vel[i],state_array_vel[i+1],state_array_acc[i]);
            }
        }
        return true;
    }

    /* add zero acc factor factor graph (for pedestrian walking, the acc tends to be zero) */
    bool addZeroAccFactors(ceres::Problem& problem)
    {
        /* add clock drift and motion model factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        int i = 0;
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            /* add doppler measurements */
            iter_prNext = iter_pr;
            iter_prNext ++;
            if(iter_prNext != gnss_raw_map.end())
            {
                double delta_t = iter_prNext->first - iter_pr->first;
                ceres::CostFunction* zeroAcc_function = new ceres::AutoDiffCostFunction<zeroAccFactor, 3 
                                                        ,3>(new 
                                                        zeroAccFactor(delta_t, 0.05));
                problem.AddResidualBlock(zeroAcc_function, loss_function, state_array_acc[i]);
            }
        }
        return true;
    }


    /* jer model factor FACTORS to factor graph (to estimate the jer) */
    bool addJerModelFactors(ceres::Problem& problem)
    {
        /* add clock drift and motion model factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        int i = 0;
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            /* add doppler measurements */
            iter_prNext = iter_pr;
            iter_prNext ++;
            if(iter_prNext != gnss_raw_map.end())
            {
                double delta_t = iter_prNext->first - iter_pr->first;

                ceres::CostFunction* jerModel_function = new ceres::AutoDiffCostFunction<jerModelFactor, 3 
                                                        , 3,3,3>(new 
                                                        jerModelFactor(delta_t, 0.1));
                problem.AddResidualBlock(jerModel_function, loss_function, state_array_acc[i],state_array_acc[i+1],state_array_jer[i]);
            }
        }
        return true;
    }

    /* solve the factor graph */
    bool solveFactorGraph(ceres::Problem& problem,ceres::Solver::Options& options, ceres::Solver::Summary& summary)
    {
        /* solve the problem*/
        ceres::Solve(options, &problem, &summary);

        *initialized = true;

        return true;
    }

    /* update residuals for GMM */
    bool updateResidualsOfGMM(ceres::Problem& problem)
    {
        /** set up objects */
        ceres::Problem::EvaluateOptions EvalOpts;
        EvalOpts.num_threads = 8;
        EvalOpts.apply_loss_function = true;

        /** only pseudorange error gets evaluated */
        EvalOpts.residual_blocks = psrIDs;

        /** calculate residuals */
        std::vector<double> residuals; // 
        
        problem.Evaluate(EvalOpts, nullptr, &residuals, nullptr, nullptr); 

        bool reduceWeight = false;
        std::vector<double> residualsTmp;
        if(reduceWeight)
        {
            for(int i = 0; i < residuals.size(); i++)
            {
                if(fabs(residuals[i] * PseudVars[i])<100)
                {
                    residualsTmp.push_back(residuals[i] * PseudVars[i]);
                }
                
            }
        }

        for(int i = 0; i < residuals.size(); i++)
        {
            if(fabs(residuals[i] )<100)
            {
                residualsTmp.push_back(residuals[i]);
            }
            // residuals[i] = 2.0;
            
        }
        residuals = residualsTmp;

        std::cout<< "size of residuals for GMM-> " << residuals.size() << std::endl;
        gaussianMixtureModel.updateGMMResiduals(residualsTmp);
        return true;
    }

    /* update the residuals of the GMM based on mannual calculation
     * Directly get the residuals based Z-h(x)
     * h(x) = ||pos_rcv-pos_sat|| + clock_bias
     */
    bool updateResidualsOfGMM_v2(ceres::Problem& problem)
    {
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        // state_array

        int i = 0;
        /** calculate residuals */
        std::vector<double> residuals; // 
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            nlosExclusion::GNSS_Raw_Array gnss_data = iter_pr->second;
            int sv_cnt = gnss_data.GNSS_Raws.size();
            for(int j =0; j < sv_cnt; j++)
            {
                std::string sat_sys;
                double pseudorange = gnss_data.GNSS_Raws[j].pseudorange;
                double clockBias = 0;
                double s_g_x = 0, s_g_y = 0,s_g_z = 0;
                if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[j].prn_satellites_index)) sat_sys = "GPS";
                else sat_sys = "BeiDou";

                s_g_x = gnss_data.GNSS_Raws[j].sat_pos_x;
                s_g_y = gnss_data.GNSS_Raws[j].sat_pos_y;
                s_g_z = gnss_data.GNSS_Raws[j].sat_pos_z;

                Eigen::Vector3d satPos = Eigen::Vector3d(s_g_x,s_g_y,s_g_z);
                Eigen::Vector3d rcvPos = Eigen::Vector3d(state_array[i][0],state_array[i][1], state_array[i][2]);
                double est_pseudorange = (satPos - rcvPos).norm();
                
                double OMGE_ = 7.2921151467E-5;
                double CLIGHT_ = 299792458.0;
                est_pseudorange = est_pseudorange + OMGE_ * (s_g_x*state_array[i][1]-s_g_y*state_array[i][0])/CLIGHT_;

                if(sat_sys == "GPS") 
                {
                    est_pseudorange = est_pseudorange - state_array[i][3];
                }
                
                else if(sat_sys == "BeiDou") 
                {
                    est_pseudorange = est_pseudorange - state_array[i][4];
                }

                double residual = est_pseudorange - pseudorange;
                // if(fabs(residual)<50) // 50 
                residuals.push_back(residual);

                // std::cout<< "residual of satellite -> " << gnss_data.GNSS_Raws[j].prn_satellites_index << "  at epoch -> " << gnss_data.GNSS_Raws[0].GNSS_time <<"  is  " << residual <<"\n";
            }
        }

        bool reduceWeight = false;
        std::vector<double> residualsTmp;
        if(reduceWeight)
        {
            for(int i = 0; i < residuals.size(); i++)
            {
                if(fabs(residuals[i] / PseudVars[i])<50)
                {
                    residualsTmp.push_back(residuals[i] / PseudVars[i]);
                }
                
            }
            std::cout<< "size of residuals for GMM-> " << residualsTmp.size() << std::endl;
            gaussianMixtureModel.updateGMMResiduals(residualsTmp);
        }
        else
        {
            for(int i = 0; i < residuals.size(); i++)
            {
                if(fabs(residuals[i])<50) // 50 
                {
                    residualsTmp.push_back(residuals[i]);
                }
                
            }
            std::cout<< "size of residuals for GMM-> " << residualsTmp.size() << std::endl;
            gaussianMixtureModel.updateGMMResiduals(residualsTmp);
        }
        
        return true;
    }

    /* update GMM with EM */
    bool estimateGMMViaEM()
    {
        /* estimate the GMM with Expectation Maximization */
        gaussianMixtureModel.estimateGMMParas();

        return true;
    }

    /* update the window size of the FGO */
    bool updateWS(double lastOptTime)
    {
        double newWS= sizeOfFactorGraph;

        double Kp = 1.7;
        double Kd = 2.0;
        double c1 = 0.1;
        double c2 = 0.15;
        double c3 = 0.2;

        int GMMSize = gaussianMixtureModel.GMM_psr.getNumberOfComponents();
        if(gaussianMixtureModel.GMM_psr.getNumberOfComponents()>=1)
        {
            std::vector<double> meanVector;
            std::vector<double> varVector;
            std::vector<double> weightVector;
            for(int i=0; i<GMMSize; i++)
            {
                Eigen::MatrixXd mean = gaussianMixtureModel.GMM_psr._Mixture.at(i).getMean();
                // meanVector.push_back(gaussianMixtureModel.GMM_psr._Mixture.at(i).getMean());
                meanVector.push_back(mean(0,0));

                Eigen::MatrixXd var = gaussianMixtureModel.GMM_psr._Mixture.at(i).getSqrtInformation().inverse();
                varVector.push_back(var(0,0));

                Eigen::MatrixXd weight = gaussianMixtureModel.GMM_psr._Mixture.at(i).getWeight();
                weightVector.push_back(weight(0,0));
            }
            int maxElementIndex = std::max_element(weightVector.begin(),weightVector.end()) - weightVector.begin();

            std::cout<< "varVector[maxElementIndex]-> " << varVector[maxElementIndex] << std::endl;

            c3 = 1.0/varVector[maxElementIndex];
            
            double deltaF = lastOptTime - 200;
            double feedback = c1 * (1-exp(c2 *deltaF))/(1-exp(c2 *50)) + c3;
            feedback = 1.0 / feedback;

            newWS = newWS + Kp * feedback + Kd * (feedback - lastfeedback);

            if(newWS<150) newWS = 150;
            if(newWS>180) newWS = 180;

            std::cout<< "newWS-> " << newWS << std::endl;

            sizeOfFactorGraph = int(newWS);

            lastfeedback = feedback;
        }

        return true;
    }

    /* save graph state to Variable for next solving */
    bool saveGraphStateToVariable()
    {
        /* save the size of current factor graph */
        // lastFactorGraphSize = measSize;

        /* save the current state of factor graph */
        last_state_array.reserve(measSize);
        int length = measSize;

        /* allocate memory */
        for(int i = 0; i < length;i++)
        {
            last_state_array[i] = new double[stateSize]; //
        }

        for(int i = 0; i < length;i++)
        {
            for(int j = 0; j < stateSize; j++)
            {
                last_state_array[i] = new double[stateSize]; //
                last_state_array[i][j] = state_array[i][j];
            }
        }
        return true;
    }

    /* initialize the satNumPerT */
    bool initializeSatNumPerT()
    {
        // for(int i = 0; i < satNumPerT.size(); i++)
        // {
        //     GNC_Geman_McClure.satNumPerT.push_back(satNumPerT[i]);
        // }
        GNC_Geman_McClure.satNumPerT = satNumPerT;
        // LOG(INFO) << "satNumPerT  length-> " <<satNumPerT.size()<<std::endl;
        return true;
    }

    /* allocate memory to weights vector */
    bool allocateMemoryForWeightings()
    {
        GNC_Geman_McClure.allocateMemorys();

        /* initialize the weightings */
        GNC_Geman_McClure.initializeWeightings(gnss_raw_map); 
        return true;
    }

    /* get the residuals after first FGO */
    bool getResiudalsOfFirstFGO(ceres::Problem& problem)
    {
        /** set up objects */
        ceres::Problem::EvaluateOptions EvalOpts;
        EvalOpts.num_threads = 8;
        EvalOpts.apply_loss_function = false;

        /** only pseudorange error gets evaluated */
        EvalOpts.residual_blocks = psrIDs;

        /** calculate residuals */
        std::vector<double> residuals, residualsTmp; // 
        
        problem.Evaluate(EvalOpts, nullptr, &residuals, nullptr, nullptr); 

        for(int i = 0; i < residuals.size(); i++)
            {
                residualsTmp.push_back(residuals[i] * PseudVars[i]);
                
            }

        GNC_Geman_McClure.inputFirstFGOResiduals(residuals);
        return true;
    }

    /* update maximum residual */
    bool updateMaxResidual()
    {
        GNC_Geman_McClure.calculateMaxResidual();
        
        return true;
    }

    /* update origin GNC_mu */
    bool updateOriginGNCMu()
    {
        GNC_Geman_McClure.updateOriginGNCMu();
        return true;
    }

    /* update first GNC-GM Weightings */
    bool updateFirstGNCGMWeightings()
    {
        GNC_Geman_McClure.updateWeightings();
        // weighings = GNC_Geman_McClure.weighings;
        // std::cout << "update weightings!!!!-> " << GNC_Geman_McClure.weighings.size() << std::endl;
        // std::cout << "update weightings!!!!-> " << GNC_Geman_McClure.weighings[0][1] << std::endl;
        return true;
    }

    /* update Weightings in GNSS raw data */
    bool updateWeightingsInGNSSRawData()
    {
        GNC_Geman_McClure.updateGNSSRawDataWeightings(gnss_raw_map);
        return true;
    }

    /* update GNC-GM iteratively */
    bool updateGNCGMIteratively(ceres::Problem& problem,ceres::Solver::Options& options, ceres::Solver::Summary& summary)
    {
        /* solve the FGO-GNC via alternate minimization */
        // GNC_Geman_McClure.GNC_mu = 1.2;
        while(GNC_Geman_McClure.GNC_mu > 1 )
        {
            /* times of GNC iterations */
            // std::cout << "GNC_iter-> " << GNC_Geman_McClure.GNC_iter<< std::endl;
            // std::cout << "max residual-> " << GNC_Geman_McClure.GNC_r_max << std::endl;
            // std::cout << "GNC_mu-> " << GNC_Geman_McClure.GNC_mu<< std::endl;
            
            /** set up objects */
            ceres::Problem::EvaluateOptions EvalOpts;
            EvalOpts.num_threads = 8;
            EvalOpts.apply_loss_function = true;

            /** only pseudorange error gets evaluated */
            EvalOpts.residual_blocks = psrIDs;

            /** calculate residuals */
            std::vector<double> Residuals; // 
            problem.Evaluate(EvalOpts, nullptr, &Residuals, nullptr, nullptr); 

            /* update new residuals */
            GNC_Geman_McClure.updateResiduals(Residuals);

            /* recover the un-weighted (weights fromn GNC) residuals */
            GNC_Geman_McClure.updateUnweightedResiduals();

            /* update new weightings */
            GNC_Geman_McClure.updateWeightings();
            // weighings = GNC_Geman_McClure.weighings;

            /* solve the factor graph */
            solveFactorGraph(problem, options, summary);

            /* SCALING the relaxization factor */
            GNC_Geman_McClure.GNC_mu = GNC_Geman_McClure.GNC_mu / 1.4;
        }

        int length = measSize;
        for(int i = 0;  i < length; i++) // initialize
        {
            // std::cout <<"satNumPerT size-> " <<satNumPerT.size()<<std::endl;
            // std::cout <<"i -> " <<i<<std::endl;
            for(int j = 0; j < satNumPerT[i]; j++)
            {
                // std::cout <<"weighings[i][j]-> " <<GNC_Geman_McClure.weighings[i][j]<<std::endl;
            }
        }

        /* set the system as uninitialized before process next batch data */
        // disableInitialized();

        return true;
    }

    /* disable the flag  */
    bool disableInitialized()
    {
        /* the memory for weightings is alocated after the first FGO */
        *initialized = false;
        return true;
    }

    /* remove large outliers and re-solve */
    bool removeLargeOutliersAndResolve(ceres::Problem& problem,ceres::Solver::Options& options, ceres::Solver::Summary& summary)
    {
        double outlier_cnt = 0;
        double pr_cnt = -1;
        int length = measSize;
        std::vector<double> PrResiduals = GNC_Geman_McClure.Residuals;
        // std::cout << "PrResiduals.size()-> " << PrResiduals.size() << std::endl;
        for(int i = 0;  i < length; i++) // 
        {
            for(int j = 0; j < satNumPerT[i]; j++)
            {
                pr_cnt++;
                // std::cout <<"  weighings[i][j]: " << GNC_Geman_McClure.weighings[i][j] <<"  PrResiduals-> " << PrResiduals[pr_cnt]<<"\n";

                // FILE* psr_residual_weighting  =fopen("/home/wws/fgo_gnss/src/fgo_gnss/result/psr_residual_weighting.csv", "w+");
                // fprintf(psr_residual_weighting, "%7.9f, %7.9f, %2.3f \n", pr_cnt, PrResiduals[pr_cnt], w[i][j]);
                // fflush(psr_residual_weighting);  

                // if(w[i][j] < 0.3)
                // if(GNC_Geman_McClure.weighings[i][j] < 0.5) //0.7 0.5
                if(PrResiduals[pr_cnt] > 100) //0.7 0.5
                {
                    // std::cout <<" remove w[i][j]: " << GNC_Geman_McClure.weighings[i][j] <<"  PrResiduals-> " << PrResiduals[pr_cnt]<<"\n";
                    if(1)
                    {
                        problem.RemoveResidualBlock(psrIDs[pr_cnt]);
                    }
                    
                    outlier_cnt++;
                }    
            }
        }

        /* solve the factor graph */
        solveFactorGraph(problem, options, summary);

        /* remember to de-initialze the flag (otherwise segment fault) */
        disableInitialized();

        return true;
    }

    /* save graph state to vector for next solving */
    bool saveGraphStateToVector()
    {
        /* save the size of current factor graph */
        lastFactorGraphSize = measSize;

        /* get time from data stream */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;
        iter_pr = gnss_raw_map.begin();
        int length = measSize;

        /* tranverse the stateArray */
        for(int m = 0;  m < length; m++,iter_pr++) // 
        {
            nlosExclusion::GNSS_Raw_Array gnss_data = (iter_pr->second);
            double time = gnss_data.GNSS_Raws[0].GNSS_time;

            /* if the state vector is empty, override */
            if(Ps.size()==0)
            {
                Ps.push_back(std::make_pair(time, Eigen::Vector3d(state_array[m][0],state_array[m][1],state_array[m][2])));
                Clocks.push_back(std::make_pair(time, Eigen::Vector2d(state_array[m][3],state_array[m][4])));
            }
            /* if the state vector is NOT empty, update */
            else
            {
                bool findTimeKey = false;
                for(int i = 0; i < Ps.size(); i++)
                {
                    if(time == Ps[i].first)
                    {
                        Ps[i] = std::make_pair(time, Eigen::Vector3d(state_array[m][0],state_array[m][1],state_array[m][2]));
                        Clocks[i] = std::make_pair(time, Eigen::Vector2d(state_array[m][3],state_array[m][4]));
                        findTimeKey = true;
                    }
                }
                /* new time frame, add to state vector*/
                if(findTimeKey==false)
                {
                    Ps.push_back(std::make_pair(time, Eigen::Vector3d(state_array[m][0],state_array[m][1],state_array[m][2])));
                    Clocks.push_back(std::make_pair(time, Eigen::Vector2d(state_array[m][3],state_array[m][4])));
                } 
            }
            
        }
        return true;
    }
    
    /* set up the reference point for ENU calculation */
    bool setupReferencePoint()
    {
        /* reference point for ENU calculation */
        ENULlhRef.resize(3,1);
        ENULlhRef<< ref_lon, ref_lat, ref_alt;
        return true;
    }

    /* get the latest state in ENU */
    Eigen::Matrix<double ,3,1> getLatestStateENU()
    {
        int length = measSize;
        Eigen::Matrix<double ,3,1> FGOENU;
        Eigen::Matrix<double, 3,1> state;
        state<< state_array[length-1][0], 
                state_array[length-1][1], 
                state_array[length-1][2];
        FGOENU = m_GNSS_Tools.ecef2enu(ENULlhRef, state);  
        latestRecCloDrift = state_array[length-1][5];
        return FGOENU;
    }

     /* print the latest state in ENU */
    bool printLatestStateENU()
    {
        int length = measSize;
        Eigen::Matrix<double ,3,1> FGOENU;
        Eigen::Matrix<double, 3,1> state;
        state<< state_array[length-1][0], 
                state_array[length-1][1], 
                state_array[length-1][2];
        FGOENU = m_GNSS_Tools.ecef2enu(ENULlhRef, state);    
        std::cout << "ENU pose estimated by FGO (East)  -> "<< FGOENU(0)<< std::endl;
        std::cout << "ENU pose estimated by FGO (North) -> "<< FGOENU(1)<< std::endl;  
        std::cout << "ENU pose estimated by FGO (Up)    -> "<< FGOENU(2)<< std::endl;   
        std::cout << "ENU vel  estimated by FGO (East)  -> "<< state_array_vel[length-1][0]<< std::endl;
        std::cout << "ENU vel  estimated by FGO (North) -> "<< state_array_vel[length-1][1]<< std::endl;  
        std::cout << "ENU vel  estimated by FGO (Up)    -> "<< state_array_vel[length-1][2]<< std::endl;  
        std::cout << "ENU acc  estimated by FGO (East)  -> "<< state_array_acc[length-1-1][0]<< std::endl;
        std::cout << "ENU acc  estimated by FGO (North) -> "<< state_array_acc[length-1-1][1]<< std::endl;  
        std::cout << "ENU acc  estimated by FGO (Up)    -> "<< state_array_acc[length-1-1][2]<< std::endl; 
        std::cout << "ENU jer  estimated by FGO (East)  -> "<< state_array_jer[length-1-1][0]<< std::endl;
        std::cout << "ENU jer  estimated by FGO (North) -> "<< state_array_jer[length-1-1][1]<< std::endl;  
        std::cout << "ENU jer  estimated by FGO (Up)    -> "<< state_array_jer[length-1-1][2]<< std::endl; 
        std::cout << "GPS clock bias  estimated by FGO  -> "<< state_array[length-1][3]<< std::endl; 
        std::cout << "BDS clock bias  estimated by FGO  -> "<< state_array[length-1][4]<< std::endl; 
        std::cout << "Clock bias drift estimated by DOP -> "<< state_array[length-1][5]<< std::endl; 
        std::cout << "Clock bias drift estimated by PSR -> "<< state_array[length-1][3]-state_array[length-1-1][3]<< std::endl; 
        return true;
    }

    /* get the path of FGO in ENU */
    nav_msgs::Path getPathENU(nav_msgs::Path& fgo_path)
    {
        ENUTrajectory.clear();

        int length = measSize;
        Eigen::Matrix<double ,3,1> FGOENU;
        Eigen::Matrix<double, 3,1> state;
        fgo_path.poses.clear();
        fgo_path.header.frame_id = "map";
        for(int i = 0; i < length;i++)
        {
            state<< state_array[i][0], 
                    state_array[i][1], 
                    state_array[i][2];
            FGOENU = m_GNSS_Tools.ecef2enu(ENULlhRef, state);  
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = FGOENU(0);
            pose_stamped.pose.position.y = FGOENU(1);
            pose_stamped.pose.position.z = 10;
            fgo_path.poses.push_back(pose_stamped);

            // std::cout << "pose_stamped- FGO-> "<< std::endl<< pose_stamped;
        }
              
        return fgo_path;
    }


    /* get the path of FGO in ENU */
    bool getENUTrajectory()
    {
        ENUTrajectory.clear();

        int length = measSize;
        Eigen::Matrix<double ,3,1> FGOENU;
        Eigen::Matrix<double, 3,1> state;
        for(int i = 0; i < length;i++)
        {
            state<< state_array[i][0], 
                    state_array[i][1], 
                    state_array[i][2];
            FGOENU = m_GNSS_Tools.ecef2enu(ENULlhRef, state);  
    
            /*  save the trajectory */
            ENUTrajectory.push_back(FGOENU);
            // std::cout << "pose_stamped- FGO-> "<< std::endl<< pose_stamped;
        }
              
        return true;
    }

    /* update the normalized residuals at each epoch (to be used as the weighting of the polynomial_estimation)
     * Directly get the residuals based Z-h(x)
     * h(x) = ||pos_rcv-pos_sat|| + clock_bias
     */
    bool getNormalizedResiduals(ceres::Problem& problem)
    {
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr, iter_prNext;

        // state_array

        int i = 0;
        /** calculate residuals */
        
        normalizedResiduals.clear();
        for(iter_pr = gnss_raw_map.begin(); iter_pr != gnss_raw_map.end();iter_pr++, i++)
        {
            std::vector<double> residuals; // 
            nlosExclusion::GNSS_Raw_Array gnss_data = iter_pr->second;
            int sv_cnt = gnss_data.GNSS_Raws.size();
            for(int j =0; j < sv_cnt; j++)
            {
                std::string sat_sys;
                double pseudorange = gnss_data.GNSS_Raws[j].pseudorange;
                double clockBias = 0;
                double s_g_x = 0, s_g_y = 0,s_g_z = 0;
                if(m_GNSS_Tools.PRNisGPS(gnss_data.GNSS_Raws[j].prn_satellites_index)) sat_sys = "GPS";
                else sat_sys = "BeiDou";

                s_g_x = gnss_data.GNSS_Raws[j].sat_pos_x;
                s_g_y = gnss_data.GNSS_Raws[j].sat_pos_y;
                s_g_z = gnss_data.GNSS_Raws[j].sat_pos_z;

                Eigen::Vector3d satPos = Eigen::Vector3d(s_g_x,s_g_y,s_g_z);
                Eigen::Vector3d rcvPos = Eigen::Vector3d(state_array[i][0],state_array[i][1], state_array[i][2]);
                double est_pseudorange = (satPos - rcvPos).norm();
                
                double OMGE_ = 7.2921151467E-5;
                double CLIGHT_ = 299792458.0;
                est_pseudorange = est_pseudorange + OMGE_ * (s_g_x*state_array[i][1]-s_g_y*state_array[i][0])/CLIGHT_;

                if(sat_sys == "GPS") 
                {
                    est_pseudorange = est_pseudorange - state_array[i][3];
                }
                
                else if(sat_sys == "BeiDou") 
                {
                    est_pseudorange = est_pseudorange - state_array[i][4];
                }

                double residual = est_pseudorange - pseudorange;
                // if(fabs(residual)<50) // 50 
                residuals.push_back(residual);

                // std::cout<< "residual of satellite -> " << gnss_data.GNSS_Raws[j].prn_satellites_index << "  at epoch -> " << gnss_data.GNSS_Raws[0].GNSS_time <<"  is  " << residual <<"\n";
            }
            
            double normalRes = 0;
            for(int k = 0; k < residuals.size();k++)
            {
                normalRes+= std::pow(residuals[k], 2);
            }
            normalRes = std::sqrt(normalRes);
            normalRes = normalRes /residuals.size();
            normalizedResiduals.push_back(normalRes);
        }

        polynomial_estimation.varSet = normalizedResiduals;
        
        return true;
    }

    bool inputTrajectory()
    {
        polynomial_estimation.getTrajectory(ENUTrajectory);
        return true;
    }

    /* get the path of FGO in ENU */
    nav_msgs::Path getPathENUPoly(nav_msgs::Path& fgo_path)
    {

        int length = measSize;
        Eigen::Matrix<double ,3,1> FGOENU;
        fgo_path.poses.clear();
        fgo_path.header.frame_id = "map";
        /* when the epoch = length, we are predicting the next epoch */
        for(int epoch = 0; epoch <= length;epoch++)
        // for(int epoch = 0; epoch < length;epoch++)
        {
            FGOENU.setZero();
            // FGOENU = m_GNSS_Tools.ecef2enu(ENULlhRef, state);  
            for(int i = 0; i < polyDim; i++)
            {
                
                FGOENU(0) += polynomial_estimation.coef_a[i] * (std::pow(epoch+1, i));
                FGOENU(1) += polynomial_estimation.coef_b[i] * (std::pow(epoch+1, i));
            }

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = FGOENU(0);
            pose_stamped.pose.position.y = FGOENU(1);
            pose_stamped.pose.position.z = 10;
            fgo_path.poses.push_back(pose_stamped);

            if(epoch==length)
            {
                posePrediction = FGOENU;
            }

            // std::cout << "pose_stamped- FGO-> "<< std::endl<< pose_stamped;
        }
              
        return fgo_path;
    }

    /**
   * @brief maintain sliding window slidingWindowSize
   * @param gnss raw msg and doppler msg
   * @return void
   @ 
   */
    void removeStatesOutsideSlidingWindow()
    {
        int numElementsToRemove = 0;
        
        /* sliding window gnss raw pseudorange*/
        numElementsToRemove = gnss_raw_map.size() - sizeOfFactorGraph;
        std::cout<<"the setted size of factor graph"<<sizeOfFactorGraph<<std::endl;
        std::cout<<"the gnss_raw_map size"<<gnss_raw_map.size() <<std::endl;
        std::cout<<"the remove elements number"<<numElementsToRemove<<std::endl;

        if(numElementsToRemove<=0) return;
        auto i = gnss_raw_map.begin();
        while (i != gnss_raw_map.end() && numElementsToRemove > 0)
        {
            i = gnss_raw_map.erase(i);
            --numElementsToRemove;
        }
       
        /* sliding window gnss raw doppler */
        numElementsToRemove = doppler_map.size() - sizeOfFactorGraph;
        if(numElementsToRemove<=0) return;
        auto j = doppler_map.begin();
        while (j != doppler_map.end() && numElementsToRemove > 0)
        {
            j = doppler_map.erase(j);
            --numElementsToRemove;
        }

    }

void saveStatesInCurrentSlidingWindow()
{
    int length = measSize;
    std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator it;
    it = gnss_raw_map.begin();

    for(int k=0; k<length; k++, it++)
    {
        double time_stamp = it->first;
        states.position =  Eigen::Vector3d(state_array[k][0],state_array[k][1],state_array[k][2]);
        states.clocks = Eigen::Vector3d(state_array[k][3],state_array[k][4],state_array[k][5]);
        states.velocity = Eigen::Vector3d(state_array_vel[k][0],state_array_vel[k][1],state_array_vel[k][2]);
        slidingwindow_map[time_stamp] = states;
            //以下仅为验证debug
        // std::cout<<"slidingwindow  same elements number"<<  slidingwindow_map.count(time_stamp) <<"the map size"<<slidingwindow_map.size()<<std::endl;
        
    }

}

    /* free memory */
    bool freeStateMemory()
    {
        int length = measSize;
        for(int i = 0; i < length;i++)
        {
            free(state_array[i]);
        }     
        return true;
    }

    /* LOG results */
    void logBatchResults(std::string logPath)
    {
        std::ofstream foutC(logPath, std::ios::ate);
        foutC.setf(std::ios::fixed, std::ios::floatfield);

        /* add clock drift and motion model factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;
        iter_pr = gnss_raw_map.begin();
        
        int length = measSize;
        for(int m = 0;  m < length; m++, iter_pr++) // 
        {
            Eigen::Matrix<double,3,1> ECEF;
            ECEF<< state_array[m][0],state_array[m][1],state_array[m][2];

            Eigen::Matrix<double,3,1> llh = m_GNSS_Tools.ecef2llh(ECEF);
            int gps_sec = state_gps_sec_vec[m];

            /* gps time */
            foutC.precision(0);             
            foutC<<gps_sec<<",";
            foutC<<gps_sec<<",";
            
            /* longitude, latitude and altitude */
            foutC.precision(10);
            foutC<<llh(1)<<",";
            foutC<<llh(0)<<",";
            foutC<<llh(2)<<",";

            /* velocity */
            foutC.precision(10);
            foutC<<state_array_vel[m][0]<<",";
            foutC<<state_array_vel[m][1]<<",";
            foutC<<state_array_vel[m][2]<<",";

            /* acc */
            foutC.precision(10);
            foutC<<state_array_acc[m][0]<<",";
            foutC<<state_array_acc[m][1]<<",";
            foutC<<state_array_acc[m][2]<<",";

            /* jer */
            foutC.precision(10);
            foutC<<state_array_jer[m][0]<<",";
            foutC<<state_array_jer[m][1]<<",";
            foutC<<state_array_jer[m][2]<<",";

            /* number of satellites */
            foutC.precision(10);
            foutC<<iter_pr->second.GNSS_Raws.size()<<std::endl;
        }
        foutC.close();
    }

    /* LOG results */
 #if enable_save_for_batch_data
    void logResults(std::string logPath)
    { 
        std::cout<<"logPath-> " << logPath << "\n";
        std::ofstream foutC(logPath, std::ios::app);
        foutC.setf(std::ios::fixed, std::ios::floatfield);

        /* add clock drift and motion model factor */
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator iter_pr;
        iter_pr = gnss_raw_map.begin();
        
        int length = measSize;
        int m = length-1;
        // for(int m = 0;  m < length; m++, iter_pr++) // 
        {
            Eigen::Matrix<double,3,1> ECEF;
            ECEF<< state_array[m][0],state_array[m][1],state_array[m][2];

            Eigen::Matrix<double,3,1> llh = m_GNSS_Tools.ecef2llh(ECEF);
            int gps_sec = state_gps_sec_vec[m];

            /* gps time */
            foutC.precision(0);             
            foutC<<gps_sec<<",";
            foutC<<gps_sec<<",";
            
            /* longitude, latitude and altitude */
            foutC.precision(10);
            foutC<<llh(1)<<",";
            foutC<<llh(0)<<",";
            foutC<<llh(2)<<",";

            /* velocity */
            foutC.precision(10);
            foutC<<state_array_vel[m][0]<<",";
            foutC<<state_array_vel[m][1]<<",";
            foutC<<state_array_vel[m][2]<<",";

            /* acc */
            foutC.precision(10);
            foutC<<state_array_acc[m][0]<<",";
            foutC<<state_array_acc[m][1]<<",";
            foutC<<state_array_acc[m][2]<<",";

            /* jer */
            foutC.precision(10);
            foutC<<state_array_jer[m][0]<<",";
            foutC<<state_array_jer[m][1]<<",";
            foutC<<state_array_jer[m][2]<<",";

            /* number of satellites */
            foutC.precision(10);
            foutC<<iter_pr->second.GNSS_Raws.size()<<std::endl;
        }
        foutC.close();
    }
#endif
};


/* check the valid epoch based on gps time span*/
bool checkValidEpoch(double gps_sec)
{
    if((gps_sec >= start_gps_sec) && (gps_sec <=end_gps_sec))
    {
        return true;
    }
    else return false;
}