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
#include <nlosExclusion/GNSS_Raw_Array.h>
// google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>
#include "../gnss_tools.h"

#include "pseudorange_factor_robust.h"
#include "../gnss_estimator/doppler_factor.hpp"
#include "../robust_models/gnc_gm.h"

#define stateSize 5

class FactorGraphRobust{
public:
    /* utilities */
    GNSS_Tools m_GNSS_Tools; 

    /* setup GNC-GM-based robuster */
    GNCGemanMcClure GNC_Geman_McClure;

    /* continuous data stream */
    std::map<double, nlosExclusion::GNSS_Raw_Array> gnss_raw_map;
    std::map<double, nav_msgs::Odometry> doppler_map;

    // /* Ceres solver object */
    // ceres::Problem problem;
    // ceres::Solver::Options options;
    // ceres::Solver::Summary summary;
    ceres::LossFunction *loss_function;

    /* size of factor graph */
    int sizeOfFactorGraph = 10000000;

    /* state array of factor graph */
    std::vector<double*> state_array, last_state_array;
    int* gps_sec_array;
    std::vector<int> satNumPerT;

    std::vector<int> state_gps_sec_vec;

    /* apply  weighting when state is initialized via FGO */
    bool* initialized;

    /* vector to save factor ids*/
    std::vector<ceres::ResidualBlockId> psrIDs;

    // /* weightings */
    // std::vector<double*> weighings; 

    /* state saved in vector pair */
    std::vector<std::pair<double, Eigen::Vector3d>> Ps;
    std::vector<std::pair<double, Eigen::Vector2d>> Clocks;

    /* size of the last factor graph optimization */
    int lastFactorGraphSize = 0;

    /* fixed variance of doppler measurements */
    double var = 0.6;

    /* reference point for ENU calculation */
    Eigen::MatrixXd ENULlhRef;

    /* measurements size */
    int measSize = 0;

    /* parameters */
    int numOfPsrFactors = 0;
    int numOfDopplerFactors = 0;
    int numOfStates =0;
    

public:
    /* input gnss doppler data */
    bool clearVariables()
    {
        psrIDs.clear();
        satNumPerT.clear();
        state_gps_sec_vec.clear();
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
        gnss_raw_map.clear();
        doppler_map.clear();
        return true;
    }

    /* set up ceres-solver options */
    bool setupSolverOptions(ceres::Solver::Options& options)
    {
        options.use_nonmonotonic_steps = true;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
        options.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
        options.num_threads = 8;
        options.max_num_iterations = 258;
        return true;
    }

    /* set up Loss functions options */
    bool setupLossFunction(std::string loss)
    {
        if(loss=="Huber")
            loss_function = new ceres::HuberLoss(1.0);
        else 
        {
            loss_function = new ceres::CauchyLoss(1.0);
        }
        return true;
    }

    /* get data stream size */  
    int getDataStreamSize()
    {
        measSize = gnss_raw_map.size();
        return measSize;
    }

    bool initializeFactorGraphParas()
    {
        numOfPsrFactors = 0;
        numOfDopplerFactors = 0;
        numOfStates =0;
    }

    /* setup state size */
    bool setupStateMemory()
    {
        state_array.reserve(measSize);
        int length = measSize;
        LOG(INFO) << "length" << length << std::endl;

        for(int i = 0; i < length;i++)
        {
            /* x, y, z, gps_cloc_bias, beidou_cloc_bias */
            state_array[i] = new double[stateSize]; //
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
                // state_array[i][0] = 0;
                // state_array[i][1] = 0; 
                // state_array[i][2] = 0;
                // state_array[i][3] = 0;
                // state_array[i][4] = 0;
                state_array[i][0] = eWLSSolutionECEF(0);
                state_array[i][1] = eWLSSolutionECEF(1); 
                state_array[i][2] = eWLSSolutionECEF(2);
                state_array[i][3] = eWLSSolutionECEF(3);
                state_array[i][4] = eWLSSolutionECEF(4);
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
                                                        , 5,5>(new 
                                                        dopplerFactor(v_x_i, v_y_i, v_z_i, delta_t,  var_vec));
                problem.AddResidualBlock(doppler_function, loss_function, state_array[i],state_array[i+1]);
                numOfDopplerFactors++;
            }
        }
        return true;
    }


    /* add pseudorange FACTORS */
    bool addPseudorangeFactors(ceres::Problem& problem)
    {
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

            int factor_index = -1;
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

                ceres::CostFunction* ps_function = new ceres::AutoDiffCostFunction<pseudorangeFactorRobust, 1 
                                                                , 5>(new 
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
                
                psrIDs.push_back(ID);
                numOfPsrFactors++;
            }
            satNumPerT.push_back(sv_cnt);            
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
        std::vector<double> residuals; // 
        
        problem.Evaluate(EvalOpts, nullptr, &residuals, nullptr, nullptr); 

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

    /* update GNC-GM iteratively */
    bool updateGNCGMIteratively(ceres::Problem& problem,ceres::Solver::Options& options, ceres::Solver::Summary& summary)
    {
        /* solve the FGO-GNC via alternate minimization */
        GNC_Geman_McClure.GNC_mu = 1.2;
        while(GNC_Geman_McClure.GNC_mu > 1 )
        {
            /* times of GNC iterations */
            // std::cout << "GNC_iter-> " << GNC_Geman_McClure.GNC_iter<< std::endl;
            // std::cout << "max residual-> " << GNC_Geman_McClure.GNC_r_max << std::endl;
            // std::cout << "GNC_mu-> " << GNC_Geman_McClure.GNC_mu<< std::endl;
            
            
            /** set up objects */
            ceres::Problem::EvaluateOptions EvalOpts;
            EvalOpts.num_threads = 8;
            EvalOpts.apply_loss_function = false;

            /** only pseudorange error gets evaluated */
            EvalOpts.residual_blocks = psrIDs;

            /** calculate residuals */
            std::vector<double> Residuals; // 
            problem.Evaluate(EvalOpts, nullptr, &Residuals, nullptr, nullptr); 

            /* update new residuals */
            GNC_Geman_McClure.updateResiduals(Residuals);

            /* update new residuals */
            GNC_Geman_McClure.updateWeightings();
            // weighings = GNC_Geman_McClure.weighings;

            /* solve the factor graph */
            solveFactorGraph(problem, options, summary);

            /* SCALING the relaxization factor */
            GNC_Geman_McClure.GNC_mu = GNC_Geman_McClure.GNC_mu / 1.4;
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
                if(GNC_Geman_McClure.weighings[i][j] < 0.5) //0.7 0.5
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
        std::cout << "FGOENU-> "<< FGOENU<< std::endl;    
        return true;
    }

    /* get the path of FGO in ENU */
    nav_msgs::Path getPathENU(nav_msgs::Path& fgo_path)
    {
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
    void logResults(std::string logPath)
    {
        std::ofstream foutC(logPath, std::ios::ate);
        foutC.setf(std::ios::fixed, std::ios::floatfield);
        
        int length = measSize;
        for(int m = 0;  m < length; m++) // 
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
            foutC<<llh(2)<<std::endl;
        }
        foutC.close();
    }

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