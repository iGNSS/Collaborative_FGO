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

#include "globalOpt.h"
#include "Factors.h"
#include "SumMixture.h"

/**
 * @brief construction function
 * @param none
 * @return none
 */
GlobalOptimization::GlobalOptimization()
{
	initGPS = false;
    newGPS = false;
	WGPS_T_Wlio = Eigen::Matrix4d::Identity(); // set the extrinsic as identity matrix
    threadOpt  = std::thread(&GlobalOptimization::optimize, this); // optimization thread
    // threadTest = std::thread(&GlobalOptimization::testPrint, this); // test thread 

    /* initialize "NumberOfComponents" GMMs */
    GMMX.initSpread(NumberOfComponents,10);
    GMMY.initSpread(NumberOfComponents,10);

}

/**
 * @brief De-construction function
 * @param none
 * @return none
 */
GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
    // threadTest.detach();
}

/**
 * @brief test the print thread
 * @param none
 * @return none
 */
void GlobalOptimization::testPrint()
{
    while (1)
    {
        std::cout << "Thread 2 is running" << std::endl;
        std::chrono::milliseconds dura(2000); // this thread sleep for 2000 ms
        std::this_thread::sleep_for(dura);   
    }
    return;
}

/**
 * @brief geodetic to local 
 * @param latitude, longitude, altitude, *xyz
 * @return none
 */
void GlobalOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initGPS)
    {
        bool setTangentPoint =1; 
        if(setTangentPoint)
        {
            /* Tagent point for dataset Jiandong 20190428 */
            latitude  = 22.3011536799;
            longitude = 114.179000783;
            altitude  = 6.338692775;
        }
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    //printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}

/**
 * @brief input lio and fitness score
 * @param none
 * @return none
 */
void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ, Eigen::Vector3d lio_cov)
{
    /* lock the thread */
	mPoseMap.lock();

    std::vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z(),lio_cov(0)};
    localPoseMap[t] = localPose; // local pose and fitness score from lio

    Eigen::Quaterniond globalQ;
    globalQ = WGPS_T_Wlio.block<3, 3>(0, 0) * OdomQ; // (from global to original of lio)*(lio odometry)
    Eigen::Vector3d globalP = WGPS_T_Wlio.block<3, 3>(0, 0) * OdomP + WGPS_T_Wlio.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;
    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);

    /* unlock the thread */
    mPoseMap.unlock();
}

/**
 * @brief get lastest Global odom
 * @param &odomP, &odomQ
 * @return none
 */
void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

/**
 * @brief get lastest Global odom
 * @param &odomP, &odomQ
 * @return none
 */
Eigen::Vector3d GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
    Eigen::Vector3d GNSSOdom;
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	// vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    
    std::default_random_engine generator;
    auto dist_E = std::bind(std::normal_distribution<double>{0, 5},
                    std::mt19937(std::random_device{}()));
    double random_psr = dist_E();
    vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    
    // std::cout<<"enu1 ->: " << enu1 << "\n";
    // trans and rotation
    double prex_ = tmp[0];
    double prey_ = tmp[1];
    // double theta = (3.65 )*( 3.141592 / 180.0 ); // kowllon ton
    double theta = (228.038 + 90 )*( 3.141592 / 180.0 ); // extrinsic (heading) between gnss and lidar odometry
    tmp[0] = prex_ * cos(theta) - prey_ * sin(theta) ;
    tmp[1] = prex_ * sin(theta) + prey_ * cos(theta) ; 
    // std::cout << "GPS in ENU ->  "<< tmp[0] << " " << tmp[1] << " " << tmp[2] << std::endl;

    /** Add noise to GPS positioning */
    // addGaussianNoise(tmp, GaussianNoiseSTD, GaussianNoiseSTD); 

	GPSPositionMap[t] = tmp;
    newGPS = true;
    GNSSOdom << tmp[0], tmp[1], tmp[2];
    return GNSSOdom;

}

void GlobalOptimization::optimize()
{
    while(true)
    {
        if(newGPS)
        {
            newGPS = false;
            // printf("Thread 1: global optimization\n");
            TicToc globalOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
            options.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
            options.num_threads = 8;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 100; // 5
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            loss_function = NULL;
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
            // ceres::LocalParameterization* local_parameterization =new EigenQuaternionParameterization; 

            //add param
            mPoseMap.lock();
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++) // initialize all the states (initial guess for optimization)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];

                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                // problem.SetParameterBlockConstant(q_array[i]);

                problem.AddParameterBlock(t_array[i], 3);
                
                /* set the first epoch constant */
                if(i==0) 
                {
                    problem.SetParameterBlockConstant(q_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }
            }

            map<double, vector<double>>::iterator iterlio, iterlioNext, iterGPS;
            int i = 0;
            int GPSMapsize = GPSPositionMap.size();
            int GPSMapIndex = 0;
            for (iterlio = localPoseMap.begin(); iterlio != localPoseMap.end(); iterlio++, i++)
            {
                /** Add lio factors */
                iterlioNext = iterlio;
                iterlioNext++;
                if(iterlioNext != localPoseMap.end()) 
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterlio->second[3], iterlio->second[4], 
                                                               iterlio->second[5], iterlio->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterlio->second[0], iterlio->second[1], iterlio->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterlioNext->second[3], iterlioNext->second[4], 
                                                               iterlioNext->second[5], iterlioNext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterlioNext->second[0], iterlioNext->second[1], iterlioNext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj; // delta transformation
                    Eigen::Quaterniond iQj; // delta rotation
                    iQj = iTj.block<3, 3>(0, 0); // delta rotation
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3); // delta translation
                    
                    /* get self-tuning cov for lio based on fitness score */
                    double adaptive_lioVarT = iterlioNext->second[7];
                    if(adaptive_lioVarT<0.00001) adaptive_lioVarT = 0.00001;
                    // adaptive_lioVarT = adaptive_lioVarT * 4;
                    ceres::CostFunction* lio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(), // translation
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(), // rotation
                                                                                // 0.1, 0.01); // original version
                                                                                // 0.1, 0.001); // scaled version
                                                                                // lioVarT, lioVarR);
                                                                                adaptive_lioVarT, lioVarR);
                    std::cout << "adaptive_lioVarT-> "<<adaptive_lioVarT<<std::endl;
                    problem.AddResidualBlock(lio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);
                }

                /** Add GPS factors */
                double t = iterlio->first;
                iterGPS = GPSPositionMap.find(t);
                int GNSSFactorModel =1; // 0: GNSS 3D, 1: GNSS 2D, 2: GNSS GaussianMixture
                if (iterGPS != GPSPositionMap.end())
                {
                    GPSMapIndex++; // index of GPS mes
                    if(GPSMapIndex==(GPSMapsize-1)) // apply GMM only for current epoch
                    // if(1) //
                    {
                        if(GNSSFactorModel==0) // GNSS 3D factor 
                        {
                            ceres::CostFunction* gps_function = new ceres::AutoDiffCostFunction<GNSSTError, 3 /* size of residual */
                                                            , 3/* size of parameter block t_array */>(new 
                                                            GNSSTError(iterGPS->second[0], iterGPS->second[1], 
                                                            iterGPS->second[2], iterGPS->second[3]));
                            problem.AddResidualBlock(gps_function, loss_function, t_array[i]); //t_array[i] is passed to the operator function
                        }
                        else if(GNSSFactorModel==1) // GNSS 2D factor
                        {
                            ceres::CostFunction* gps_function = new ceres::AutoDiffCostFunction<GNSSTError2D, 2 /* size of residual */
                                                            , 3/* size of parameter block t_array */>(new 
                                                            GNSSTError2D(iterGPS->second[0], iterGPS->second[1], 
                                                            // iterGPS->second[3]));
                                                            GPSSTD));
                            problem.AddResidualBlock(gps_function, loss_function, t_array[i]); //t_array[i] is passed to the operator function
                        }
                        else if(GNSSFactorModel==2) // GNSS 2D with Gaussian Mixture Model (GMM)
                        {
                            libRSF::GaussianMixture<1> GMMXTmp, GMMYTmp;
                            // fixedGMM.initSpread(2,iterGPS->second[3]); // 0.1
                            // fixedGMM.initSpread(2,0.1); // 0.1
                            // std::cout << "/* getNumberOfComponents GMM X->*/" << GMMX.getNumberOfComponents()<< std::endl;
                            if(GMMX.getNumberOfComponents()>=1) // at least one Gaussian component
                            // if(GMMXMap[iterGPS->first].getNumberOfComponents()>1) // at least one Gaussian component
                            {
                                GMMXTmp = GMMX;
                                // GMMXTmp = GMMXMap[iterGPS->first];
                            }
                            else // directly use Gaussian Model
                            {
                                // GMMXTmp.initSpread(1,iterGPS->second[3] * 0.1); // 0.1
                                GMMXTmp.initSpread(1,GPSSTD); // 0.1
                            }

                            if(GMMY.getNumberOfComponents()>=1) // at least one Gaussian component
                            // if(GMMYMap[iterGPS->first].getNumberOfComponents()>1) // at least one Gaussian component
                            {
                                GMMYTmp = GMMY;
                                // GMMYTmp = GMMYMap[iterGPS->first];
                            }
                            else // directly use Gaussian Model
                            {
                                // GMMYTmp.initSpread(1,iterGPS->second[3] * 0.1); // 0.1
                                GMMYTmp.initSpread(1,GPSSTD); // 0.1
                            }
                            
                            ceres::CostFunction* gps_function = new ceres::AutoDiffCostFunction<GNSSTError2DGMM, 2 /* size of residual */
                                                                , 3/* size of parameter block t_array */>(new 
                                                                GNSSTError2DGMM(iterGPS->second[0], iterGPS->second[1], 
                                                                iterGPS->second[3],GMMXTmp, GMMYTmp));
                            problem.AddResidualBlock(gps_function, loss_function, t_array[i]); //t_array[i] is passed to the operator function
                        }
                        else if(GNSSFactorModel==-1) // GNSS 3D factor (original version from vins-fusion)
                        {
                            ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                                                                        iterGPS->second[2], GPSSTD);
                            problem.AddResidualBlock(gps_function, loss_function, t_array[i]); //t_array[i] is passed to the operator function
                        }
                    }
                    else
                    {
                        ceres::CostFunction* gps_function = new ceres::AutoDiffCostFunction<GNSSTError2D, 2 /* size of residual */
                                                            , 3/* size of parameter block t_array */>(new 
                                                            GNSSTError2D(iterGPS->second[0], iterGPS->second[1], 
                                                            GPSSTD));
                        problem.AddResidualBlock(gps_function, loss_function, t_array[i]); //t_array[i] is passed to the operator function
                    }
                    
                }

            }
            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";
            
            /* Get GNSS residual*/ 
            Eigen::MatrixXd ErrorXY; // gps residual
            ErrorXY.resize(GPSPositionMap.size(),3);
            double dErrorX =0, dErrorY =0, dErrorZ =0;
            int gpsCnt = 0;
            i =0;
            FILE* GNSSLooseResidual = fopen("../ion_PLANS_2020_GNSS_LiDAR/src/result/GNSSLooseResidual.csv", "w+");
            for (iterlio = localPoseMap.begin(); iterlio != localPoseMap.end(); iterlio++, i++)
            {
                //lio factor
                iterlioNext = iterlio;
                iterlioNext++;
                double t = iterlio->first;
                iterGPS = GPSPositionMap.find(t);
                // std::cout << "/* GPSPositionMap.size() */" <<GPSPositionMap.size()<< std::endl;
                if (iterGPS != GPSPositionMap.end())
                {
                    ErrorXY(gpsCnt,0) = (iterGPS->second[0] - t_array[i][0]) * residualscalingFactor;
                    ErrorXY(gpsCnt,1) = (iterGPS->second[1] - t_array[i][1]) * residualscalingFactor;
                    ErrorXY(gpsCnt,2) = (iterGPS->second[2] - t_array[i][2]) * residualscalingFactor;
                    // std::cout << "/* ErrorXY(gpsCnt,0) */" <<ErrorXY(gpsCnt,0)<< std::endl;
                    // std::cout << "/* ErrorXY(gpsCnt,1) */" <<ErrorXY(gpsCnt,1)<< std::endl;
                    // std::cout << "/* ErrorXY(gpsCnt,2) */" <<ErrorXY(gpsCnt,2)<< std::endl;
                    fprintf(GNSSLooseResidual, "%d,%7.5f,%7.5f,%7.5f  \n", gpsCnt, ErrorXY(gpsCnt,0),ErrorXY(gpsCnt,1),ErrorXY(gpsCnt,2));
                    fflush(GNSSLooseResidual);
                    gpsCnt++;
                    std::cout << "/* gpsCnt */ " <<gpsCnt<< std::endl;
                }

                ++iterGPS;
                if (iterGPS == GPSPositionMap.end())
                {
                    --iterGPS;
                    dErrorX = (iterGPS->second[0] - t_array[i][0]) * residualscalingFactor;
                    dErrorY = (iterGPS->second[1] - t_array[i][1]) * residualscalingFactor;
                    dErrorZ = (iterGPS->second[2] - t_array[i][2]) * residualscalingFactor;
                    std::cout << "/* find the last GPS */" << "dErrorX-> "<< dErrorX<< "dErrorY-> "<< dErrorY<<  std::endl;
                }
            }

            
            TicToc EMTime;
            // std::vector<double> errorX;
            // std::vector<double> errorY;

            // errorX = getVectorFromEigen(ErrorXY,0);
            // errorY = getVectorFromEigen(ErrorXY,1);
            
            bool useSquareResial =0;
            if(useSquareResial)
            {
                double square_res = sqrt(pow(dErrorX,2) + pow(dErrorY,2));
                if(square_res>6) square_res =6;
                errorX.push_back(square_res);
                errorY.push_back(square_res);
            }
            else 
            {
                errorX.push_back(dErrorX);
                errorY.push_back(dErrorY);
            }
            

            /** keep the residual inside the window */
            removeResidualsOutsideWindow(errorX, GMMResidualWindowSize);
            removeResidualsOutsideWindow(errorY, GMMResidualWindowSize);
            
            /** create default model */
            GMMX.initSpread(NumberOfComponents, DefaultStdDev);
            GMMY.initSpread(NumberOfComponents, DefaultStdDev);

            /** adapt error model */
            // std::cout << "/* errorX.size()->  */" <<errorX.size() << std::endl;
            GMMX.estimateWithEM(errorX, ReduceComponents);
            // std::cout << "/* errorY.size()->  */" <<errorY.size() << std::endl;
            GMMY.estimateWithEM(errorY, ReduceComponents);

            GMMXMap[GPSPositionMap.end()->first] = GMMX;
            GMMYMap[GPSPositionMap.end()->first] = GMMY;

            // std::cout << "GPSPositionMap.size()-> " <<GPSPositionMap.size() <<"  GMMXMap.size()-> " <<GMMXMap.size() <<std::endl;

            // printf("Time for Expectation maximization: %f ms \n", EMTime.toc());

            // if(RemoveOffset)
            // {
            //     /** try to set main component to zero : specially for pseudorange measurements*/
            //     GMM.removeOffset();
            // }

            /** update global pose */
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d Wlio_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    Wlio_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    Wlio_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    
                    WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    
                    WGPS_T_Wlio = WGPS_T_body * Wlio_T_body.inverse();
                    // WGPS_T_Wlio = Wlio_T_body.inverse() * WGPS_T_body;
            	}
            }
            updateGlobalPath();
            // printf("Time for global optimization %f ms\n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(1); // this thread sleep for 100 ms
        std::this_thread::sleep_for(dura);
    }
	return;
}


void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }
}