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


#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"
#include "GMMTools.h"

#include "ErrorModel.h"
#include "Gaussian.h"
#include "GaussianComponent.h"
#include "GaussianMixture.h"
#include "SumMixture.h"
#include "VectorMath.h"

using namespace std;

/**
 * @brief fuse multiple measurements with fator graph
 * @param multiple sensor measurements
 * @return none
 */
class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	Eigen::Vector3d inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ, Eigen::Vector3d lio_cov);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
	nav_msgs::Path global_path;

	void testPrint(); // testing for a independent thread 

	/**
	 * @brief get a certain vector from matrix
	 * @param matrix, cols ID
	 * @return vector
	 */
	std::vector<double> getVectorFromEigen(Eigen::MatrixXd ErrorXY, int colsID)
	{
		int rows = ErrorXY.rows();
		// int cols = ErrorXY.cols();
		std::vector<double> errorVector;
		for(int i = 0; i < rows; i++)
		{
			errorVector.push_back(ErrorXY(i,colsID));
		}
		return errorVector;
	}

	/**
	 * @brief remove elements outside a given window
	 * @param vector, window size
	 * @return none
	 */
	void removeResidualsOutsideWindow(std::vector<double>& error, int windowSize)
	{
		std::vector<double> errorTmp;
		if(error.size()> windowSize)
		{
			for (int i = 0; i < error.size(); i++)
			{
				if(i > (error.size()-windowSize))
				{
					errorTmp.push_back(error[i]);
				}			
			}
			error = errorTmp;
		}
	}
	
	/**
	 * @brief add Gaussian noise to East and North directions
	 * @param vector, std1, std2
	 * @return none
	 */
	void addGaussianNoise(std::vector<double> &data, double std1, double std2)
	{
		/* get noised E */
		auto dist_E = std::bind(std::normal_distribution<double>{0, std1},
						std::mt19937(std::random_device{}()));
		double random_E = dist_E();
		data[0] = data[0] + random_E;

		/* get noised N */
		auto dist_N = std::bind(std::normal_distribution<double>{0, std2},
						std::mt19937(std::random_device{}()));
		double random_N = dist_N();
		data[1] = data[1] + random_N;
	}
	

private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
	void optimize();
	void updateGlobalPath();
	

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap; // pose from lio 
	map<double, vector<double>> globalPoseMap; // global pose after optimization
	map<double, vector<double>> GPSPositionMap; // pose from gps

	bool initGPS;
	bool newGPS;
	GeographicLib::LocalCartesian geoConverter;

	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_Wlio; // transformation between original of gps and lio (extrinsic)
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;

	std::thread threadOpt;
	std::thread threadTest;

	int NumberOfComponents = 3;
	double DefaultStdDev = 7.0;
	bool ReduceComponents =true;
	bool RemoveOffset = true; // set true?
	double residualscalingFactor = 1; // 0,1, 0.01

	libRSF::GaussianMixture<1> GMMX;
	libRSF::GaussianMixture<1> GMMY;

	std::vector<double> errorX;
    std::vector<double> errorY;

	int GMMResidualWindowSize = 10; // 35 

	map<double, libRSF::GaussianMixture<1>> GMMXMap;
	map<double, libRSF::GaussianMixture<1>> GMMYMap;

	double lioVarT = 0.8; // 0.1  (0.3 seems to be good) 0.6 0.5 
	double lioVarR = 0.001; // 0.001
	double GPSSTD = 7.0; // 6

	double GaussianNoiseSTD = 20;

};