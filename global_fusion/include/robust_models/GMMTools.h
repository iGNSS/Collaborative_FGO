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

#include <ctime>
#include <cstdlib>
#include <chrono>

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

#include <mutex>
#include <thread>

using namespace std;
using namespace Eigen;

// typedef struct
// {
//     nlosExclusion::GNSS_Raw_Array GNSS_data;
//     nlosExclusion::GNSS_Raw_Array GNSS_dataCorrected;
//     Eigen::MatrixXd WLSSolution;
//     std::vector<double> residualArray;
//     std::vector<Eigen::Vector4d> SatVirtualPoseArray;
//     std::vector<Eigen::Vector3d> reflectingPointArray;
//     std::vector<int> NLOSIDArray;
//     std::vector<double> alphaArray;
//     bool updated;
// }SatData; // 

// double getDistanceFrom2Points(Eigen::Vector3d p1, Eigen::Vector3d p2)
// {
//     double xMod = pow((p1.x() - p2.x()), 2);
//     double yMod = pow((p1.y() - p2.y()), 2);
//     double zMod = pow((p1.z() - p2.z()), 2);
//     double mod = sqrt(xMod + yMod + zMod);
//     return mod;
// }



