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

#include <ceres/ceres.h>

 
#define polyDim 5 // 3 7 (3 is square loss) 7

/* doppler factor*/
struct poseFittingFactor3
{
    poseFittingFactor3(double px, double py, double var, double epoch)
                :px(px),py(py), var(var), epoch(epoch){}

    template <typename T>
    bool operator()(const T* state_a, const T* state_b, T* residuals) const
    {

        T est_px = T(0);
        for(int i = 0; i < polyDim; i++)
        {
            est_px += state_a[i] * T(ceres::pow(epoch, i));
        }

        T est_py = T(0);
        for(int i = 0; i < polyDim; i++)
        {
            est_py += state_b[i] * T(ceres::pow(epoch, i));
        }

        residuals[0] = (est_px - T(px)) / T(var);
        residuals[1] = (est_py - T(py)) / T(var);

        return true;
    }

    double px, py, pz, var;
    double epoch;
};


/* pseudorange factor*/
class PolynomialEstimation{
public:
    /* trajectory */
    std::vector<Eigen::Matrix<double ,3,1>> ENUTrajectory;
    std::vector<double> varSet;

    double coef_a[polyDim]; // coefficients for east direction
    double coef_b[polyDim]; // coefficients for north direction

    bool getTrajectory(std::vector<Eigen::Matrix<double ,3,1>> data)
    {
        ENUTrajectory.clear();
        ENUTrajectory = data;

        return true;
    }

    bool optimizePolyParas()
    {
        /* define the problem */
        ceres::Problem problem;
        ceres::Solver::Options options;
        ceres::Solver::Summary summary;
        options.use_nonmonotonic_steps = true;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
        options.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
        options.num_threads = 8;
        options.max_num_iterations = 258;

        problem.AddParameterBlock(coef_a,polyDim);
        problem.AddParameterBlock(coef_b,polyDim); 

        int size = ENUTrajectory.size();

        for(int i = 0; i < size; i++)
        {
            double px = ENUTrajectory[i](0);
            double py = ENUTrajectory[i](1);
            double var = varSet[i];
            // std::cout << "varSet[i]->  "<< var << std::endl;

            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);

            ceres::CostFunction* pose_function = new ceres::AutoDiffCostFunction<poseFittingFactor3, 2 
                                                                , polyDim,polyDim>(new 
                                                                poseFittingFactor3(px, py, var, i+1));
            auto ID = problem.AddResidualBlock(pose_function, loss_function, coef_a, coef_b);
        }

        /* solve the problem*/
        ceres::Solve(options, &problem, &summary);
        for(int i = 0; i < polyDim; i++)
        {
            // std::cout << "coef_a[i]-> " << coef_a[i] << "\n";
            // std::cout << "coef_b[i]-> " << coef_b[i] << "\n";
        }

        return true;
    }

};


