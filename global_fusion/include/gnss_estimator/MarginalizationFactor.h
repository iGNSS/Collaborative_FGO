/*******************************************************
 * Copyright (C) 2021, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University
 * 
 * This file is part of la_gnss.
 *
 * Author: Weisong Wen (weisong.wen@connect.polyu.hk)
 * 
 * We make reference to the RTKLIB, GraphGNSSLib, etc. 
 * 
 * Wen, Weisong, and Li-Ta Hsu. "Towards Robust GNSS Positioning and Real-time Kinematic Using Factor Graph Optimization." arXiv preprint arXiv:2106.01594 (2021).
 * 
 * Weisong Wen, Xiwei Bai, Li-ta Hsu. (2021). 3D Vision Aided
GNSS Real-time Kinematic Positioning for Autonomous Systems in Urban
Canyons. IEEE Transactions on Robotics (Under Review).
 *
 * */


#ifndef MARGINALIZATIONFACTOR_H_
#define MARGINALIZATIONFACTOR_H_

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include <unordered_map>
#include <ros/ros.h>


#include "common.h"
#include "math_tools.h"

const int NUM_THREADS = 4;

struct ResidualBlockInfo {
    ResidualBlockInfo(ceres::CostFunction *_cost_function,
                      ceres::LossFunction *_loss_function,
                      std::vector<double *> _parameter_blocks,
                      std::vector<int> _drop_set)
        : cost_function(_cost_function),
          loss_function(_loss_function),
          parameter_blocks(_parameter_blocks),
          drop_set(_drop_set) {}

    void Evaluate();

    ceres::CostFunction *cost_function;
    ceres::LossFunction *loss_function;
    std::vector<double *> parameter_blocks;
    std::vector<int> drop_set;

    double **raw_jacobians;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    Eigen::VectorXd residuals;
};

class MarginalizationInfo {
public:
    ~MarginalizationInfo();
    int LocalSize(int size) const;
    void AddResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    void PreMarginalize();
    void Marginalize();
    std::vector<double *> GetParameterBlocks(std::unordered_map<long, double *> &addr_shift);

    std::vector<ResidualBlockInfo *> factors;
    int m, n;
    std::unordered_map<long, int> parameter_block_size; //global size
    int sum_block_size;
    std::unordered_map<long, int> parameter_block_idx; //local size
    std::unordered_map<long, double *> parameter_block_data;

    std::vector<int> keep_block_size; //global size
    std::vector<int> keep_block_idx;  //local size
    std::vector<double *> keep_block_data;

    Eigen::MatrixXd linearized_jacobians;
    Eigen::VectorXd linearized_residuals;
    const double eps = 1e-8;
};

class MarginalizationFactor : public ceres::CostFunction {
public:
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    MarginalizationInfo* marginalization_info;
};

struct ThreadsStruct {
    std::vector<ResidualBlockInfo *> sub_factors;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::unordered_map<long, int> parameter_block_size; //global size
    std::unordered_map<long, int> parameter_block_idx; //local size
};

#endif //MARGINALIZATIONFACTOR_H_
