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

/* doppler factor*/
struct dopplerFactor
{
    dopplerFactor(double v_x, double v_y, double v_z, double delta_t, Eigen::Vector3d var_vector)
                :v_x(v_x),v_y(v_y), v_z(v_z), delta_t(delta_t), var_vector(var_vector){}

    template <typename T>
    bool operator()(const T* state_i, const T* state_j, T* residuals) const
    {
        T est_v_x = (state_j[0] - state_i[0])/ delta_t;
        T est_v_y = (state_j[1] - state_i[1])/ delta_t;
        T est_v_z = (state_j[2] - state_i[2])/ delta_t;

        // est_pseudorange = sqrt(delta_x+ delta_y + delta_z);

        residuals[0] = (est_v_x - T(v_x)) / T(var_vector(0));
        residuals[1] = (est_v_y - T(v_y)) / T(var_vector(1));
        residuals[2] = (est_v_z - T(v_z)) / T(var_vector(2));

        return true;
    }

    double v_x, v_y, v_z, var;
    Eigen::Vector3d var_vector;
    double delta_t;
    std::string sat_sys; // satellite system
};

/* tightly coupled doppler factor*/
struct tcdopplerFactor
{
    tcdopplerFactor(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double s_v_x, double s_v_y, double s_v_z, double svddt, double doppler, double delta_t, double var)
                :sat_sys(sat_sys),s_g_x(s_g_x), s_g_y(s_g_y), s_g_z(s_g_z), s_v_x(s_v_x), s_v_y(s_v_y), s_v_z(s_v_z), svddt(svddt), doppler(doppler),delta_t(delta_t), var(var){}

    template <typename T>
    bool operator()(const T* statePj,const T* stateVj, T* residuals) const
    {
        T est_range; 
        T delta_x = pow((statePj[0] - s_g_x),2);
        T delta_y = pow((statePj[1] - s_g_y),2);
        T delta_z = pow((statePj[2] - s_g_z),2);
        est_range = sqrt(delta_x+ delta_y + delta_z);

        T los_x = T(1) * (s_g_x - statePj[0]) / est_range;
        T los_y = T(1) * (s_g_y - statePj[1]) / est_range;
        T los_z = T(1) * (s_g_z - statePj[2]) / est_range;

        T est_dop = (s_v_x - stateVj[0]) * los_x + (s_v_y - stateVj[1]) * los_y + (s_v_z - stateVj[2]) * los_z; 

        double OMGE_ = 7.2921151467E-5;
        double CLIGHT_ = 299792458.0;

        // T relatisDop = OMGE_ * (sv_vel(0)*P_ecef(1)+
        //     sv_pos(0)*V_ecef(1) - sv_vel(1)*P_ecef(0) - sv_pos(1)*V_ecef(0))/CLIGHT_; 

        T relatisDop = OMGE_/CLIGHT_ * (s_v_x*statePj[1] + s_g_x*stateVj[1] - s_v_y*statePj[0] - s_g_y*stateVj[0]); // 3DVA GNSS-RTK
        
        // T relatisDop = OMGE_/CLIGHT_ * (s_v_y*statePj[0] + s_g_y*stateVj[0] - s_v_x*statePj[1] - s_g_x*stateVj[1]); // RTKLIB

        est_dop = est_dop + relatisDop - svddt;   

        // est_dop = est_dop - svddt;   
        
        // receiver clock drift
        if(sat_sys == "GPS") 
        {
            est_dop = est_dop + statePj[5];
        }
        
        else if(sat_sys == "BeiDou") 
        {
            est_dop = est_dop + statePj[5];
        }
        
        /* in RTKLIB, the covariance of the Doppler measurements is set as fixed covariance */
        residuals[0] = (est_dop - doppler) / T(var); // for huawei watch dataset
        // residuals[0] = (est_dop + doppler) / T(var); // for huawei p40 phone and u-blox data 0.2 0.3 

        // residuals[0] = (est_dop + doppler) / T(0.02); // for huawei p40 phone and u-blox data

        // std::cout<< "Doppler weight-> " << var << std::endl;

        // std::cout << "residuals[0]-> "<< residuals[0]<<std::endl;

        return true;
    }

    double s_g_x, s_g_y, s_g_z, svddt, var;
    double s_v_x, s_v_y, s_v_z; // satellite velocity
    std::string sat_sys; // satellite system

    double delta_t;
    double doppler;

};

/* doppler factor*/
struct dopplerConstraint
{
    typedef ceres::DynamicAutoDiffCostFunction<dopplerConstraint, 10>
      dopplerDynaCostFunction;
    dopplerConstraint(double v_x, double v_y, double v_z, double delta_t, Eigen::Vector3d var_vector, int keyIndex)
                :v_x(v_x),v_y(v_y), v_z(v_z), delta_t(delta_t), var_vector(var_vector),keyIndex(keyIndex){}

    template <typename T>
    bool operator()(T const* const* state, T* residuals) const
    {
        /* state[keyIndex][0]   -> state_i
           state[keyIndex+1][0] -> state_j 
        */
        T est_v_x = (state[keyIndex+1][0] - state[keyIndex][0])/ delta_t;
        T est_v_y = (state[keyIndex+1][1] - state[keyIndex][1])/ delta_t;
        T est_v_z = (state[keyIndex+1][2] - state[keyIndex][2])/ delta_t;

        // est_pseudorange = sqrt(delta_x+ delta_y + delta_z);

        residuals[0] = (est_v_x - T(v_x)) / T(var_vector(0));
        residuals[1] = (est_v_y - T(v_y)) / T(var_vector(1));
        residuals[2] = (est_v_z - T(v_z)) / T(var_vector(2));

        return true;
    }

    // Factory method to create a CostFunction from a DDCarrierPhaseConstraint to
    // conveniently add to a ceres problem.
    static dopplerDynaCostFunction* Create(double v_x, double v_y, double v_z, double delta_t, Eigen::Vector3d var_vector, int keyIndex, std::vector<double*>* state_array, std::vector<double*>* pose_parameter_blocks, std::vector<int*>* ar_state_num) {
        
        dopplerConstraint* constraint = new dopplerConstraint(
            v_x, v_y, v_z, delta_t, var_vector, keyIndex);
        
        dopplerDynaCostFunction* cost_function = new dopplerDynaCostFunction(constraint);
        
        pose_parameter_blocks->clear();
        // double a[5] = {1,2,3,4,5};
        // parameter_blocks->push_back(a);
        // parameter_blocks->push_back(&((*state_array)[keyIndex]));
        // parameter_blocks->push_back(state_array[keyIndex]);
        
        /* push back one more state ("+1") corresponding to state_j */
        for(int i = 0; i <(keyIndex+1+1); i++)
        {
            pose_parameter_blocks->push_back((*state_array)[i]);
            cost_function->AddParameterBlock(3 + (*ar_state_num)[i][0]);
        }
        // std::cout << "parameter_blocks.size()-> " << parameter_blocks->size() << std::endl;
        // cost_function->AddParameterBlock(1);
        // cost_function->AddParameterBlock(5);
        
        cost_function->SetNumResiduals(3);
        return (cost_function);
    }

    double v_x, v_y, v_z, var;
    Eigen::Vector3d var_vector;
    double delta_t;
    std::string sat_sys; // satellite system
    int keyIndex;
};