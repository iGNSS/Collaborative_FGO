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

#include "../robust_models/SumMixture.h"
#include "../robust_models/GaussianMixture.h"
#include "../robust_models/GaussianComponent.h"
#include "../robust_models/MaxMixture.h"

/* pseudorange factor modeled with GMM */
struct pseudorangeFactor_GMM
{
    pseudorangeFactor_GMM(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double pseudorange, double var, libRSF::GaussianMixture<1> GMM_p)
                :sat_sys(sat_sys),s_g_x(s_g_x), s_g_y(s_g_y), s_g_z(s_g_z), pseudorange(pseudorange),var(var),GMM_p(GMM_p){}

    template <typename T>
    bool operator()(const T* state, T* residuals) const
    {
        T est_pseudorange; 
        T delta_x = pow((state[0] - s_g_x),2);
        T delta_y = pow((state[1] - s_g_y),2);
        T delta_z = pow((state[2] - s_g_z),2);

        est_pseudorange = sqrt(delta_x+ delta_y + delta_z);

        double OMGE_ = 7.2921151467E-5;
        double CLIGHT_ = 299792458.0;
        est_pseudorange = est_pseudorange + OMGE_ * (s_g_x*state[1]-s_g_y*state[0])/CLIGHT_;
        
        if(sat_sys == "GPS") 
        {
            est_pseudorange = est_pseudorange - state[3];
        }
        
        else if(sat_sys == "BeiDou") 
        {
            est_pseudorange = est_pseudorange - state[4];
        }

        // residuals[0] = (est_pseudorange - T(pseudorange)) / T(1);
        residuals[0] = (est_pseudorange - T(pseudorange)) / T(var);

        if(residuals[0]< T(100) && residuals[0]> T(-100))
        // if(0)
        {
            libRSF::GaussianMixture<1> GMM_tmp;
            libRSF::SumMix1 SM_p;
            // libRSF::SumMixRay1 SM_p;
            
            // libRSF::MaxMix1 SM_p;
            
            
            GMM_tmp = GMM_p;
            SM_p.addMixture(GMM_tmp);
            SM_p.Evaluate(residuals);
        }
        

        return true;
    }

    double s_g_x, s_g_y, s_g_z, pseudorange, var;
    std::string sat_sys; // satellite system
    libRSF::GaussianMixture<1> GMM_p;

};