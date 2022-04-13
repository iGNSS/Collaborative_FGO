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

/* pseudorange factor*/
struct pseudorangeFactor
{
    pseudorangeFactor(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double pseudorange, double var)
                :sat_sys(sat_sys),s_g_x(s_g_x), s_g_y(s_g_y), s_g_z(s_g_z), pseudorange(pseudorange),var(var){}

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

        /* test the erfc functions */
        // double pc =  std::erfc(-2/std::sqrt(2))/2;
        // est_pseudorange = est_pseudorange + OMGE_ * (s_g_x*state[1]-s_g_y*state[0])/CLIGHT_  + ceres::erfc(-state[1]);

        
        
        est_pseudorange = est_pseudorange + OMGE_ * (s_g_x*state[1]-s_g_y*state[0])/CLIGHT_ ;

        
        
        
        if(sat_sys == "GPS") 
        {
            est_pseudorange = est_pseudorange - state[3];
        }
        
        else if(sat_sys == "BeiDou") 
        {
            est_pseudorange = est_pseudorange - state[4];
        }

        residuals[0] = (est_pseudorange - T(pseudorange)) / T(var);
        // std::cout << "residuals[0]-> "<< residuals[0]<<std::endl;

        return true;
    }

    double s_g_x, s_g_y, s_g_z, pseudorange, var;
    std::string sat_sys; // satellite system

};

/* tightly coupled doppler factor*/
struct TDCPFactor
{
    TDCPFactor(std::string sat_sys, double s_g_x, double s_g_y, double s_g_z, double pre_s_g_x, double pre_s_g_y, double pre_s_g_z, double car_measure, double pre_car_measure, double lambda, double var)
                :sat_sys(sat_sys),s_g_x(s_g_x), s_g_y(s_g_y), s_g_z(s_g_z), pre_s_g_x(pre_s_g_x), pre_s_g_y(pre_s_g_y), pre_s_g_z(pre_s_g_z), car_measure(car_measure),pre_car_measure(pre_car_measure), lambda(lambda), var(var){}

    template <typename T>
    bool operator()(const T* statePj,const T* statePi, T* residuals) const
    {
        T est_range; 
        T delta_x = pow((statePj[0] - s_g_x),2);
        T delta_y = pow((statePj[1] - s_g_y),2);
        T delta_z = pow((statePj[2] - s_g_z),2);
        est_range = sqrt(delta_x+ delta_y + delta_z);

        T pre_est_range; 
        T pre_delta_x = pow((statePi[0] - pre_s_g_x),2);
        T pre_delta_y = pow((statePi[1] - pre_s_g_y),2);
        T pre_delta_z = pow((statePi[2] - pre_s_g_z),2);
        pre_est_range = sqrt(pre_delta_x+ pre_delta_y + pre_delta_z);
        
        /* using the clock bias estimated by the pseudorange: suggest not to use this, as the pseudorange is much noisy */
        // if(sat_sys == "GPS") 
        // {
        //     est_range = est_range - statePj[3];
        //     pre_est_range = pre_est_range - statePi[3];
        // }
        
        // else if(sat_sys == "BeiDou") 
        // {
        //     est_range = est_range - statePj[4];
        //     pre_est_range = pre_est_range - statePi[4];
        // }

        residuals[0] = (est_range - pre_est_range + statePi[5]) - (car_measure - pre_car_measure);
        
        /* using the clock drift estimated by the Doppler */
        // residuals[0] = (est_range - pre_est_range + statePj[5]) - (car_measure - pre_car_measure);

        residuals[0] = residuals[0] /T(var);
        /* in RTKLIB, the covariance of the Doppler measurements is set as fixed covariance */

        return true;
    }

    double s_g_x, s_g_y, s_g_z, svddt, var;
    double pre_s_g_x, pre_s_g_y, pre_s_g_z; // satellite position
    std::string sat_sys; // satellite system

    double car_measure;
    double pre_car_measure;

    double lambda;

};

/* constant clock drift factor*/
struct constantClockDriftFactor
{
    constantClockDriftFactor(double var)
                :var(var){}

    template <typename T>
    bool operator()(const T* rcvDDti,const T* rcvDDtj, T* residuals) const
    {

        residuals[0] = (rcvDDti[5] - rcvDDtj[5])/ T(var);

        return true;
    }

    double var;
};

/* time-correlated receievr clock drift FACTORS */
struct timeCorrelatedClockDriftFactor
{
    timeCorrelatedClockDriftFactor(double dt, double var)
                :dt(dt), var(var){}

    template <typename T>
    bool operator()(const T* rcvi,const T* rcvj, T* residuals) const
    {
        T rcvDti_GPS = rcvi[3];
        T rcvDti_BDS = rcvi[4];
        T rcvDDti = rcvi[5];

        T rcvDtj_GPS = rcvj[3];
        T rcvDtj_BDS = rcvj[4];
        T rcvDDtj = rcvj[5];

        T aveDDt = T(0.5) * (rcvDDti - rcvDDtj);

        residuals[0] = (rcvDtj_GPS - rcvDti_GPS - aveDDt * dt)/ T(var);

        residuals[1] = (rcvDtj_BDS - rcvDti_BDS - aveDDt * dt)/ T(var);


        // residuals[0] = (rcvDDti[5] - rcvDDtj[5])/ T(var);

        return true;
    }

    double var;
    double dt;
};

/* Motion model FACTORS */
struct motionModelFactor
{
    motionModelFactor(double dt, double var)
                :dt(dt), var(var){}

    template <typename T>
    bool operator()(const T* statePi,const T* statePj,const T* stateVi, T* residuals) const
    {
        T est_v_x = (statePj[0] - statePi[0])/ dt;
        T est_v_y = (statePj[1] - statePi[1])/ dt;
        T est_v_z = (statePj[2] - statePi[2])/ dt;

        // est_pseudorange = sqrt(delta_x+ delta_y + delta_z);

        residuals[0] = (est_v_x - stateVi[0]) / T(var);
        residuals[1] = (est_v_y - stateVi[1]) / T(var);
        residuals[2] = (est_v_z - stateVi[2]) / T(var);

        return true;
    }

    double var;
    double dt;
};


/* Motion model FACTORS */
struct motionModelFactorSmooth
{
    motionModelFactorSmooth(double dt, double var)
                :dt(dt), var(var){}

    template <typename T>
    bool operator()(const T* statePi,const T* statePj,const T* stateVi, const T* stateVj, T* residuals) const
    {
        T est_v_x = (statePj[0] - statePi[0])/ dt;
        T est_v_y = (statePj[1] - statePi[1])/ dt;
        T est_v_z = (statePj[2] - statePi[2])/ dt;

        // est_pseudorange = sqrt(delta_x+ delta_y + delta_z);

        residuals[0] = (est_v_x - ((stateVi[0]+stateVi[0])/T(2))) / T(var);
        residuals[1] = (est_v_y - ((stateVi[1]+stateVi[1])/T(2))) / T(var);
        residuals[2] = (est_v_z - ((stateVi[2]+stateVi[2])/T(2))) / T(var);

        return true;
    }

    double var;
    double dt;
};


/* 
**  Function: motion constraint with different uncertainty on xyz.
**  parameters[0]: position at time k
**  parameters[1]: position at time k+1
**  parameters[1]: velocity at time k
**  parameters[1]: velocity at time k+1
**  residual[0]: the integer ambiguity value VS P_k
*/
class AnalyticalMotionModelFactor : public ceres::SizedCostFunction<3, 6, 6, 3, 3>
{
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        AnalyticalMotionModelFactor() = delete;
        AnalyticalMotionModelFactor( const double _dt, const Eigen::MatrixXd &_W_matrix):  dt(_dt), W_matrix(_W_matrix)
        {
            
        }
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
            Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);

            Eigen::Vector3d Vi(parameters[2][0], parameters[2][1], parameters[2][2]);
            Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
            
            // est_pseudorange = sqrt(delta_x+ delta_y + delta_z);

            residuals[0] = (Pj[0] - Pi[0])/ dt - ((Vi[0]+Vj[0])/(2));
            residuals[1] = (Pj[1] - Pi[1])/ dt - ((Vi[1]+Vj[1])/(2));
            residuals[2] = (Pj[2] - Pi[2])/ dt - ((Vi[2]+Vj[2])/(2));

            if (jacobians)
            {
                // J_Pi
                if (jacobians[0])
                {
                    Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J_Pi(jacobians[0]);
                    J_Pi.setZero();

                    J_Pi(0,0) = -1.0 / dt;
                    J_Pi(1,1) = -1.0 / dt;
                    J_Pi(2,2) = -1.0 / dt;

                    J_Pi = W_matrix * J_Pi;
                }

                // J_Pj
                if (jacobians[1])
                {
                    Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> J_Pj(jacobians[1]);
                    J_Pj.setZero();

                    J_Pj(0,0) = 1.0 / dt;
                    J_Pj(1,1) = 1.0 / dt;
                    J_Pj(2,2) = 1.0 / dt;

                    J_Pj = W_matrix * J_Pj;
                }

                // J_Vi
                if (jacobians[2])
                {
                    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_Vi(jacobians[2]);
                    J_Vi.setZero();

                    J_Vi(0,0) = -1.0/2.0;
                    J_Vi(1,1) = -1.0/2.0;
                    J_Vi(2,2) = -1.0/2.0;

                    J_Vi = W_matrix * J_Vi;
                }

                // J_Vj
                if (jacobians[3])
                {
                    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> J_Vj(jacobians[3]);
                    J_Vj.setZero();

                    J_Vj(0,0) = -1.0/2.0;
                    J_Vj(1,1) = -1.0/2.0;
                    J_Vj(2,2) = -1.0/2.0;

                    J_Vj = W_matrix * J_Vj;
                }

                Eigen::Map<Eigen::Matrix<double, 3, 1>> residual(residuals);
                residual = W_matrix * residual;

                
            }
            return true;
        }

        bool check_gradients(const std::vector<const double*> &parameters) const;
    private:

        double dt;
        Eigen::MatrixXd W_matrix;
};


/*add 3DMA as a prior factor to each frame*/
struct Factor3dma
{
    Factor3dma(double ox, double oy,double oz,double var)
    :ox(ox),oy(oy),oz(oz),var(var){}

    template <typename T>
    bool operator()(const T* state, T* residuals) const //const T* stateV
    {
        residuals[0] = (ox - state[0]) / T(var);
        residuals[1] = (oy- state[1])  / T(var);
        residuals[2] = (oz - state[2]) / T(var);
        return true;

    }

  double ox; 
  double oy;
  double oz;
  double var;

};


/* constant vel FACTORS */
struct constantVelFactor
{
    constantVelFactor(double dt, double var)
                :dt(dt), var(var){}

    template <typename T>
    bool operator()(const T* stateVi, T* residuals) const
    {
        residuals[0] = (T(1) - stateVi[0]) / T(var);
        residuals[1] = (T(1) - stateVi[1]) / T(var);
        residuals[2] = (T(1) - stateVi[2]) / T(var);

        return true;
    }

    double var;
    double dt;
};

/* vel limit FACTORS */
struct velLimitFactor
{
    velLimitFactor(double dt, double var, double alpha, double sigma, double velLimit)
                :dt(dt), var(var), alpha(alpha), sigma(sigma), velLimit(velLimit) {}

    template <typename T>
    bool operator()(const T* stateVi, T* residuals) const
    {
        // residuals[0] = (T(1) - stateVi[0]) / T(var);
        // residuals[1] = (T(1) - stateVi[1]) / T(var);
        // residuals[2] = (T(1) - stateVi[2]) / T(var);

        T upplerLimit = T(velLimit);
        T lowLimit    = T(-1 * velLimit);

        /* residual for x direction */
        if((stateVi[0])<=(lowLimit + T(sigma)))
        {
            residuals[0] = T(alpha) * (lowLimit-stateVi[0]+sigma);
            residuals[0] = residuals[0] / T(var);
        }
        else if((stateVi[0])>=(upplerLimit - T(sigma)))
        {
            residuals[0] = T(alpha) * (stateVi[0] - upplerLimit +sigma);
            residuals[0] = residuals[0] / T(var);
        }
        else
        {
            residuals[0] = T(0);
            residuals[0] = residuals[0] / T(var);
        }
        
        /* residual for y direction */
        if((stateVi[1])<=(lowLimit + T(sigma)))
        {
            residuals[1] = T(alpha) * (lowLimit-stateVi[1]+sigma);
            residuals[1] = residuals[1] / T(var);
        }
        else if((stateVi[1])>=(upplerLimit - T(sigma)))
        {
            residuals[1] = T(alpha) * (stateVi[1] - upplerLimit +sigma);
            residuals[1] = residuals[1] / T(var);
        }
        else
        {
            residuals[1] = T(0);
            residuals[1] = residuals[1] / T(var);
        }

        /* residual for z direction */
        if((stateVi[2])<=(lowLimit + T(sigma)))
        {
            residuals[2] = T(alpha) * (lowLimit-stateVi[1]+sigma);
            residuals[2] = residuals[2] / T(var);
        }
        else if((stateVi[2])>=(upplerLimit - T(sigma)))
        {
            residuals[2] = T(alpha) * (stateVi[2] - upplerLimit +sigma);
            residuals[2] = residuals[2] / T(var);
        }
        else
        {
            residuals[2] = T(0);
            residuals[2] = residuals[2] / T(var);
        }
        

        return true;
    }

    double var;
    double dt;
    double alpha;
    double sigma;
    double velLimit;
};

/* Acc model FACTORS */
struct accModelFactor
{
    accModelFactor(double dt, double var)
                :dt(dt), var(var){}

    template <typename T>
    bool operator()(const T* stateVi,const T* stateVj,const T* stateAi, T* residuals) const
    {
        T est_acc_x = (stateVj[0] - stateVi[0])/ dt;
        T est_acc_y = (stateVj[1] - stateVi[1])/ dt;
        T est_acc_z = (stateVj[2] - stateVi[2])/ dt;

        // est_pseudorange = sqrt(delta_x+ delta_y + delta_z);

        residuals[0] = (est_acc_x - stateAi[0]) / T(var);
        residuals[1] = (est_acc_y - stateAi[1]) / T(var);
        residuals[2] = (est_acc_z - stateAi[2]) / T(var);

        return true;
    }

    double var;
    double dt;
};

/* zero Acc FACTORS */
struct zeroAccFactor
{
    zeroAccFactor(double dt, double var)
                :dt(dt), var(var){}

    template <typename T>
    bool operator()(const T* stateAi, T* residuals) const
    {
        residuals[0] = (T(0) - stateAi[0]) / T(var);
        residuals[1] = (T(0) - stateAi[1]) / T(var);
        residuals[2] = (T(0) - stateAi[2]) / T(var);

        return true;
    }

    double var;
    double dt;
};

/* sliding window prior FACTORS */
struct priorFactor
{
    priorFactor(double position_x, double position_y,double position_z, double gps_bias,double beidou_bias, double bias_drift, double  velocity_x, double velocity_y, double velocity_z,double var)
                :position_x(position_x), position_y(position_y),position_z(position_z),gps_bias(gps_bias),beidou_bias(beidou_bias),bias_drift(bias_drift),velocity_x(velocity_x),velocity_y(velocity_y),velocity_z(velocity_z),var(var){}

    template <typename T>
    bool operator()(const T* state, T* residuals) const //const T* stateV
    {
        residuals[0] = (position_x - state[0]) / T(var);
        residuals[1] = (position_y- state[1]) / T(var);
        residuals[2] = (position_z - state[2]) / T(var);

        // residuals[3] = (gps_bias-state[3])/T(var);
        // residuals[4] = (beidou_bias-state[4])/T(var);
        //  residuals[5] = (bias_drift-state[5])/T(var);

        // residuals[6] = (velocity_x-stateV[0])/T(var);
        // residuals[7] =(velocity_y-stateV[1])/T(var);
        // residuals[8] = (velocity_z-stateV[2])/T(var);

        // std::cout<<"the residual x is----->"<<residuals[0]<<"the residual y is----->"<<residuals[1]<<"the residual z is----->"<<residuals[2]<<std::endl;

        return true;
    }

    double var;
    double position_x, position_y,position_z,gps_bias,beidou_bias,bias_drift;
    double velocity_x, velocity_y, velocity_z;
};

/* sliding window prior FACTORS */
struct polyposePriorFactor
{
    polyposePriorFactor(double position_x, double position_y,double var)
                :position_x(position_x), position_y(position_y),var(var){}

    template <typename T>
    bool operator()(const T* state, T* residuals) const //const T* stateV
    {
        residuals[0] = (position_x - state[0]) / T(var);
        residuals[1] = (position_y- state[1]) / T(var);

        return true;
    }

    double var;
    double position_x, position_y;
};


/* jer model FACTORS */
struct jerModelFactor
{
    jerModelFactor(double dt, double var)
                :dt(dt), var(var){}

    template <typename T>
    bool operator()(const T* stateAi,const T* stateAj,const T* stateJi, T* residuals) const
    {
        T est_jer_x = (stateAj[0] - stateAi[0])/ dt;
        T est_jer_y = (stateAj[1] - stateAi[1])/ dt;
        T est_jer_z = (stateAj[2] - stateAi[2])/ dt;

        // est_pseudorange = sqrt(delta_x+ delta_y + delta_z);

        residuals[0] = (est_jer_x - stateJi[0]) / T(var);
        residuals[1] = (est_jer_y - stateJi[1]) / T(var);
        residuals[2] = (est_jer_z - stateJi[2]) / T(var);

        return true;
    }

    double var;
    double dt;
};



/* DDpseudorangeFactor factor :: different satellite system different sat pose
    * M: master satellite
    * i: satellite to be double-difference based on master satellite
    * r: reference satellite
    * u: use end satellite (GNSS receiver)
*/
struct DDpseudorangeVSVFactor
{
    DDpseudorangeVSVFactor(DDMeasurement dd_measurement, Eigen::Vector3d base_pos)
                :dd_measurement(dd_measurement),base_pos(base_pos){}

    template <typename T>
    bool operator()(const T* state, T* residuals) const
    {

        /**/
        Eigen::Vector3d pose_r = base_pos;
        GNSS_Tools m_GNSS_Tools1; // utilities
        double OMGE_ = 7.2921151467E-5;
        double CLIGHT_ = 299792458.0;


        /* satellite position*/
        Eigen::Vector3d u_pose_m(dd_measurement.u_master_SV.sat_pos_x, dd_measurement.u_master_SV.sat_pos_y,dd_measurement.u_master_SV.sat_pos_z);
        double var_u2m = m_GNSS_Tools1.getVarofpr_ele_SNR(dd_measurement.u_master_SV);

        Eigen::Vector3d u_pose_i(dd_measurement.u_iSV.sat_pos_x, dd_measurement.u_iSV.sat_pos_y,dd_measurement.u_iSV.sat_pos_z); 
        double var_u2i = m_GNSS_Tools1.getVarofpr_ele_SNR(dd_measurement.u_iSV);
        // LOG(INFO)<<"var_u2i-> " <<var_u2i;

        Eigen::Vector3d r_pose_m(dd_measurement.r_master_SV.sat_pos_x, dd_measurement.r_master_SV.sat_pos_y,dd_measurement.r_master_SV.sat_pos_z);
        double var_r2m = m_GNSS_Tools1.getVarofpr_ele_SNR(dd_measurement.r_master_SV);
        // LOG(INFO)<<"var_r2m-> " <<var_r2m;

        Eigen::Vector3d r_pose_i(dd_measurement.r_iSV.sat_pos_x, dd_measurement.r_iSV.sat_pos_y,dd_measurement.r_iSV.sat_pos_z); 
        double var_r2i = m_GNSS_Tools1.getVarofpr_ele_SNR(dd_measurement.r_iSV);
        // LOG(INFO)<<"var_r2i-> " <<var_r2i;
        
        T est_p_r2m = T(m_GNSS_Tools1.getDistanceFrom2Points(pose_r, r_pose_m));
        // est_p_r2m = est_p_r2m + OMGE_ * (rs[0]*rr[1]-rs[1]*rr[0])/CLIGHT_;
        est_p_r2m = est_p_r2m + OMGE_ * (r_pose_m(0)*pose_r(1)-r_pose_m(1)*pose_r(0))/CLIGHT_;
        
        T est_p_r2i = T(m_GNSS_Tools1.getDistanceFrom2Points(pose_r, r_pose_i)); 
        est_p_r2i = est_p_r2i + OMGE_ * (r_pose_i(0)*pose_r(1)-r_pose_i(1)*pose_r(0))/CLIGHT_;

        T est_p_u2m;
        T delta_x1 = pow((state[0] - u_pose_m(0)),2);
        T delta_y1 = pow((state[1] - u_pose_m(1)),2);
        T delta_z1 = pow((state[2] - u_pose_m(2)),2);
        est_p_u2m = sqrt(delta_x1+ delta_y1 + delta_z1);
        est_p_u2m = est_p_u2m + OMGE_ * (u_pose_m(0)*state[1]-u_pose_m(1)*state[0])/CLIGHT_;

        T est_p_u2i;
        T delta_x2 = pow((state[0] - u_pose_i(0)),2);
        T delta_y2 = pow((state[1] - u_pose_i(1)),2);
        T delta_z2 = pow((state[2] - u_pose_i(2)),2);
        est_p_u2i = sqrt(delta_x2+ delta_y2 + delta_z2);

        /* relatistic effects*/
        est_p_u2i = est_p_u2i + OMGE_ * (u_pose_i(0)*state[1]-u_pose_i(1)*state[0])/CLIGHT_;
        
        /* expected DD measurments */
        T est_DD_pr = (est_p_u2i - est_p_r2i) - (est_p_u2m - est_p_r2m);

        T p_r2m = T(dd_measurement.r_master_SV.raw_pseudorange);
        T p_r2i = T(dd_measurement.r_iSV.raw_pseudorange);
        T p_u2m = T(dd_measurement.u_master_SV.raw_pseudorange);
        T p_u2i = T(dd_measurement.u_iSV.raw_pseudorange);
        
        /*observation of DD measurement*/
        T DD_pr = (p_u2i - p_r2i) - (p_u2m - p_r2m);

        // double var = 1000;
        double var = var_u2m + var_u2i + var_r2m + var_r2i;
        var = var / 4.0;
        // residuals[0] = (est_DD_pr - DD_pr) /T(var);
        #if use_fixed_cov
        residuals[0] = (est_DD_pr - DD_pr) /T(pow(0.4, 2)); 
        #else
        // residuals[0] = (est_DD_pr - DD_pr) /T(var_u2i);
        residuals[0] = (est_DD_pr - DD_pr) /T(var);
        // std::cout<<"residuals[0]-> " << (est_DD_pr - DD_pr) << std::endl;
        #endif

        return true;
    }

    DDMeasurement dd_measurement;
    Eigen::Vector3d base_pos;

};



/* DDpseudorangeVSVConstraint  :: different satellite system different sat pose
    * M: master satellite
    * i: satellite to be double-difference based on master satellite
    * r: reference satellite
    * u: use end satellite (GNSS receiver)
*/
struct DDpseudorangeVSVConstraint
{
    typedef ceres::DynamicAutoDiffCostFunction<DDpseudorangeVSVConstraint, 10>
      DDpseudorangeVSVDynaCostFunction;
    DDpseudorangeVSVConstraint(DDMeasurement dd_measurement, Eigen::Vector3d base_pos, int keyIndex)
                :dd_measurement(dd_measurement),base_pos(base_pos),keyIndex(keyIndex){}

    template <typename T>
    bool operator()(T const* const* state, T* residuals) const
    {
        /**/
        Eigen::Vector3d pose_r = base_pos;
        GNSS_Tools m_GNSS_Tools1; // utilities
        double OMGE_ = 7.2921151467E-5;
        double CLIGHT_ = 299792458.0;


        /* satellite position*/
        Eigen::Vector3d u_pose_m(dd_measurement.u_master_SV.sat_pos_x, dd_measurement.u_master_SV.sat_pos_y,dd_measurement.u_master_SV.sat_pos_z);
        double var_u2m = m_GNSS_Tools1.getVarofpr_ele_SNR(dd_measurement.u_master_SV);

        Eigen::Vector3d u_pose_i(dd_measurement.u_iSV.sat_pos_x, dd_measurement.u_iSV.sat_pos_y,dd_measurement.u_iSV.sat_pos_z); 
        double var_u2i = m_GNSS_Tools1.getVarofpr_ele_SNR(dd_measurement.u_iSV);
        // LOG(INFO)<<"var_u2i-> " <<var_u2i;

        Eigen::Vector3d r_pose_m(dd_measurement.r_master_SV.sat_pos_x, dd_measurement.r_master_SV.sat_pos_y,dd_measurement.r_master_SV.sat_pos_z);
        double var_r2m = m_GNSS_Tools1.getVarofpr_ele_SNR(dd_measurement.r_master_SV);
        // LOG(INFO)<<"var_r2m-> " <<var_r2m;

        Eigen::Vector3d r_pose_i(dd_measurement.r_iSV.sat_pos_x, dd_measurement.r_iSV.sat_pos_y,dd_measurement.r_iSV.sat_pos_z); 
        double var_r2i = m_GNSS_Tools1.getVarofpr_ele_SNR(dd_measurement.r_iSV);
        // LOG(INFO)<<"var_r2i-> " <<var_r2i;
        
        T est_p_r2m = T(m_GNSS_Tools1.getDistanceFrom2Points(pose_r, r_pose_m));
        // est_p_r2m = est_p_r2m + OMGE_ * (rs[0]*rr[1]-rs[1]*rr[0])/CLIGHT_;
        est_p_r2m = est_p_r2m + OMGE_ * (r_pose_m(0)*pose_r(1)-r_pose_m(1)*pose_r(0))/CLIGHT_;
        
        T est_p_r2i = T(m_GNSS_Tools1.getDistanceFrom2Points(pose_r, r_pose_i)); 
        est_p_r2i = est_p_r2i + OMGE_ * (r_pose_i(0)*pose_r(1)-r_pose_i(1)*pose_r(0))/CLIGHT_;

        T est_p_u2m;
        T delta_x1 = pow((state[keyIndex][0] - u_pose_m(0)),2);
        T delta_y1 = pow((state[keyIndex][1] - u_pose_m(1)),2);
        T delta_z1 = pow((state[keyIndex][2] - u_pose_m(2)),2);
        est_p_u2m = sqrt(delta_x1+ delta_y1 + delta_z1);
        est_p_u2m = est_p_u2m + OMGE_ * (u_pose_m(0)*state[keyIndex][1]-u_pose_m(1)*state[keyIndex][0])/CLIGHT_;

        T est_p_u2i;
        T delta_x2 = pow((state[keyIndex][0] - u_pose_i(0)),2);
        T delta_y2 = pow((state[keyIndex][1] - u_pose_i(1)),2);
        T delta_z2 = pow((state[keyIndex][2] - u_pose_i(2)),2);
        est_p_u2i = sqrt(delta_x2+ delta_y2 + delta_z2);

        /* relatistic effects*/
        est_p_u2i = est_p_u2i + OMGE_ * (u_pose_i(0)*state[keyIndex][1]-u_pose_i(1)*state[keyIndex][0])/CLIGHT_;
        
        /* expected DD measurments */
        T est_DD_pr = (est_p_u2i - est_p_r2i) - (est_p_u2m - est_p_r2m);

        T p_r2m = T(dd_measurement.r_master_SV.raw_pseudorange);
        T p_r2i = T(dd_measurement.r_iSV.raw_pseudorange);
        T p_u2m = T(dd_measurement.u_master_SV.raw_pseudorange);
        T p_u2i = T(dd_measurement.u_iSV.raw_pseudorange);
        
        /*observation of DD measurement*/
        T DD_pr = (p_u2i - p_r2i) - (p_u2m - p_r2m);

        // double var = 1000;
        double var = var_u2m + var_u2i + var_r2m + var_r2i;
        var = var / 4.0;
        // residuals[0] = (est_DD_pr - DD_pr) /T(var);
        #if use_fixed_cov
        residuals[0] = (est_DD_pr - DD_pr) /T(pow(0.4, 2)); 
        #else
        // residuals[0] = (est_DD_pr - DD_pr) /T(var_u2i);
        residuals[0] = (est_DD_pr - DD_pr) /T(var);
        #endif

        return true;
    }

    // Factory method to create a CostFunction from a DDpseudorangeVSVConstraint to
    // conveniently add to a ceres problem.
    static DDpseudorangeVSVDynaCostFunction* Create(DDMeasurement dd_measurement, Eigen::Vector3d base_pos, int keyIndex, std::vector<double*>* state_array, std::vector<double*>* pose_parameter_blocks, std::vector<int*>* ar_state_num) {
        
        DDpseudorangeVSVConstraint* constraint = new DDpseudorangeVSVConstraint(
            dd_measurement, base_pos, keyIndex);
        
        DDpseudorangeVSVDynaCostFunction* cost_function = new DDpseudorangeVSVDynaCostFunction(constraint);
        
        pose_parameter_blocks->clear();
        // double a[5] = {1,2,3,4,5};
        // parameter_blocks->push_back(a);
        // parameter_blocks->push_back(&((*state_array)[keyIndex]));
        // parameter_blocks->push_back(state_array[keyIndex]);
        
        for(int i = 0; i <(keyIndex+1); i++)
        {
            pose_parameter_blocks->push_back((*state_array)[i]);
            cost_function->AddParameterBlock(3 + (*ar_state_num)[i][0]);
        }
        // std::cout << "parameter_blocks.size()-> " << parameter_blocks->size() << std::endl;
        // cost_function->AddParameterBlock(1);
        // cost_function->AddParameterBlock(5);
        
        cost_function->SetNumResiduals(1);
        return (cost_function);
    }

    DDMeasurement dd_measurement;
    Eigen::Vector3d base_pos;
    int keyIndex;

};

