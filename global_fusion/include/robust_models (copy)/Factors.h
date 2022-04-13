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
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "SumMixture.h"

extern libRSF::SumMix1 SMx, SMy;

template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
	q_inverse[0] = q[0];
	q_inverse[1] = -q[1];
	q_inverse[2] = -q[2];
	q_inverse[3] = -q[3];
};


struct TError
{
	TError(double t_x, double t_y, double t_z, double var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), var(var){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var);
		residuals[1] = (tj[1] - T(t_y)) / T(var);
		residuals[2] = (tj[2] - T(t_z)) / T(var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z, const double var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          TError, 3, 3>(
	          	new TError(t_x, t_y, t_z, var)));
	}

	double t_x, t_y, t_z, var;

};


// from Weisong
struct GNSSTError
{
	GNSSTError(double t_x, double t_y, double t_z, double var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), var(var){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var);
		residuals[1] = (tj[1] - T(t_y)) / T(var);
		residuals[2] = (tj[2] - T(t_z)) / T(var);

		return true;
	}

	double t_x, t_y, t_z, var;

};

// from Weisong
struct GNSSTError2D
{
	GNSSTError2D(double t_x, double t_y, double var)
				  :t_x(t_x), t_y(t_y), var(var){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		residuals[0] = (tj[0] - T(t_x)) / T(var);
		residuals[1] = (tj[1] - T(t_y)) / T(var);

		// residuals[0] = (tj[0] - T(t_x));
		// residuals[1] = (tj[1] - T(t_y));

		// SMx.disable();

		// std::cout << "disable works..."<< std::endl;

		// std::cout << "residuals[0]..."<< residuals[0]<< "   residuals[1]..."<< residuals[1]<<std::endl;

		// libRSF::GaussianMixture<1> GMM;
		// GMM.initSpread(3,10);
		// SMx.addMixture(GMM);

		// SMx.Evaluate(residuals);
		//SMy.Evaluate(residuals+1);

		return true;
	}

	double t_x, t_y, var;

};


// from Weisong
struct GNSSTError2DGMM
{
	GNSSTError2DGMM(double t_x, double t_y, double var, libRSF::GaussianMixture<1> GMMX, libRSF::GaussianMixture<1> GMMY)
				  :t_x(t_x), t_y(t_y), var(var), GMMX(GMMX), GMMY(GMMY){}

	template <typename T>
	bool operator()(const T* tj, T* residuals) const
	{
		/*residuals[0] = (tj[0] - T(t_x)) / T(var);
		residuals[1] = (tj[1] - T(t_y)) / T(var);*/

		residuals[0] = (tj[0] - T(t_x));
		residuals[1] = (tj[1] - T(t_y));

		// SMx.disable();

		// std::cout << "disable works..."<< std::endl;

		// std::cout << "residuals[0]..."<< residuals[0]<< "   residuals[1]..."<< residuals[1]<<std::endl;

		libRSF::GaussianMixture<1> GMMXTmp,GMMYTmp;
		GMMXTmp = GMMX;
		GMMYTmp = GMMY;
		SMx.addMixture(GMMXTmp);
		SMy.addMixture(GMMYTmp);

		SMx.Evaluate(residuals);
		SMy.Evaluate(residuals+1);

		return true;
	}

	double t_x, t_y, var;
	libRSF::GaussianMixture<1> GMMX, GMMY;

};


struct RelativeRTError
{
	RelativeRTError(double t_x, double t_y, double t_z, 
					double q_w, double q_x, double q_y, double q_z,
					double t_var, double q_var)
				  :t_x(t_x), t_y(t_y), t_z(t_z), 
				   q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z),
				   t_var(t_var), q_var(q_var){}

	template <typename T>
	bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const // these are from initial guess of states 
	{
		T t_w_ij[3]; // predicted trans residual in the global frame 
		t_w_ij[0] = tj[0] - ti[0];
		t_w_ij[1] = tj[1] - ti[1];
		t_w_ij[2] = tj[2] - ti[2];

		T i_q_w[4]; 
		QuaternionInverse(w_q_i, i_q_w); // 

		T t_i_ij[3];
		// i_q_w (imu to world)
		ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij); // rotate the relative T from global (t_w_ij) to local (t_i_ij)

		residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_var);
		residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_var);
		residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_var);

		T relative_q[4]; // relative q from vio
		relative_q[0] = T(q_w); // 
		relative_q[1] = T(q_x);
		relative_q[2] = T(q_y);
		relative_q[3] = T(q_z);

		T q_i_j[4];
		ceres::QuaternionProduct(i_q_w // quaternion from i to w :state related 
								, w_q_j // quaternion from w to j :state related 
								, q_i_j); // quaternion from i to j :state related 

		T relative_q_inv[4];
		QuaternionInverse(relative_q, relative_q_inv);

		T error_q[4];
		ceres::QuaternionProduct(relative_q_inv, // relative q from vio
								 q_i_j // relative q from states
								 , error_q); // q_i_j “-” relative_q_inv

		residuals[3] = T(2) * error_q[1] / T(q_var);
		residuals[4] = T(2) * error_q[2] / T(q_var);
		residuals[5] = T(2) * error_q[3] / T(q_var);

		return true;
	}

	static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
									   const double q_w, const double q_x, const double q_y, const double q_z,
									   const double t_var, const double q_var) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          RelativeRTError, 6, 4, 3, 4, 3>(
	          	new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z, t_var, q_var)));
	}

	double t_x, t_y, t_z, t_norm;
	double q_w, q_x, q_y, q_z;
	double t_var, q_var; // information matrix

};