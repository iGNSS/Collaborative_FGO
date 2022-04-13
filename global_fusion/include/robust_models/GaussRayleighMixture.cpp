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

#include "error_models/GaussRayleighMixture.h"

namespace libRSF
{
  bool GaussRayleighMixture::estimateWithEM(std::vector<double> const &Data)
  {
    size_t N = Data.size();

    Eigen::VectorXd DataVector = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(Data.data(), N);

    /** convergence criteria*/
    double LikelihoodSumOld = 1e40;
    double LikelihoodSumNew;

    /** N x M matrix */
    Eigen::MatrixXd Likelihood;
    Likelihood.resize(N, 2);

    /** repeat until convergence or 100 iterations*/
    for (int i = 0; i < 100; ++i)
    {
      /** E-step */
      Likelihood.col(0) = _Gaussian.computeProbability(DataVector);
      Likelihood.col(1) = _Rayleigh.computeProbability(DataVector);

      /** remove NaNs */
      Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 0.5);

      /** calculate relative likelihood  */
      LikelihoodSumNew = Likelihood.sum();
      Eigen::VectorXd LikelihoodRowSum = Likelihood.rowwise().sum();

      Likelihood.col(0).array() /= LikelihoodRowSum.array();
      Likelihood.col(1).array() /= LikelihoodRowSum.array();

      /** remove NaNs */
      Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 0.0);

      /** M-Step */
      _Gaussian.estimateParameters(DataVector, Likelihood.col(0), true);
      _Rayleigh.estimateParameters(DataVector, Likelihood.col(1));

      /** check convergence */
      if(std::abs(LikelihoodSumOld - LikelihoodSumNew)/LikelihoodSumNew > 1e-6)
      {
        LikelihoodSumOld = LikelihoodSumNew;
      }
      else
      {
        return true;
      }
    }

    std::cerr << "No Convergence in GaussRayleighMixture::estimateWithEM!" << std::endl;
    return false;
  }
}
