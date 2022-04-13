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


#ifndef NUMERICALRANGE_H
#define NUMERICALRANGE_H

#include <ceres/ceres.h>

namespace libRSF
{
  template <typename T>
  T ScaledLogSumExp(const T* Exponents, const T* Scaling, const double NumberOfGaussians)
  {
    T Sum, MaxExp;

    Sum = T(0.0);
    MaxExp = Exponents[0];

    for (size_t nGauss = 1; nGauss < NumberOfGaussians; ++nGauss)
    {
      if (MaxExp < Exponents[nGauss])
        MaxExp = Exponents[nGauss];
    }

    for (size_t nGauss = 0; nGauss < NumberOfGaussians; ++nGauss)
    {
      Sum += ceres::exp(Exponents[nGauss] - MaxExp) * Scaling[nGauss];
    }
    return ceres::log(Sum) + MaxExp;
  }
}

#endif // NUMERICALRANGE_H

