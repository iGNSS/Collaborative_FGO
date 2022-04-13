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



#ifndef SUMMIXTURE_H
#define SUMMIXTURE_H

#include <ceres/ceres.h>
#include "ErrorModel.h"
#include "GaussianMixture.h"
#include "GaussRayleighMixture.h"
#include "NumericalRobust.h"



namespace libRSF
{
  /** \brief The robust Sum-Mixture error model
   * Based on:
   * D. M. Rosen, M. Kaess, and J. J. Leonard
   * “Robust incremental online inference over sparse factor graphs: Beyond the Gaussian case”
   * Proc. of Intl. Conf. on Robotics and Automation (ICRA), Karlsruhe, 2013.
   * DOI: 10.1109/ICRA.2013.6630699
   *
   * \param Mixture Underlying mixture distribution
   *
   */
  template <int Dimension, typename MixtureType>
  class SumMixture : public ErrorModel <Dimension, 1>
  {
  public:

      SumMixture()
      {
        _Normalization = 0;
      };

     explicit SumMixture(MixtureType &Mixture)
     {
       addMixture(Mixture);
     };

      void addMixture (MixtureType &Mixture)
      {
        _Mixture = Mixture;

        _Normalization = 0;

        size_t NumberOfComponents = _Mixture.getNumberOfComponents();
        for (int nComponent = 1; nComponent <= NumberOfComponents; ++nComponent)
        {
          _Normalization += _Mixture.getMaximumOfComponent(nComponent);
        }
      }

      void clear()
      {
        _Normalization = 0;
        _Mixture.clear();
      }

      template <typename T>
      bool Evaluate(T* Error) const
      {
        if (this->_Enable)
        {
          Eigen::Matrix<T, Eigen::Dynamic, 1> Scalings;
          Eigen::Matrix<T, Eigen::Dynamic, 1> Exponents;
          T SquaredError;

          size_t NumberOfComponents = _Mixture.getNumberOfComponents();

          Scalings.resize(NumberOfComponents,1);
          Exponents.resize(NumberOfComponents,1);

          /** calculate all exponents and scalings */
          for(int nComponent = 1; nComponent <= NumberOfComponents; ++nComponent)
          {
            Exponents(nComponent-1,0) = - 0.5 * _Mixture.getExponentialPartOfComponent(nComponent, Error).squaredNorm();
            Scalings(nComponent-1,0) = T(_Mixture.getLinearPartOfComponent(nComponent, Error)/_Normalization);
            
          }

          /** combine them numerically robust */
          SquaredError = - ScaledLogSumExp(Exponents.data(), Scalings.data(), NumberOfComponents);
          *Error = ceres::sqrt(SquaredError * T(2.0));
        }
        return true;
      };

  private:
    MixtureType _Mixture;
    double _Normalization;
  };

  typedef SumMixture<1, GaussianMixture<1>> SumMix1;
  typedef SumMixture<1, GaussRayleighMixture> SumMixRay1;
}





#endif // SUMMIXTURE_H
