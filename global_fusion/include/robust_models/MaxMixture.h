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



#ifndef MAXMIXTURE_H
#define MAXMIXTURE_H

#include <ceres/ceres.h>
#include "ErrorModel.h"
#include "GaussianMixture.h"

namespace libRSF
{
  /** \brief The robust Max-Mixture error model
   * Based on:
   * E. Olson and P. Agarwal
   * “Inference on networks of mixtures for robust robot mapping”
   * Proc. of Robotics: Science and Systems (RSS), Sydney, 2012.
   * DOI: 10.15607/RSS.2012.VIII.040
   *
   * \param Mixture Underlying mixture distribution
   *
   */
  template <int Dimension, typename MixtureType>
  class MaxMixture : public ErrorModel <Dimension, Dimension+1>
  {
    public:

      MaxMixture()
      {
        _Normalization = -std::numeric_limits<double>::infinity();
      }

      explicit MaxMixture(MixtureType &Mixture)
      {
        addMixture(Mixture);
      }

      void clear()
      {
        _Normalization = -std::numeric_limits<double>::infinity();
        _Mixture.clear();
      }

      void addMixture(MixtureType &Mixture)
      {
        _Mixture = Mixture;

        _Normalization = _Mixture.getMaximumOfComponent(1);

        size_t NumberOfComponents = _Mixture.getNumberOfComponents();
        for(int nComponent = 2; nComponent <= NumberOfComponents; ++nComponent)
        {
          _Normalization = std::max(_Normalization, _Mixture.getMaximumOfComponent(nComponent));
        }
      }

      template <typename T>
      bool Evaluate(T* Error) const
      {
        if(this->_Enable)
        {
          T Loglike = T(NAN);

          /** map the error pointer to a matrix */
          Eigen::Map<Eigen::Matrix<T, Dimension + 1, 1> > ErrorMap(Error);
          Eigen::Matrix<T, Dimension + 1, 1> ErrorShadow, ErrorShadowBest;

          size_t NumberOfComponents = _Mixture.getNumberOfComponents();

          /** calculate Log-Likelihood for each Gaussian component */
          for(int nComponent = 1; nComponent <= NumberOfComponents; ++nComponent)
          {
            ErrorShadow << _Mixture.getExponentialPartOfComponent(nComponent, Error),
                           sqrt(ceres::fmax(T(-log(_Mixture.getLinearPartOfComponent(nComponent, Error) / _Normalization)), T(1e-10)));/** fmax() is required to handle numeric tolerances */

            /** keep only the most likely component */
            if(ErrorShadow.squaredNorm() < Loglike || ceres::IsNaN(Loglike))
            {
              Loglike = ErrorShadow.squaredNorm();
              ErrorShadowBest = ErrorShadow;
            }
          }

          ErrorMap = ErrorShadowBest;
        }
        else
        {
          /** write something to the second component, if the error model is disabled */
          Error[Dimension] = T(0.0);
        }

        return true;
      };

    private:
      MixtureType _Mixture;
      double _Normalization;

  };

  typedef MaxMixture<1, GaussianMixture<1>> MaxMix1;
}

#endif // MAXMIXTURE_H
