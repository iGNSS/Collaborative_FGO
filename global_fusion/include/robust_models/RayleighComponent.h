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



#ifndef RAYLEIGHCOMPONENT_H
#define RAYLEIGHCOMPONENT_H


#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "VectorMath.h"

namespace libRSF
{
  template <int Dimension>
  class RayleighComponent
  {
    public:

      RayleighComponent()
      {
        _Scaling = 1;
      };
      virtual ~RayleighComponent(){};

      /** set parameters */
      void setParamsStdDev(ceres::Vector StdDev, ceres::Vector Weight)
      {
        _Weight = Weight;
        _SqrtInformation = StdDev.cwiseInverse().asDiagonal();

        updateScaling();
      }

      void setParamsSqrtInformation(ceres::Matrix SqrtInformation, ceres::Vector Weight)
      {
        _Weight = Weight;
        _SqrtInformation = SqrtInformation;

        updateScaling();
      }

      /** get parameters */
      Eigen::Matrix<double, 1, 1> getWeight() const
      {
        return _Weight;
      }

      Eigen::Matrix<double, Dimension, Dimension> getSqrtInformation() const
      {
        return _SqrtInformation;
      }


      /** return maximum likelihood value */
      double getMaximum() const
      {
        return _SqrtInformation.determinant() * std::exp(-0.5);
      }

      /** return elements for error model */

      /**only valid for 1D case!!! */
      template <typename T>
      Eigen::Matrix < T, Dimension, 1 > getExponentialPart(T * const Error) const
      {
        /**< Rayleigh distribution is not defined for negative values */
        if (Error[0] > T(0.0))
        {
          return Eigen::Matrix<T, Dimension, 1>::Zero();
        }
        else
        {
          Eigen::Map <Eigen::Matrix<T, Dimension, 1> > ErrorMap(Error);

          /** scale with information matrix */
          return _SqrtInformation.template cast<T>() * ErrorMap;
        }
      }

      /**only valid for 1D case!!! */
      template <typename T>
      T getLinearPart(T * const Error) const
      {
        if (Error[0] > T(0.0))
          return T(0.0); /**< Rayleigh distribution is not defined for negative values */
        else
          return -Error[0] * _Scaling;
      }

       /** compute probability for E step of EM */
      Eigen::VectorXd computeProbability (Eigen::Matrix<double, Eigen::Dynamic, Dimension> const &Errors) const
      {
        Eigen::VectorXd Result(Errors.rows());

        /** apply mean */
        Eigen::Matrix<double, Eigen::Dynamic, Dimension> WeightedError = Errors;

        /** multiply each row with square root information */
        for (int n = WeightedError.rows()-1; n >= 0; n--)
        {
          WeightedError.row(n) = WeightedError.row(n) * _SqrtInformation;
        }

        Result = ((WeightedError.array().square().rowwise().sum() / -2.0).exp() * _Scaling * -Errors.array()).matrix();

        /** set probability of negative components to zero */
        Result = (Result.array() < 0.0).select(0.0, Result);

        return Result;
      }

       /** estimate parameters for M step of EM */
      void estimateParameters (Eigen::Matrix<double, Eigen::Dynamic, Dimension> const  &Errors,
                               Eigen::Matrix<double, Eigen::Dynamic, 1> const  &Likelihoods)
      {
        double LikelihoodSum = Likelihoods.sum();

        _Weight[0] = LikelihoodSum / Likelihoods.rows();

        Eigen::Matrix<double, Dimension, Dimension> Covariance = Eigen::Matrix<double, Dimension, Dimension>::Zero();
        for (int n = Errors.rows()-1; n >= 0; n--)
        {
          Covariance += Errors.row(n).transpose() * Errors.row(n) * Likelihoods[n];
        }
        Covariance.array() /= LikelihoodSum*2;
        _SqrtInformation = InverseSquareRoot(Covariance);

        updateScaling();
      }

    private:
      void updateScaling()
      {
        _Scaling = calculateNormalization();
      }

      double calculateNormalization()
      {
        return _Weight(0, 0) * std::pow(_SqrtInformation.determinant(), 2);
      }

      Eigen::Matrix<double, 1, 1> _Weight;
      Eigen::Matrix<double, Dimension, Dimension> _SqrtInformation;
      double _Scaling;
  };
}

#endif // RAYLEIGHCOMPONENT_H
