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

#ifndef GAUSSIANCOMPONENT_H
#define GAUSSIANCOMPONENT_H

#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "VectorMath.h"
using namespace ceres;

namespace libRSF
{
  template <int Dimension>
  class GaussianComponent
  {
    public:
      GaussianComponent()
      {
        _Scaling = 1;
      }

      ~GaussianComponent(){};

      void setParamsStdDev(Vector StdDev, Vector Mean, Vector Weight)
      {
        _Mean = Mean;
        _Weight = Weight;

        /** write information matrix directly */
        _SqrtInformation = StdDev.cwiseInverse().asDiagonal();

        updateScaling();
      }

      void setParamsCovariance(ceres::Matrix Covariance, Vector Mean, Vector Weight)
      {
        _Mean = Mean;
        _Weight = Weight;

        /** use cholesky decomposition to get square root information matrix */
        _SqrtInformation = InverseSquareRoot(Covariance);

        updateScaling();
      }

      void setParamsInformation(ceres::Matrix Information, Vector Mean, Vector Weight)
      {
        _Mean = Mean;
        _Weight = Weight;
        _SqrtInformation = SquareRoot(Information);

        updateScaling();
      }

      void setParamsSqrtInformation(ceres::Matrix SqrtInformation, Vector Mean, Vector Weight)
      {
        _Mean = Mean;
        _Weight = Weight;
        _SqrtInformation = SqrtInformation;

        updateScaling();
      }

      Eigen::Matrix<double, Dimension, 1> getMean() const
      {
        return _Mean;
      }

      Eigen::Matrix<double, 1, 1> getWeight() const
      {
        return _Weight;
      }

      Eigen::Matrix<double, Dimension, Dimension> getSqrtInformation() const
      {
        return _SqrtInformation;
      }

      Eigen::Matrix<double, Dimension, Dimension> getCovariance() const
      {
        return (_SqrtInformation.transpose() * _SqrtInformation).inverse();
      }

      void setMean(Eigen::Matrix<double, Dimension, 1> Mean)
      {
        _Mean = Mean;
      }

      /** return the part inside the exp() function */
      template <typename T>
      Eigen::Matrix < T, Dimension, 1 > getExponentialPart(T * const Error) const
      {
        Eigen::Map <const Eigen::Matrix<T, Dimension, 1>> ErrorMap(Error);
        Eigen::Matrix <T, Dimension, 1> WeightedError;

        /** shift by mean */
        WeightedError = ErrorMap + _Mean.template cast<T>();

        /** scale with information matrix */
        WeightedError = _SqrtInformation.template cast<T>() * WeightedError;

        return WeightedError;
      }

      /** return the part before the exp() function */
      template <typename T>
      double getLinearPart(T * const Error) const
      {
        return _Scaling;
      }

      /** return the maximum value at the mode of the probability function*/
      double getMaximum() const
      {
        return _Scaling;
      }

      /** compute probability for E step of EM */
      Eigen::VectorXd computeProbability (Eigen::Matrix<double, Eigen::Dynamic, Dimension> const &Errors) const
      {
        /** apply mean */
        Eigen::Matrix<double, Eigen::Dynamic, Dimension> WeightedError = (Errors.array().rowwise() + _Mean.transpose().array()).matrix();

        /** multiply each row with square root information */
        for (size_t n = 0; n < WeightedError.rows(); ++n)
        {
          WeightedError.row(n) = WeightedError.row(n) * _SqrtInformation;
        }

        return ((WeightedError.array().square().rowwise().sum() / -2.0).exp() * _Scaling).matrix();
      }

      /** estimate parameters for M step of EM */
      void estimateParameters (Eigen::Matrix<double, Eigen::Dynamic, Dimension> const  &Errors,
                               Eigen::Matrix<double, Eigen::Dynamic, 1> const  &Likelihoods,
                               bool KeepMean)
      {
        double LikelihoodSum = Likelihoods.sum();

        _Weight(0) = LikelihoodSum / Likelihoods.rows();

        if (KeepMean == false)
        {
          _Mean = -((Errors.array().colwise() * Likelihoods.array()).colwise().sum() / LikelihoodSum).matrix().transpose();
        }

        Eigen::Matrix<double, Dimension, Dimension> Covariance = Eigen::Matrix<double, Dimension, Dimension>::Zero();
        for (size_t n = 0; n < Errors.rows(); ++n)
        {
          Covariance += (Errors.row(n) + _Mean).transpose() * (Errors.row(n) + _Mean) * Likelihoods(n);
        }
        Covariance.array() /= LikelihoodSum;
        _SqrtInformation = InverseSquareRoot(Covariance);

        /** check for degenerated SqrtInfo matrix */
        if(!(_SqrtInformation.array().isFinite().all()))
        {
          /** disable component if degenerated */
          _Weight = Eigen::Matrix<double, 1, 1>::Zero();
          _Mean = Eigen::Matrix<double, Dimension, 1>::Zero();
          _SqrtInformation = Eigen::Matrix<double, Dimension, Dimension>::Identity();
        }

        updateScaling();
      }

    private:
      void updateScaling()
      {
        _Scaling = calculateNormalization();
      }

      double calculateNormalization()
      {
        return _Weight(0, 0) * _SqrtInformation.determinant();
      }


      Eigen::Matrix<double, Dimension, 1> _Mean;
      Eigen::Matrix<double, 1, 1> _Weight;
      Eigen::Matrix<double, Dimension, Dimension> _SqrtInformation;
      double _Scaling;
  };

  /** compare functions for ordering */
  template <int Dimension>
  bool compareByMode(const GaussianComponent<Dimension> & a, const GaussianComponent<Dimension> & b)
  {
    return (a.getWeight()(0) * a.getSqrtInformation().determinant()) > (b.getWeight()(0) * b.getSqrtInformation().determinant());
  }

  template <int Dimension>
  bool compareByMean(const GaussianComponent<Dimension> & a, const GaussianComponent<Dimension> & b)
  {
    return a.getMean().sum() < b.getMean().sum();
  }

  template <int Dimension>
  bool compareByAbsMean(const GaussianComponent<Dimension> & a, const GaussianComponent<Dimension> & b)
  {
    return std::abs(a.getMean().sum()) < std::abs(b.getMean().sum());
  }

  template <int Dimension>
  bool compareByWeight(const GaussianComponent<Dimension> & a, const GaussianComponent<Dimension> & b)
  {
    return a.getWeight()(0) > b.getWeight()(0);
  }

  template <int Dimension>
  bool compareByLOSness(const GaussianComponent<Dimension> & a, const GaussianComponent<Dimension> & b)
  {
    return (a.getWeight()(0) * a.getSqrtInformation().determinant() / (std::abs(a.getMean().sum()) + 1e-6))
           >
           (b.getWeight()(0) * b.getSqrtInformation().determinant() / (std::abs(b.getMean().sum()) + 1e-6));
  }

  /** Bhattacharyya distance for merging */
  template <int Dimension>
  double CalculateBhattacharyyaDistance(const GaussianComponent<Dimension> & Gaussian1, const GaussianComponent<Dimension> & Gaussian2)
  {
    Eigen::Matrix<double, Dimension, Dimension> Cov1 = Gaussian1.getCovariance();
    Eigen::Matrix<double, Dimension, Dimension> Cov2 = Gaussian2.getCovariance();
    Eigen::Matrix<double, Dimension, Dimension> CovMean = (Cov1 + Cov2).array() / 2.0;

    Eigen::Matrix<double, Dimension, 1> MeanDiff = Gaussian1.getMean() - Gaussian2.getMean();

    Eigen::Matrix<double, 1, 1> Mahala = 0.125 * MeanDiff.transpose() * CovMean.inverse() * MeanDiff;

    return Mahala(0) + 0.5 * log(CovMean.determinant() / sqrt(Cov1.determinant()*Cov2.determinant()));
  }

  /** merge two gaussians */
  template <int Dimension>
  GaussianComponent<Dimension> MergeGaussians (const GaussianComponent<Dimension> & Gaussian1, const GaussianComponent<Dimension> & Gaussian2)
  {
    Eigen::Matrix<double, 1, 1> Weight12 = Gaussian1.getWeight() + Gaussian2.getWeight();
    Eigen::Matrix<double, Dimension, 1> Mean12 = (Gaussian1.getMean() * Gaussian1.getWeight() + Gaussian2.getMean() * Gaussian2.getWeight()).array() / Weight12(0);
    Eigen::Matrix<double, Dimension, Dimension> SqrtInfo12 = (Gaussian1.getSqrtInformation() * Gaussian1.getWeight() + Gaussian2.getSqrtInformation() * Gaussian2.getWeight()).array() / Weight12(0);

    GaussianComponent<Dimension> Gaussian12;
    Gaussian12.setParamsSqrtInformation(SqrtInfo12, Mean12, Weight12);

    return Gaussian12;
  }

}

#endif // GAUSSIANCOMPONENT_H
