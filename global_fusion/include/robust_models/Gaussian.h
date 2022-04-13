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

#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include <Eigen/Dense>
#include "ErrorModel.h"
#include "VectorMath.h"

namespace libRSF
{
  template <int Dimension>
  class GaussianDiagonal : public ErrorModel <Dimension, Dimension>
  {
  public:

      GaussianDiagonal(){};

      explicit GaussianDiagonal(Eigen::Matrix<double, Dimension, 1> StdDev)
      {
        setStdDev(StdDev);
      };

      void setStdDev(Eigen::Matrix<double, Dimension, 1> StdDev)
      {
        _SqrtInformation = StdDev.cwiseInverse().asDiagonal();
      }

      Eigen::Matrix<double, Dimension, 1> getStdDev()
      {

      }

      template <typename T>
      bool Evaluate(T* Error) const
      {
        Eigen::Map<Eigen::Matrix<T, Dimension, 1>> ErrorMap(Error);

        /** scale with information matrix */
        ErrorMap = _SqrtInformation.template cast<T>() * ErrorMap;

        return true;
      };

  private:
    Eigen::Matrix<double,Dimension,Dimension> _SqrtInformation;
  };

  template <int Dimension>
  class GaussianFull : public ErrorModel <Dimension, Dimension>
  {
  public:

      GaussianFull()
      {};

      void setCovarianceMatrix(Eigen::Matrix<double, Dimension, Dimension> CovMat)
      {
        _SqrtInformation = InverseSquareRoot(CovMat);
      }

      template <typename T>
      bool Evaluate(T* Error) const
      {
        Eigen::Map<Eigen::Matrix<T, Dimension, 1>> ErrorMap(Error);

        ErrorMap = _SqrtInformation.template cast<T>() * ErrorMap;

        return true;
      };

  private:

      Eigen::Matrix<double, Dimension, Dimension> _SqrtInformation;
  };
}

#endif // GAUSSIAN_H
