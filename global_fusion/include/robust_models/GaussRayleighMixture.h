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

#ifndef GAUSSRAYLEIGHMIXTURE_H
#define GAUSSRAYLEIGHMIXTURE_H


#include "GaussianComponent.h"
#include "RayleighComponent.h"

#include <Eigen/Dense>

namespace libRSF
{
  class GaussRayleighMixture
  {
    public:

      virtual ~GaussRayleighMixture() {};

      GaussRayleighMixture() {};

      /** construct mixture */
      GaussRayleighMixture(Eigen::Matrix<double,2,1> StdDev, Eigen::Matrix<double,2,1> Weight): _Gaussian(GaussianComponent<1>()), _Rayleigh(RayleighComponent<1>())
      {
        _Gaussian.setParamsStdDev(StdDev.segment<1>(0), Eigen::Matrix<double,1,1>::Zero(), Weight.segment<1>(0));
        _Rayleigh.setParamsStdDev(StdDev.segment<1>(1), Weight.segment<1>(1));
      }

      size_t getNumberOfComponents() const
      {
        return 2;
      }

      /** query error values for a specific component */
      template <typename T>
      Eigen::Matrix < T, 1, 1 > getExponentialPartOfComponent(size_t NumberOfComponent, T * const Error) const
      {
        if (NumberOfComponent == 1)
        {
          return _Gaussian.getExponentialPart(Error);
        }
        else if (NumberOfComponent == 2)
        {
          return _Rayleigh.getExponentialPart(Error);
        }
      }

      template <typename T>
      T getLinearPartOfComponent(size_t NumberOfComponent, T * const Error) const
      {
        if (NumberOfComponent == 1)
        {
          return T(_Gaussian.getLinearPart(Error));
        }
        else if (NumberOfComponent == 2)
        {
          return _Rayleigh.getLinearPart(Error);
        }
      }

      double getMaximumOfComponent(size_t NumberOfComponent) const
      {
        if (NumberOfComponent == 1)
        {
          return _Gaussian.getMaximum();
        }
        else if (NumberOfComponent == 2)
        {
          return _Rayleigh.getMaximum();
        }
      }

      void printParameter()
      {
        std::cout << "Mean" <<' ' << "StdDev" <<' ' << "Weight" <<std::endl;
        std::cout << _Gaussian.getMean() <<' ' << _Gaussian.getSqrtInformation().inverse() <<' ' << _Gaussian.getWeight() <<std::endl;
        std::cout << "StdDev" <<' ' << "Weight" <<std::endl;
        std::cout << _Rayleigh.getSqrtInformation().inverse() <<' ' << _Rayleigh.getWeight() <<std::endl;
      }

      /** fit to data */
      bool estimateWithEM(std::vector<double> const &Data);

    private:
      GaussianComponent<1> _Gaussian;
      RayleighComponent<1> _Rayleigh;
  };
}

#endif // GAUSSRAYLEIGHMIXTURE_H
