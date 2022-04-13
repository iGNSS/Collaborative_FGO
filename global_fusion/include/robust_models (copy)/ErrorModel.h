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

#ifndef ERRORMODEL_H
#define ERRORMODEL_H


#include <ceres/ceres.h>
// #include "../StateData.h"

using ceres::CostFunctionToFunctor;
using ceres::Vector;

namespace libRSF
{
  class ErrorModelBase
  {
    public:
      ErrorModelBase() {};
      virtual ~ErrorModelBase() {};

      void enable()
      {
        _Enable = true;
      }

      void disable()
      {
        _Enable = false;
      }

    protected:
      bool _Enable = true;
  };


  template <int InputDim, int OutputDim, int ...StateDims>
  class ErrorModel: public ErrorModelBase
  {
    public:
      ErrorModel() {};
      virtual ~ErrorModel() {};

      const int getInputDim() const
      {
        return _InputDim;
      };

      const int getOutputDim() const
      {
        return _OutputDim;
      };

      static const int _OutputDim = OutputDim;
      static const int _InputDim = InputDim;

      /** store the dimension of variables at compile time */
      // using _StateDims = std::integer_sequence<int, StateDims...>;
  };
}

#endif // ERRORMODEL_H
