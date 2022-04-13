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

#ifndef GAUSSIANMIXTURE_H
#define GAUSSIANMIXTURE_H

#include <algorithm>
//#include <thread>
//#include <future>
#include <ceres/ceres.h>

#include <unsupported/Eigen/SpecialFunctions>

#include "GaussianComponent.h"
// #include "../StateData.h"
#include "misc.h"
// #include "Messages.h"


namespace libRSF
{
  template <int Dimension>
  class GaussianMixture
  {
    public:
      GaussianMixture(){};
      virtual ~GaussianMixture(){};

      /** init with increasing uncertainties */
      void initSpread(const size_t Components, const double BaseStdDev)
      {
        libRSF::GaussianComponent<Dimension> Component;

        Eigen::Matrix<double, Dimension, 1> Mean = Eigen::Matrix<double, Dimension, 1>::Zero();
        Eigen::Matrix<double, 1, 1> Weight = Eigen::Matrix<double, 1, 1>::Ones() / Components;
        Eigen::Matrix<double, Dimension, Dimension> SqrtInfo = Eigen::Matrix<double, Dimension, Dimension>::Identity() * (1.0/BaseStdDev);

        this->clear();
        for(int nComponent = 0; nComponent < Components; ++nComponent)
        {
          Component.setParamsSqrtInformation(SqrtInfo*std::pow(0.1, nComponent), Mean, Weight);
          this->addComponent(Component);
        }

      }

      /** only for 1D GMMs */
      GaussianMixture(ceres::Vector Mean, ceres::Vector StdDev, ceres::Vector Weight);
      void addDiagonal(ceres::Vector Mean, ceres::Vector StdDev, ceres::Vector Weight);

      void addComponent(GaussianComponent<Dimension> Gaussian)
      {
        _Mixture.emplace_back(Gaussian);
      }

      void removeLastComponent()
      {
        _Mixture.pop_back();
      }

      void clear()
      {
        _Mixture.clear();
      }

      size_t getNumberOfComponents() const
      {
        return _Mixture.size();
      }

      typedef Eigen::Matrix<double, Eigen::Dynamic, Dimension> ErrorMatType;
//      double computeLikelihood(ErrorMatType & DataVector, Eigen::MatrixXd &Likelihood, size_t MaxNumThreads)
//      {
//        size_t N = Likelihood.rows();
//        size_t M = Likelihood.cols();
//        std::vector<std::thread> Threads;
//        std::vector<std::future<Eigen::VectorXd>> LikelihoodFutures;
//
//        /** E-step (multi-threaded) */
//        for(size_t m = 0; m < M; ++m)
//        {
//          /** pack function in task object*/
//          std::packaged_task<Eigen::VectorXd(ErrorMatType)> NewTask(std::bind(&GaussianComponent<Dimension>::computeProbability,
//              std::cref(_Mixture.at(m)),
//              std::placeholders::_1));
//          LikelihoodFutures.push_back(NewTask.get_future());
//
//          /** create computation threads */
//          Threads.push_back(std::move(std::thread(std::move(NewTask), std::cref(DataVector))));
//
//          /** finish all threads if maximum number is reached before a new set is created */
//          if(m % MaxNumThreads == MaxNumThreads-1 || m == M-1)
//          {
//            for(int nThread = (m / MaxNumThreads)*MaxNumThreads; nThread <= m; ++nThread)
//            {
//              Likelihood.col(nThread) = LikelihoodFutures.at(nThread).get();
//            }
//          }
//        }
//
//        /** delete threads when results are computed*/
//        for(std::thread &Thread : Threads)
//        {
//          if(Thread.joinable())
//            Thread.join();
//        }
//
//        Threads.clear();
//        LikelihoodFutures.clear();
//
//        /** remove NaNs */
//        Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 0.0);
//
//        /** calculate relative likelihood  */
//        double LikelihoodSumNew = Likelihood.sum();
//        Eigen::VectorXd LikelihoodRowSum = Likelihood.rowwise().sum();
//
//        for(size_t m = 0; m < M; ++m)
//        {
//          Likelihood.col(m).array() /= LikelihoodRowSum.array();
//        }
//
//        /** remove NaNs (occur if sum of likelihoods is zero) */
//        Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 1.0/M);
//
//        return LikelihoodSumNew;
//      }

//      void computeMixtureParameters(ErrorMatType & DataVector, Eigen::MatrixXd & Likelihood, size_t MaxNumThreads)
//      {
//        size_t N = Likelihood.rows();
//        size_t M = Likelihood.cols();
//        std::vector<std::thread> Threads;
//
//        for(size_t m = 0; m < M; ++m)
//        {
//          /** create computation threads */
//          Threads.push_back(std::thread(std::bind(&GaussianComponent<Dimension>::estimateParameters,
//                                                  std::ref(_Mixture.at(m)),
//                                                  std::cref(DataVector),
//                                                  Eigen::Ref<const Eigen::Matrix<double, Eigen::Dynamic, 1>>(Likelihood.col(m)),
//                                                  false)));
//
//          /** finish all thread if maximum number is reached before a new set is created */
//          if (m % MaxNumThreads == MaxNumThreads-1 || m == M-1)
//          {
//            for (std::thread & Thread : Threads)
//            {
//              if (Thread.joinable())
//                Thread.join();
//            }
//            Threads.clear();
//          }
//        }
//      }

      double computeLikelihood(ErrorMatType &DataVector, Eigen::MatrixXd &Likelihood)
      {
        size_t M = Likelihood.cols();

        /** E-step */
        for(size_t m = 0; m < M; ++m)
        {
          Likelihood.col(m) = _Mixture.at(m).computeProbability(DataVector);
        }

        /** remove NaNs */
        Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 0.0);

        /** calculate relative likelihood  */
        double LikelihoodSumNew = Likelihood.sum();
        Eigen::VectorXd LikelihoodRowSum = Likelihood.rowwise().sum();

        for(size_t m = 0; m < M; ++m)
        {
          Likelihood.col(m).array() /= LikelihoodRowSum.array();
        }

        /** remove NaNs (occur if sum of likelihoods is zero) */
        Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 1.0/M);

        return LikelihoodSumNew;
      }

      /** single threaded version */
      void computeMixtureParameters(ErrorMatType &DataVector, Eigen::MatrixXd &Likelihood)
      {
        size_t M = Likelihood.cols();

        for(size_t m = 0; m < M; ++m)
        {
          _Mixture.at(m).estimateParameters(DataVector,Likelihood.col(m),false);
        }
      }

      bool estimateWithEM(std::vector<double> &Data, bool Adaptive = false)
      {
        size_t N = Data.size();
        size_t M = _Mixture.size();

        /** map data to eigen vector */
        ErrorMatType DataVector = Eigen::Map<const ErrorMatType, Eigen::Unaligned>(Data.data(), N, Dimension);

        /** convergence criteria*/
        double LikelihoodSumOld = 1e40;
        double LikelihoodSumNew;
        bool PerfomedRemove = false;
        bool PerfomedMerge = false;
        bool ReachedConvergence = false;

        /** N x M matrix */
        Eigen::MatrixXd Likelihood;

        /** use at least one thread */
//        size_t MaxNumThreads = 1;//std::max(std::thread::hardware_concurrency(), static_cast<unsigned int>(1));

        /** repeat until convergence or 200 iterations*/
        for(int i = 0; i < 200; ++i)
        {
          M = _Mixture.size();
          Likelihood.resize(N, M);
          PerfomedRemove = false;

          /** E-step (multi-threaded) */
          LikelihoodSumNew = computeLikelihood(DataVector, Likelihood);

          /** M-Step (multi-threaded) */
          computeMixtureParameters(DataVector, Likelihood);

          double LikelihoodChange = std::abs(LikelihoodSumOld - LikelihoodSumNew)/LikelihoodSumNew;

          /** remove small components */
          if(Adaptive)
          {
            for(size_t m = 0; m < M; ++m)
            {
              if (_Mixture.at(m).getWeight()(0) < (Dimension+1.0)/N)
              {
                _Mixture.erase(_Mixture.begin() + m);
                M -= 1;
                PerfomedRemove = true;
              }
            }
          }

          /** check for convergence */
          if(LikelihoodChange < 1e-5)
          {
            ReachedConvergence = true;
          }
          else
          {
            ReachedConvergence = false;
          }

          /** terminate loop */
          if (ReachedConvergence && !PerfomedMerge && !PerfomedRemove)
          {
            // removeMean();
           std::cerr << "EM ended with " << _Mixture.size() <<" Components." << std::endl; /**< Debugging */
           printParameterByCout();
            return true;
          }

          LikelihoodSumOld = LikelihoodSumNew;
        }

       std::cerr << "EM did not converge with " << _Mixture.size() <<" Components." << std::endl; /**< Debugging */
        return false;
      }

      /** merge components that are identical */
      bool reduceMixture(double BhattacharyyaLimit)
      {
        bool PerformedMerge = false;
        size_t M = _Mixture.size();

        for(size_t m1 = 0; m1 < M-1; ++m1)
        {
          for(size_t m2 = m1+1; m2 < M; ++m2)
          {
            /** calculate metric */
            double d = CalculateBhattacharyyaDistance(_Mixture.at(m1), _Mixture.at(m2));

//            std::cerr << "Distance d from " << m1 <<" to " << m2 << " of " << M << " is " << exp(-d) << std::endl; /**< Debugging */

            /** perform merge if the distance exceed a specific tuning parameter*/
            if(exp(-d) > BhattacharyyaLimit)
            {
              _Mixture.at(m1) = MergeGaussians(_Mixture.at(m1), _Mixture.at(m2));
              _Mixture.erase(_Mixture.begin() + m2);

              M -= 1;
              m2 -= 1;

              PerformedMerge = true;
            }
          }
        }
        return PerformedMerge;
      }

      /** variational bayesian inference */
      void estimateWithVBI(std::vector<double> &Data, double Nu);

      void printParameter()
      {
        // PRINT_LOGGING("Mean    StdDev    Weight");
        for(size_t n = 0; n < _Mixture.size(); ++n)
        {
          // PRINT_LOGGING(_Mixture.at(n).getMean(), "  ", _Mixture.at(n).getSqrtInformation().inverse(),"  ", _Mixture.at(n).getWeight());
        }
      }

      void printParameterByCout()
      {
        std::cout << "/* Mean    StdDev    Weight */" << std::endl;
        for(size_t n = 0; n < _Mixture.size(); ++n)
        {
          // PRINT_LOGGING(_Mixture.at(n).getMean(), "  ", _Mixture.at(n).getSqrtInformation().inverse(),"  ", _Mixture.at(n).getWeight());
          std::cout <<_Mixture.at(n).getMean()<< "   " <<_Mixture.at(n).getSqrtInformation().inverse()<< "   " <<_Mixture.at(n).getWeight()<< std::endl;
        }
      }

      /** query error values for a specific component */
      template <typename T>
      Eigen::Matrix < T, Dimension, 1 > getExponentialPartOfComponent(size_t NumberOfComponent, T * const Error) const
      {
        return _Mixture.at(NumberOfComponent-1).getExponentialPart(Error);
      }

      template <typename T>
      double getLinearPartOfComponent(size_t NumberOfComponent, T * const Error) const
      {
        return _Mixture.at(NumberOfComponent-1).getLinearPart(Error);
      }

      double getMaximumOfComponent(size_t NumberOfComponent) const
      {
        return _Mixture.at(NumberOfComponent-1).getMaximum();
      }

      /** special function for pseudo range stuff */
      Eigen::VectorXd removeOffset()
      {
        size_t NumberOfComponents = this->getNumberOfComponents();
        Eigen::VectorXd MeanLOS;

        this->sortComponentsByWeight();
        double MinimumWeight = std::min(_Mixture.at(0).getWeight()(0), 0.2);

        /** remove offset of the first "LOS" component */
        this->sortComponentsByMean();
        for(int i = 0; i < NumberOfComponents; ++i)
        {
          if(_Mixture.at(i).getWeight()(0) >= MinimumWeight)
          {
            MeanLOS = _Mixture.at(i).getMean();
            break;
          }
        }

        for(int i = 0; i < NumberOfComponents; ++i)
        {
          _Mixture.at(i).setMean(_Mixture.at(i).getMean() - MeanLOS);
        }
        return MeanLOS;
      }
      void removeGivenOffset(Eigen::VectorXd Offset);

      void removeMean()
      {
        for(size_t i = 0; i < _Mixture.size(); ++i)
        {
          _Mixture.at(i).setMean(Eigen::Matrix<double, Dimension,1>::Zero());
        }
      }

      /** sort for better readability */
      void sortComponentsByMean()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByMean<Dimension>);
      }

      void sortComponentsByAbsMean()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByAbsMean<Dimension>);
      }

      void sortComponentsByWeight()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByWeight<Dimension>);
      }

      void sortComponentsByMode()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByMode<Dimension>);
      }

      void sortComponentsByLOSness()
      {
        std::sort(_Mixture.begin(), _Mixture.end(), compareByLOSness<Dimension>);
      }

      /** export GMM into state data for logging */
      // StateData exportToStateData(double Timestamp);

      public:
        std::vector<GaussianComponent<Dimension>> _Mixture;
  };
}

#endif // GAUSSIANMIXTURE_H
