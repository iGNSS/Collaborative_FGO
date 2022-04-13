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

// #include "error_models/GaussianMixture.h"
#include "GaussianMixture.h"

namespace libRSF
{
  template<>
  void GaussianMixture<1>::addDiagonal(ceres::Vector Mean, ceres::Vector StdDev, ceres::Vector Weight)
  {
    GaussianComponent<1> Gaussian;
    if (StdDev.size() == Mean.size() && StdDev.size() == Weight.size())
    {
      for(size_t nDim = 0; nDim < StdDev.size(); ++nDim)
      {
        Gaussian.setParamsStdDev(StdDev.segment<1>(nDim), Mean.segment<1>(nDim), Weight.segment<1>(nDim));
        addComponent(Gaussian);
      }
    }
    else
    {
      std::cerr << "Error in GaussianMixture::addDiagonal(): Parameter dimension isn't equal! "
                <<"StdDev: "  << StdDev.size()  << " "
                <<"Mean: "    << Mean.size()    << " "
                <<"Weight: "  << Weight.size()  << " " << std::endl;

    }

  }

  template<>
  GaussianMixture<1>::GaussianMixture(ceres::Vector Mean, ceres::Vector StdDev, ceres::Vector Weight)
  {
    addDiagonal(Mean, StdDev, Weight);
  }

  // template<>
  // Eigen::VectorXd GaussianMixture<1>::removeOffset()
  // {
  //   size_t NumberOfComponents = this->getNumberOfComponents();
  //   Eigen::VectorXd MeanLOS;

  //   this->sortComponentsByWeight();
  //   double MinimumWeight = std::min(_Mixture.at(0).getWeight()(0), 0.2);

  //   /** remove offset of the first "LOS" component */
  //   this->sortComponentsByMean();
  //   for(int i = 0; i < NumberOfComponents; ++i)
  //   {
  //     if(_Mixture.at(i).getWeight()(0) >= MinimumWeight)
  //     {
  //       MeanLOS = _Mixture.at(i).getMean();
  //       break;
  //     }
  //   }

  //   for(int i = 0; i < NumberOfComponents; ++i)
  //   {
  //     _Mixture.at(i).setMean(_Mixture.at(i).getMean() - MeanLOS);
  //   }
  //   return MeanLOS;
  // }

  template<>
  void GaussianMixture<1>::removeGivenOffset(Eigen::VectorXd Offset)
  {
    size_t NumberOfComponents = this->getNumberOfComponents();

    /** remove offset of the given component */
    for(int i = 0; i < NumberOfComponents; ++i)
    {
      _Mixture.at(i).setMean(_Mixture.at(i).getMean() - Offset);
    }
  }

  // template <>
  // StateData GaussianMixture<1>::exportToStateData(double Timestamp)
  // {
  //   size_t NumberOfComponents = this->getNumberOfComponents();

  //   /** save to vectors */
  //   Eigen::VectorXd Means(NumberOfComponents);
  //   Eigen::VectorXd StdDevs(NumberOfComponents);
  //   Eigen::VectorXd Weights(NumberOfComponents);
  //   for(int i = 0; i < NumberOfComponents; ++i)
  //   {
  //     Means(i) = _Mixture.at(i).getMean()[0];
  //     StdDevs(i) = 1/_Mixture.at(i).getSqrtInformation()[0];
  //     Weights(i) = _Mixture.at(i).getWeight()[0];
  //   }

  //   switch(NumberOfComponents)
  //   {
  //     case 1:
  //       {
  //         StateData GMMState(StateType::GM1, Timestamp);
  //         GMMState.setMean(Means);
  //         GMMState.setValue(libRSF::StateElement::GMM_StdDev, StdDevs);
  //         GMMState.setValue(libRSF::StateElement::GMM_Weight, Weights);
  //         return GMMState;
  //       }
  //       break;
  //     case 2:
  //       {
  //         StateData GMMState(StateType::GM2, Timestamp);
  //         GMMState.setMean(Means);
  //         GMMState.setValue(libRSF::StateElement::GMM_StdDev, StdDevs);
  //         GMMState.setValue(libRSF::StateElement::GMM_Weight, Weights);
  //         return GMMState;
  //       }
  //       break;
  //     case 3:
  //       {
  //         StateData GMMState(StateType::GM3, Timestamp);
  //         GMMState.setMean(Means);
  //         GMMState.setValue(libRSF::StateElement::GMM_StdDev, StdDevs);
  //         GMMState.setValue(libRSF::StateElement::GMM_Weight, Weights);
  //         return GMMState;
  //       }
  //       break;
  //     case 4:
  //       {
  //         StateData GMMState(StateType::GM4, Timestamp);
  //         GMMState.setMean(Means);
  //         GMMState.setValue(libRSF::StateElement::GMM_StdDev, StdDevs);
  //         GMMState.setValue(libRSF::StateElement::GMM_Weight, Weights);
  //         return GMMState;
  //       }
  //       break;
  //     case 5:
  //       {
  //         StateData GMMState(StateType::GM5, Timestamp);
  //         GMMState.setMean(Means);
  //         GMMState.setValue(libRSF::StateElement::GMM_StdDev, StdDevs);
  //         GMMState.setValue(libRSF::StateElement::GMM_Weight, Weights);
  //         return GMMState;
  //       }
  //       break;
  //     default:
  //         std::cerr << "Error in StateData GaussianMixture<1>::exportToStateData: wrong number of GMM components: " << NumberOfComponents << std::endl;
  //       break;
  //   }
  // }

  /** variational bayesian inference */
  template <>
  void GaussianMixture<1>::estimateWithVBI(std::vector<double> &Data, double Nu)
  {
    /** only 1D problems !!! */
    #define Dimension 1

    /** set up problem */
    size_t N = Data.size();
    size_t M = _Mixture.size();

    /** N x M matrix */
    Eigen::MatrixXd Likelihood;
    Likelihood.resize(N, M);

    /** map data to eigen vector */
    ErrorMatType DataVector = Eigen::Map<const ErrorMatType, Eigen::Unaligned>(Data.data(), N, Dimension);
    /** change sign to match own GMM definition */
    DataVector.array() *= -1;

    double DataMean = DataVector.array().mean();
    double DataCov = (DataVector.array() - DataMean).square().sum()/(DataVector.size()-1);

//    std::cerr << "Mean  is: " << DataMean << std::endl; /**< Debugging */
//    std::cerr << "Cov   is: " << DataCov << std::endl; /**< Debugging */
//    std::cerr << "Cov_r is: " << DataCovRobust << std::endl; /**< Debugging */

    /** convergence criterion */
    double const LikelihoodTolerance = 1e-6;
    int const MaxIterations = 1000;

    /** pruning criterion */
    double MinWeight = 1.0/N;

    /** define priors */
    double const Beta = 1e-6;                 /**< prior over mean */
    //double const Nu = 10;                      /**< Wishart prior for I */
    double const V = DataCov/Nu;              /**< Wishart prior for I */

    /** initialize  variables */
    std::vector<double> NuInfo;
    std::vector<double> VInfo;
    std::vector<double> InfoMean;
    std::vector<double> MeanMean;
    std::vector<double> Weights;

    /** initialize estimated parameters */
    for(size_t m = 0; m < M; ++m)
    {
      NuInfo.push_back(Nu);
      VInfo.push_back(V);
      InfoMean.push_back(0.0); /**< will be overwritten */
      MeanMean.push_back(0.0); /**< will be overwritten */
      Weights.push_back(_Mixture.at(m).getWeight()(0));
    }

    /** convergence criteria (not variational!)*/
    double LikelihoodSumOld = 1e40;
    double LikelihoodSumNew;

    /** first likelihood estimation is not variational! */
    LikelihoodSumNew = computeLikelihood(DataVector, Likelihood);

    /** repeat until convergence or 100 iterations*/
    int i;
    for(i = 0; i < MaxIterations; ++i)
    {
      bool ReachedConvergence = false;
      bool PerfomedRemove = false;

      /** Expectation step */

      /** pre-calculate some multi-use variables */
      Eigen::VectorXd SumLike = Likelihood.colwise().sum();
      Eigen::VectorXd SumLikeX = (Likelihood.array().colwise() * DataVector.array()).colwise().sum();
      Eigen::VectorXd SumLikeXX = (Likelihood.array().colwise() * DataVector.array().square()).colwise().sum();

      double MuMuEx, InfoEx, LogInfoEx;
      /** iterate over GMM components */
      for(size_t m = 0; m < M; ++m)
      {

        /** calculate mean */
        InfoMean.at(m) = Beta + NuInfo.at(m) / VInfo.at(m) * SumLike(m);
        MeanMean.at(m) = NuInfo.at(m) / VInfo.at(m) / InfoMean.at(m) * SumLikeX(m);

        /** calculate variance */
        NuInfo.at(m) = Nu + SumLike(m);
        MuMuEx = 1.0/InfoMean.at(m) + MeanMean.at(m)*MeanMean.at(m);

        VInfo.at(m) = V + SumLikeXX(m) - 2*MeanMean.at(m)*SumLikeX(m) + MuMuEx * SumLike(m);

        /** variational likelihood */
        InfoEx = NuInfo.at(m)/VInfo.at(m);
        LogInfoEx = Eigen::numext::digamma(NuInfo.at(m)/2.0) + log(2.0) - log(VInfo.at(m));

        Likelihood.col(m) = (LogInfoEx/2.0
                             + log(Weights.at(m))
                             - 0.5*InfoEx*
                             (
                               DataVector.array().square()
                               - 2.0 * MeanMean.at(m) * DataVector.array()
                               + MuMuEx
                             )
                            ).array().exp();
      }

      /** "Maximization step" */

      /** remove NaNs */
      Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 0.0);

      /** calculate relative likelihood  */
      LikelihoodSumNew = Likelihood.sum();
      Eigen::VectorXd LikelihoodRowSum = Likelihood.rowwise().sum();

      for(size_t m = 0; m < M; ++m)
      {
        Likelihood.col(m).array() /= LikelihoodRowSum.array();
      }

      /** remove NaNs (occur if sum of likelihoods is zero) */
      Likelihood = (Likelihood.array().isFinite()).select(Likelihood, 1.0/M);

      /** calculate weights */
      for(size_t m = 0; m < M; ++m)
      {
        Weights.at(m)= Likelihood.col(m).sum() / Likelihood.rows();
      }

      /** remove useless components */
      for(int m = M-1; m >=0 ; --m)
      {
        if(Weights.at(m) < MinWeight)
        {
          NuInfo.erase(NuInfo.begin() + m);
          VInfo.erase(VInfo.begin() + m);
          InfoMean.erase(InfoMean.begin() + m);
          MeanMean.erase(MeanMean.begin() + m);
          Weights.erase(Weights.begin() + m);
          RemoveColumn(Likelihood,m);
          _Mixture.erase(_Mixture.begin() + m);

          M -= 1;
          /** enforce an additional iteration after removal and reset likelihood */
          PerfomedRemove = true;
          LikelihoodSumNew = 1e40;
        }
      }

      /** check for convergence */
      double LikelihoodChange = std::abs(LikelihoodSumOld - LikelihoodSumNew)/LikelihoodSumNew;

      if(LikelihoodChange < LikelihoodTolerance)
      {
        ReachedConvergence = true;
      }
      else
      {
        ReachedConvergence = false;
      }

      /** terminate loop */
      if(ReachedConvergence && !PerfomedRemove)
      {
        break;
      }
      else
      {
        LikelihoodSumOld = LikelihoodSumNew;
      }

    }

    /** update mixtures */
    for(size_t m = 0; m < M; ++m)
    {
      Eigen::Matrix<double,1,1> Mean;
      Eigen::Matrix<double,1,1> Info;
      Eigen::Matrix<double,1,1> Weight;

      Mean << MeanMean.at(m);
      Info << NuInfo.at(m)/VInfo.at(m);
      Weight << Weights.at(m);

      _Mixture.at(m).setParamsInformation(Info, Mean, Weight);
    }

    /** Debugging output */
//    std::cerr << "VBI started with  " << Data.size() <<" samples."<< std::endl; /**< Debugging */
//    std::cerr << "VBI ended with " << _Mixture.size() <<" Components, after " << i << " Iterations."<< std::endl; /**< Debugging */

  }
}
