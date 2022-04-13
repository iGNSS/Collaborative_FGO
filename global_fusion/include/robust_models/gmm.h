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


#define D2R 3.1415926/180.0
#include <nlosExclusion/GNSS_Raw_Array.h>
// google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>

#include "SumMixture.h"
#include "GaussianMixture.h"
#include "GaussianComponent.h"

class GaussianMixtureModel{
public:
    /* satellite (measurement) number (index) per epoch */
    std::vector<int> satNumPerT;

    /* residuals at the each FGO (weighted residuals) */
    std::vector<double> Residuals;

    /* setup Gaussian mixture model parameters */
    libRSF::GaussianMixture<1> GMM_psr;
    int NumberOfComponents = 3;
    double DefaultStdDev = 1; // 1 0.7
    bool ReduceComponents = true;
    bool removeOffset = true;
    int GMMResidualWindowSize = 1000; // widow size for the resiudal of GMM

public:
    /* setup GNC-GM parameters */
    bool clearGMMVariables()
    {
        /* setup parameters */
        satNumPerT.clear();
        Residuals.clear();
        
        return true;
    }

    /* setup the Gaussian component numbers */
    bool setNumberOfComponents(int num)
    {
        NumberOfComponents = num;
        return true;
    }

    /* setup the DefaultStdDev of Gaussian mixture  */
    bool setDefaultStdDev(double value)
    {
        DefaultStdDev = value;
        return true;
    }

    /* setup the ReduceComponents of Gaussian mixture  */
    bool setReduceComponents(bool value)
    {
        ReduceComponents = value;
        return true;
    }

    /* setup the ReduceComponents of Gaussian mixture  */
    bool setremoveOffset(bool value)
    {
        removeOffset = value;
        return true;
    }

    /* setup the GMMResidualWindowSize of Gaussian mixture  */
    bool setGMMResidualWindowSize(int value)
    {
        GMMResidualWindowSize = value;
        return true;
    }

    /* initialize Gaussian mixture  */
    bool initializeGMM()
    {
        GMM_psr.initSpread(NumberOfComponents,DefaultStdDev); // 0.1
        return true;
    }

    /* update residuals  */
    bool updateGMMResiduals(std::vector<double> residuals)
    {
        Residuals = residuals;

        std::cout<< "residual size -> " << Residuals.size() << std::endl;
        
        return true;
    }

    /* estimateGMMParas  */
    bool estimateGMMParas()
    {
        TicToc EMTime;

        /* initialize Gaussian mixture  */
        initializeGMM();

        /* estimate the GMM parameters using Expectation maximization algorithm */
        GMM_psr.estimateWithEM(Residuals, ReduceComponents);
        
        /* remove the offset of the major Gaussian component */
        if(removeOffset)
        {
            // GMM_psr.removeOffset();
            GMM_psr.removeMean();
            std::cout<< "remove mean!! -> " << std::endl;
        } 

        /* print the parameters of GMM */
        GMM_psr.printParameterByCout();

        std::cout << "EM Time (second)-> "<< EMTime.toc() / 1000<< std::endl;
        
        return true;
    }


    

};