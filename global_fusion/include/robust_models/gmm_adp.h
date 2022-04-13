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
    /* time, satellite (measurement) number (index) per epoch */
    std::vector<std::pair<double, int>> satNumPerT;

    /* residuals at the each FGO (weighted residuals) */
    std::vector<double> Residuals;

    /* setup Gaussian mixture model parameters */
    libRSF::GaussianMixture<1> GMM_psr;

    /* epoch, residuals, GMM */
    struct GMM_set
    {
        /* data */
        double time;
        std::vector<double> residualSet;
        libRSF::GaussianMixture<1> GMMPara;
    };
    
    std::vector<std::pair<double, GMM_set>> GMMSets, lastGMMSets;
    int lastGMMSetSize = 0;

    int     NumberOfComponents = 3;
    double  DefaultStdDev = 1; // 1 0.7
    bool    ReduceComponents = true;
    bool    removeOffset = true;
    int     GMMResidualWindowSize = 50; // widow size for the resiudal of GMM

public:
    /* setup GNC-GM parameters */
    bool clearGMMVariables()
    {
        /* setup parameters */
        satNumPerT.clear();
        Residuals.clear();
        // GMMSet.clear();
        
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

    /* initialize Gaussian mixture sets  */
    bool initializeGMMSets()
    {
        // GMM_psr.initSpread(NumberOfComponents,DefaultStdDev); // 0.1
        int length = GMMSets.size();
        for(int i = 0; i < length; i++)
        {
            GMMSets[i].second.GMMPara.initSpread(NumberOfComponents,DefaultStdDev);
        }
        return true;
    }

    /* update satNumPerT */
    bool updatesatNumPerT(std::vector<std::pair<double, int>> value)
    {
        /* update the satNumPerT */
        satNumPerT = value;

        return true;
    }

    /* update residuals  */
    bool updateGMMResiduals(std::vector<double> residuals)
    {
        Residuals = residuals;
        std::cout<< "residual size -> " << Residuals.size() << std::endl;
        int epochSize = satNumPerT.size();

        /* update the GMMSets parameters */
        GMMSets.resize(epochSize);
        
        for(int k = 0; k < epochSize; k++)
        {
            if(k>GMMResidualWindowSize && epochSize>GMMResidualWindowSize)
            {
                GMM_set GMMSet;
                GMMSet.time = satNumPerT[k].first;
                int resIndext = -1;
                for(int i = k-GMMResidualWindowSize; i < k; i++)
                    for(int j = 0; j <satNumPerT[i].second; j++)
                    {
                        resIndext++;
                        double residual = Residuals[resIndext];
                        GMMSet.residualSet.push_back(residual);
                    }
                GMMSets[k] = std::make_pair(satNumPerT[k].first,GMMSet);
            }
        }

        /* assign the very early epoch GMM*/
        for(int k = 0; k < epochSize; k++)
        {
            if(k<=GMMResidualWindowSize && epochSize>(GMMResidualWindowSize+1))
            {
                GMMSets[k] = GMMSets[GMMResidualWindowSize+1];
            }
        }
        
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

        std::cout << "EM Time-> "<< EMTime.toc()<< std::endl;
        
        return true;
    }

    /* estimateGMMsets Paras  */
    bool estimateGMMSetsParas()
    {
        TicToc GMMSetEMTime;

        /* initialize Gaussian mixture sets */
        // initializeGMMSets();
        
        /* estimate the GMM parameters using Expectation maximization algorithm */
        int length = GMMSets.size();
        for(int i = 0;  i < lastGMMSetSize; i++)
        {
            GMMSets[i] = lastGMMSets[i]; 
        }
        
        for(int i = 0; i < length; i++)
        {
            /* do not re-estimate the old GMMs */
            if(i<lastGMMSetSize ) continue;

            GMMSets[i].second.GMMPara.initSpread(NumberOfComponents,DefaultStdDev);
            GMMSets[i].second.GMMPara.estimateWithEM(GMMSets[i].second.residualSet,ReduceComponents);

            /* remove the offset of the major Gaussian component */
            if(removeOffset)
            {
                // GMM_psr.removeOffset();
                GMMSets[i].second.GMMPara.removeMean();
                // std::cout<< "remove mean!! -> " << std::endl;
            } 
            /* print the parameters of GMM */
            std::cout << "estiamte GMM epoch> "<< GMMSets[i].first << "  " << i<< std::endl;
            GMMSets[i].second.GMMPara.printParameterByCout();
        }

        /* assign the very early epoch GMM*/
        if(length>(GMMResidualWindowSize+1))
        {
            for(int k = 0; k < (GMMResidualWindowSize+1); k++)
            {
                GMMSets[k] = GMMSets[GMMResidualWindowSize+1];
            }
        }
        
        std::cout << "GMMSetEMTime Time-> "<< GMMSetEMTime.toc()<< std::endl;
        
        lastGMMSetSize = GMMSets.size();
        lastGMMSets.resize(lastGMMSetSize);
        lastGMMSets = GMMSets;
        
        return true;
    }


    

};