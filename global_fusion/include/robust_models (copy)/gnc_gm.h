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


#include <nlosExclusion/GNSS_Raw_Array.h>
// google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>

class GNCGemanMcClure{
public:
    /* parameters for GNC infinite: convex; zero: non-convex */
    double GNC_mu = 1000000; 

    /* the maximum error you can accept (0.7) */
    double GNC_c = 2; 

    /* the maximum residual at first solving */
    double GNC_r_max = 0; 

    /* the weightings corresponding to measurements (vector) each epoch (double *) */
    std::vector<double*> weighings; 

    /* satellite (measurement) number (index) per epoch */
    std::vector<int> satNumPerT;

    /* residuals at first FGO (weighted residuals) */
    std::vector<double> origResiduals;

    /* residuals at the each FGO (weighted residuals) */
    std::vector<double> Residuals;

    /* number of iterations of GNC */
    int GNC_iter = 0;

public:
    /* setup GNC-GM parameters */
    bool setGNCGMParas(double _GNC_mu, double _GNC_c, double _GNC_r_max)
    {
        /* setup parameters */
        GNC_mu = _GNC_mu;
        GNC_c = _GNC_c;
        GNC_r_max = _GNC_r_max;

        weighings.clear();
        satNumPerT.clear();
        origResiduals.clear();
        Residuals.clear();
        GNC_iter = 0;
        
        return true;
    }

    /* alocate memory */
    bool allocateMemorys()
    {
        /* allocate memory based on satNumPerT */
        int length = satNumPerT.size();
        weighings.resize(length);
        for(int i = 0;  i < length; i++) // initialize
        {
            weighings[i] = new double[satNumPerT[i]];
            // double a[satNumPerT[i]] = {0};
            // weighings.push_back(a);
            for(int j = 0; j < satNumPerT[i]; j++)
            {
                weighings[i][j] = 1.0; 
            }
        }       
        // LOG(INFO) << "allocate memory size  length-> " <<length<<std::endl;
        // LOG(INFO) <<"allocate memory size-> " <<weighings.size()<<std::endl;
        return true;
    }

    /* input residuals after first FGO */
    bool inputFirstFGOResiduals(std::vector<double> residuals)
    {
        origResiduals = residuals;

        Residuals = residuals;
        
        return true;
    }

    /* calculate the maximum residual after first FGO */
    bool calculateMaxResidual()
    {
        GNC_r_max = *max_element(origResiduals.begin(), origResiduals.end());
        // GNC_r_max = 3000;
        if(GNC_r_max>200) GNC_r_max = 200;
        std::cout << "max residual is-------------------------------> "<< GNC_r_max << std::endl;
        return true;
    }

    /* calculate the GNC_mu after first FGO */
    bool updateOriginGNCMu()
    {        
        GNC_mu = (2 * pow(GNC_r_max, 2)) / pow(GNC_c, 2);
        
        return true;
    }

    /* calculate weightings corresponding to each measurements */
    bool updateWeightings()
    {
        int res_index = -1;
        int length = satNumPerT.size();
        for(int i = 0;  i < length; i++) // initialize
        {
            // std::cout <<"satNumPerT size-> " <<satNumPerT.size()<<std::endl;
            // std::cout <<"i -> " <<i<<std::endl;
            for(int j = 0; j < satNumPerT[i]; j++)
            {
                res_index++;
                weighings[i][j] = (GNC_mu * pow(GNC_c, 2)) / (pow(Residuals[res_index],2) + GNC_mu * pow(GNC_c, 2)); 
            }
        }
        GNC_iter++;
        return true;
    }

    /* update the weightings to the GNSS raw data structure */
    bool updateGNSSRawDataWeightings(std::map<double, nlosExclusion::GNSS_Raw_Array>& gnss_raw_map)
    {
        int res_index = -1;
        int length = satNumPerT.size();
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator itR;
        itR = gnss_raw_map.begin();

        for(int i = 0;  i < length; i++) // initialize
        {
            // std::cout <<"satNumPerT size-> " <<satNumPerT.size()<<std::endl;
            // std::cout <<"i -> " <<i<<std::endl;
            for(int j = 0; j < satNumPerT[i]; j++)
            {
                res_index++;
                itR->second.GNSS_Raws[j].raw_pseudorange = weighings[i][j];
            }
            ++itR;
        }
        return true;
    }

    /* initialize weightings corresponding to each measurements */
    bool initializeWeightings(std::map<double, nlosExclusion::GNSS_Raw_Array> gnss_raw_map)
    {
        int res_index = -1;
        int length = satNumPerT.size();
        std::map<double, nlosExclusion::GNSS_Raw_Array>::iterator itR;
        itR = gnss_raw_map.begin();

        for(int i = 0;  i < length; i++) // initialize
        {
            // std::cout <<"satNumPerT size-> " <<satNumPerT.size()<<std::endl;
            // std::cout <<"i -> " <<i<<std::endl;
            for(int j = 0; j < satNumPerT[i]; j++)
            {
                res_index++;
                // itR->second.GNSS_Raws[j].raw_pseudorange = weighings[i][j];
                weighings[i][j] = itR->second.GNSS_Raws[j].raw_pseudorange;
            }
            ++itR;
        }
        return true;
    }

    

    /* update residuals */
    bool updateResiduals(std::vector<double> residuals)
    {
        /* update residuals */
        Residuals = residuals;
        
        return true;
    }

    /* update un-weighted residuals */
    bool updateUnweightedResiduals()
    {
        int length = satNumPerT.size();
        int index = -1;
        for(int i = 0;  i < length; i++) // initialize
        {
            for(int j = 0; j < satNumPerT[i]; j++)
            {
                index++;
                Residuals[index] = Residuals[index] / weighings[i][j];
            }
        }
        
        return true;
    }
};