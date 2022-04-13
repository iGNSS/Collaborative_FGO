/*------------------------------------------------------------------------------
* pntpos.c : standard positioning
*
*          Copyright (C) 2007-2015 by T.TAKASU, All rights reserved.
*
* version : $Revision:$ $Date:$
* history : 2010/07/28 1.0  moved from rtkcmn.c
*                           changed api:
*                               pntpos()
*                           deleted api:
*                               pntvel()
*           2011/01/12 1.1  add option to include unhealthy satellite
*                           reject duplicated observation data
*                           changed api: ionocorr()
*           2011/11/08 1.2  enable snr mask for single-mode (rtklib_2.4.1_p3)
*           2012/12/25 1.3  add variable snr mask
*           2014/05/26 1.4  support galileo and beidou
*           2015/03/19 1.5  fix bug on ionosphere correction for GLO and BDS
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

// add by weisong
#include <algorithm>
// google eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include<Eigen/Core>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
// google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include <novatel_msgs/BESTPOS.h> // novatel_msgs/INSPVAX

#include "../../include/gnss_tools.h"
#include <nlosExclusion/GNSS_Raw_Array.h>
#include <nlosExclusion/GNSS_Raw.h>

FILE* gnss_ublox_wls = fopen("gnss_ublox_wls.csv", "w+");


static const char rcsid[]="$Id:$";

/* constants -----------------------------------------------------------------*/

#define SQR(x)      ((x)*(x))

#define NX          (4+3)       /* # of estimated parameters */

#define MAXITR      10          /* max number of iteration for point pos */
#define ERR_ION     5.0         /* ionospheric delay std (m) */
#define ERR_TROP    3.0         /* tropspheric delay std (m) */
#define ERR_SAAS    0.3         /* saastamoinen model error std (m) */
#define ERR_BRDCI   0.5         /* broadcast iono model error factor */
#define ERR_CBIAS   0.3         /* code bias error std (m) */
#define REL_HUMI    0.7         /* relative humidity for saastamoinen model */

ros::Publisher pub_pntpos_odometry;
ros::Publisher pub_wls_odometry;

ros::Publisher pub_gnss_raw;
ros::Publisher pub_velocity_from_doppler;

GNSS_Tools m_GNSS_Tools; // utilities

extern void pntposRegisterPub(ros::NodeHandle &n)
{
    pub_pntpos_odometry = n.advertise<nav_msgs::Odometry>("WLSENURTKLIB", 1000);
    pub_gnss_raw = n.advertise<nlosExclusion::GNSS_Raw_Array>("GNSSPsrCarRov1", 1000);
    pub_wls_odometry = n.advertise<nav_msgs::Odometry>("WLSENUGoGPS", 1000);
    pub_velocity_from_doppler = n.advertise<nav_msgs::Odometry>("GNSSDopVelRov1", 1000); // velocity_from_doppler
}


/* pseudorange measurement error variance ------------------------------------*/
static double varerr(const prcopt_t *opt, double el, int sys)
{
    double fact,varr;
    fact=sys==SYS_GLO?EFACT_GLO:(sys==SYS_SBS?EFACT_SBS:EFACT_GPS);
    varr=SQR(opt->err[0])*(SQR(opt->err[1])+SQR(opt->err[2])/sin(el));
    if (opt->ionoopt==IONOOPT_IFLC) varr*=SQR(3.0); /* iono-free */
    return SQR(fact)*varr;
}
/* get tgd parameter (m) -----------------------------------------------------*/
static double gettgd(int sat, const nav_t *nav)
{
    int i;
    for (i=0;i<nav->n;i++) {
        if (nav->eph[i].sat!=sat) continue;
        return CLIGHT*nav->eph[i].tgd[0];
    }
    return 0.0;
}
/* psendorange with code bias correction -------------------------------------*/
static double prange(const obsd_t *obs, const nav_t *nav, const double *azel,
                     int iter, const prcopt_t *opt, double *var)
{
    const double *lam=nav->lam[obs->sat-1];
    double PC,P1,P2,P1_P2,P1_C1,P2_C2,gamma;
    int i=0,j=1,sys;
    
    *var=0.0;
    
    if (!(sys=satsys(obs->sat,NULL))) return 0.0;
    
    /* L1-L2 for GPS/GLO/QZS, L1-L5 for GAL/SBS */
    if (NFREQ>=3&&(sys&(SYS_GAL|SYS_SBS))) j=2;
    
    if (NFREQ<2||lam[i]==0.0||lam[j]==0.0) return 0.0;
    
    /* test snr mask */
    if (iter>0) {
        if (testsnr(0,i,azel[1],obs->SNR[i]*0.25,&opt->snrmask)) {
            trace(4,"snr mask: %s sat=%2d el=%.1f snr=%.1f\n",
                  time_str(obs->time,0),obs->sat,azel[1]*R2D,obs->SNR[i]*0.25);
            return 0.0;
        }
        if (opt->ionoopt==IONOOPT_IFLC) {
            if (testsnr(0,j,azel[1],obs->SNR[j]*0.25,&opt->snrmask)) return 0.0;
        }
    }
    gamma=SQR(lam[j])/SQR(lam[i]); /* f1^2/f2^2 */
    P1=obs->P[i];
    P2=obs->P[j];
    P1_P2=nav->cbias[obs->sat-1][0];
    P1_C1=nav->cbias[obs->sat-1][1];
    P2_C2=nav->cbias[obs->sat-1][2];
    // LOG(INFO)<<""
    
    /* if no P1-P2 DCB, use TGD instead */
    if (P1_P2==0.0&&(sys&(SYS_GPS|SYS_GAL|SYS_QZS))) {
        P1_P2=(1.0-gamma)*gettgd(obs->sat,nav);
    }
    if (opt->ionoopt==IONOOPT_IFLC) { /* dual-frequency */
        
        if (P1==0.0||P2==0.0) return 0.0;
        if (obs->code[i]==CODE_L1C) P1+=P1_C1; /* C1->P1 */
        if (obs->code[j]==CODE_L2C) P2+=P2_C2; /* C2->P2 */
        
        /* iono-free combination */
        PC=(gamma*P1-P2)/(gamma-1.0);
    }
    else { /* single-frequency */
        
        if (P1==0.0) return 0.0;
        if (obs->code[i]==CODE_L1C) P1+=P1_C1; /* C1->P1 */
        PC=P1-P1_P2/(1.0-gamma);
    }
    if (opt->sateph==EPHOPT_SBAS) PC-=P1_C1; /* sbas clock based C1 */
    
    *var=SQR(ERR_CBIAS);
    
    return PC;
}
/* ionospheric correction ------------------------------------------------------
* compute ionospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          int    sat       I   satellite number
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    ionoopt   I   ionospheric correction option (IONOOPT_???)
*          double *ion      O   ionospheric delay (L1) (m)
*          double *var      O   ionospheric delay (L1) variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int ionocorr(gtime_t time, const nav_t *nav, int sat, const double *pos,
                    const double *azel, int ionoopt, double *ion, double *var)
{
    trace(4,"ionocorr: time=%s opt=%d sat=%2d pos=%.3f %.3f azel=%.3f %.3f\n",
          time_str(time,3),ionoopt,sat,pos[0]*R2D,pos[1]*R2D,azel[0]*R2D,
          azel[1]*R2D);
    
    /* broadcast model */
    if (ionoopt==IONOOPT_BRDC) {
        *ion=ionmodel(time,nav->ion_gps,pos,azel);
        *var=SQR(*ion*ERR_BRDCI);
        return 1;
    }
    /* sbas ionosphere model */
    if (ionoopt==IONOOPT_SBAS) {
        return sbsioncorr(time,nav,pos,azel,ion,var);
    }
    /* ionex tec model */
    if (ionoopt==IONOOPT_TEC) {
        return iontec(time,nav,pos,azel,1,ion,var);
    }
    /* qzss broadcast model */
    if (ionoopt==IONOOPT_QZS&&norm(nav->ion_qzs,8)>0.0) {
        *ion=ionmodel(time,nav->ion_qzs,pos,azel);
        *var=SQR(*ion*ERR_BRDCI);
        return 1;
    }
    /* lex ionosphere model */
    if (ionoopt==IONOOPT_LEX) {
        return lexioncorr(time,nav,pos,azel,ion,var);
    }
    *ion=0.0;
    *var=ionoopt==IONOOPT_OFF?SQR(ERR_ION):0.0;
    return 1;
}
/* tropospheric correction -----------------------------------------------------
* compute tropospheric correction
* args   : gtime_t time     I   time
*          nav_t  *nav      I   navigation data
*          double *pos      I   receiver position {lat,lon,h} (rad|m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          int    tropopt   I   tropospheric correction option (TROPOPT_???)
*          double *trp      O   tropospheric delay (m)
*          double *var      O   tropospheric delay variance (m^2)
* return : status(1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int tropcorr(gtime_t time, const nav_t *nav, const double *pos,
                    const double *azel, int tropopt, double *trp, double *var)
{
    trace(4,"tropcorr: time=%s opt=%d pos=%.3f %.3f azel=%.3f %.3f\n",
          time_str(time,3),tropopt,pos[0]*R2D,pos[1]*R2D,azel[0]*R2D,
          azel[1]*R2D);
    
    /* saastamoinen model */
    if (tropopt==TROPOPT_SAAS||tropopt==TROPOPT_EST||tropopt==TROPOPT_ESTG) {
        *trp=tropmodel(time,pos,azel,REL_HUMI);
        *var=SQR(ERR_SAAS/(sin(azel[1])+0.1));
        return 1;
    }
    /* sbas troposphere model */
    if (tropopt==TROPOPT_SBAS) {
        *trp=sbstropcorr(time,pos,azel,var);
        return 1;
    }
    /* no correction */
    *trp=0.0;
    *var=tropopt==TROPOPT_OFF?SQR(ERR_TROP):0.0;
    return 1;
}
/* pseudorange residuals -----------------------------------------------------*/
static int rescode(int iter, const obsd_t *obs, int n, const double *rs,
                   const double *dts, const double *vare, const int *svh,
                   const nav_t *nav, const double *x, const prcopt_t *opt,
                   double *v, double *H, double *var, double *azel, int *vsat,
                   double *resp, int *ns)
{
    double r,dion,dtrp,vmeas,vion,vtrp,rr[3],pos[3],dtr,e[3],P,lam_L1;
    int i,j,nv=0,sys,mask[4]={0};
    
    trace(3,"resprng : n=%d\n",n);
    
    for (i=0;i<3;i++) rr[i]=x[i]; dtr=x[3];
    
    ecef2pos(rr,pos);
    
    for (i=*ns=0;i<n&&i<MAXOBS;i++) {
        vsat[i]=0; azel[i*2]=azel[1+i*2]=resp[i]=0.0;
        
        if (!(sys=satsys(obs[i].sat,NULL))) continue;
        
        
        /* reject duplicated observation data */
        if (i<n-1&&i<MAXOBS-1&&obs[i].sat==obs[i+1].sat) {
            trace(2,"duplicated observation data %s sat=%2d\n",
                  time_str(obs[i].time,3),obs[i].sat);
            i++;
            LOG(INFO) <<"duplicated observation data!!!!!!!!!!";
            continue;
        }
        /* geometric distance/azimuth/elevation angle */
        if ((r=geodist(rs+i*6,rr,e))<=0.0||
            satazel(pos,e,azel+i*2)<opt->elmin) continue;
        
        /* psudorange with code bias correction */
        if ((P=prange(obs+i,nav,azel+i*2,iter,opt,&vmeas))==0.0) continue;
        // P = obs[i].P[0];

        // printf("original pseudorange P0 %f \n", obs[i].P[0]);
        // printf("original pseudorange P1 %f \n", obs[i].P[1]);
        // printf("original pseudorange P2 %f \n", obs[i].P[2]);
        // printf("corrected pseudorange P %f \n", P);
        
        /* excluded satellite? */
        if (satexclude(obs[i].sat,svh[i],opt)) 
        {
            LOG(INFO) <<"exclude satellite!!!!!!!!!!";
            continue;
        }
        
        
        /* ionospheric corrections 
        * ionospheric correction only do once (iter=0) 
        */
        if (!ionocorr(obs[i].time,nav,obs[i].sat,pos,azel+i*2,
                      iter>0?opt->ionoopt:IONOOPT_BRDC,&dion,&vion)) continue;
        
        /* GPS-L1 -> L1/B1 */
        if ((lam_L1=nav->lam[obs[i].sat-1][0])>0.0) {
            dion*=SQR(lam_L1/lam_carr[0]);
        }
        /* tropospheric corrections 
        *  tropospheric delay is correlated with the pose of receiver,
        * therefore, the delay is estimated again based on newly estimated 
        * pose of receievr.
        */
        if (!tropcorr(obs[i].time,nav,pos,azel+i*2,
                      iter>0?opt->tropopt:TROPOPT_SAAS,&dtrp,&vtrp)) {
            continue;
        }
        /* pseudorange residual 
        * dtr receiver clock bias 
        * dts satellite clock bias
        * dion estimated ionospheric delay 
        * dtrp estimated dtrp delay
        */
        v[nv]=P-(r+dtr-CLIGHT*dts[i*2]+dion+dtrp);

        // printf("dtr %f \n", dtr );
        // printf("dts %f \n", dts );
        // printf("dion %f \n", dion);
        // printf("dtrp %f \n", dtrp);
        
        /* design matrix */
        for (j=0;j<NX;j++) H[j+nv*NX]=j<3?-e[j]:(j==3?1.0:0.0);
        
        /* time system and receiver bias offset correction */
        if      (sys==SYS_GLO) {v[nv]-=x[4]; H[4+nv*NX]=1.0; mask[1]=1;}
        else if (sys==SYS_GAL) {v[nv]-=x[5]; H[5+nv*NX]=1.0; mask[2]=1;}
        else if (sys==SYS_CMP) {v[nv]-=x[6]; H[6+nv*NX]=1.0; mask[3]=1;}
        else mask[0]=1;
        
        vsat[i]=1; resp[i]=v[nv]; (*ns)++;
        
        /* error variance */
        var[nv++]=varerr(opt,azel[1+i*2],sys)+vare[i]+vmeas+vion+vtrp;
        
        trace(4,"sat=%2d azel=%5.1f %4.1f res=%7.3f sig=%5.3f\n",obs[i].sat,
              azel[i*2]*R2D,azel[1+i*2]*R2D,resp[i],sqrt(var[nv-1]));
    }
    /* constraint to avoid rank-deficient */
    for (i=0;i<4;i++) {
        if (mask[i]) continue;
        v[nv]=0.0;
        for (j=0;j<NX;j++) H[j+nv*NX]=j==i+3?1.0:0.0;
        var[nv++]=0.01;
    }
    return nv;
}
/* validate solution ---------------------------------------------------------*/
static int valsol(const double *azel, const int *vsat, int n,
                  const prcopt_t *opt, const double *v, int nv, int nx,
                  char *msg)
{
    double azels[MAXOBS*2],dop[4],vv;
    int i,ns;
    
    trace(3,"valsol  : n=%d nv=%d\n",n,nv);
    
    /* chi-square validation of residuals */
    vv=dot(v,v,nv);
    if (nv>nx&&vv>chisqr[nv-nx-1]) {
        sprintf(msg,"chi-square error nv=%d vv=%.1f cs=%.1f",nv,vv,chisqr[nv-nx-1]);
        return 0;
    }
    /* large gdop check */
    for (i=ns=0;i<n;i++) {
        if (!vsat[i]) continue;
        azels[  ns*2]=azel[  i*2];
        azels[1+ns*2]=azel[1+i*2];
        ns++;
    }
    dops(ns,azels,opt->elmin,dop);
    if (dop[0]<=0.0||dop[0]>opt->maxgdop) {
        sprintf(msg,"gdop error nv=%d gdop=%.1f",nv,dop[0]);
        return 0;
    }
    return 1;
}
/* estimate receiver position ------------------------------------------------*/
static int estpos(const obsd_t *obs, int n, const double *rs, const double *dts,
                  const double *vare, const int *svh, const nav_t *nav,
                  const prcopt_t *opt, sol_t *sol, double *azel, int *vsat,
                  double *resp, char *msg)
{
    double x[NX]={0},dx[NX],Q[NX*NX],*v,*H,*var,sig;
    int i,j,k,info,stat,nv,ns;
    
    trace(3,"estpos  : n=%d\n",n);
    
    v=mat(n+4,1); H=mat(NX,n+4); var=mat(n+4,1);
    
    for (i=0;i<3;i++) x[i]=sol->rr[i];
    
    for (i=0;i<MAXITR;i++) { // iterations for WLS
        
        /* pseudorange residuals */
        nv=rescode(i,obs,n,rs,dts,vare,svh,nav,x,opt,v,H,var,azel,vsat,resp,
                   &ns);
        
        if (nv<NX) {
            sprintf(msg,"lack of valid sats ns=%d",nv);
            break;
        }
        /* weight by variance */
        for (j=0;j<nv;j++) {
            sig=sqrt(var[j]);
            v[j]/=sig;
            for (k=0;k<NX;k++) H[k+j*NX]/=sig;
        }
        /* least square estimation */
        if ((info=lsq(H,v,NX,nv,dx,Q))) {
            sprintf(msg,"lsq error info=%d",info);
            break;
        }
        for (j=0;j<NX;j++) x[j]+=dx[j]; // state increment
        
        if (norm(dx,NX)<1E-4) {
            sol->type=0;
            sol->time=timeadd(obs[0].time,-x[3]/CLIGHT);
            sol->dtr[0]=x[3]/CLIGHT; /* receiver clock bias (s) */
            sol->dtr[1]=x[4]/CLIGHT; /* glo-gps time offset (s) */
            sol->dtr[2]=x[5]/CLIGHT; /* gal-gps time offset (s) */
            sol->dtr[3]=x[6]/CLIGHT; /* bds-gps time offset (s) */
            for (j=0;j<6;j++) sol->rr[j]=j<3?x[j]:0.0;
            for (j=0;j<3;j++) sol->qr[j]=(float)Q[j+j*NX];
            sol->qr[3]=(float)Q[1];    /* cov xy */
            sol->qr[4]=(float)Q[2+NX]; /* cov yz */
            sol->qr[5]=(float)Q[2];    /* cov zx */
            sol->ns=(unsigned char)ns;
            sol->age=sol->ratio=0.0;
            
            /* validate solution */
            if ((stat=valsol(azel,vsat,n,opt,v,nv,NX,msg))) {
                sol->stat=opt->sateph==EPHOPT_SBAS?SOLQ_SBAS:SOLQ_SINGLE;
            }
            // stat = 1; // add by weisong
            free(v); free(H); free(var);
            
            return stat;
        }
    }
    if (i>=MAXITR) sprintf(msg,"iteration divergent i=%d",i);
    
    free(v); free(H); free(var);
    
    return 0;
}
/* raim fde (failure detection and exclution) -------------------------------*/
static int raim_fde(const obsd_t *obs, int n, const double *rs,
                    const double *dts, const double *vare, const int *svh,
                    const nav_t *nav, const prcopt_t *opt, sol_t *sol,
                    double *azel, int *vsat, double *resp, char *msg)
{
    obsd_t *obs_e;
    sol_t sol_e={{0}};
    char tstr[32],name[16],msg_e[128];
    double *rs_e,*dts_e,*vare_e,*azel_e,*resp_e,rms_e,rms=100.0;
    int i,j,k,nvsat,stat=0,*svh_e,*vsat_e,sat=0;
    
    trace(3,"raim_fde: %s n=%2d\n",time_str(obs[0].time,0),n);
    
    if (!(obs_e=(obsd_t *)malloc(sizeof(obsd_t)*n))) return 0;
    rs_e = mat(6,n); dts_e = mat(2,n); vare_e=mat(1,n); azel_e=zeros(2,n);
    svh_e=imat(1,n); vsat_e=imat(1,n); resp_e=mat(1,n); 
    
    for (i=0;i<n;i++) {
        
        /* satellite exclution */
        for (j=k=0;j<n;j++) {
            if (j==i) continue;
            obs_e[k]=obs[j];
            matcpy(rs_e +6*k,rs +6*j,6,1);
            matcpy(dts_e+2*k,dts+2*j,2,1);
            vare_e[k]=vare[j];
            svh_e[k++]=svh[j];
        }
        /* estimate receiver position without a satellite */
        if (!estpos(obs_e,n-1,rs_e,dts_e,vare_e,svh_e,nav,opt,&sol_e,azel_e,
                    vsat_e,resp_e,msg_e)) {
            trace(3,"raim_fde: exsat=%2d (%s)\n",obs[i].sat,msg);
            continue;
        }
        for (j=nvsat=0,rms_e=0.0;j<n-1;j++) {
            if (!vsat_e[j]) continue;
            rms_e+=SQR(resp_e[j]);
            nvsat++;
        }
        if (nvsat<5) {
            trace(3,"raim_fde: exsat=%2d lack of satellites nvsat=%2d\n",
                  obs[i].sat,nvsat);
            continue;
        }
        rms_e=sqrt(rms_e/nvsat);
        
        trace(3,"raim_fde: exsat=%2d rms=%8.3f\n",obs[i].sat,rms_e);
        
        if (rms_e>rms) continue;
        
        /* save result */
        for (j=k=0;j<n;j++) {
            if (j==i) continue;
            matcpy(azel+2*j,azel_e+2*k,2,1);
            vsat[j]=vsat_e[k];
            resp[j]=resp_e[k++];
        }
        stat=1;
        *sol=sol_e;
        sat=obs[i].sat;
        rms=rms_e;
        vsat[i]=0;
        strcpy(msg,msg_e);
    }
    if (stat) {
        time2str(obs[0].time,tstr,2); satno2id(sat,name);
        trace(2,"%s: %s excluded by raim\n",tstr+11,name);
    }
    free(obs_e);
    free(rs_e ); free(dts_e ); free(vare_e); free(azel_e);
    free(svh_e); free(vsat_e); free(resp_e);
    return stat;
}
/* doppler residuals ---------------------------------------------------------*/
static int resdop(const obsd_t *obs, int n, const double *rs, const double *dts,
                  const nav_t *nav, const double *rr, const double *x,
                  const double *azel, const int *vsat, double *v, double *H)
{
    double lam,rate,pos[3],E[9],a[3],e[3],vs[3],cosel;
    int i,j,nv=0;
    
    trace(3,"resdop  : n=%d\n",n);
    
    ecef2pos(rr,pos); xyz2enu(pos,E);
    
    for (i=0;i<n&&i<MAXOBS;i++) {
        
        lam=nav->lam[obs[i].sat-1][0];
        
        if (obs[i].D[0]==0.0||lam==0.0||!vsat[i]||norm(rs+3+i*6,3)<=0.0) {
        // if (obs[i].D[0]==0.0||lam==0.0||!vsat[i]||norm(rs+3+i*6,3)<=0.0 || obs[i].SNR[0] * 0.25]<25) {
            continue;
        }
        /* line-of-sight vector in ecef */
        cosel=cos(azel[1+i*2]);
        a[0]=sin(azel[i*2])*cosel;
        a[1]=cos(azel[i*2])*cosel;
        a[2]=sin(azel[1+i*2]);
        matmul("TN",3,1,3,1.0,E,a,0.0,e);
        
        /* satellite velocity relative to receiver in ecef */
        for (j=0;j<3;j++) vs[j]=rs[j+3+i*6]-x[j];
        
        /* range rate with earth rotation correction */
        rate=dot(vs,e,3)+OMGE/CLIGHT*(rs[4+i*6]*rr[0]+rs[1+i*6]*x[0]-
                                      rs[3+i*6]*rr[1]-rs[  i*6]*x[1]);
        
        /* doppler residual */
        // v[nv]=-lam*obs[i].D[0]-(rate+x[3]-CLIGHT*dts[1+i*2]);
        v[nv]=lam*obs[i].D[0]-(rate+x[3]-CLIGHT*dts[1+i*2]);  /* for 2022 SH & Xi'an data */
        
        /* design matrix */
        for (j=0;j<4;j++) H[j+nv*4]=j<3?-e[j]:1.0;
        
        nv++;
    }
    return nv;
}
/* estimate receiver velocity ------------------------------------------------*/
static void estvel(const obsd_t *obs, int n, const double *rs, const double *dts,
                   const nav_t *nav, const prcopt_t *opt, sol_t *sol,
                   const double *azel, const int *vsat, double *dop_res)
{
    double x[4]={0},dx[4],Q[16],*v,*H;
    int i,j,nv;
    
    trace(3,"estvel  : n=%d\n",n);
    
    v=mat(n,1); H=mat(4,n);
    
    for (i=0;i<MAXITR;i++) {
        
        /* doppler residuals */
        if ((nv=resdop(obs,n,rs,dts,nav,sol->rr,x,azel,vsat,v,H))<4) {
            break;
        }
        /* least square estimation */
        if (lsq(H,v,4,nv,dx,Q)) break;
        
        for (j=0;j<4;j++) x[j]+=dx[j];
        
        if (norm(dx,4)<1E-6) {
            for (i=0;i<3;i++) sol->rr[i+3]=x[i];
            break;
        }
    }
    dop_res = v;
    free(v); free(H);
}
/* single-point positioning ----------------------------------------------------
* compute receiver position, velocity, clock bias by single-point positioning
* with pseudorange and doppler observables
* args   : obsd_t *obs      I   observation data
*          int    n         I   number of observation data
*          nav_t  *nav      I   navigation data
*          prcopt_t *opt    I   processing options
*          sol_t  *sol      IO  solution
*          double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)
*          ssat_t *ssat     IO  satellite status              (NULL: no output)
*          char   *msg      O   error message for error exit
* return : status(1:ok,0:error)
* notes  : assuming sbas-gps, galileo-gps, qzss-gps, compass-gps time offset and
*          receiver bias are negligible (only involving glonass-gps time offset
*          and receiver bias)
*-----------------------------------------------------------------------------*/
extern int pntpos(const obsd_t *obs, int n, const nav_t *nav,
                  const prcopt_t *opt, sol_t *sol, double *azel, ssat_t *ssat,
                  char *msg)
{
    prcopt_t opt_=*opt;
    double *rs,*dts,*var,*azel_,*resp;
    int i,stat,vsat[MAXOBS]={0},svh[MAXOBS];
    
    sol->stat=SOLQ_NONE;
    
    if (n<=0) {strcpy(msg,"no observation data"); return 0;}
    
    trace(3,"pntpos  : tobs=%s n=%d\n",time_str(obs[0].time,3),n);
    
    sol->time=obs[0].time; msg[0]='\0';
    
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n); azel_=zeros(2,n); resp=mat(1,n);
    
    if (opt_.mode!=PMODE_SINGLE) { /* for precise positioning */
#if 0
        opt_.sateph =EPHOPT_BRDC;
#endif
        opt_.ionoopt=IONOOPT_BRDC;
        opt_.tropopt=TROPOPT_SAAS;
    }

    /* construct data for WLS with nlosExclusion::GNSS_Raw_Array*/
    nlosExclusion::GNSS_Raw_Array gnss_data;
    int current_week = 0;
    double current_tow = time2gpst(obs[0].time, &current_week);
    double epoch_time[100];
    time2epoch(obs[0].time, epoch_time);

    /* satellite positons, velocities and clocks */
    satposs(sol->time,obs,n,nav,opt_.sateph,rs,dts,var,svh);
    
    /* estimate receiver position with pseudorange by RTKLIB */
    stat=estpos(obs,n,rs,dts,var,svh,nav,&opt_,sol,azel_,vsat,resp,msg);

    // satazel();

    /* estimate receiver position with pseudorange by WLS and Eigen */
    bool haveOneBeiDou = false;
    int CMP_cnt = 0, GPS_cnt = 0;
    for(int s_i=0;s_i<n; s_i++)
    {
        nlosExclusion::GNSS_Raw gnss_raw;
        gnss_raw.GNSS_time = current_tow;
        gnss_raw.prE3dMA = current_week;
        gnss_raw.total_sv = float(n);
        gnss_raw.prn_satellites_index = float(obs[s_i].sat);
        // std::cout<<"sat prn -> " << float(obs[s_i].sat) << std::endl;
        
        /* get snr*/
        double snr = 0;
        for (int j=0,snr=0.0;j<NFREQ;j++) if ((snr=obs[s_i].SNR[j])!=0.0) break;
        gnss_raw.snr = obs[s_i].SNR[0] * 0.25;
        // gnss_raw.snr = snr;
        
        #if 0 // reculate the elevation and azimuth angles
        double rsTmp[3] = {rs[0 + s_i * 6], rs[1 + s_i * 6],rs[2 + s_i * 6]}; //sate pose
        double rrTmp[3] = {sol->rr[0],sol->rr[1],sol->rr[2]};
        double eTmp[3]={0};
        double rTmp = geodist(rsTmp,rrTmp,eTmp);
        double posTmp[3] = {0};
        double azelTmp[3] = {0};
        ecef2pos(rrTmp,posTmp);
        satazel(posTmp, eTmp,azelTmp);
        LOG(INFO)<<"azel->--------------------- "<<azelTmp[0] * R2D;
        LOG(INFO)<<"azel->-------------------- "<<azelTmp[1] * R2D;
        #endif

        gnss_raw.azimuth = azel_[0 + s_i*2] * R2D;
        gnss_raw.elevation = azel_[1 + s_i * 2] * R2D;
        
        double dion,dtrp,vmeas,vion,vtrp,rr[3],pos[3],e[3],P,lam_L1;
        /* psudorange with code bias correction */
        if ((P=prange(obs+s_i,nav,azel_+s_i*2,2,&opt_,&vmeas))==0.0) continue;

        /* ionospheric corrections */
        for (int i=0;i<3;i++) rr[i]=sol->rr[i];
        ecef2pos(rr,pos);
        if (!ionocorr(obs[s_i].time,nav,obs[s_i].sat,pos,azel_+s_i*2,
                      opt->ionoopt,&dion,&vion)) continue;
        // printf("dion-> WLS %f \n", dion);
        // printf("dtrp %f \n", dtrp);

        /* GPS-L1 -> L1/B1 */
        if ((lam_L1=nav->lam[obs[s_i].sat-1][0])>0.0) {
            dion*=SQR(lam_L1/lam_carr[0]);
        }

        /* tropospheric corrections 
        * pose of receievr.
        */
        if (!tropcorr(obs[s_i].time,nav,pos,azel_+s_i*2,
                      opt->tropopt,&dtrp,&vtrp)) {
            continue;
        }
        // printf("dtrp %f \n", dtrp);
        gnss_raw.err_tropo = dtrp;
        gnss_raw.err_iono = dion;

        gnss_raw.sat_clk_err = dts[0+ s_i * 2] * CLIGHT;
        gnss_raw.ttx = 0;
        gnss_raw.dt = dts[0+ s_i * 2] * CLIGHT;
        gnss_raw.ddt = dts[1+ s_i * 2] * CLIGHT;
        gnss_raw.tgd = 0;
        gnss_raw.sat_pos_x = rs[0 + s_i * 6];
        gnss_raw.sat_pos_y = rs[1 + s_i * 6];
        gnss_raw.sat_pos_z = rs[2 + s_i * 6];
        gnss_raw.vel_x = rs[3 + s_i * 6];
        gnss_raw.vel_y = rs[4 + s_i * 6];
        gnss_raw.vel_z = rs[5 + s_i * 6];

        /* get pr*/
        double pr = 0;
        for (int j=0;j<NFREQ;j++) if ((pr=obs[s_i].P[j])!=0.0) break;
        gnss_raw.pseudorange = obs[s_i].P[0];
        // gnss_raw.pseudorange = pr;
        // gnss_raw.pseudorange = gnss_raw.pseudorange + gnss_raw.sat_clk_err - dion - dtrp;
        /* remove the satellite clock bias, atmosphere error here */
        // gnss_raw.pseudorange = P + gnss_raw.sat_clk_err - dion - dtrp;
        
        /* if the satellite clock bias is already removed */
        gnss_raw.pseudorange = P + 0 - dion - dtrp;


        // gnss_raw.raw_pseudorange = obs[s_i].P[0];
        gnss_raw.raw_pseudorange = 1; // for GNC, this is used to save the weighting
        gnss_raw.carrier_phase = obs[s_i].L[0];
        
        /* remove the satellite clock error/atmosphere errors */
        // gnss_raw.carrier_phase = gnss_raw.carrier_phase + gnss_raw.sat_clk_err - dion + dtrp;

        gnss_raw.doppler = 1 * obs[s_i].D[0];

        gnss_raw.LLI = obs[s_i].LLI[0];
        //simple detection based on LLI, we can add other logic to detect CS
        gnss_raw.slip = (obs[s_i].LLI[0] & 3) > 0 ? 1 : 0;

        gnss_raw.lamda = nav->lam[obs[s_i].sat-1][0];
        // LOG(INFO) << "nav->lam[obs[s_i].sat-1][0];  "<<nav->lam[obs[s_i].sat-1][0];
        // LOG(INFO) << "nav->lam[obs[s_i].sat][0];  "<<nav->lam[obs[s_i].sat][0];
        int sys=satsys(obs[s_i].sat,NULL);   
        if(gnss_raw.elevation>15 && gnss_raw.snr>25) // 15 25 30 
        {
            gnss_data.GNSS_Raws.push_back(gnss_raw); 

            if(sys==SYS_GPS)
            {
                // LOG(INFO) << "GPS Satellite  "<<current_tow;
                GPS_cnt++;
            }
            else if(sys==SYS_CMP)
            {
                // LOG(INFO) << "BeiDou Satellite   "<<current_tow;
                haveOneBeiDou = true;
                CMP_cnt++;
            }
            else
            {
                LOG(INFO) << "Unknow!!!!! Satellite   "<<current_tow;
            }

            #if 0
            std::cout << "epoch_time-> "<< epoch_time[0]<<"/"<<epoch_time[1]<<"/"<<epoch_time[2]<< " "<<epoch_time[3]<<":"<<epoch_time[4]<<":"<<epoch_time[5]<<std::endl;
            LOG(INFO) << "obs[s_i].P[0];  "<<obs[s_i].P[0];
            LOG(INFO) << "obs[s_i].L[0];  "<<obs[s_i].L[0];
            #endif
        }
        else
        {
            #if 1
            // LOG(INFO)<<"The elevation angle is less than 15!!!!!" << gnss_raw.elevation;
            // LOG(INFO) << "obs[s_i].P[0];  "<<obs[s_i].P[0];;
            // LOG(INFO) << "obs[s_i].L[0];  "<<obs[s_i].L[0];;
            #endif
        }
        
        #if 0 // debug satellite information
        // LOG(INFO) << "obs[s_i].sat  "<<float(obs[s_i].sat);
        if(sys==SYS_GPS)
        {
            // LOG(INFO) << "GPS Satellite  "<<current_tow;
            GPS_cnt++;
        }
        else if(sys==SYS_CMP)
        {
            // LOG(INFO) << "BeiDou Satellite   "<<current_tow;
            haveOneBeiDou = true;
            CMP_cnt++;
        }
        else
        {
            LOG(INFO) << "Unknow!!!!! Satellite   "<<current_tow;
        }
        #endif
        
    }

    LOG(INFO) << "GPS_cnt   "<<GPS_cnt;
    LOG(INFO) << "CMP_cnt   "<<CMP_cnt;
    
    // pub_gnss_raw.publish(gnss_data);
    
    nlosExclusion::GNSS_Raw_Array gnss_dataTmp = gnss_data;
    #if 1 // PNT from WLS using Eigen
    {
        Eigen::Matrix<double, 3,1> ENU_ref;
        // ENU_ref<< 114.190297420,22.301487386,0;
        ENU_ref<< ref_lon, ref_lat, ref_alt;
        Eigen::Matrix<double, 3, 1> ENU;
        Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare(
                                            m_GNSS_Tools.getAllPositions(gnss_data),
                                            m_GNSS_Tools.getAllMeasurements(gnss_data),
                                            gnss_data, "WLS");
        
        
        
        double dop = m_GNSS_Tools.getDop(m_GNSS_Tools.getAllPositions(gnss_dataTmp),
                                            m_GNSS_Tools.getAllMeasurements(gnss_dataTmp),
                                            gnss_dataTmp, "LS");
        double miniSnr = 100;
        while(dop<3 && gnss_dataTmp.GNSS_Raws.size()>6)
        {
            miniSnr = m_GNSS_Tools.removeLowSnrSat(gnss_dataTmp);
            dop = m_GNSS_Tools.getDop(m_GNSS_Tools.getAllPositions(gnss_dataTmp),
                                            m_GNSS_Tools.getAllMeasurements(gnss_dataTmp),
                                            gnss_dataTmp, "LS");
        }

        // Eigen::MatrixXd eWLSSolutionECEF = m_GNSS_Tools.WeightedLeastSquare_GPS(
        //                                     m_GNSS_Tools.getAllPositions(gnss_data),
        //                                     m_GNSS_Tools.getAllMeasurements(gnss_data),
        //                                     gnss_data);
        ENU = m_GNSS_Tools.ecef2enu(ENU_ref, eWLSSolutionECEF);
        // std::cout << "eWLSSolutionECEF (wls)-> "<< eWLSSolutionECEF << std::endl;
        std::cout << "dop---------------------------> "<< dop << std::endl;
        std::cout << "miniSnr---------------------------> "<< miniSnr << std::endl;
        std::cout << "gnss_dataTmp.GNSS_Raws.size()---------------------------> "<< gnss_dataTmp.GNSS_Raws.size() << std::endl;
        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "map";
        odometry.child_frame_id = "map";
        odometry.pose.pose.position.x = ENU(0);
        odometry.pose.pose.position.y = ENU(1);
        odometry.pose.pose.position.z = 1;
        pub_wls_odometry.publish(odometry);
    }

    // pub_gnss_raw.publish(gnss_dataTmp);
    pub_gnss_raw.publish(gnss_data);
    #endif
    
    /* raim fde */
    if (!stat&&n>=6&&opt->posopt[4]) { // disabled
        stat=raim_fde(obs,n,rs,dts,var,svh,nav,&opt_,sol,azel_,vsat,resp,msg);
        std::cout<<"Fault Detection and Exclusion\n";
    }

    #if 1 /* estimate receiver velocity with doppler */
    double *dop_res;
    dop_res=mat(n,1); 
    
    /* estimate solution only when the pnt validation is successful */
    if (stat) 
    {
        estvel(obs,n,rs,dts,nav,&opt_,sol,azel_,vsat, dop_res);
    }
    else 
    {
        // std::cout<<" the doppler velocity is not estimated due to failure of RAIM"<<std::endl;
        estvel(obs,n,rs,dts,nav,&opt_,sol,azel_,vsat, dop_res);
    }
    // if (1) estvel(obs,n,rs,dts,nav,&opt_,sol,azel_,vsat, dop_res);
    // std::cout << "norm(dx,n)-> "<<norm(dop_res,n)<<std::endl;
    
    nav_msgs::Odometry odometry;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "map";
    odometry.pose.pose.position.x = current_tow;
    odometry.twist.twist.linear.x = sol->rr[3];
    odometry.twist.twist.linear.y = sol->rr[4];
    odometry.twist.twist.linear.z = sol->rr[5];
    odometry.twist.covariance[0] = norm(dop_res,n);

    if(1) // use the snr and ele to model the uncertainty of doppler
    {
        Eigen::MatrixXd covarianceMatrix = m_GNSS_Tools.getCovarianceMatrix(
                                            m_GNSS_Tools.getAllPositions(gnss_data),
                                            m_GNSS_Tools.getAllMeasurements(gnss_data),
                                            gnss_data, "WLS");
        // std::cout << "covarianceMatrix-> \n"<<covarianceMatrix << std::endl;
        odometry.twist.covariance[0] = covarianceMatrix(0,0);
        odometry.twist.covariance[1] = covarianceMatrix(1,1);
        odometry.twist.covariance[2] = covarianceMatrix(2,2);

        // odometry.twist.covariance[0] = sqrt(covarianceMatrix(0,0));
        // odometry.twist.covariance[1] = sqrt(covarianceMatrix(1,1));
        // odometry.twist.covariance[2] = sqrt(covarianceMatrix(2,2));
    }
    
    pub_velocity_from_doppler.publish(odometry);
    #endif


    #if 1
    /* Weisong: publish the solution
    *  no matter the solution is good or not
     */
    if(1) // from RTKLIB PNT
    {
        Eigen::Matrix<double, 3,1> ENU_ref;
        // ENU_ref<< 114.190297420,22.301487386,0;
        ENU_ref<< ref_lon, ref_lat, ref_alt;
        Eigen::Matrix<double, 3, 1> ENU;
        Eigen::Matrix<double, 3, 1> ECEF;
        ECEF<<sol->rr[0], sol->rr[1], sol->rr[2];
        ENU = m_GNSS_Tools.ecef2enu(ENU_ref, ECEF);
        // std::cout << "ENU (RTKLIB)-> "<< ENU << std::endl;
        nav_msgs::Odometry odometry;
        odometry.header.frame_id = "map";
        odometry.child_frame_id = "map";
        odometry.pose.pose.position.x = ENU(0);
        odometry.pose.pose.position.y = ENU(1);
        odometry.pose.pose.position.z = 1;
        for(int i = 0; i < 6; i++)
        {
            // odometry.pose.covariance[i] = sol->qr[i];
        }

        odometry.twist.twist.linear.x = sol->rr[3];
        odometry.twist.twist.linear.y = sol->rr[4];
        odometry.twist.twist.linear.z = sol->rr[5];
        pub_pntpos_odometry.publish(odometry);

        if((int(current_tow) > start_gps_sec) && (int(current_tow) < end_gps_sec))
        {
            double pos[3];
            double rr[3]={sol->rr[0], sol->rr[1], sol->rr[2]};
            ecef2pos(sol->rr,pos);
            fprintf(gnss_ublox_wls, "%d,%d,%7.9f,%7.9f,%7.9f,", 2096, int(current_tow),pos[0]*R2D,pos[1]*R2D,pos[2]);
            fprintf(gnss_ublox_wls, "%7.9f,%7.9f,%7.9f \n", sol->qr[0], sol->qr[1],sol->qr[2]);
            fflush(gnss_ublox_wls);
        }
        

    }
    #endif
    
    if (azel) {
        for (i=0;i<n*2;i++) azel[i]=azel_[i];
    }
    if (ssat) {
        for (i=0;i<MAXSAT;i++) {
            ssat[i].vs=0;
            ssat[i].azel[0]=ssat[i].azel[1]=0.0;
            ssat[i].resp[0]=ssat[i].resc[0]=0.0;
            ssat[i].snr[0]=0;
        }
        for (i=0;i<n;i++) {
            ssat[obs[i].sat-1].azel[0]=azel_[  i*2];
            ssat[obs[i].sat-1].azel[1]=azel_[1+i*2];
            ssat[obs[i].sat-1].snr[0]=obs[i].SNR[0];
            if (!vsat[i]) continue;
            ssat[obs[i].sat-1].vs=1;
            ssat[obs[i].sat-1].resp[0]=resp[i];
        }
    }
    free(rs); free(dts); free(var); free(azel_); free(resp);
    return stat;
}
