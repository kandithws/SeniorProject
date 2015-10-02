/*
  * VisualOdometry.h
  *
  * Created on: Jan , 2014
  * Author: Kandith
  * 
  *
  */ 

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <functional>  
#include <iostream>
#include <ctype.h>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include <exception>
#define MAX_COUNT 500
#define PI 3.14159265

#define PRINT_DISABLE 0x00
#define PRINT_ENABLE 0x01

#define DRAW_DISABLE 0x00
#define DRAW_ENABLE 0x10

#define DEGREE true
#define RADIAN false

#define COV_CALC_FEATUREPTS 12

class VisualOdometry{

private:
    /*Private variables*/
    cv::Mat rgbFrames, grayFrames, prevGrayFrame;
    cv::Mat H,R,t,n;

    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    std::vector<cv::Point2f> ransac_ptsold;
    std::vector<cv::Point2f> ransac_ptsnew;
    std::vector<cv::Point2f> drawpts1;
    std::vector<cv::Point2f> drawpts2;

    std::vector<uchar> status;
    std::vector<float> err;

    
    bool needToInit;
    cv::TermCriteria termcrit;
    cv::Size winSize;
    bool result;
   

    bool decomp_check;
    unsigned char printflag;
    unsigned char drawableflag;
    //Homodecomposer

    cv::Mat A;
    cv::Mat Ainv;
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat n1;
    cv::Mat n2;
    cv::Mat t1;
    cv::Mat t2;
    cv::SVD svd;
    cv::Mat U;
    cv::Mat V;
    cv::Mat D;
    /*correspond alpha,beta,delta,s to correct ans*/
    double c_alpha;
    double c_beta;
    double c_delta;
    double c_s;
    double c_omega;

    cv::SVD dsvd;
    cv::Mat mat_1;
    cv::Mat mat_2;
    cv::Mat Hc;
    cv::Mat eye9; 
    //Surf Matcher
    std::vector<cv::KeyPoint> keypoints_new;
    std::vector<cv::KeyPoint> keypoints_prev;
    //cv::FlannBasedMatcher matcher;
    cv::BFMatcher matcher;
    std::vector< cv::DMatch > matches;
    std::vector< cv::DMatch > good_matches;
    cv::Mat descriptors_new;
    cv::Mat descriptors_prev;
    
    //HomoDecomposer hmdecom;
    /*Abot CovMat*/

    cv::Mat cov_matchedfeature;
    //cv::Mat ransac_ptsnew;
    //cv::Mat ransac_ptsold;
    /*Private Function*/

    bool iszero(cv::Point2f x);
    double homography_calcdenormalizedfactor(cv::Mat Hp);
    bool homodecomp_compute(cv::Mat H,cv::Mat& R,cv::Mat& t,cv::Mat& n);

    bool estimate_homographycovariancematrix(cv::Mat& covH);
    bool estimate_svdelementjacobian(cv::Mat inputD,cv::Mat& Ju,cv::Mat& Jd,cv::Mat& Jv);
    bool computejacobian_fromsvd(cv::Mat Ju,cv::Mat Jd,cv::Mat Jv,cv::Mat inputD,cv::Mat& Jh);
    bool find_translationCov(cv::Mat covUVD,cv::Mat& covt);
    bool find_yawcovariance(cv::Mat covUVD,double& covYaw);

    template <class T>
    void reshape_vect2square(cv::Mat Inputmat,cv::Mat& Outputmat,int square_size,unsigned int CV_TYPE);
    
    template <class J>
    void reshape_square2vect(cv::Mat Inputmat,cv::Mat& Outputmat,unsigned int CV_TYPE);
public:
    VisualOdometry(void);
    VisualOdometry(cv::Mat Intrinsic);
    VisualOdometry(cv::Mat Intrinsic,unsigned char setup);
    virtual ~VisualOdometry();
    
   // bool compute_odometry(cv::Mat InputFrame,cv::Mat& Rotation,cv::Mat& Translation,cv::Mat& Normal);
    bool compute_odometry_lkoptflow(cv::Mat InputFrame,cv::Mat& Drawframe,cv::Mat& Rotation,cv::Mat& Translation,cv::Mat& Normal);
    
    bool compute_odometry_lkoptflowCov(cv::Mat InputFrame,cv::Mat& Drawframe,cv::Mat& Rotation,cv::Mat& Translation,cv::Mat& Normal,cv::Mat& Covt,double& covYaw);

    bool compute_odometry_SURF(cv::Mat InputFrame,cv::Mat& Drawframe,cv::Mat& lastDrawframe,cv::Mat& Rotation,cv::Mat& Translation,cv::Mat& Normal);
    
    void printcvMat(cv::Mat P);
    void dcm2angle(cv::Mat R,double& roll,double& pitch,double& yaw,bool degorrad);
    void drawmatch(cv::Mat& rgbFrame,cv::Mat& lastdraw);
    void drawoptflow(cv::Mat& rgbFrame);
    cv::Mat rpy2homography(double roll,double pitch,double yaw,bool mode);
    

    //GET , SET
    cv::Mat get_t1(void);
    cv::Mat get_t2(void);
    cv::Mat get_R1(void);
    cv::Mat get_R2(void);
    cv::Mat get_n1(void);
    cv::Mat get_n2(void);
    cv::Mat get_H(void);

};


#endif
