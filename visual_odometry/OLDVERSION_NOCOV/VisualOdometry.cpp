/*
  * VisualOdometry.h
  *
  * Created on: Jan , 2014
  * Author: Kandith
  * 
  *
  */ 



#include "VisualOdometry.h"

 
using namespace cv;
using namespace std;

VisualOdometry::VisualOdometry(void){}
 

VisualOdometry::VisualOdometry(Mat Intrinsic)
:termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03),
 winSize(31, 31),
 svd(),
 dsvd(),
 mat_1(3, 3,CV_64F,Scalar(0.0)),
 mat_2(3, 3,CV_64F,Scalar(0.0)),
 detector(400),
 matcher(NORM_L2)
 {
    this->needToInit = true;
    this->decomp_check = true;
    this->A = Intrinsic;
    this->Ainv = this->A.inv(DECOMP_LU);
    this->mat_1.at<double>(1,1) = 1;
    this->mat_2.at<double>(1,1) = 1;
    this->printflag = PRINT_ENABLE;
    this->drawableflag = DRAW_DISABLE;
    
 }

VisualOdometry::VisualOdometry(Mat Intrinsic,unsigned char setup)
:termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03),
 winSize(31, 31),
 svd(),
 dsvd(),
 mat_1(3, 3,CV_64F,Scalar(0.0)),
 mat_2(3, 3,CV_64F,Scalar(0.0)),
 detector(400),
 matcher(NORM_L2)
 {
    this->needToInit = true;
    this->decomp_check = true;
    this->A = Intrinsic;
    this->Ainv = this->A.inv(DECOMP_LU);
    this->mat_1.at<double>(1,1) = 1;
    this->mat_2.at<double>(1,1) = 1;
    this->printflag = setup & PRINT_ENABLE;
    this->drawableflag = setup & DRAW_ENABLE;
 }

VisualOdometry::~VisualOdometry(){}

//Private Function

bool VisualOdometry::iszero(Point2f x)
{
	return ((x.x == 0.0) && (x.y == 0.0));
}

double VisualOdometry::homography_calcdenormalizedfactor(Mat Hp)
{
    Mat Hl,S,U,Vt,V;
    
    Hl = (this->Ainv) * Hp * (this->A);
    this->dsvd.compute(Hl,S,U,Vt);

    return sqrt(S.at<double>(2));
}


bool VisualOdometry::homodecomp_compute(cv::Mat H,cv::Mat& R,cv::Mat& t,cv::Mat& n)
{
 //Matrix Decomposition
    
    this-> Hc = (this->Ainv) * H * (this->A);
    Mat S,U,Vt,V;
   
    this->svd.compute(this->Hc,S,U,Vt);
    
    transpose(Vt,V); // V = (Vt)';
    
    double eigen[3] = {S.at<double>(0),S.at<double>(1),S.at<double>(2)};
   
    double delta = -1.0*sqrt( ((eigen[0]*eigen[0]) - (eigen[1]*eigen[1])) / ((eigen[1]*eigen[1]) - (eigen[2]*eigen[2])) );
   // double (delta2) = -delta;
    
    double s = determinant(U)*determinant(V);
    double alpha = (eigen[0] + (s*eigen[2]*(delta*delta)))/(eigen[1]*(1 + delta*delta));
    
    double beta = sqrt(1.0- (alpha*alpha));
   // double (beta2) = -beta;

 
    
   this->mat_1.at<double>(0,0) = alpha;
   this->mat_2.at<double>(0,0) = alpha;
   this->mat_1.at<double>(0,2) = beta;
   this->mat_2.at<double>(0,2) = (-beta);
   this->mat_1.at<double>(2,0) = -1.0*s*beta;
   this->mat_2.at<double>(2,0) = -1.0*s*(-beta);
   this->mat_1.at<double>(2,2) = s*alpha;
   this->mat_2.at<double>(2,2) = s*alpha;
   this->R1 = U*(mat_1)*Vt;
   this->R2 = U*(mat_2)*Vt;
   this->n1 = delta*V.col(0)+V.col(2);
   this->n2 = (-delta)*V.col(0)+V.col(2);
   double w1 = sqrt(1.0/((n1.at<double>(0)*n1.at<double>(0))+(n1.at<double>(1)*n1.at<double>(1))+(n1.at<double>(2)*n1.at<double>(2))));
   double w2 = sqrt(1.0/((n2.at<double>(0)*n2.at<double>(0))+(n2.at<double>(1)*n2.at<double>(1))+(n2.at<double>(2)*n2.at<double>(2))));
   this->n1 = w1*n1;
   this->n2 = w2*n2;
   this->t1 = (1.0/w1)*(-1.0*beta*U.col(0)+((eigen[2]/eigen[1])-(s*alpha))*U.col(2));
   this->t2 = (1.0/w2)*(-1.0*(-beta)*U.col(0)+((eigen[2]/eigen[1])-(s*alpha))*U.col(2));
    
   

    double roll1,pitch1,yaw1,roll2,pitch2,yaw2;
    this->dcm2angle(R1,roll1,pitch1,yaw1,DEGREE);   
    this->dcm2angle(R2,roll2,pitch2,yaw2,DEGREE);
    
         
    
    double norm1,norm2;
    //finding minimum norm between [roll1 pitch1] and [roll2 pitch2]
    norm1 = sqrt((roll1*roll1)+(pitch1*pitch1));
    norm2 = sqrt((roll2*roll2)+(pitch2*pitch2));
    
    //Have std:: due to c++11 problem
    
    if(std::isnan(norm1)||std::isnan(norm2))
      {
        cout << "-- Output nan Error Wait for next sequence --" << endl;
        return false;
      } 
  
       
    if(norm1 < norm2)
    {
        //cout << "Your Correct Answer is 1" << endl;
        R = this->R1;
        n = this->n1;
        t = this->t1;
        //Z-axis normal vector should point to the surface planar as theory : n(z) never be < 0
        if(n.at<double>(2) < 0.0) 
        { 
          t = -1.0*t; 
          n = -1.0*n;
        }
        return true;
       
    }
    else if(norm1 > norm2)
    {
        //cout << "Your Correct Answer is 2" << endl;
        R = this->R2;
        n = this->n2;
        t = this->t2;
        //Z-axis normal vector should point to the surface planar as theory : n(z) never be < 0
        if(n.at<double>(2) < 0.0) 
        { 
          t = -1.0*t; 
          n = -1.0*n;
        }
        return true;
    }
    else
    {
        cout << "No Correct answer Select" << endl;
        return false;
    }

}



//----------------------------Public Function-------------------------------------------------------

/*----------------- Using Opitcal Flow Tracker -------------------------------------------------*/

bool VisualOdometry::compute_odometry_lkoptflow(Mat InputFrame,Mat& Drawframe,Mat& Rotation,Mat& Translation,Mat& Normal)
{
	InputFrame.copyTo(this->rgbFrames);
	cvtColor(this->rgbFrames,this->grayFrames, CV_BGR2GRAY);

	if(this->needToInit)
    {
    	goodFeaturesToTrack(this->grayFrames, this->points1, MAX_COUNT, 0.01, 5, Mat(),3, 0, 0.04);
        
        if(this->drawableflag)
        {
            drawoptflow(Drawframe);    
        }

        this->needToInit = false;
    } 

    else if (!(this->points2.empty()))
    {
                   
    	calcOpticalFlowPyrLK(this->prevGrayFrame, this->grayFrames, this->points2, this->points1,this->status, this->err, this->winSize, 3, this->termcrit, 0, 0.001);
        

        if(this->drawableflag)
        {
            drawoptflow(Drawframe);    
        }            
    	
          
        //Repacking tracked points form status

        for (int j = 0; j < this->points2.size(); j++ ) 
        {
        	if( !(this->status[j]) )
            {
            	this->points1[j].x = 0.0;
                this->points1[j].y = 0.0;
                this->points2[j].x = 0.0;
                this->points2[j].y = 0.0;
            }
        }
                  
        vector<Point2f>::iterator newIter1 = std::remove_if( this->points1.begin() , this->points1.end() , std::bind(&VisualOdometry::iszero,this, std::placeholders::_1 ));
        vector<Point2f>::iterator newIter2 = std::remove_if( this->points2.begin() , this->points2.end() , std::bind(&VisualOdometry::iszero,this, std::placeholders::_1 ));
        this->points1.resize( newIter1 -  this->points1.begin() );
        this->points2.resize( newIter2 -  this->points2.begin() );
                   
                   
        this->H = findHomography(this->points2,this->points1,CV_RANSAC,3,cv::noArray() );

        /*cout << "-----------Homography Matrix:H----------" << endl;
             
        this->printcvMat(H);*/

        //this->decomp_check = this->hmdecom.compute(this->H,this->R,this->t,this->n);
          this->decomp_check = this->homodecomp_compute(this->H,this->R,this->t,this->n); 

        if(this->decomp_check)
        { 
            //cout << "---------- R--------"<< endl;
            //this->printcvMat(R);
            Rotation = this->R;
           //cout << "---------- T--------"<< endl;
           //this->printcvMat(t);
            Translation = -this->t; // To make the output direction match camera coordinate
           //cout << "---------- N--------"<< endl;
           //this->printcvMat(n);
            Normal = this->n;
            
            this->result = true;
        
        }
        else
        {
        	cout << "wait for next sequence" << endl;
            this->result = false; 



        }

        goodFeaturesToTrack(this->grayFrames, this->points1, MAX_COUNT, 0.01, 10, Mat(),3, 0, 0.04);
       // std::copy(this->points1.begin(),this->points1.end(),std::back_inserter(this->drawpts1));
       // std::copy(this->points2.begin(),this->points2.end(),std::back_inserter(this->drawpts2));    
     
    } // end if pointempty
    
    if(this->decomp_check) //if Decomp Error -> Go get new sample image
    {
        
        swap(this->points2,this->points1);
        this->points1.clear();   //pts2 == prev, pts1 == newpts
        this->grayFrames.copyTo(this->prevGrayFrame);  
        return this->result;
    }




}


/*----------------------   Using SURF Feature --------------------------*/

bool VisualOdometry::compute_odometry_SURF(cv::Mat InputFrame,cv::Mat& Drawframe,cv::Mat& lastDrawframe,cv::Mat& Rotation,cv::Mat& Translation,cv::Mat& Normal)
{

    InputFrame.copyTo(this->rgbFrames);
    cvtColor(this->rgbFrames,this->grayFrames, CV_BGR2GRAY);
    
    if(this->needToInit)
    {
        
        
      //-- Step 0: Detect the keypoints using SURF Detector of Previous image
      this->detector.detect(this->grayFrames , this->keypoints_new );

      this->extractor.compute(this->grayFrames,this->keypoints_new,this->descriptors_new);
 

        this->needToInit = false;
    } 
    else if(!(this->keypoints_prev.empty()))
    {
      
     

      //-- Step 1: Detect the keypoints using SURF Detector
      this->detector.detect(this->grayFrames,this->keypoints_new);
     
      //-- Step 2: Calculate descriptors (feature vectors)
      this->extractor.compute(this->grayFrames,this->keypoints_new,this->descriptors_new);
       

      //-- Step 3: Matching descriptor vectors using BFmatcher
      
      matcher.match( descriptors_prev, descriptors_new, matches );
     
         

      for( int i = 0; i < matches.size(); i++ )
      {

    //-- Get the keypoints from the good matches
       this->points2.push_back( keypoints_prev[ matches[i].queryIdx ].pt );
       this->points1.push_back( keypoints_new[ matches[i].trainIdx ].pt );
      }
      

      if(this->drawableflag)
      {
          
          this->drawmatch(Drawframe,lastDrawframe);    
      }            


      this->H = findHomography(this->points2,this->points1,CV_RANSAC,3,cv::noArray() );

      cout << "---------Homography--------" << endl;
      this->printcvMat(this->H);

      this->decomp_check = this->homodecomp_compute(this->H,this->R,this->t,this->n); 

        if(this->decomp_check)
        { 
          cout << "---------- R--------"<< endl;
            this->printcvMat(R);
            Rotation = this->R;
            cout << "---------- T--------"<< endl;
            this->printcvMat(t);
            Translation = this->t;
            cout << "---------- N--------"<< endl;
            this->printcvMat(n);
            Normal = this->n;
            
            this->result = true;
        
        }
  
    }
      
    if(this->decomp_check) //if Decomp Error -> Go get new sample image
    {
        
        swap(this->points2,this->points1);
        swap(this->keypoints_prev,this->keypoints_new);
        this->descriptors_prev = this->descriptors_new;

        this->descriptors_new = Mat();
        this->keypoints_new.clear();
        this->points1.clear();   //pts2 == prev, pts1 == newpts
        this->grayFrames.copyTo(this->prevGrayFrame);  
       
        return this->result;
    }
      
      
      //-- Get the corners from the image_1 ( the object to be "detected" )

}

void VisualOdometry::printcvMat(Mat P)
{
	Size s = P.size();
    for(int i = 0;i < s.height;i++) //rows
    {
        for(int j = 0; j < s.width; j++)
        {
            cout<< P.at<double>(i,j) << " ";
        }
        cout << endl;
    }
    cout << endl;
}

void VisualOdometry::dcm2angle(Mat R,double& roll,double& pitch,double& yaw,bool degorrad)
{
	//roll = atan2(R.at<double>(2,1),R.at<double>(2,2))*180.0/PI;
    //pitch = atan2(-1*R.at<double>(2,0),sqrt((R.at<double>(2,1)*R.at<double>(2,1))+(R.at<double>(2,2)*R.at<double>(2,2))))*180.0/PI;
    //yaw = atan2(R.at<double>(1,0),R.at<double>(0,0))*180.0/PI;
    yaw = atan2(R.at<double>(0,1),R.at<double>(0,0));
    pitch = asin(-R.at<double>(0,2));
    roll = atan2(R.at<double>(1,2),R.at<double>(2,2));

    if(degorrad == DEGREE)
    {
        yaw *= 180.0/PI;
        pitch *= 180.0/PI;
        roll *= 180/PI;
    }    

}

Mat VisualOdometry::rpy2homography(double roll,double pitch,double yaw,bool mode)
{
    if(mode == DEGREE)
    {
        //if DEGREE -> make it in radians
       roll *= PI/180.0;
       pitch *= PI/180.0;
       yaw *= PI/180.0;
    }

    Mat Rgen(3,3,CV_64F);
    double sx,sy,sz,cx,cy,cz;
    sx = sin(roll);
    cx = cos(roll);
    sy = sin(pitch);
    cy = cos(pitch);
    sz = sin(yaw);
    cz = cos(yaw);
    /* x-axis = 1 ; y - axis = 2 ; z - axis = 3
       [
            c2c3             -c2s3               s2

            c1s3+c3s1s2      c1c3 - s1s2s3       -c2s1

            s1s3 - c1c3s2    c3s1 + c1s2s3       c1c2

        ]
    */
    

    Rgen.at<double>(0,0) = cy*cz;
    Rgen.at<double>(0,1) = -cy*sz;
    Rgen.at<double>(0,2) = sy;

    Rgen.at<double>(1,0) = cx*sz + cz*sx*sy;
    Rgen.at<double>(1,1) = cx*cz - sx*sy*sz;
    Rgen.at<double>(1,2) = -cy*sx;

    Rgen.at<double>(2,0) = sx*sz - cx*cz*sy;
    Rgen.at<double>(2,1) = cz*sx + cx*sy*sz;
    Rgen.at<double>(2,2) = cx*cy;

    
    return (this->A*Rgen*this->Ainv);

}

void VisualOdometry::drawmatch(Mat& rgbFrame,Mat& lastdraw)
{
   

    for (int i = 0; i < this->points2.size(); i++) 
    {
           

        //if(this->status[i])
        //{
            if ((this->points1[i].x - this->points2[i].x) > 0) //nextpts - prevpts > 0 := feature pts move to right Scalar(R,G,B)  
            {
                line(rgbFrame, this->points1[i], this->points2[i], Scalar(0, 0, 255), 1, 1, 0);
                circle(rgbFrame, this->points1[i], 2, Scalar(255, 0, 0), 1, 1, 0);
                circle(lastdraw, this->points2[i], 2, Scalar(255, 0, 0), 1, 1, 0);
            } 
            else 
            {
                line(rgbFrame, this->points1[i], this->points2[i], Scalar(0, 255, 0),1, 1, 0);
                circle(rgbFrame, this->points1[i], 2, Scalar(255, 0, 0), 1, 1,0);
                circle(lastdraw, this->points2[i], 2, Scalar(255, 0, 0), 1, 1, 0);
            }

       // }
    }
}

void VisualOdometry::drawoptflow(Mat& rgbFrame)
{
   

    for (int i = 0; i < this->points2.size(); i++) 
    {
           

        if(this->status[i])
        {
            if ((this->points1[i].x - this->points2[i].x) > 0) //nextpts - prevpts > 0 := feature pts move to right Scalar(R,G,B)  
            {
                line(rgbFrame, this->points1[i], this->points2[i], Scalar(0, 0, 255), 1, 1, 0);
                circle(rgbFrame, this->points1[i], 2, Scalar(255, 0, 0), 1, 1, 0);
            } 
            else 
            {
                line(rgbFrame, this->points1[i], this->points2[i], Scalar(0, 255, 0),1, 1, 0);
                circle(rgbFrame, this->points1[i], 2, Scalar(255, 0, 0), 1, 1,0);
            }

        }
    }
}

//----GET,SET function
Mat VisualOdometry::get_t1(void)
{
    return this->t1;
}

Mat VisualOdometry::get_t2(void)
{
    return this->t2;
}

Mat VisualOdometry::get_R1(void)
{
    return this->R1;
}

Mat VisualOdometry::get_R2(void)
{
    return this->R2;
}

Mat VisualOdometry::get_n1(void)
{
    return this->n1;
}

Mat VisualOdometry::get_n2(void)
{
    return this->n2;
}

Mat VisualOdometry::get_H(void)
{
    return this->H;
}


