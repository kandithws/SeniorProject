/*
  * VisualOdometry.h
  *
  * Created on: Jan , 2015
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

    this->cov_matchedfeature = (1.0/36.0)*Mat::eye(24,24,CV_64F);
    this->eye9 = Mat::eye(9,9,CV_64F);
    
 }

VisualOdometry::VisualOdometry(Mat Intrinsic,unsigned char setup)
:termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03),
 winSize(31, 31),
 svd(),
 dsvd(),
 mat_1(3, 3,CV_64F,Scalar(0.0)),
 mat_2(3, 3,CV_64F,Scalar(0.0)),
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

    /*Covariance matrix initiator 6sigma*/
    this->cov_matchedfeature = (1.0/36.0)*Mat::eye(24,24,CV_64F); // Select feature 12 pts
    this->eye9 = Mat::eye(9,9,CV_64F);
    //this->printcvMat(eye8);

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

template <class T>
void VisualOdometry::reshape_vect2square(cv::Mat Inputmat,cv::Mat& Outputmat,int square_size,unsigned int CV_TYPE)
{
  if((Inputmat.rows % square_size) == 0 )
  {
    Outputmat = Mat(square_size,square_size,CV_TYPE,Scalar(0.0)); 
    for(int i =0; i < square_size ; i++ )
    {
        //Reshape to square Mat
        for(int j=0; j< square_size ; j++)
        {
          Outputmat.at<T>(i,j) = Inputmat.at<T>(j+i*square_size);
          //cout <<"Inmat : i,j val  = " << i << "," << j << "," << Inputmat.at<T>(j+i*square_size) <<endl;
          //cout <<"Outputmat : i,j val  = " << i << "," << j << "," << Outputmat.at<T>(i,j) <<endl;
        }        
    } 
  }
  else cout << "Reshape : inputsize un available" << endl;
}

template <class T>
void VisualOdometry::reshape_square2vect(cv::Mat Inputmat,cv::Mat& Outputmat,unsigned int CV_TYPE)
{
    
    Size s = Inputmat.size();
    Outputmat = Mat(s.height*s.width,1,CV_TYPE,Scalar(0.0));
    int it = 0;
    for(int i = 0;i < s.height;i++) //rows
    {
        for(int j = 0; j < s.width; j++)
        {
           Outputmat.at<T>(it++,0) = Inputmat.at<T>(i,j);
        }
    }
}


bool VisualOdometry::homodecomp_compute(cv::Mat H,cv::Mat& R,cv::Mat& t,cv::Mat& n)
{
 //Matrix Decomposition
    
    this-> Hc = (this->Ainv) * H * (this->A);
    Mat Vt;
    
    this->svd.compute(this->Hc,this->D,this->U,Vt);
      
    transpose(Vt,this->V); // V = (Vt)';
    //cout << "Size V from SVD : " << this->V.size() << endl; 
    double eigen[3] = {this->D.at<double>(0),this->D.at<double>(1),this->D.at<double>(2)};
   
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
        this->c_alpha = alpha;
        this->c_beta = beta;
        this->c_delta = delta;
        this->c_s = s;
        this->c_omega = w1;
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
        
        this->c_alpha = alpha;
        this->c_beta = -1.0*beta;
        this->c_delta = -1.0*delta;
        this->c_s = s;
        this->c_omega = w2;
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

/*-----------Covariance Matrix Estimator------------*/
bool VisualOdometry::estimate_homographycovariancematrix(cv::Mat& covH)
{
  cv::Mat Jp(24,32,CV_64F,Scalar(0.0));
  
  /*---Esitmate Homography+Featurepts Jacobian---*/
  try
  {
    int j = 0;

    for(int i=0;i < Jp.rows;i++)
    {
      
      if((i+1)%2 != 0) //if its is a odd rows (Even Index) --> Jp element will be dx[i]'/d??;
      {
        //Jp.at<double>(i,0) = this->ransac_ptsold.at<double>(2*i,0); // dx[i]'/dh11 = x[i]
        //Jp.at<double>(i,1) = this->ransac_ptsold.at<double>(2*i+1,0); // dx[i]'/dh12 = y[i]
        Jp.at<double>(i,0) = this->ransac_ptsold[j].x ;// dx[i]'/dh11 = x[i]
        Jp.at<double>(i,1) = this->ransac_ptsold[j].y ; // dx[i]'/dh12 = y[i]
        Jp.at<double>(i,2) = 1; // dx[i]'/dh13 = 1
       
        Jp.at<double>(i,i+8) = this->H.at<double>(0,0); // dx[i]'/dx[i] = h11
        Jp.at<double>(i,i+9) = this->H.at<double>(0,1); // dx[i]'/dy[i] = h12
      //  cout << "i odd = " << i << "," << i+8 << "," << i+9 <<endl;
        //Other Element in Jp in this row is 0
      }
      else //even row
      {
        Jp.at<double>(i,3) = this->ransac_ptsold[j].x; // dy[i]'/dh21 = x[i]
        Jp.at<double>(i,4) = this->ransac_ptsold[j].y; // dy[i]'/dh22 = y[i]
        Jp.at<double>(i,5) = 1; // dy[i]'/dh23 = 1
        j++;
        Jp.at<double>(i,(i-1)+8) = this->H.at<double>(1,0); // dy[i]'/dx[i] = h21
        Jp.at<double>(i, (i-1)+9) = this->H.at<double>(1,1); // dy[i]'/dy[i] = h22
        // std::cout << "i even = " << i << "," << (i-1)+8 << "," << (i-1)+9 << endl
      } 
    }
  }
  catch (exception &e)
  {
    cout << e.what() << endl;
  }
  //cout << " --Jp--" <<endl;
  //printcvMat(Jp);
  /*Backward Error Propagation*/
  cv::Mat covP = (Jp.t()*this->cov_matchedfeature*Jp).inv(DECOMP_SVD); // (Jpt * CovX * Jp)^+ psudoinverse of cv must use inv SVD
    //cout << " --Jp--" <<endl;
    //      printcvMat(Jp);

          covH = covP(Range(0,9),Range(0,9));
          // Make the None diagonal element to be 0 note : .mul = per element operation
          covH = covH.mul(this->eye9);
          covH.at<double>(8,8) = 0;
  

  
  //std::cout << "Homography Covariance Matrix" << std::endl;
  //printcvMat(covH);
}

bool VisualOdometry::estimate_svdelementjacobian(cv::Mat inputD,cv::Mat& Ju,cv::Mat& Jd,cv::Mat& Jv)
{
  cv::Mat omegaUij(3,3,CV_64F,Scalar(0.0));
  cv::Mat omegaVij(3,3,CV_64F,Scalar(0.0));
          Ju = Mat(9,9,CV_64F,Scalar(0.0));
          Jv = Mat(9,9,CV_64F,Scalar(0.0));
          Jd= Mat(9,9,CV_64F,Scalar(0.0));
  int jcb_col = 0 ;
  int jd_col = 0;

  for(int i = 0;i < 3; i++)
  {
    for(int j=0;j < 3;j++)
    {
      for(int k=0;k < 3;k++)
      {
        int l = (i+k)%3;
       // cout << "iteration i j k l: " << i << ',' << j << ',' << k << ',' << l << ',' << endl;
       // if(l != k)
       // {
              // l = i +1 .. 2
            double dl,dk,uik,vjl,uil,vjk;
            dl = inputD.at<double>(l);
            dk = inputD.at<double>(k);
            uik = this->U.at<double>(i,k);
            vjl = this->V.at<double>(j,l);
            uil = this->U.at<double>(i,l);
            vjk = this->V.at<double>(j,k);
            double det = dl*dl+ dk*dk;
            if(det != 0) 
            {
              omegaUij.at<double>(k,l) = (uik*vjl*dl - dk*(-uil)*vjk)/det;
              omegaVij.at<double>(k,l) = (dl*(-uil)*vjk - uik*vjl*dk)/det;
              
            } 
            else 
            {
              cout << "Solving Error" << endl;
            }
      //  }
        /*else
        {
          // l == k the jacobian element of ij will be 0
          omegaUij.at<double>(k,l) = 0.0;
          omegaVij.at<double>(k,l) = 0.0;
        }*/
          //--------Find Jacobian of D 9x9 ------
          Jd.at<double>(k*4,jcb_col) = uik*vjk; 

      
      } // end k
    
      cv::Mat Ju_buff(3,3,CV_64F,Scalar(0.0));
      cv::Mat Jv_buff(3,3,CV_64F,Scalar(0.0));
   
      
      Ju_buff = this->U*omegaUij;
      Jv_buff = -1.0*this->V*omegaVij;



      cv::Mat vect_Ju,vect_Jv;

      //reshape column into J
      for(int i=0;i <Ju_buff.rows ;i++)
      {
        for(int j=0;j <Ju_buff.cols ;j++)
        {
          vect_Ju.push_back(Ju_buff.at<double>(i,j));
          vect_Jv.push_back(Jv_buff.at<double>(i,j));
        }
      }
        vect_Ju.copyTo(Ju.col(jcb_col));
      vect_Jv.copyTo(Jv.col(jcb_col));
      jcb_col++;


    } // end j
  } //end i

  return true;
}

/*bool VisualOdometry::computejacobian_fromsvd(cv::Mat Ju,cv::Mat Jd,cv::Mat Jv,cv::Mat inputD,cv::Mat& Jh)
{
   Jh = Mat();
   Mat Ju_square,Jd_square,Jv_square,Jh_square,Vt; 
   //Make inputD from Vector to diagonal Matrix
   Mat inputD_square(3,3,CV_64F,Scalar(0.0));
   inputD_square.at<double>(0,0) = inputD.at<double>(0);
   inputD_square.at<double>(1,1) = inputD.at<double>(1);
   inputD_square.at<double>(2,2) = inputD.at<double>(2);
   Vt = this->V.t();
   for(int i=0 ; i < 9 ; i++)
   {
      this->reshape_vect2square<double>(Ju.col(i),Ju_square,3,CV_64F);
      this->reshape_vect2square<double>(Jv.col(i),Jv_square,3,CV_64F);
      this->reshape_vect2square<double>(Jd.col(i),Jd_square,3,CV_64F);

      Jh_square = (Ju_square*inputD_square*Vt) + (this->U*Jd_square*Vt) + (this->U*inputD_square*Jd_square.t());  
      Mat Jcol;
      this->reshape_square2vect<double>(Jh_square,Jcol,CV_64F);
      cout << "Print Vect : " << i << endl;
      printcvMat(Jcol);

      if(i==0)
      {
        Jcol.copyTo(Jh);
      }
      else 
      {
        cv::hconcat(Jh,Jcol,Jh);
      }
    
    }

}*/

bool VisualOdometry::find_translationCov(cv::Mat covUVD,cv::Mat& covt)
{
  Mat Jt(3,21,CV_64F,Scalar(0.0));
  double detv = determinant(this->V) ;
  double detu = determinant(this->U) ;

  double u11,u12,u13,u21,u22,u23,u31,u32,u33,d1,d2,d3;
  double nx,ny,nz,v11,v12,v13,v21,v22,v23,v31,v32,v33;

  u11 = this->U.at<double>(0,0);
  u12 = this->U.at<double>(0,1);
  u13 = this->U.at<double>(0,2);
  u21 = this->U.at<double>(1,0);
  u22 = this->U.at<double>(1,1);
  u23 = this->U.at<double>(1,2);
  u31 = this->U.at<double>(2,0);
  u32 = this->U.at<double>(2,1);
  u33 = this->U.at<double>(2,2);

  v11 = this->V.at<double>(0,0);
  v12 = this->V.at<double>(0,1);
  v13 = this->V.at<double>(0,2);
  v21 = this->V.at<double>(1,0);
  v22 = this->V.at<double>(1,1);
  v23 = this->V.at<double>(1,2);
  v31 = this->V.at<double>(2,0);
  v32 = this->V.at<double>(2,1);
  v33 = this->V.at<double>(2,2);

  nx = this->n.at<double>(0);
  ny = this->n.at<double>(1);
  nz = this->n.at<double>(2);

  Jt.at<double>(0,0) = (-this->c_beta/this->c_omega)-((this->c_alpha*u13*detv)*(u22*u33-u32*u23)); //dx/du11
  Jt.at<double>(1,0) = (-this->c_alpha*u23*detv)*(u22*u33-u32*u23);//dy/du11
  Jt.at<double>(2,0) = (Jt.at<double>(1,0) /u23)*u33 ; //dz/du11

  //cout << "out   : " << Jt.at<double>(0,0) << " " << Jt.at<double>(1,0) << " " << Jt.at<double>(2,0) <<endl;
  
  Jt.at<double>(0,1) = -this->c_alpha*detv*(u23*u31 - u33*u21); //dx/u12
  Jt.at<double>(1,1) = Jt.at<double>(0,1); //dy/u12
  Jt.at<double>(2,1) = Jt.at<double>(0,1); //dz/u12

  //cout << "out   : " << Jt.at<double>(0,1) << " " << Jt.at<double>(1,1) << " " << Jt.at<double>(2,1) <<endl;

  Jt.at<double>(0,2) = (-this->c_alpha/this->c_omega)*(((u21*u32-u31*u22)*u13*detv) + this->c_s); //dx/u13
  Jt.at<double>(1,2) = (-this->c_alpha/this->c_omega)*((u21*u32-u31*u22)*u13*detv); //dy/u13
  Jt.at<double>(2,2) = Jt.at<double>(1,2); //dz/u13  

  Jt.at<double>(0,3) = -this->c_alpha*u13*detv*(u13*u32-u33*u12); //dx/u21
  Jt.at<double>(1,3) = (-this->c_beta/this->c_omega)-((this->c_alpha)*u23*detv*(u13*u32-u33*u12)); //dy/u21
  Jt.at<double>(2,3) = (Jt.at<double>(0,3)/u13)*u33; //dz/u21

  Jt.at<double>(0,4) = -this->c_alpha*detv*(u11*u33 - u31*u13); //dx/u22
  Jt.at<double>(1,4) = Jt.at<double>(0,4); //dy/u22
  Jt.at<double>(2,4) = Jt.at<double>(0,4); //dz/u22

  Jt.at<double>(0,5) = (-this->c_alpha/this->c_omega)*u13*(u12*u31 - u32*u11)*detv; //dx/u23
  Jt.at<double>(1,5) = (-this->c_alpha/this->c_omega)*((u23*(u12*u31 - u32*u11)*detv) + this->c_s); //dy/u23
  Jt.at<double>(2,5) = (Jt.at<double>(0,5)/u13)*u33; //dz/u23

  Jt.at<double>(0,6) = -this->c_alpha*u13*detv*(u12*u23 - u22*u13); //dx/u31
  Jt.at<double>(1,6) = (Jt.at<double>(0,6)/u13)*u23; //dy/u31
  Jt.at<double>(2,6) = (-this->c_beta/this->c_omega)-((this->c_alpha)*u33*detv*(u12*u32-u22*u13)); //dz/u31

  Jt.at<double>(0,7) = this->c_alpha*detv*(u13*u21 - u23*u11); //dx/u32
  Jt.at<double>(1,7) = Jt.at<double>(0,7); //dy/u32
  Jt.at<double>(2,7) = Jt.at<double>(0,7); //dz/u32

  Jt.at<double>(0,8) = (-this->c_alpha/this->c_omega)*u13*(u11*u22 - u21*u12)*detv; //dx/u33
  Jt.at<double>(1,8) = (Jt.at<double>(0,8)/u13)*u23; //dy/u33
  Jt.at<double>(2,8) = (-this->c_alpha/this->c_omega)* ((u33*(u11*u22 - u21*u12)*detv)+this->c_s); //dz/u33
 
  double d1s = d1*d1;
  double d2s = d2*d2;
  double d3s = d3*d3;
  double d2s_d3s = d2s - d3s;
  double d1s_d2s = d1s - d2s;

  //d(delta) / d (lambda1)
  double curl_deltad1 = d1/sqrt((d1s_d2s)*(d2s_d3s));
  //d(delta) / d (lambda2)
  double curl_deltad2 = (d2*(d3s - d1s) ) / (d2s_d3s*d2s_d3s*this->c_delta);
  //d(delta) / d (lambda3)
  double curl_deltad3 = this->c_delta*d3 / (d2s_d3s);
  //d(alpha) / d (lambda1)
  double curl_alphad1 = ((2.0*this->c_s*d1*d2*d3*(d2s_d3s)) - (d1s*d2s) + (d2*d3s)*(d1s_d2s + d3s)) / pow((d1s*d2 - d2*d3s),2.0);
  //d(alpha) / d (lambda2)
  double curl_alphad2 = ((d1*(d1s*(d2s_d3s)) - d2s*d3s - d3s*d3s) + this->c_s*(d2s*d3s - d1s*d2s*d3 - d1s*d1s*d3 + d1s*d3s)) / pow((d1s*d2 - d2*d3s),2.0);
  //d(alpha) / d (lambda3)
  double curl_alphad3 = (2.0*d1*d2*d3*(d2*d2 - d1*d1) + this->c_s*d1s*d2*(d1s-d2) + (2.0*this->c_s)*(d1s*d2*d3s) - (this->c_s* d2s * d2 * d3s)) / pow((d1s*d2 - d2*d3s),2.0);

  double den = (1.0/sqrt(1.0-this->c_alpha*this->c_alpha));

  double curl_betad1 = curl_alphad1*den;
  double curl_betad2 = curl_alphad2*den;
  double curl_betad3 = curl_alphad3*den;

  
  
  double temp_1 = (this->c_omega*this->c_omega)* (nx*v11 + ny * v21 + nz* v31);
  //dx / d(lambda1)
  Jt.at<double>(0,18) = temp_1 * (this->t.at<double>(0) * curl_deltad1) - (1.0/this->c_omega)*(u11*curl_betad1 + u13*this->c_s*curl_alphad1); 
  //dy / d(lambda2)
  Jt.at<double>(1,18) = temp_1 * (this->t.at<double>(1) * curl_deltad1)  - (1.0/this->c_omega)*(u21*curl_betad1 + u23*this->c_s*curl_alphad1); 
  //dz / d(lambda3)
  Jt.at<double>(2,18) = temp_1 *(this->t.at<double>(2) * curl_deltad1)  - (1.0/this->c_omega)*(u31*curl_betad1 + u33*this->c_s*curl_alphad1);
  
  //dx / d(lambda2)
  Jt.at<double>(0,19) = temp_1 * (this->t.at<double>(0) * curl_deltad2) - (1.0/this->c_omega)*( (curl_betad2)*u11 + ((d3/d2s) - this->c_s*curl_alphad2)*u13 ) ; 
  //dy / d(lambda2)
  Jt.at<double>(1,19) = temp_1 * (this->t.at<double>(1) * curl_deltad2) - (1.0/this->c_omega)*( (curl_betad2)*u21 + ((d3/d2s) - this->c_s*curl_alphad2)*u23 ); 
  //dz / d(lambda2)
  Jt.at<double>(2,19) = temp_1 * (this->t.at<double>(2) * curl_deltad2) - (1.0/this->c_omega)*( (curl_betad2)*u31 + ((d3/d2s) - this->c_s*curl_alphad2)*u33 ) ; 
  
  //dx / d(lambda3)
  Jt.at<double>(0,20) = temp_1 * (this->t.at<double>(0) * curl_deltad3) - (1.0/this->c_omega)*( (curl_betad3)*u11 + ((-1.0/d2) + this->c_s*curl_alphad3)*u13);
  //dy / d(lambda3)
  Jt.at<double>(1,20) = temp_1 * (this->t.at<double>(1) * curl_deltad3) - (1.0/this->c_omega)*( (curl_betad3)*u21 + ((-1.0/d2) + this->c_s*curl_alphad3)*u23);
  //dz / d(lambda3)
  Jt.at<double>(2,20) = temp_1 * (this->t.at<double>(2) * curl_deltad3) - (1.0/this->c_omega)*( (curl_betad3)*u31 + ((-1.0/d2) + this->c_s*curl_alphad3)*u33);
  
  double temp_2 = (1.0/this->c_omega)* (-this->c_alpha*detu);
  double xx = v22*v33 - v32*v23;
  double c_omega_s = this->c_omega*this->c_omega;
  //dx/dv11
  Jt.at<double>(0,9) =  temp_2*u13*xx + c_omega_s*nx*this->c_delta*this->t.at<double>(0);
  //dy/dv11
  Jt.at<double>(1,9) =  temp_2*u23*xx + c_omega_s*nx*this->c_delta*this->t.at<double>(1);
  //dz/dv11
  Jt.at<double>(2,9) =  temp_2*u33*xx + c_omega_s*nx*this->c_delta*this->t.at<double>(2);
  
  //dx,y,z/dv12
  xx = v23*v31 - v33*v21;
  Jt.at<double>(0,10) = temp_2*u13*xx;
  Jt.at<double>(1,10) = temp_2*u23*xx;
  Jt.at<double>(2,10) = temp_2*u33*xx;
  
  //dx,y,z/dv13
  xx = v21*v32 - v31*v22;
  Jt.at<double>(0,11) = temp_2*u13*xx + c_omega_s*nx*this->c_delta*this->t.at<double>(0);
  Jt.at<double>(1,11) = temp_2*u23*xx + c_omega_s*nx*this->c_delta*this->t.at<double>(1);
  Jt.at<double>(2,11) = temp_2*u33*xx + c_omega_s*nx*this->c_delta*this->t.at<double>(2);
  
  
  //dx,y,z/dv21
  xx = v13*v32 - v33*v12;
  Jt.at<double>(0,12) = temp_2*u13*xx + c_omega_s*ny*this->c_delta*this->t.at<double>(0);
  Jt.at<double>(1,12) = temp_2*u23*xx + c_omega_s*ny*this->c_delta*this->t.at<double>(1);
  Jt.at<double>(2,12) =  temp_2*u33*xx + c_omega_s*ny*this->c_delta*this->t.at<double>(2);

  //dx,y,z/dv22
  xx = v11*v13 - v31*v13;
  Jt.at<double>(0,13) = temp_2*u13*xx;
  Jt.at<double>(1,13) = temp_2*u23*xx;
  Jt.at<double>(2,13) = temp_2*u33*xx;

  //dx,y,z/dv23
  xx = v12*v31 - v32*v11;
  Jt.at<double>(0,14) = temp_2*u13*xx + c_omega_s*ny*this->c_delta*this->t.at<double>(0);
  Jt.at<double>(1,14) = temp_2*u23*xx + c_omega_s*ny*this->c_delta*this->t.at<double>(1);
  Jt.at<double>(2,14) =  temp_2*u33*xx + c_omega_s*ny*this->c_delta*this->t.at<double>(2);

  //dx,y,z/dv31
  xx = v12*v23 - v22*v13;
  Jt.at<double>(0,15) = temp_2*u13*xx + c_omega_s*nz*this->c_delta*this->t.at<double>(0);
  Jt.at<double>(1,15) = temp_2*u23*xx + c_omega_s*nz*this->c_delta*this->t.at<double>(1);
  Jt.at<double>(2,15) =  temp_2*u33*xx + c_omega_s*nz*this->c_delta*this->t.at<double>(2);
  
  //dx,y,z/dv32
  xx = v13*v21 - v23*v11;
  Jt.at<double>(0,16) = temp_2*u13*xx;
  Jt.at<double>(1,16) = temp_2*u23*xx;
  Jt.at<double>(2,16) = temp_2*u33*xx;

  //dx,y,z/dv33
  xx = v11*v22 - v21*v12;
  Jt.at<double>(0,17) = temp_2*u13*xx + c_omega_s*nz*this->c_delta*this->t.at<double>(0);
  Jt.at<double>(1,17) = temp_2*u23*xx + c_omega_s*nz*this->c_delta*this->t.at<double>(1);
  Jt.at<double>(2,17) =  temp_2*u33*xx + c_omega_s*nz*this->c_delta*this->t.at<double>(2);

  /*--- Remove Nan and infinity ---*/
  for(int i=0; i < Jt.rows ; i++)
  {
    for(int j=0; j < Jt.cols ; j++)
    {
      if(std::isnan(Jt.at<double>(i,j)) || std::isinf(Jt.at<double>(i,j)) )
      Jt.at<double>(i,j) = 0;
      
    }
  }

  /*Forward Error Propagation*/
  //cout << "Jt --- " << endl;
  //printcvMat(Jt);

  covt = Jt*covUVD*Jt.t();
}

bool VisualOdometry::find_yawcovariance(cv::Mat covUVD,double& covYaw)
{
  Mat JYaw(1,21,CV_64F,Scalar(0.0));
  
  double v11,v12,v13,v21,v22,v23;
  v11 = this->V.at<double>(0,0);
  v12 = this->V.at<double>(0,1);
  v13 = this->V.at<double>(0,2);
  v21 = this->V.at<double>(1,0);
  v22 = this->V.at<double>(1,1);
  v23 = this->V.at<double>(1,2);

  double diff_buff = 1.0/(1.0 + pow(this->R.at<double>(1,0)*this->R.at<double>(0,0),2.0) );
  //cout << "diff_buff = " << diff_buff << endl;
  JYaw.at<double>(0,9) = (-v21/(v11*v11))*diff_buff ; //dyaw/dv11
  JYaw.at<double>(0,10) = (-v22/(v12*v12))*diff_buff ; //dyaw/dv12
  JYaw.at<double>(0,11) = (-v23/(v13*v13))*diff_buff ; //dyaw/dv13
  JYaw.at<double>(0,12) = (1.0/v11)*diff_buff ; //dyaw/dv21
  JYaw.at<double>(0,13) = (1.0/v12)*diff_buff ; //dyaw/dv22
  JYaw.at<double>(0,14) = (1.0/v13)*diff_buff ; //dyaw/dv23
  
  //cout << "JYaw ----" << endl;
  //printcvMat(JYaw);
  //Forward Err Propagate

  Mat covYawbuff = JYaw * covUVD * JYaw.t();
  covYaw = covYawbuff.at<double>(0,0); 
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

bool VisualOdometry::compute_odometry_lkoptflowCov(cv::Mat InputFrame,cv::Mat& Drawframe,cv::Mat& Rotation,
                                                   cv::Mat& Translation,cv::Mat& Normal,cv::Mat& Covt,double& covYaw)
{
	InputFrame.copyTo(this->rgbFrames);
	cvtColor(this->rgbFrames,this->grayFrames, CV_BGR2GRAY);

	if(this->needToInit)
    {
    	goodFeaturesToTrack(this->grayFrames, this->points1, MAX_COUNT, 0.01, 5, Mat(),3, 0, 0.04);
        
        if(this->drawableflag)
        {
            //cout << "--drawing1--" << endl ; 
            drawoptflow(Drawframe);    
        }

        this->needToInit = false;
    } 

    else if (!(this->points2.empty()))
    {
    	calcOpticalFlowPyrLK(this->prevGrayFrame, this->grayFrames, this->points2, this->points1,this->status,
                             this->err, this->winSize, 3, this->termcrit, 0, 0.001);
        if(this->drawableflag)
        {
            //cout << "--drawing2--" << endl ; 
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
        std::vector<uchar> Ransac_mask;
        //this->H = findHomography(this->points2,this->points1,CV_RANSAC,3,cv::noArray() );
        this->H = findHomography(this->points2,this->points1,CV_RANSAC,3,Ransac_mask );
        //cout << "-----------Homography Matrix:H----------" << endl;
        //this->printcvMat(H);

        /*-----Ransac mask----*/
        for (int j = 0; j < this->points2.size(); j++ ) 
        {
          if( !(Ransac_mask[j]) )
            {
              this->points1[j].x = 0.0;
                this->points1[j].y = 0.0;
                this->points2[j].x = 0.0;
                this->points2[j].y = 0.0;
            }
        }
                  
        vector<Point2f>::iterator newIter3 = std::remove_if( this->points1.begin() , this->points1.end() , std::bind(&VisualOdometry::iszero,this, std::placeholders::_1 ));
        vector<Point2f>::iterator newIter4 = std::remove_if( this->points2.begin() , this->points2.end() , std::bind(&VisualOdometry::iszero,this, std::placeholders::_1 ));
        this->points1.resize( newIter3 -  this->points1.begin() );
        this->points2.resize( newIter4 -  this->points2.begin() );
        
        //this->ransac_ptsold = this->points2;
        //this->ransac_ptsnew = this->points1;

        //Random 12 pts and put in vector
       
        for (int k=0 ; k < 12 ; k++)
        {
          
          int ran_buff = rand() % this->points1.size();
          /*this->ransac_ptsold.at<double>(2*k,0) = this->points2[ran_buff].x;
          this->ransac_ptsnew.at<double>(2*k,0) = this->points1[ran_buff].x; 

          this->ransac_ptsold.at<double>(2*k+1,0) = this->points2[ran_buff].y;
          this->ransac_ptsnew.at<double>(2*k+1,0) = this->points1[ran_buff].y;*/
          cv::Point2f buff;
          buff.x = this->points2[ran_buff].x;
          buff.y = this->points2[ran_buff].y;
          this->ransac_ptsold.push_back(buff);
          //this->ransac_ptsold[k].x = this->points2[ran_buff].x;
          //this->ransac_ptsnew.at<double>(2*k,0) = this->points1[ran_buff].x; 

          //this->ransac_ptsold[k].y = this->points2[ran_buff].y;
          //this->ransac_ptsnew.at<double>(2*k+1,0) = this->points1[ran_buff].y;
        }
       // cout << "RANSAC SELECTED FEATURE" << endl;
        //this->printcvMat(this->ransac_ptsold);
       
        cv::Mat covHomography;
        this->estimate_homographycovariancematrix(covHomography);
        
        this->decomp_check = this->homodecomp_compute(this->H,this->R,this->t,this->n); 

        Mat Ju,Jd,Jv,covU,covV,covD;
        this->estimate_svdelementjacobian(this->D,Ju,Jd,Jv);
        //--------Forward Error Propagation---------------
        covU = Ju*covHomography*Ju.t();
        covV = Jv*covHomography*Jv.t();
        covD = Jd*covHomography*Jd.t();
        
        //covU = covU.mul(this->eye9);
        //covV = covV.mul(this->eye9);
        //covD = covD.mul(this->eye9);

        /*cout << "cov U : " << endl;
        printcvMat(covU);
            cout << "cov V : " << endl;
        printcvMat(covV);
            cout << "cov D : " << endl;
        printcvMat(covD);*/

        Mat covUVD(21,21,CV_64F,Scalar(0.0));
        
        //covU.copyTo(covUVD);
        for(int i = 0;i < 9;i++)
        {
          for(int j=0;j < 9;j++)
          {
            
              covUVD.at<double>(i,j) = covU.at<double>(i,j);
              //cout << "element " << i << ',' << j << " = " << covV.at<double>(i,j) << endl;
            
          }
        }

        for(int i = 9;i < 21;i++)
        {
          for(int j=9;j < 21;j++)
          {
            if((i<=17) && (j <=17))
            {
              covUVD.at<double>(i,j) = covV.at<double>(i-9,j-9);
              //cout << "element " << i << ',' << j << " = " << covV.at<double>(i-9,j-9) << endl;
            }
          }
        }

        for(int i = 18;i < 21;i++)
        {
          for(int j=18;j < 21;j++)
          {
            if((i>=18) && (j >=18))
            {
              covUVD.at<double>(i,j) = covD.at<double>((i-18)*4,(j-18)*4);
              //cout << "element " << i << ',' << j << " = " << covV.at<double>(i-9,j-9) << endl;
            }
          }
        }
       /* cout << "cov UVD : " << endl;
        printcvMat(covUVD);*/

        
        find_translationCov(covUVD,Covt);

        // t Covariance -> find Jt(3x9) = dt/d(u,v,lambda) first and Forward propagate  covt = Jt*covUVD*Jt^t
    
        find_yawcovariance(covUVD,covYaw);
          cout << "Cov Yaw :: " << covYaw << endl;
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