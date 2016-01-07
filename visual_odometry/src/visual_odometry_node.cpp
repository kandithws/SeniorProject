#include <cstdlib>
#include "ros/ros.h"
#include <vector>
#include <image_transport/image_transport.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <ctype.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <fstream>

#include "boost/thread.hpp"
#include "VisualOdometry.h"
#include "visual_odometry/DistanceCovStamped.h"
#include "geometry_msgs/QuaternionStamped.h"

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h> 

#include <ros/package.h>

#define DEFAULT_IMAGE_TOPIC "/image_raw"
#define DEFAULT_OUTPUT_TOPIC "distcov"

#define MAX_COUNT 500
#define DEFAULT_LOOP_RATE 10  //ROS Run-Rate in Hertz

#define FIXED_CAM_DEPTH 5.0
#define TRANSLATION_BIAS 0.0754

using namespace std;
using namespace cv;
using namespace boost;

double camera_intrinsic_parameter[3][3] = {{623.911529 , 0.000000  , 315.403612},
                                           {0.000000   , 622.391874, 249.380732},
                                           {0.000000   , 0.000000  , 1.000000  }   };


class OdometryRunner {
    private :
        ros::NodeHandle nh;
        image_transport::ImageTransport it;
        image_transport::Subscriber image_sub;
        
        //ros::Publisher twistcov_pub;
        ros::Publisher distcov_pub;
        //geometry_msgs::TwistWithCovarianceStamped twistcov; // THIS IS OUR NODE MAIN MSG
        visual_odometry::DistanceCovStamped distcov;

        ros::Time current_time, last_time;
        
        double depth;
        double translationbias;
        double rate;

        double Covyaw;
        cv::Mat CovT;
        
        
        cv_bridge::CvImagePtr cv_in_ptr;
        std::string img_topic;
        std::string rawWindow;
        boost::thread ui_thread;
        VisualOdometry vid_odo;
        cv::Mat R,t,n;
        cv::Mat Intrinsic;
        cv::Mat inimg;
        cv::Mat uiimg;
        ros::Rate loop_rate;

        bool is_first_frame;

    public  : 
        OdometryRunner()
            : nh("~"),
              it(nh),
              Intrinsic(3, 3, CV_64F, camera_intrinsic_parameter),
              loop_rate(DEFAULT_LOOP_RATE)
        {
            std::string filename = ros::package::getPath("visual_odometry") + "/outcov.txt";
            this->outfile.open(filename.c_str());
            
            /* Subscribe to image topic */
                
            this->img_topic = DEFAULT_IMAGE_TOPIC ;
            this->image_sub = it.subscribe( img_topic, 1, &OdometryRunner::imageCallback, this );

            //this->imu_sub = nh.subscribe(DEFAULT_IMU_TOPIC,1,&OdometryRunner::imuCallback,this);
            
            this->vid_odo   = VisualOdometry(Intrinsic,DRAW_ENABLE);
            
            ui_thread = thread(bind(&OdometryRunner::uiThread,this));

            this->nh.param("bias", this->translationbias, TRANSLATION_BIAS);
            
            ROS_INFO( "translation bias percentage: %lf", this->translationbias );
            

            /*Odometry Publisher Initialize*/
            //this->twistcov_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>(DEFAULT_OUTPUT_TOPIC, 50);
            this->distcov_pub = nh.advertise<visual_odometry::DistanceCovStamped>(DEFAULT_OUTPUT_TOPIC, 50);
            this->current_time = ros::Time::now();
            this->last_time = ros::Time::now();

            /*OpenCV UI Window Initalization*/      
            rawWindow = "Visual Odometry Frame output";
            namedWindow(rawWindow, CV_WINDOW_AUTOSIZE);

            /*DEPTH and BIAS is FIXED*/

            this->depth = FIXED_CAM_DEPTH; //60cm
            //this->translationbias = TRANSLATION_BIAS; // %15.8705 bias value of translation vector
            
            /*Need to publish the first frame time to the system*/
            this->is_first_frame = true;
        }

        ~OdometryRunner(){}
        
        ofstream outfile;
        

    void imageCallback( const sensor_msgs::ImageConstPtr& msg )
    {
        
        /*------START IMAGE PROCESSING HERE---------*/
       
        try {
                this->cv_in_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
                this->inimg = this->cv_in_ptr->image;
                
                this->uiimg = this->inimg;
                
                
                //this->vid_odo.compute_odometry_lkoptflow(this->inimg,this->uiimg,this->R,this->t,this->n);
                this->vid_odo.compute_odometry_lkoptflowCov(this->inimg,this->uiimg,this->R,this->t,this->n,CovT,Covyaw);
                //this->outfile << "Test" <<endl;
                /*
                if(!CovT.empty())
                {
                    //writing output to file

                    this->outfile << CovT.at<double>(0,0) << "," << CovT.at<double>(1,1) << "," << CovT.at<double>(2,2) << ","<< Covyaw << std::endl;  
                }
                */
                
                ROS_INFO("----IMG Process OKAY----");
                
           
                if(this->is_first_frame)
                {
                    /*---- Publish time and zero velocity to initiate odom system ----*/
                    ROS_INFO("--Publishing Start MSG--");
                    this->current_time = ros::Time::now();
                    /*twistcov.header.stamp = this->current_time;
                    twistcov.twist.twist.linear.x = 0;
                    twistcov.twist.twist.linear.y = 0;
                    twistcov.twist.twist.linear.z = 0;


                    twistcov.twist.twist.angular.x = 0;
                    twistcov.twist.twist.angular.y = 0;
                    twistcov.twist.twist.angular.z = 0;

                    twistcov_pub.publish(twistcov);*/
                    this->distcov.header.stamp = this->current_time;
                    this->distcov.header.frame_id = msg->header.frame_id;
                    this->distcov.translation.x = 0;
                    this->distcov.translation.y = 0;
                    this->distcov.translation.z = 0;

                    this->distcov.angular.x = 0;
                    this->distcov.angular.y = 0;
                    this->distcov.angular.z = 0;

                    this->distcov_pub.publish(this->distcov);

                    this->last_time = this->current_time;
                    this->is_first_frame = false;
                }


                /*-----------Publishing Odometry MSG-------------*/
                if(! this->t.empty() )    
                {          
                    ROS_INFO("----CALCULATING----");
                    
                    double roll,pitch,yaw;
                    vid_odo.dcm2angle(this->R,roll,pitch,yaw,RADIAN);

                    
                    /*Change output vector to ROS twist velocity*/
                    this->current_time = ros::Time::now();
                    ros::Duration dur = this->current_time - this->last_time;
                    
                    double deltatime = dur.toSec();
                    
                    
                    //deltatime = 1.0/(double)DEFAULT_LOOP_RATE;

                    
                    //cv::Mat output_twist = ((this->depth*this->t)*(1.0 + this->translationbias))/ deltatime;  
                    cv::Mat output_dist = (this->depth*this->t)*(1.0 + this->translationbias); 
                    
                    /*FOR THISCASE we assume that optflow calc fast ->> SO we rexlaxed deltatime to const*/

                    ROS_INFO("---Translation Out----");
                    vid_odo.printcvMat(output_dist);
                    ROS_INFO("Yaw = %lf",yaw);
                    //ROS_INFO("Deltatime in sec : %lf",deltatime);
                    /*twistcov.header.stamp = this->current_time;
                    twistcov.twist.twist.linear.x = output_twist.at<double>(0);
                    twistcov.twist.twist.linear.y = output_twist.at<double>(1);
                    twistcov.twist.twist.linear.z = 0;


                    twistcov.twist.twist.angular.x = 0;
                    twistcov.twist.twist.angular.y = 0;
                    twistcov.twist.twist.angular.z = yaw/deltatime;
                    this->last_time = this->current_time;
                    twistcov_pub.publish(twistcov);*/

                    this->distcov.header.stamp = this->current_time;
                    //this->distcov.deltatime = dur;
                    this->distcov.deltatime.data = dur;
                    this->distcov.translation.x = output_dist.at<double>(0);
                    this->distcov.translation.y = output_dist.at<double>(1);
                    this->distcov.translation.z = 0;

                    this->distcov.angular.x = 0;
                    this->distcov.angular.y = 0;
                    this->distcov.angular.z = yaw;

                    this->distcov_pub.publish(this->distcov);
                    

                    this->last_time = this->current_time;
                }


            }

        catch( cv_bridge::Exception &e ) 
            {
                ROS_ERROR( "cv_bridge exception: %s", e.what() );
                return;
            }     
           
           
     
            this->loop_rate.sleep();
    }

    void imuCallback(const geometry_msgs::QuaternionStampedConstPtr& msg)
    {
        ROS_INFO("-----DUMMY RECIEVE IMU-----");
    }

    void uiThread()
    {
         while(ros::ok())
          {
            if(this->uiimg.empty())
            {
              //ROS_INFO("PROCESSING");
            }
            else
            {
                
               imshow(rawWindow,this->uiimg);
                waitKey(5);
            }   
          }
    }

    
    
 
}; //end class

int main( int argc, char **argv ) {

        ros::init( argc, argv, "gun_optical_flow" );
        OdometryRunner runner;
        ROS_INFO("---Visual Odometry Node---");
        ros::spin();
        runner.outfile.close();
        return 0;
    
} 

