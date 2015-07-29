#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "custom_msgs/DistanceCovStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "boost/thread.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include <ros/package.h>
#define PI 3.14159
#define DEFAULT_LOOP_RATE 4  //VISUAL ODOMETRY NODE Run-Rate in Hertz

//#define START_ANGLE PI/2.0000// Heading on Y-AXIS
#define START_ANGLE PI
#define DEFAULT_FRAME_HEIGHT 0.6
using namespace std;

class RobotRunner{

  private:
          //World Position
          double x;
          double y;
          double th;

          double vx;
          double vy;
          double vth;

          double dx;
          double dy;
          double dth;
          double dt;
          double frame_height;
          bool firstmsgs;
          bool firstimudata;
          ros::Time laststamp,currentstamp;
          ros::Time timeoutreset;

          geometry_msgs::Vector3 lastrpy; // Remember laststate of imu roll pitch yaw
          geometry_msgs::Vector3 currentrpy;
          int i;

          ros::NodeHandle n;
          ros::Publisher odom_pub;
          ros::Publisher comparepose_pub;
          tf::TransformBroadcaster tf_broadcaster;
        
          ros::Subscriber sub;
          ros::Subscriber imusub; 

          geometry_msgs::Quaternion odom_quat;
          
          ros::Time current_time;
          boost::thread tfpub_thread;



  public: 
          ofstream ret;
          

          RobotRunner()
            :n("~")
          {
              this->laststamp = ros::Time::now(); 
              this->currentstamp = ros::Time::now();
              this->odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

              this->comparepose_pub = n.advertise<geometry_msgs::Vector3Stamped>("compare_pose", 10);
             // this->imupose_pub = n.advertise<geometry_msgs::Vector3Stamped>("compare_pose", 10);  
              //this->sub = n.subscribe("/visual/twistcov", 20, &RobotRunner::twistcovCallback,this);
              this->sub = n.subscribe("/visual/distcov", 20, &RobotRunner::distcovCallback,this);
              //this->imusub = n.subscribe("/vectornav/rpy", 1, &RobotRunner::imuCallback,this);
              this->firstimudata = true;
              //this->tfpub_thread = boost::thread(boost::bind(&RobotRunner::tfpubthread,this));
              
              this->th = START_ANGLE; //set start angle at y 
              this->currentrpy.z = -START_ANGLE; //set start angle at y
              this->n.param( "frame_height", this->frame_height, DEFAULT_FRAME_HEIGHT);
              ROS_INFO( "frame_height: %lf", this->frame_height );

              std::string filename = ros::package::getPath("robot_odometry") + "/outodom.txt";
              ret.open(filename.c_str());

              //std::string filename2 = ros::package::getPath("robot_odometry") + "/imucompare.txt";
              //retimu.open(filename2.c_str());
      
          }
          ~RobotRunner(){}



          void distcovCallback(const custom_msgs::DistanceCovStampedConstPtr& msg)
          {
            /*msg -> element , */
            custom_msgs::DistanceCovStamped distcov_buf;
            
            this->dx = msg->translation.x;
            this->dy = msg->translation.y;
            this->dth = msg->angular.z;

            this->currentstamp = msg->header.stamp;
            this->dt = msg->deltatime.toSec();

            /*Change to Robot Transform*/  
            this->dx = this->dx;
            this->dy = -1.0*this->dy;
            this->dth = -1.0*this->dth;
            



            this->th += this->dth;

            /*Relative path method*/
            //change baselink to odometry frame with out using tf listener 
            //this->x = this->x + (this->dy*cos(this->th) + this->dx*sin(this->th));
            //this->y = this->y + (this->dy*sin(this->th) + this->dx*cos(this->th));
            this->x = this->x + -(this->dy*sin(this->th) + this->dx*cos(this->th));
            this->y = this->y + -(this->dy*cos(this->th) + this->dx*sin(this->th));
            /*
            this->x += this->dx;
            this->y += this->dy;
            */

            this->odom_quat = tf::createQuaternionMsgFromYaw(this->th);
            this->current_time = ros::Time::now();



            //next, we'll publish the odometry message over ROS
            nav_msgs::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";

            //set the position
            odom.pose.pose.position.x = this->x;
            odom.pose.pose.position.y = this->y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = this->odom_quat;

            //set the velocity
            
            odom.twist.twist.linear.x = this->dx/this->dt;
            odom.twist.twist.linear.y = this->dy/this->dt;
            odom.twist.twist.angular.z = this->dth/this->dt;

            //std::cout << "vy after = " << this->vy << std::endl;
            
            //this->ret << (double)this->x << "," << (double)this->y << "," << (double)this->th << "," << (double)this->dx  << "," << (double)this->dy << "," << (double)this->dth << "," << (double)this->currentrpy.z << std::endl;
            
            //publish the message
            odom_pub.publish(odom);
            this->currentstamp = this->laststamp;
         }

         void imuCallback(const geometry_msgs::Vector3ConstPtr& msg)
         {
            double rollupdate;
            double pitchupdate;
            double yawupdate;
            if(this->firstimudata)
            {
              this->firstimudata =false;
              this->lastrpy.x = msg->x;
              this->lastrpy.y = msg->y;
              this->lastrpy.z = msg->z;
              return;
            }
            if( ((this->lastrpy.z > 2.967) && (this->lastrpy.z < 3.1416)) && ((msg->z < -2.967) && (msg->z > -3.1416))      )
            {
              yawupdate = (3.1416 - this->lastrpy.z) + (msg->z + 3.1416);
             // cout << "updatecase 1 " << endl;
            }
            else if(   ((this->lastrpy.z < -2.967) && (this->lastrpy.z > -3.1416)) && ((msg->z > 2.967) && (msg->z < 3.1416))   )
            {
              yawupdate = (-3.1416 - this->lastrpy.z) + (msg->z - 3.1416);
              //cout << "updatecase 2 " << endl;
            }
            else
            {
              yawupdate = msg->z - this->lastrpy.z;
              //cout << "updatecase normal:" << endl;
            }
            
            rollupdate = msg->x - this->lastrpy.x;
            pitchupdate = msg->y - this->lastrpy.y;

            /*double rollupdate = msg->x - this->lastrpy.x;
            double pitchupdate = msg->y - this->lastrpy.y;
            double yawupdate = msg->z - this->lastrpy.z;*/

            this->currentrpy.x += rollupdate;
            this->currentrpy.y += pitchupdate;
            this->currentrpy.z += yawupdate;

            //Realtime compare with VO evaluation
            geometry_msgs::Vector3Stamped cmpmsg;
            
            cmpmsg.header.frame_id = "base_link";
            cmpmsg.header.stamp = ros::Time::now();
            cmpmsg.vector.x = -this->th;
            cmpmsg.vector.y = this->currentrpy.z;
            cmpmsg.vector.z = -START_ANGLE; // reference 0 for realtime plotting
            this->comparepose_pub.publish(cmpmsg); 

            //this->retimu << (double)this->th << "," << (double)this->currentrpy.z << std::endl;
            
            this->lastrpy.x = msg->x;
            this->lastrpy.y = msg->y;
            this->lastrpy.z = msg->z;

         }

          void tfpubthread()
          {
            static double tfpubrate = 1.0/10.0; //20Hz
            while(ros::ok())
            {
              //Baselink and camera_link
              tf::Transform transform;
              transform.setOrigin( tf::Vector3(0.0, 0.0, this->frame_height) );
              tf::Quaternion q;
              q.setRPY(PI, 0, 0);
              //q.setRPY(0, 0, 0);
              transform.setRotation(q);
              
              tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
              
              //---------IMULINK ---------------
              tf::Transform imutf;
              imutf.setOrigin( tf::Vector3(0.0, 0.0, 0.08) );
              tf::Quaternion qimu;
              qimu.setRPY(PI, 0, 0);
              imutf.setRotation(qimu);
              tf_broadcaster.sendTransform(tf::StampedTransform(imutf, ros::Time::now(), "camera_link", "imu_link"));



              //---------ODOMLINK ---------------
              tf::Transform odomtf;
              odomtf.setOrigin( tf::Vector3(this->x,  this->y, 0.0) );
              tf::Quaternion quatodom;
              quatodom.setRPY(0, 0, this->th);
              odomtf.setRotation(quatodom);
              tf_broadcaster.sendTransform(tf::StampedTransform(odomtf, ros::Time::now(), "odom", "base_link"));
              
             /* //first, we'll publish the transform over tf
              geometry_msgs::TransformStamped odom_trans;
              odom_trans.header.stamp = current_time;
              odom_trans.header.frame_id = "odom";
              odom_trans.child_frame_id = "base_link";

              odom_trans.transform.translation.x = this->x;
              odom_trans.transform.translation.y = this->y;
              odom_trans.transform.translation.z = 0.0;
              odom_trans.transform.rotation = odom_quat;

              //send the transform
              tf_broadcaster.sendTransform(odom_trans);*/
              boost::this_thread::sleep(boost::posix_time::milliseconds(tfpubrate));
            }

          }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "~");
  
  RobotRunner runner;
  ROS_INFO("---Start Odometry Publisher Node---");
  ros::spin();
  runner.ret.close();
  //runner.retimu.close();

}
