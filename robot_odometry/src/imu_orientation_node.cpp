#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include <vectornav/sensors.h>
#include <robot_odometry/BiasImu.h>
#include <cstdlib>
#include <math.h>
#include <Movingavg.h>
#include <fstream>
#include <iostream>


using namespace std;
using namespace ros;

#define PI 3.1416



#define DEFAULT_LOOP_RATE 20
#define COMPLEMENTARY_GAIN 0.90 
#define DEFAULT_BIAS_ANGLE_YAW 0
#define DEFAULT_IMU_TOPIC "/vectornav/imu"

//double dt = (1.00/ROS_LOOP_RATE)/1000.0;



class ImuOrientation
{
  private:
          double roll;
          double pitch;
          double yaw;
          double yawbias;
          double biasangle_radian;
          double commGain;
          double dt;
          double current_yaw;
          double last_yaw;
          bool firstdata;
          double yawabs;
          Movingavg moveavgyaw;
          ros::NodeHandle n;
          ros::Publisher quat_pub;
          geometry_msgs::Quaternion quatmsg;
          
          ros::Time current;
          ros::Time lasttime;
          ros::Subscriber sub;
          ros::ServiceServer service;


  public: 
          ros::Rate loop_rate;
          ofstream ret;
          ImuOrientation()
            :n("~"),
            loop_rate(DEFAULT_LOOP_RATE),
            moveavgyaw(10)
            {
              this->sub = n.subscribe(DEFAULT_IMU_TOPIC, 1, &ImuOrientation::imuCallback,this);
              this->quat_pub = n.advertise<geometry_msgs::Quaternion>("/imu/orientation", 50);
              this->service = n.advertiseService("/robot_odometry/bias_imu_degree", &ImuOrientation::imubiassrvCallback,this);
              this->current = ros::Time::now();
              this->lasttime = ros::Time::now();
              this->firstdata = true;
             // this->ret.open("~/imu1.txt",ios::app);
       
            }

          ~ImuOrientation(){}

          bool imubiassrvCallback(robot_odometry::BiasImu::Request &req,robot_odometry::BiasImu::Response &res)
          {
              this->biasangle_radian = (double)req.bias*(PI/180.00); 
     
              std::string stat;
              std::stringstream ss;

              ss << "Current Yaw Bias Degree : " << this->biasangle_radian*180.00/PI << std::endl;
              stat = ss.str();
              
              res.status = stat;
              return true;
          }

          void imuCallback(const vectornav::sensors::ConstPtr& msg)
          {
            this->current = ros::Time::now();
            this->dt = this->current.toSec() - this->lasttime.toSec();
            this->roll = (this->commGain * (this->roll + msg->Gyro.x * this->dt)  + (1-this->commGain)  * atan2(msg->Accel.z,msg->Accel.y));
            this->pitch = (this->commGain * (this->pitch + msg->Gyro.y * this->dt)  + (1-this->commGain)  * atan2(msg->Accel.z,msg->Accel.x));
            this->yaw = (this->commGain * (this->yaw + msg->Gyro.z * this->dt)  + (1-this->commGain)  * atan2(msg->Accel.y,msg->Accel.x));
           
            
            double outyaw;
            bool isfull = this->moveavgyaw.compute(this->yaw,outyaw);

            //if(isfull)
            //{  
                this->yawbias = outyaw + this->biasangle_radian;
                
                /*ROS_INFO("acc value x : [%f] \n" ,msg->Accel.x);
                ROS_INFO("acc value y : [%f] \n" ,msg->Accel.y);
                ROS_INFO("acc value z : [%f] \n" ,msg->Accel.z);

                ROS_INFO("gyro value x : [%f] \n" ,msg->Gyro.x);
                ROS_INFO("gyro value y : [%f] \n" ,msg->Gyro.y);
                ROS_INFO("gyro value z : [%f] \n" ,msg->Gyro.z);*/
                //ROS_INFO("roll : [%lf] \n" ,(this->roll*180.00/PI) );
                //ROS_INFO("pitch : [%lf] \n" ,(this->pitch*180.00/PI));
                //ROS_INFO("yaw : [%lf] \n" ,(outyaw*180.00/PI));
                //ROS_INFO("yawbias : [%lf] \n" ,(this->yawbias*180.00/PI));

                if(this->firstdata)
                {
                    this->current_yaw = this->yawbias;
                    this->firstdata = false;
                }
                else
                {
                  this->current_yaw = this->yawbias;
                  //this->yawabs +=   this->current_yaw - this->last_yaw;
                  //this->ret << (double)this->yaw<< std::endl;
                  //ROS_INFO("current acc delta_yaw : [%lf]",this->yaw*180.00/PI);
                  cout << this->yaw*180.00/PI << std::endl;
                }
               

                this->quatmsg = tf::createQuaternionMsgFromRollPitchYaw(this->roll,this->pitch,this->yawbias);
                this->quat_pub.publish(this->quatmsg);
                this->last_yaw = this->current_yaw;
                this->lasttime = this->current;
            //}
          }



};




int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu");
  ImuOrientation imuo;
  /*while(ros::ok())
  {
    ros::spinOnce();
    imuo.loop_rate.sleep();
   // ROS_INFO("Test");
  }*/
  ros::spin();
  imuo.ret.close();
  return 0;
}