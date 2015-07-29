#include <cstdlib>
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <vector>
#include <sstream>
#include <tf/transform_broadcaster.h>

#include "geometry_msgs/QuaternionStamped.h"
#define PI 3.1415926

using namespace std;
unsigned char calculateChecksum(unsigned char data[], unsigned int length)
{
	unsigned int i;
	unsigned char cksum = 0;
	for(i=0; i<length; i++){
	cksum ^= data[i];
	}
	return cksum;
}

int main( int argc, char **argv ) 
{

    ros::init( argc, argv, "vectornav_driver" );
    ros::NodeHandle n;
    //ros::Publisher imu_pub = n.advertise<geometry_msgs::QuaternionStamped>("/vectornav/rpy", 1);
    ros::Publisher imu_pub = n.advertise<geometry_msgs::Vector3>("/vectornav/rpy", 1);
    ros::Rate loop_rate(50);

    ROS_INFO("-------VECTORNAV_DRIVER----------");
    
    fstream serialport;
	
	std::vector<string> recievecontent;
   system("stty -F /dev/ttyUSB0 sane raw pass8 -echo -hupcl clocal 115200");
   std::this_thread::sleep_for(std::chrono::milliseconds(500));
   serialport.open("/dev/ttyUSB0",ios::out | ios::in);

   double yaw,pitch,roll;


    while (ros::ok())
 	{

	 	/*if(serialport)
		{
			string command("VNWRG,75,2,16,01,0008");
				
			unsigned char cstr_cmd[] = "VNWRG,75,2,16,01,0008";
				 	
					
			unsigned char chksum = calculateChecksum(cstr_cmd,command.size());
			char chkstr[2];
			//itoa(chksum,chkstr,16);
			sprintf(chkstr, "%x", chksum);

			serialport << '$' << command << '*' << chkstr <<std::endl;
			std::cout << "Send :" << '$' << command << '*' << chkstr << std::endl;
		}*/
	
	
	
		if(serialport)
		{
			string sbuf;
			getline( serialport, sbuf );

			if(sbuf.empty())
			{

			}
			else
			{
				//std::cout << "Read :" << sbuf << std::endl;
				if(sbuf[0] == '$')
				{
					std::stringstream ss(sbuf);
					//ss.str();
					string buffcontent;

					while(getline(ss,buffcontent,','))
					{
						recievecontent.push_back(buffcontent);
					}
		
					// [1] = Yaw deg , [2] = pitchdeg , [3] = rolldeg
					if(recievecontent[0] == "$VNYMR")
					{
						yaw = atof(recievecontent[1].c_str());
						pitch = atof(recievecontent[2].c_str()); 
						roll = atof(recievecontent[3].c_str());
						//cout << "ANGLE YPR: " << yaw << "/" << pitch << "/" << roll << endl; 
					}

					recievecontent.clear();
				}
				else
				{
					std::cout << "Drop this : " << sbuf << std::endl;
				}	
		
			}
		}

				
		

    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    	//geometry_msgs::QuaternionStamped msg;
    	double rollrad,pitchrad,yawrad;
    	rollrad = roll*PI/180.00;
    	pitchrad = pitch*PI/180.00;
    	yawrad = yaw*PI/180.00;

    	cout << "ANGLE YPR: " << yawrad << "/" << pitchrad << "/" << rollrad << endl; 
		//geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromRollPitchYaw(rollrad ,pitchrad,yawrad);
		//msg.header.stamp = ros::Time::now();
		//msg.header.frame_id = "imu_link";
     	//msg.quaternion = quat;
    	geometry_msgs::Vector3 msg;
    	msg.x = rollrad;
    	msg.y = pitchrad;
    	msg.z = yawrad;

    	imu_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
    }
    serialport.close(); 
    return 0;
   
} 


