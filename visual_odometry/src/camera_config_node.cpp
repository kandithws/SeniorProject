/*
 * camera_config_node.cpp
 *
 *  Created on: Mar 23, 2015
 *      Author: Kandith
 */

#include <cstdlib>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#define EXPOSURE_ADJUST 5
#define GAIN_ADJUST 1

/* Default values */
#define DEFAULT_DEVICE_ID "1"

/*
 * Configurations for Logitech C920
 *  - exposure [0,10000]
 *  - gain [0,255]
 *  - brightness/contrast/saturation [0,255]
 */

#define DEFAULT_MAX_EXPOSURE 500 //2047
#define DEFAULT_MIN_EXPOSURE 3
#define DEFAULT_MAX_GAIN 200 //255
#define DEFAULT_MIN_GAIN 0
#define DEFAULT_BRIGHTNESS 128
#define DEFAULT_CONTRAST 128
#define DEFAULT_SATURATION 128
#define DEFAULT_THRESHOLD_MIN 100
#define DEFAULT_THRESHOLD_MAX 200


class CameraAdjuster {

private:
	int exposure;
	int gain;
	int max_exposure;
	int min_exposure;
	int max_gain;
	int min_gain;
	int threshold_max;
	int threshold_min;
    int counter;
	char command_str[500];
	std::string device;

	ros::NodeHandle nh;


public:
	CameraAdjuster()
		: nh( "~" )
	{
        this->counter = 0;
		FILE *ret;

		/******* Get node's parameters ********/
		/* device */
		nh.param<std::string>( "device", this->device, DEFAULT_DEVICE_ID );
		ROS_INFO( "device: %s", this->device.c_str() );



		/* max_exp min_exp */
		nh.param( "max_exp", this->max_exposure, DEFAULT_MAX_EXPOSURE );
		ROS_INFO( "max_exp: %d", this->max_exposure );
		nh.param( "min_exp", this->min_exposure, DEFAULT_MIN_EXPOSURE );
		ROS_INFO( "min_exp: %d", this->min_exposure );

		/* max_gain min_gain */
		nh.param( "max_gain", this->max_gain, DEFAULT_MAX_GAIN );
		ROS_INFO( "max_gain: %d", this->max_gain );
		nh.param( "min_gain", this->min_gain, DEFAULT_MIN_GAIN );
		ROS_INFO( "min_gain: %d", this->min_gain );

		/* brightness contrast saturation */
		int brightness;
		int contrast;
		int saturation;
		nh.param( "brightness", brightness, DEFAULT_BRIGHTNESS );
		ROS_INFO( "brightness: %d", brightness );
		nh.param( "contrast", contrast, DEFAULT_CONTRAST );
		ROS_INFO( "contrast: %d", contrast );
		nh.param( "saturation", saturation, DEFAULT_SATURATION );
		ROS_INFO( "saturation: %d", saturation );

		/* threshold_min threshold_max */
		nh.param( "threshold_min", this->threshold_min, DEFAULT_THRESHOLD_MIN );
		ROS_INFO( "threshold_min: %d", this->threshold_min );
		nh.param( "threshold_max", this->threshold_max, DEFAULT_THRESHOLD_MAX );
		ROS_INFO( "threshold_max: %d", this->threshold_max );

		/* Disable auto focus Set powerlinefrequency*/
		std::cout << "Disable Auto-focus && SetPowerLine frequency @50Hz"<< std::endl;
		
		/*sprintf( this->command_str, "v4l2-ctl --device=%s --set-ctrl=white_balance_temperature_auto=0 --set-ctrl=exposure_auto=0 --set-ctrl=focus_auto=0",
				this->device.c_str() );*/

		sprintf( this->command_str, "v4l2-ctl --device=%s --set-ctrl=focus_auto=0 --set-ctrl=power_line_frequency=1",this->device.c_str() );
		

		if(system( this->command_str ) == 0 )
			std::cout << "OK" << std::endl;
		else
			std::cout << "Failed" << std::endl;

			

		/* Disable auto_exposure/auto_white_balance */
		std::cout << "Disable auto_exposure/auto_white_balance"<< std::endl;
		sprintf( this->command_str, "v4l2-ctl --device=%s --set-ctrl=white_balance_temperature_auto=0 --set-ctrl=exposure_auto=1",this->device.c_str() );
		

		if(system( this->command_str ) == 0 )
			std::cout << "OK-Exposure" << std::endl;
		else
			std::cout << "Failed-Exposure" << std::endl;

		/* Get current gain */
		sprintf( this->command_str, "v4l2-ctl --device=%s --get-ctrl=gain", this->device.c_str() );
		ret = popen( this->command_str, "r" );
		if( ret ) {
			fscanf( ret, "%s %d", this->command_str, &(this->gain) );
			ROS_INFO( "Current gain = %d", this->gain );
		}

		/* Get current exposure_absolute */
		sprintf( this->command_str, "v4l2-ctl --device=%s --get-ctrl=exposure_absolute", this->device.c_str() );
		ret = popen( this->command_str, "r" );
		if( ret ) {
			fscanf( ret, "%s %d", this->command_str, &(this->exposure) );
			ROS_INFO( "Current exposure = %d", this->exposure );
		}

		/*Get current power line freq*/
		sprintf( this->command_str, "v4l2-ctl --device=%s --get-ctrl=power_line_frequency", this->device.c_str() );
		ret = popen( this->command_str, "r" );
		if( ret ) {
			int powerlinefreq = 1;
			fscanf( ret, "%s %d", this->command_str, &(powerlinefreq) );
			ROS_INFO( "Current powerlinefrequency = %d", powerlinefreq );
		}

		/*Get current Focus*/
		sprintf( this->command_str, "v4l2-ctl --device=%s --get-ctrl=focus_auto", this->device.c_str() );
		ret = popen( this->command_str, "r" );
		if( ret ) {
			int focus_auto = 0;
			fscanf( ret, "%s %d", this->command_str, &(focus_auto) );
			ROS_INFO( "Current Focus = %d", focus_auto );
		}




		/* Set brightness/contrast/saturation */
		std::cout << "Set Brightness/Contrast/Saturation... ";
		sprintf( this->command_str, "v4l2-ctl --device=%s --set-ctrl=brightness=%d --set-ctrl=contrast=%d --set-ctrl=saturation=%d",
				this->device.c_str(), brightness, contrast, saturation );
		if( 0 == system( this->command_str ) )
			std::cout << "OK" << std::endl;
		else
			std::cout << "Failed" << std::endl;



		std::cout << "Finished Initializing Camera" << std::endl;

		/**/
	}

	~CameraAdjuster(){}
};

int main( int argc, char **argv )
{
	ros::init( argc, argv, "camera_adjuster" );
	CameraAdjuster adjuster;
	ros::spinOnce();
	return 0;
}
