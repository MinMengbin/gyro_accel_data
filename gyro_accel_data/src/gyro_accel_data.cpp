/*
 * gyro_accel_data.cpp
 *
 *  Created on: Aug 22, 2017
 *      Author: aspa1
 */

#include "ros/ros.h"

#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"  //Unsigned int 32 data type
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include <iostream>

#include <math.h>
#include "string.h"
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#define BUFFER_SIZE 23   // The data size received from UM7 are 23.

// How many times we will try a blocking read of the
// device before we call it a failed operation.
const int MAX_READ_TRIES = 5;

// Default port. Assumes one UM7 plugged in
// and no other USB devices posing as serial
// devices.
#define SERIAL_PORT "/dev/ttyUSB0"


//the function used to read buffer from UM7
void readUM7Response(int UM7, unsigned char* response, int responseSize)
{
    int i = 0;
    int ioResult;
    do
    {
        ioResult = read(UM7, response, responseSize);
        i++;
    } while (ioResult < 0 && errno == EAGAIN && i < MAX_READ_TRIES);

    if (i == MAX_READ_TRIES)
    {
        ROS_INFO("Error reading from QSB device");
    }
    response[23] = '\0';
}


struct um7data {
   unsigned int SS[3]; // 's''n''p'
   unsigned int PT; // one byte
   unsigned int	Address; // one byte
   unsigned int Data[16]; // 16 bytes
   unsigned int Checksum[2]; // z bytes
};


int main (int argc, char *argv[])
{
    // Register the Ctrl-C handler.
    //signal(SIGINT, ctrlCHandler);

    unsigned char response[BUFFER_SIZE]; // the buffer size for the input data
    unsigned char response_pack[24]; // the buffer size for the input data
	// Data type of the published
    std_msgs::Float32 gyro_accel;
    gyro_accel.data = 0.0;

    sensor_msgs::Imu imu_msg;
	int UM7; // serial handle for UM7

	//set up ros
	ros::init(argc, argv, "gyro_accel");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<sensor_msgs::Imu>("data", 1000);
	ros::Rate loop_rate(100);

	// Open the serial port.
	ROS_INFO("Trying to open a serial port.");
	UM7 = open(SERIAL_PORT, O_RDWR);// | O_NOCTTY | O_NDELAY | ICANON | O_NONBLOCK
	if (UM7 < 0)
	{
		ROS_INFO("Fail to open UM7 devices, please check that UM7 is connected or not.");
		return 0;
	}
    ROS_INFO("Serial ports are connected.");
    //Open the serial port.

	// Configure serial communication.
	struct termios UM7Configuration;
	tcgetattr(UM7, &UM7Configuration);

	UM7Configuration.c_cflag = B115200 | CS8;
	UM7Configuration.c_lflag = 0; //non-canonical, no echo
	UM7Configuration.c_cc[VTIME] = 0; //inter-character timer unused

    // The data size received from UM7 are 23.
	UM7Configuration.c_cc[VMIN] = 23; //blocking read until 23 chars received
	cfsetospeed(&UM7Configuration, B115200);
	tcflush(UM7, TCIFLUSH);
	tcsetattr(UM7, TCSANOW, &UM7Configuration);
	// Configure serial communication.

    //std::stringstream ss_gyro_x;
    //std::string s_gyro_x;
	// define the 4 bytes gyro (x y z) data
/*    char gyro_x[4] =  {'0','0','0','0'};
    char gyro_y[4] =  {'0','0','0','0'};
    char gyro_z[4] =  {'0','0','0','0'};*/
    unsigned char gyro[4] =  {'0','0','0','0'};
    //unsigned char gyro_y[4] =  {'0','0','0','0'};
    //unsigned char gyro_z[4] =  {'0','0','0','0'};

    //std::stringstream ss_accel_x;
    //std::string s_accel_x;
    char converted[4*2 + 1];
    // define the 4 bytes accel (x y z) data
/*    char accel_x[4] =  {'0','0','0','0'};
    char accel_y[4] =  {'0','0','0','0'};
    char accel_z[4] =  {'0','0','0','0'};*/
    unsigned char accel[4] =  {'0','0','0','0'};
    //unsigned char accel_y[4] =  {'0','0','0','0'};
    //unsigned char accel_z[4] =  {'0','0','0','0'};

   // Read input and publish the data.
   while ( ros::ok())
    {
		 // Read steam data.
		 readUM7Response(UM7, response, BUFFER_SIZE);
		 //ROS_INFO("Response is : %s", response);
		 //ROS_INFO("Response[0] is : %d", response[0]);
		 if (response[0] != 's' ) {
			 //test for display all the input
/*			 for (int k = 0; k < 23; k++){
				 ROS_INFO("Response[%d] is in hex : %x", k, response[k]);
			}*/
		 	continue;
		 }
		 //ROS_INFO("................................");
	     // Extract the accel data
	     if (response[4] == 101){
				 for (int k = 0; k < 4; k++){
					 accel[k] = response[8-k];
				 }
				 accel[4] = '\0';
				 // Transfer 4 bytes char data into the float value (IEEE 754)
				 // https://en.wikipedia.org/wiki/Single-precision_floating-point_format
				 memcpy(&gyro_accel.data, &accel, sizeof(gyro_accel.data));
				 imu_msg.linear_acceleration.x = gyro_accel.data;
				 //ROS_INFO("Converter %s to %f.", accel, gyro_accel.data);
				 ROS_INFO("accel_x is %f.", gyro_accel.data);

				 for (int k = 0; k < 4; k++){
					 accel[k] = response[12-k];
				 }
				 // Transfer 4 bytes char data into the float value (IEEE 754)
				 // https://en.wikipedia.org/wiki/Single-precision_floating-point_format
				 memcpy(&gyro_accel.data, &accel, sizeof(gyro_accel.data));
				 imu_msg.linear_acceleration.y = gyro_accel.data;
				 //ROS_INFO("Converter %s to %f.", accel, gyro_accel.data);
				 ROS_INFO("accel_y is %f.", gyro_accel.data);

				 for (int k = 0; k < 4; k++){
					 accel[k] = response[16-k];
				 }
				 // Transfer 4 bytes char data into the float value (IEEE 754)
				 // https://en.wikipedia.org/wiki/Single-precision_floating-point_format
				 memcpy(&gyro_accel.data, &accel, sizeof(gyro_accel.data));
				 imu_msg.linear_acceleration.z = gyro_accel.data;
				 //ROS_INFO("Converter %s to %f.", accel, gyro_accel.data);
				 ROS_INFO("accel_z is %f.", gyro_accel.data);
	        }
		 // Extract the gyro data
        if (response[4] == 97){
        	 for (int k = 0; k < 4; k++){
        		 gyro[k] = response[8-k];
			 }
        	 gyro[4] = '\0';
			 // Transfer 4 bytes char data into the float value (IEEE 754)
			 // https://en.wikipedia.org/wiki/Single-precision_floating-point_format
			 memcpy(&gyro_accel.data, &gyro, sizeof(gyro_accel.data));
			 imu_msg.angular_velocity.x = gyro_accel.data;
			 //ROS_INFO("Converter %s to %f.", accel, gyro_accel.data);
			 ROS_INFO("Gyro_x is %f.", gyro_accel.data);

        	 for (int k = 0; k < 4; k++){
        		 gyro[k] = response[12-k];
			 }
        	 gyro[4] = '\0';
			 // Transfer 4 bytes char data into the float value (IEEE 754)
			 // https://en.wikipedia.org/wiki/Single-precision_floating-point_format
			 memcpy(&gyro_accel.data, &gyro, sizeof(gyro_accel.data));
			 imu_msg.angular_velocity.y = gyro_accel.data;
			 //ROS_INFO("Converter %s to %f.", accel, gyro_accel.data);
			 ROS_INFO("Gyro_y is %f.", gyro_accel.data);

        	 for (int k = 0; k < 4; k++){
        		 gyro[k] = response[16-k];
			 }
        	 gyro[4] = '\0';
			 // Transfer 4 bytes char data into the float value (IEEE 754)
			 // https://en.wikipedia.org/wiki/Single-precision_floating-point_format
			 memcpy(&gyro_accel.data, &gyro, sizeof(gyro_accel.data));
			 imu_msg.angular_velocity.z = gyro_accel.data;
			 //ROS_INFO("Converter %s to %f.", accel, gyro_accel.data);
			 ROS_INFO("Gyro_z is %f.", gyro_accel.data);
        }
		 chatter_pub.publish(imu_msg);
		 //loop_rate.sleep();
    }
   // Read input and publish the data.

    close(UM7); // close the serial communication.
    ROS_INFO("Serial ports are disconnected.");

    return(0);
}
