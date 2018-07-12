/*
 * IMU_360D.cpp
 *  This cpp file is used for reading Linear acceleration (x, y , z),
 *  Angular rate (x, y, z) from IMU360D-F99-B20-V15
 *
 *  Created by Mengbin Min (Mike) based on the communication protocol
 *  from the datasheet of Inertial measurement unit IMU360D-F99-B20-V15
 *  https://www.pepperl-fuchs.com/global/en/classid_6422.htm?view=productdetails&prodid=81885
 *
 *  Oct 5, 2017
 *  Email: prlatlab@gmail.com; robotintheworld@163.com
 *  http://prllab.wixsite.com/prl-lab-en
 */

// header files for standard input and output, standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "ros/ros.h"// header file for ros

// header file for ros data type
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32.h"  //Unsigned int 32 data type
#include "std_msgs/Int16.h"  //Unsigned int 32 data type

#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

// header file for CAN frame structure and sockaddr structure and socket com
#include <linux/can.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>

#include <net/if.h>  //header file for ifreq structure

#define  g 9.81 // Acceleration of gravity
#define pi 3.1415926 //

int main(int argc, char **argv)
{

    int s; // creat a handler of CAN communication
    int nbytes; // byte numbers of received and sent messages
    std_msgs::Float32  Int_data;
    //define the structure used for CAN bus
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct timeval tv;

    // gyro data and accel data
    std_msgs::Float32 gyro_accel, gyro_accel_1, gyro_accel_2, accel_z;
    std_msgs::Float32 velocity_z;
    std_msgs::Float32 time_1, time_2;
    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = "world";

    // variables used for converting received data to IMU ros topic
    char data[2] = {0,0};
    //std_msgs::Int16 x; // ros type
    int16_t x_int;  //c++ signed int type
    uint16_t x_uint;//c++ unsigned int type
    int16_t test_int = -1;  //c++ signed int type

    printf("%x \n", test_int);

    float y; // used to transfer data to float value

    int flag = 1; // flag for debug

    //set up ros
	ros::init(argc, argv, "IMU_360D");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("data", 1000);
	//ros::Publisher chatter_pub = n.advertise<std_msgs::Float32 >("int_data", 1000);

	ros::Rate loop_rate(100); // running frequency

	// CAN bus communication initialization.
	ROS_INFO("Trying to create CAN bus connection.");

	s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // creat a handler of CAN communication

    strcpy(ifr.ifr_name, "can0" );
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr *)&addr, sizeof(addr)); //binding s to socket
    ROS_INFO("CAN communication connected.");
    time_2.data = 0; //initialize time value
    gyro_accel_2.data = 0;

// Read inputs and publish acceleration, gyro, orientation and Euler Angles data.
    while ( ros::ok()) {
    //while (flag) { // for debug
          for(int k = 0; k < 8; k++){  // read 8 frames data
				nbytes = read(s, &frame, sizeof(struct can_frame));
				if (ioctl(s,SIOCGSTAMP, &tv) == -1){
				   ROS_INFO ("Getting time stamp wrongly"); // for debug
				} else
				{
					ROS_INFO("%d.%d",tv.tv_sec,tv.tv_usec);
					time_1.data = time_2.data;
					time_2.data = tv.tv_sec+tv.tv_usec/1000;
				}
				//ROS_INFO ("nbytes = %d", nbytes); // for debug

				if (nbytes < 0) {
						perror("can raw socket read fail");
						return 1;
				}

				if (nbytes < sizeof(struct can_frame)) {
						fprintf(stderr, "read: incomplete CAN frame\n");
						return 1;
				}

// Rotation Rate data
				if ((frame.can_id & 0x00ffff00) == 0x00ff0100){
					//ROS_INFO("CAN ID = %08x",frame.can_id); //for debug
					for (int i = 0; i < 3; i++) {
						data[0] = frame.data[2*i];
						data[1] = frame.data[2*i+1];
						data[2] = '\0';

/*
						ROS_INFO ("nbytes[%d] = %2x\n",2*i, frame.data[2*i]);//for debug
						ROS_INFO ("nbytes[%d] = %2x\n",2*i+1, frame.data[2*i+1]);//for debug
*/

						switch (i){
						case (0):
								memcpy(&x_int, &data, 2);
								//ROS_INFO ("gyro[x] in hex = (%x), gyro[x] in dec = (%i)",x_int, x_int);//for debug
								y = (x_int - 0)/100.0;
								//ROS_INFO ("gyro[x] = %f deg/s ",y);//for debug
								imu_msg.angular_velocity.x = (y/180) * pi;
								break;
						case (1):
								memcpy(&x_int, &data, 2);
								//ROS_INFO ("gyro[y] in hex = (%x), gyro[y] in dec = (%i)",x_int, x_int);//for debug
								y = (x_int - 0)/100.0;
								//ROS_INFO ("gyro[y] = %f deg/s ",y);//for debug
								imu_msg.angular_velocity.y = (y/180) * pi;
								break;
						case (2):
								memcpy(&x_int, &data, 2);
								//ROS_INFO ("gyro[z] in hex = (%x), gyro[z] in dec = (%i)",x_int, x_int);//for debug
								y = (x_int - 0)/100;
								//ROS_INFO ("gyro[z] = %f deg/s ",y);//for debug
								imu_msg.angular_velocity.z = (y/180) * pi;
								break;
						default:
							    break;
						}

					}
				}
// Rotation Rate data

// Linear Acceleration data
				if ((frame.can_id & 0x00ffff00) == 0x00ff0400){
					for (int i = 0; i < 3; i++) {
						data[0] = frame.data[2*i];
						data[1] = frame.data[2*i+1];
						data[2] = '\0';

/*
						ROS_INFO ("nbytes[%d] = %x\n",2*i, frame.data[2*i]);//for debug
						ROS_INFO ("nbytes[%d] = %x\n",2*i+1, frame.data[2*i+1]);//for debug
*/

						switch (i){
						case (0):
								memcpy(&x_int, &data, 2);
								//ROS_INFO ("accel[x] in dec = (%i)", x_int);//for debug
								y = ((x_int - 0)/1000.0) * g;
								imu_msg.linear_acceleration.x = y;
								ROS_INFO ("accel[x] = %f",y);//for debug
								break;
						case (1):
								memcpy(&x_int, &data, 2);
								//ROS_INFO("accel[y] in hex = (%x), accel[y] in dec = (%i) \n",x_int, x_int);//for debug
								//printf ("accel[y] in hex = (%x), accel[y] in dec = (%i) \n",x_int, x_int);//for debug
								//ROS_INFO("the size of x_int is %lu", sizeof(x_int)); //for debug
								y = ((x_int - 0)/1000.0) * g;
								imu_msg.linear_acceleration.y = y;
								ROS_INFO ("accel[y] = %f",y);//for debug
								break;
						case (2):
								memcpy(&x_int, &data, 2);
								//ROS_INFO ("accel[z] in hex = (%x), accel[z] in dec = (%i)", x_int, x_int);//for debug
								//printf ("accel[z] in hex = (%x), accel[z] in dec = (%i)\n", x_int, x_int);//for debug
								y = ((x_int - 0)/1000.0) * g;
								imu_msg.linear_acceleration.z = y;
								accel_z.data = y;
								gyro_accel_1.data = gyro_accel_2.data;
								gyro_accel_2.data = y;
								//gyro_accel.data = y;
								ROS_INFO ("accel[z] = %f",y);//for debug
								break;
						default:
							    break;
						}

					}
				}
// Linear Acceleration data

// Euler Angels data
				if ((frame.can_id & 0x00ffff00) == 0x00ff0500){
					//ROS_INFO("CAN ID = %08x",frame.can_id); //for debug
					for (int i = 0; i < 3; i++) {
						data[0] = frame.data[2*i];
						data[1] = frame.data[2*i+1];
						data[2] = '\0';

/*
						ROS_INFO ("nbytes[%d] = %2x\n",2*i, frame.data[2*i]);//for debug
						ROS_INFO ("nbytes[%d] = %2x\n",2*i+1, frame.data[2*i+1]);//for debug
*/

						switch (i){
						case (0):
								memcpy(&x_uint, &data, 2);
								//ROS_INFO ("Roll in hex = (%x), Roll in dec = (%i)",x_uint, x_uint);//for debug
								y = (x_uint - 0)/100.0;
								ROS_INFO ("Roll = (%f degree)", y);
								break;
						case (1):
								memcpy(&x_uint, &data, 2);
								//ROS_INFO ("Pitch in hex = (%x), Pitch in dec = (%i)",x_uint, x_uint);//for debug
								//ROS_INFO("the size of x_uint is %lu", sizeof(x_uint)); //for debug
								y = (x_uint - 0)/100.0;
								ROS_INFO ("Pitch = (%f degree)", y);
								break;
						case (2):
								memcpy(&x_uint, &data, 2);
								//ROS_INFO ("Heading in hex = (%x), Heading in dec = (%i)",x_uint, x_uint);//for debug
								y = (x_uint - 0)/100.0;
								ROS_INFO ("Heading = (%f degree)", y);
								break;
						default:
								break;
						}

					}
				}
// Euler Angels data

// Quaternion data
				if ((frame.can_id & 0x00ffff00) == 0x00ff0600){
					//ROS_INFO("CAN ID = %08x",frame.can_id); //for debug
					for (int i = 0; i < 4; i++) {
						data[0] = frame.data[2*i];
						data[1] = frame.data[2*i+1];
						data[2] = '\0';

/*
						ROS_INFO ("nbytes[%d] = %2x\n",2*i, frame.data[2*i]);//for debug
						ROS_INFO ("nbytes[%d] = %2x\n",2*i+1, frame.data[2*i+1]);//for debug
*/

						switch (i){
						case (0):
								memcpy(&x_int, &data, 2);
								//ROS_INFO (" in_data Quat[x] in dec = (%i)",x_int);//for debug
								y = (x_int - 0)/1000.0;
								imu_msg.orientation.x = y;
								//ROS_INFO ("Quat[x] = %f",y);//for debug
								break;
						case (1):
								memcpy(&x_int, &data, 2);
								//ROS_INFO (" in_data Quat[y] in dec = (%i)",x_int);//for debug
								y = (x_int - 0)/1000.0;
								imu_msg.orientation.y = y;
								//ROS_INFO (" in_data Quat[y] = %f",y);//for debug
								break;
						case (2):
								memcpy(&x_int, &data, 2);
								//ROS_INFO ("in_data Quat[z] in dec = (%i)", x_int);//for debug
								y = (x_int - 0)/1000.0;
								imu_msg.orientation.z = y;
								//ROS_INFO ("Quat[z] = %f",y);//for debug
								break;
						case (3):
								memcpy(&x_int, &data, 2);
								//ROS_INFO ("in_data Quat[w] in dec = (%i)", x_int);//for debug
								y = (x_int - 0)/1000.0;
								imu_msg.orientation.w = y;
								//ROS_INFO ("Quat[w] = %f",y);//for debug
								break;
						default:
								break;
						}

					}
				}
// Quaternion data

          }
// Read inputs and publish acceleration, gyro, orientation and Euler Angles data.
             //gyro_accel.data += (gyro_accel_2.data + gyro_accel_1.data) * (time_2.data - time_1.data);
		     chatter_pub.publish(accel_z);
		     //chatter_pub.publish(Int_data);
		     flag = 0; // a flag for debugging
		}
	close(s);
	printf ("CAN communication disconnected\n");
    return 0;
}

