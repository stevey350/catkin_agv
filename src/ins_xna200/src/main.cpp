#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include "AsyncSerial.h"


#define FRAME_LEN	42

using namespace std;
using namespace boost;

float rate_x, rate_y, rate_z;
float accel_x, accel_y, accel_z;
float roll, pitch, yaw;

void readCallback(const char *data_packet, size_t size)
{
    short header;
    short packet_index;
    short check_sum;

	// Verify data packet header 
	memcpy(&header, data_packet, sizeof(short));
	if(header != (short)0xFFFF)
	{
		cout << "Header error ----------------------------------------------------!!!\n";
		return ;
	}
	
	// Verify data checksum
	check_sum=data_packet[2];
	for(int i=3;i<FRAME_LEN-1;i++)
		check_sum^=data_packet[i];

	if( ((unsigned char)check_sum) != data_packet[FRAME_LEN-1])
	{ 
		cout<< "Checksum error ***************************************************!!\n";
		return ;
	}

	// Copy values from data string and apply scale factors

	int float_size = sizeof(float);
	int parse_index = 2; //Skip header
	// Packet counter
	memcpy(&packet_index,data_packet+parse_index,sizeof(short));

	// Gryo_x rate
	parse_index = 5; //Skip Dat Information byte
	memcpy(&rate_x,data_packet+parse_index,float_size);
	// Gryo_Y rate
	parse_index += float_size;
	memcpy(&rate_y,data_packet+parse_index,float_size);
	// Gryo_z rate
	parse_index += float_size;
	memcpy(&rate_z,data_packet+parse_index,float_size);

	// Acc_X rate
	parse_index += float_size;
	memcpy(&accel_x,data_packet+parse_index,float_size);
	// Acc_Y rate
	parse_index += float_size;
	memcpy(&accel_y,data_packet+parse_index,float_size);
	// Acc_Y rate
	parse_index += float_size;
    memcpy(&accel_z ,data_packet+parse_index,float_size);

	// Roll
	parse_index += float_size;
	memcpy(&roll,data_packet+parse_index,float_size);
	// Pitch
	parse_index += float_size;
	memcpy(&pitch,data_packet+parse_index,float_size);
	// Yaw
	parse_index += float_size;
	memcpy(&yaw,data_packet+parse_index,float_size);

//	cout << "Rates [deg/sec]:" << rate_x << " " << rate_y << " " << rate_z <<endl;
//	cout << "Accel [m/sec^2]:" << accel_x << " " << accel_y << " " << accel_z <<endl;
//	cout << "Attitude [deg]:" << roll <<" "<< pitch << " " << yaw <<endl;
}



int main(int argc, char* argv[])
{
    // 1-ros initial
    ros::init(argc, argv, "ins_node");
    ros::NodeHandle nh;

//    ros::Publisher ins_pub = nh.advertise<std_msgs::String>("imu_data", 10);
    ros::Publisher ins_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::Rate loop_rate(20);

    // 2-xna200 initial
    CallbackAsyncSerial serial;

    try
	{
        serial.open("/dev/ttyTHS2",115200);
    }
	catch(boost::system::system_error& e)
    {
        //Errors during open
		cout<<"Error: "<<e.what()<<endl;
        return 1;
    }

    serial.setCallback(bind(readCallback, _1, _2));//&
    
    // rpy -> quaternion
    tf::Quaternion q;

    while(ros::ok())
    {
/*
        std_msgs::String msg;

        std::stringstream ss;
        ss << "roll="<<roll_out<<" pitch="<<pitch_out<<" yaw="<<yaw_out;

        msg.data = ss.str();

        ins_pub.publish(msg);
*/
        sensor_msgs::Imu imu_data;
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "base_link";
        q.setRPY(roll, pitch, yaw);
        imu_data.orientation.x = q.getX();
        imu_data.orientation.y = q.getY();
        imu_data.orientation.z = q.getZ();
        imu_data.orientation.w = q.getW();
        
        imu_data.angular_velocity.x = rate_x;
        imu_data.angular_velocity.y = rate_y; 
        imu_data.angular_velocity.z = rate_z; 

        imu_data.linear_acceleration.x = accel_x; 
        imu_data.linear_acceleration.y = accel_y; 
        imu_data.linear_acceleration.z = accel_z; 
        
        ins_pub.publish(imu_data);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
