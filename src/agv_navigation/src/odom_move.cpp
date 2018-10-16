#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "odom_move.h"
#include <tf/transform_listener.h>
#include <iostream>
using namespace std;

bool flag_posi_complete = false; //first excute
int agv_id = 0;
int agv_id_before = 0;
//geometry_msgs::Pose2D tag_pose2D;
double tag_pose2D[3] = {0};
double del_yg = 0;


OdomMove::OdomMove(ros::NodeHandle& nh):_nh(nh)
{
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

    odom_frame = "odom";
    base_frame = "base_footprint";

   // listener = tf.TransformListener();
    ros::Time(2);

    listener.waitForTransform(odom_frame,base_frame,ros::Time(0),ros::Duration(1.0));
    

}

OdomMove::~OdomMove()
{
    shutdown();
}

void OdomMove::shutdown()
{
    ROS_INFO("Stopping the robot...");

    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.angular.z = 0;

    cmd_vel_pub.publish(twist);
}

int OdomMove::get_odom(tf::Point &point, tf::Quaternion &quat)
{
    tf::StampedTransform transform;

    try
    {
        listener.lookupTransform(odom_frame,base_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("TF exception: %s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
    }

    point = transform.getOrigin();
    quat = transform.getRotation();

    return 0;
}

void OdomMove::go_distance(float linear_speed, float goal_distance)
{
    // 1-init move_cmd
    geometry_msgs::Twist move_cmd;

    move_cmd.linear.x = linear_speed;
    move_cmd.angular.z = 0;//M_PI/2;//0;

    float acc = 20; //rps/s;
    float  d = 0.18; //diameter
    float ra = 16; //reduction ratio
    float distance_slide = pow(linear_speed, 2)/(40*acc*M_PI*d/ra)*32;//pow(linear_speed, 2)/(2*acc*pi*d/ra)

    // 2-Get the starting position values
    float x_start, y_start;
    tf::Point position;
    tf::Quaternion quat;

    get_odom(position, quat);
    x_start = position.x();
    y_start = position.y();

    // 3-Enter the loop to move along a side
    float distance = 0;
    ros::Rate rate(10);
//    float goal_d = goal_distance-distance_slide;
//    ROS_INFO("1  %f :",goal_d);
    while(_nh.ok() && (distance < (goal_distance-distance_slide)))
    {
        cmd_vel_pub.publish(move_cmd);
        rate.sleep();

        get_odom(position, quat);
        distance = sqrt( pow( (position.x() - x_start), 2) + pow((position.y() - y_start), 2));
   //     ROS_INFO("222 possition_distance:  %f ",distance);
    }

    // Stop the robot
    move_cmd.linear.x = 0;
    move_cmd.angular.z = 0;
    cmd_vel_pub.publish(move_cmd);
}

void OdomMove::turn_angle(float angular_speed, float goal_angle)
{
    float angular_tolerance = 0.5/180.0*M_PI;
    
    // 1-init move_cmd
    geometry_msgs::Twist move_cmd;

    move_cmd.linear.x = 0;
    move_cmd.angular.z = angular_speed;

    // 2-Track the last angle measured
    tf::Point position;
    tf::Quaternion quat;
    float last_angle;

    get_odom(position, quat);

 //   ROS_INFO("333turn_test1:  %f ",angular_tolerance);
   // last_angle = quat.getAxis().z();
    last_angle = tf::getYaw(quat);
//    cout<<"333turn_test2:  " << last_angle <<endl;
    // 3-turn the goal angle
    float turn_angle = 0, delta_angle = 0;
    ros::Rate rate(10);

    while(_nh.ok() && (abs(turn_angle + angular_tolerance) < abs(goal_angle)) )
    {
 //	cout<<"333turn_test3:  " << turn_angle + angular_tolerance << " goal: "<<goal_angle <<endl;
	ros::Duration(0.05).sleep(); 
        cmd_vel_pub.publish(move_cmd);
        rate.sleep();

        get_odom(position, quat);
        //delta_angle = normalize_angle(quat.getAxis().z()-last_angle);
        delta_angle = normalize_angle(tf::getYaw(quat)-last_angle);

        turn_angle += delta_angle;
        //last_angle = quat.getAxis().z();

        last_angle =tf::getYaw(quat);
    //    ROS_INFO("333turn_angle:  %f ",turn_angle*180/M_PI);
        //ROS_INFO("2   %f:\n",goal_angle);
    }

    // Stop the robot
    move_cmd.linear.x = 0;
    move_cmd.angular.z = 0;
    cmd_vel_pub.publish(move_cmd);
}

float OdomMove::normalize_angle(float angle)
{
    float res = angle;
   // ROS_INFO("3  %f :",angle);
    while(res > M_PI)
        res -= 2.0 * M_PI;

    while(res < -M_PI)
        res += 2.0 * M_PI;

    return res;
}




