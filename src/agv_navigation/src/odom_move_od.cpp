#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "odom_move.h"

OdomMove::OdomMove(ros::NodeHandle& nh):_nh(nh)
{
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);

    odom_frame = "odom";
    base_frame = "base_footprint";

    listener.waitForTransform(odom_frame, base_frame, ros::Time(0), ros::Duration(1.0));
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
        listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
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
    move_cmd.angular.z = 0;

    // 2-Get the starting position values
    float x_start, y_start;
    tf::Point position;
    tf::Quaternion quat;

    get_odom(position, quat);
    x_start = position.x();
    y_start = position.y();

    // 3-Enter the loop to move along a side
    float distance = 0;
    ros::Rate rate(20);
    while(_nh.ok() && (distance < goal_distance))
    {
        cmd_vel_pub.publish(move_cmd);
        rate.sleep();

        get_odom(position, quat);
        distance = sqrt( pow( (position.x() - x_start), 2) + pow((position.y() - y_start), 2));
    }

    // Stop the robot
    move_cmd.linear.x = 0;
    move_cmd.angular.z = 0;
    cmd_vel_pub.publish(move_cmd);
}

void OdomMove::turn_angle(float angular_speed, float goal_angle)
{
    float angular_tolerance = 1.0/180.0*M_PI;

    // 1-init move_cmd
    geometry_msgs::Twist move_cmd;

    move_cmd.linear.x = 0;
    move_cmd.angular.z = angular_speed;

    // 2-Track the last angle measured
    tf::Point position;
    tf::Quaternion quat;
    float last_angle;

    get_odom(position, quat);

//    last_angle = quat.getAxis().z();
    last_angle = tf::getYaw(quat);

    // 3-turn the goal angle
    float turn_angle = 0, delta_angle = 0;
    ros::Rate rate(20);
    while(_nh.ok() && (abs(turn_angle + angular_tolerance) < abs(goal_angle)) )
    {
        cmd_vel_pub.publish(move_cmd);
        rate.sleep();

        get_odom(position, quat);
        delta_angle = normalize_angle(quat.getAxis().z()-last_angle);

        turn_angle += delta_angle;
        last_angle = quat.getAxis().z();
    }

    // Stop the robot
    move_cmd.linear.x = 0;
    move_cmd.angular.z = 0;
    cmd_vel_pub.publish(move_cmd);
}

float OdomMove::normalize_angle(float angle)
{
    float res = angle;

    while(res > M_PI)
        res -= 2.0 * M_PI;

    while(res < -M_PI)
        res += 2.0 * M_PI;

    return res;
}

