#ifndef ODOM_MOVE_H
#define ODOM_MOVE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

class OdomMove{
public:
    OdomMove(ros::NodeHandle &nh);
    ~OdomMove();

    void shutdown();

    int get_odom(tf::Point &point, tf::Quaternion &quat);

    void go_distance(float linear_speed, float goal_distance);

    void turn_angle(float angular_speed, float goal_angle);

    float normalize_angle(float angle);

private:
    ros::NodeHandle _nh;
    ros::Publisher cmd_vel_pub;
    tf::TransformListener listener;

    std::string odom_frame;
    std::string base_frame;
};


#endif // ODOM_MOVE_H
