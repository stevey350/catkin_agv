#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

#include "std_msgs/String.h"
#include "dgvmsg/DriverVelocity.h"
#include "dgvmsg/DriverNode.h"
#include "dgvmsg/Encounter.h"
#include "dgvmsg/ctrl_ModeMessage.h"

#include "../../zmq_ros/include/zmqclient.h"
#include <iostream>
using namespace std;


#define TICK2RAD 0.00062832      // 360/10000 * 3.1416/180    one pulse -> degree -> radian

static int count_sum = 0;


class Odom_Transformer
{
 public:
   Odom_Transformer()
   {
      encoder_l = 0;
      encoder_r = 0;
      time_re = 0.0;

      wheel_l = 0.0;
      wheel_r = 0.0;
      v = w =0.0;

      odom_vel[0] =0.0;
      odom_vel[1] =0.0;
      odom_vel[2] =0.0;

      odom_pose[0] =0.0;
      odom_pose[1] =0.0;
      odom_pose[2] =0.0;

      last_tick[0] = 0.0;
      last_tick[1] = 0.0;
      last_diff_tick[0] =0.0;
      last_diff_tick[1] =0.0;

      joint_rad[0] = 0.0;
      joint_rad[1] = 0.0;
      joint_velocity[0] = 0.0;
      joint_velocity[1] = 0.0;

      current_tick = 0.0;

      joint_states.header.frame_id = "base_footprint";
      joint_states.name.resize(2);
      joint_states.position.resize(2);
      joint_states.velocity.resize(2);
      joint_states.effort.resize(2);

      joint_states.name[0] = "base_wheel_l";
      joint_states.name[1] = "base_wheel_r";


      current_time = ros::Time::now();

      odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

      joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

      driver_encoder_sub = nh.subscribe("driverencounter", 10, &Odom_Transformer::driver_encoder_callback,this);

   }



   void driver_encoder_callback(const dgvmsg::Encounter& msg)
   {

    if(msg.devadd == 1)
        {
          encoder_r = msg.encounter;
          ROS_INFO("encounter_r = %d.\n", encoder_r);

        }
    else
       {
         encoder_l = msg.encounter;
         ROS_INFO("encounter_l = %d.\n", encoder_l);

         time_re = msg.now.toSec();
       }

      count_sum ++;

   if(count_sum % 2 ==0)
  {

   //  ROS_INFO("encounter = %d.\n", encoder);

       ROS_INFO("time = %f.\n", time_re);

   // left encoder inner value

    current_time = ros::Time::now();

    current_tick = encoder_l;
    last_diff_tick[0] = current_tick - last_tick[0];
    last_tick[0] = current_tick;

    joint_rad[0] += TICK2RAD * last_diff_tick[0] / 16;  // joint_states  left_wheeel_angle

   // right encoder inner value

    current_tick = -encoder_r;
    last_diff_tick[1] = current_tick - last_tick[1];
    last_tick[1] = current_tick;

    joint_rad[1] += TICK2RAD * last_diff_tick[1] / 16;

    wheel_l = TICK2RAD * last_diff_tick[0] / 16; // pulses converted angle  , 16 is reduction ratio

    ROS_INFO("wheel_l = %f.\n", wheel_l);

    wheel_r = TICK2RAD * last_diff_tick[1] / 16;

    ROS_INFO("wheel_r = %f.\n", wheel_r);

    time_now =time_re;
    step_time = time_now - prev_time;  //
    prev_time = time_now;

 // if(step_time ==0)
 //     return false;

    v = 0.09 * (wheel_r + wheel_l) / 2 /step_time;     //0.09 is the radius of wheel, l = r * angle
    w = 0.09 * (wheel_r - wheel_l) / 0.57 / step_time;  // 0.638 is the distances of wheels

    joint_velocity[0] = wheel_l / step_time;   // joint_states  left_wheeel_velocity
    joint_velocity[1] = wheel_r / step_time;


    ROS_INFO("v = %f.\n",v);

    odom_pose[0] += v * step_time * cos(odom_pose[2] + (w * step_time / 2));
    odom_pose[1] += v * step_time * sin(odom_pose[2] + (w * step_time / 2));
    odom_pose[2] += w * step_time;

    ROS_INFO("position.x= %f.\n",odom_pose[0]);
    ROS_INFO("yaw = %f.\n",odom_pose[2]);

    odom_vel[0] = v;
    odom_vel[1] = 0.0;
    odom_vel[2] = w;


    geometry_msgs::Quaternion odom_rotation = tf::createQuaternionMsgFromYaw(odom_pose[2]);

  // odom_tf

    geometry_msgs::TransformStamped odom_tf;

    odom_tf.header.stamp = current_time;

    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = odom_pose[0];
    odom_tf.transform.translation.y = odom_pose[1];
    odom_tf.transform.rotation = odom_rotation;

    tfbroadcaster.sendTransform(odom_tf);


   // publish odom;

    odom.header.frame_id = "odom";
    odom.header.stamp = current_time;
    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_rotation;

    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];

    odom_pub.publish(odom);

    // publish joint_states

    joint_states.header.stamp = current_time;
    joint_states.position[0] = joint_rad[0];
    joint_states.position[1] = joint_rad[1];

    joint_states.velocity[0] = joint_velocity[0];
    joint_states.velocity[1] = joint_velocity[0];

    joint_pub.publish(joint_states);


    }

  }
 private:

       ros::NodeHandle nh;
       ros::Publisher odom_pub;
       ros::Publisher joint_pub;
       ros::Subscriber driver_encoder_sub;
       ros::Time current_time;
       tf::TransformBroadcaster tfbroadcaster;
       nav_msgs::Odometry odom;
       sensor_msgs::JointState joint_states;

       double odom_vel[3];
       double odom_pose[3];
       double wheel_l,wheel_r;


       double v,w;
       double last_tick[2];
       double last_diff_tick[2];
       double current_tick;

       double joint_rad[2];
       double joint_velocity[2];

       double time_now;
       double step_time;
       double prev_time;

       int encoder_l;
       int encoder_r;
       double time_re;
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "drivertest_node");

 //   ros::NodeHandle nh;
cout<<"hello1"<<endl;

    Odom_Transformer Odom_mh;

 //   ros::Subscriber driver_status_sub = nh.subscribe("Kincodriverfeedback", 10, driver_status_callback);

    ros::spin();

    return 0;
}
