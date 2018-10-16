#include <ros/ros.h>
#include "odom_move.h"
#include <math.h>


int sign(float number)
{
    if(number < 0)
	return -1;
    else
	return 1;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_move");
    ros::NodeHandle nh;
    
   
        ROS_INFO("Hello world2!");

        OdomMove odom_move(nh);
	odom_move.go_distance(0.5, 2.5);
   /*  for(int i=0;i<2; i++)
{
        odom_move.go_distance(0.5, 2.5);
        ros::Duration(1.0).sleep();
	
        odom_move.turn_angle(1, M_PI/2);
        ros::Duration(1.0).sleep();
     
   
      odom_move.go_distance(0.5, 1.0);
       ros::Duration(1.0).sleep();
    
	float yaw = M_PI;
       odom_move.turn_angle(sign(yaw) * 1, M_PI/2);//(-1)*
      ros::Duration(1.0).sleep();
}
 */
    ros::spin();
}
