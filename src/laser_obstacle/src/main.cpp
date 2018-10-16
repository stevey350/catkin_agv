#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "jetsonGPIO.h"

int main(int argc, char **argv)
{
    // 1-ros initial
    ros::init(argc, argv, "laser_obstacle");
    ros::NodeHandle nh;

    ros::Publisher laser_obstacle_pub = nh.advertise<std_msgs::String>("laser_obstacle_pub", 1000);

    ros::Rate loop_rate(10);

    // 2-laser obstacle
    jetsonTX2GPIONumber read1 = gpio486;
    jetsonTX2GPIONumber read2 = gpio480;
    jetsonTX2GPIONumber read3 = gpio388;

    gpioExport(read1);
    gpioExport(read2);
    gpioExport(read3);
    gpioSetDirection(read1, inputPin);
    gpioSetDirection(read2, inputPin);
    gpioSetDirection(read3, inputPin);

    unsigned int value1 = low;
    unsigned int value2 = low;
    unsigned int value3 = low;

    std_msgs::String msg;
    std::string a;
    while (ros::ok())
    {
        gpioGetValue(read1, &value1);
        gpioGetValue(read2, &value2);
        gpioGetValue(read3, &value3);


        value1 == high ? a = "1" : a = "0";
        value2 == high ? a += "1" : a += "0";
        value3 == high ? a += "1" : a += "0";

        msg.data = a;

        laser_obstacle_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    gpioUnexport(read1);
    gpioUnexport(read2);
    gpioUnexport(read3);

    return 0;
}
