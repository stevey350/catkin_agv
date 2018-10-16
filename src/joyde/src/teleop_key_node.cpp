/**********************************************************
方向键控制移动平台的前进/后退，原地左转/右转。
Q-左转前进，E-右转前进，A-左转后退，D-右转后退，S-停止
*************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_d 0x64
#define ANGULAR 0.4
#define LINEAR 0.2
class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:


  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;

};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("agv/cmd_vel", 10);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();

  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = ANGULAR;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -ANGULAR;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = LINEAR;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -LINEAR;
        dirty = true;
        break;

        case KEYCODE_S:
        ROS_DEBUG("STOP");
        linear_ = 0.0;
        dirty = true;
        break;

    case KEYCODE_Q:
        ROS_DEBUG("LEFT-UP");
        angular_ = ANGULAR;
        linear_ = LINEAR;
        dirty = true;
        break;

    case KEYCODE_E:
        ROS_DEBUG("RIGHT-UP");
        angular_ = -ANGULAR;
        linear_ = LINEAR;
        dirty = true;
        break;

    case KEYCODE_A:
        ROS_DEBUG("LEFT-DOWN");
        angular_ = ANGULAR;
        linear_ = -LINEAR;
        dirty = true;
        break;

    case KEYCODE_d:
        ROS_DEBUG("RIGHT-DOWN");
        angular_ = -ANGULAR;
        linear_ = -LINEAR;
        dirty = true;
        break;

    }


    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);
      dirty=false;
    }
  }


  return;
}
