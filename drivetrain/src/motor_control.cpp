#include "ros/ros.h"
#include "std_msgs/String.h"
#include "wiringPi.h"
#include <sstream>
#include <chrono>
#include <thread>

#include "motor_control.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_control");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(100);

  // Initialize motors
  RRB3 rrb3;

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "motors going..." << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    rrb3.set_motors(0, 1, 0, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    rrb3.set_motors(0.5, 1, 0.5, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    ros::spinOnce();
    ++count;
  }

  return 0;
}
