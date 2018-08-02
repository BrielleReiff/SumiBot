#include "ros/ros.h"
#include "wiringPi.h"
#include <iostream>
#include <chrono>
#include <thread>

#include "motor_control.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_control");

  ros::NodeHandle n;

  // Initialize motors
  RRB3 rrb3(12, 12);

  // Test motion functions
  // Forward
  std::cout << "Run both motors forward two seconds at half speed." << std::endl;
  rrb3.forward(2, 0.5);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Reverse
  std::cout << "Run both motors reverse two seconds at half speed." << std::endl;
  rrb3.reverse(2, 0.5);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Left
  std::cout << "Run left motor reverse, right motor forward two seconds at half speed.";
  std::cout << std::endl;
  rrb3.left(2, 0.5);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Right
  std::cout << "Run right motor reverse, left motor forward two seconds at half speed.";
  std::cout << std::endl;
  rrb3.right(2, 0.5);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  return 0;
}
