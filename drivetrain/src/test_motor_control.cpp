#include "ros/ros.h"
#include "wiringPi.h"
#include <iostream>

#include "motor_control.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_control");

  ros::NodeHandle n;

  // Initialize motors
  RRB3 rrb3;

  const float circum = 45.6;  //cm

  // Test motion functions
  // Forward
  std::cout << "Run both motors forward one rotation at half speed." << std::endl;
  rrb3.forward(circum, 0.5);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Reverse
  std::cout << "Run both motors reverse one rotation at half speed." << std::endl;
  rrb3.reverse(circum, 0.5);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Left
  std::cout << "Turn approximately 90 degrees to the left at half speed." << std::endl;
  rrb3.turn_left(circum / 2.0, 0.5);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Right
  std::cout << "Turn approximately 90 degrees to the right at half speed." << std::endl;
  rrb3.turn_right(circum / 2.0, 0.5);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  return 0;
}
