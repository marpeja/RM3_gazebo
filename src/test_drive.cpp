#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("low_level_velocity_controller");

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/velocity_controller/commands", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  std_msgs::msg::Float64MultiArray commands;

  using namespace std::chrono_literals;

  for(int i = 0; i<4; i++)
  {
      commands.data.push_back(0);
  }
  publisher->publish(commands);
  std::this_thread::sleep_for(1s);

  // Screw velocities [FR, BR, BL, FL]
  float vel = 3;
  commands.data[0] = vel;
  commands.data[1] = -vel;
  commands.data[2] = -vel;
  commands.data[3] = vel;

  // Lateral virtual wheels velocities
  // [FR1, BR1, BL1, FL1]

  publisher->publish(commands);
  std::this_thread::sleep_for(5s);

  commands.data[0] = vel;
  commands.data[1] = vel;
  commands.data[2] = -vel;
  commands.data[3] = -vel;

  publisher->publish(commands);
  std::this_thread::sleep_for(5s);
  rclcpp::shutdown();

  return 0;
}
