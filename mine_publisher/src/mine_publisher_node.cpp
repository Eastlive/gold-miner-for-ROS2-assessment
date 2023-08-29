// Copyright 2023 Chen Tingxu
// MIT License

#include "mine_publisher/mine_publisher_node.hpp"

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "miner_interfaces/msg/ore.hpp"
#include "miner_interfaces/msg/ores.hpp"

namespace mine_publisher
{
MinePublisherNode::MinePublisherNode(const rclcpp::NodeOptions& options) : Node("mine_publisher", options)
{
  init_mine_msg();

  publisher_ = this->create_publisher<miner_interfaces::msg::Ores>("mines", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MinePublisherNode::timer_callback, this));
}

void MinePublisherNode::init_mine_msg()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dis(0.0, 1.0);

  ores_msg_.header.stamp = this->now();
  ores_msg_.header.frame_id = "mine_map";
  int mine_num = declare_parameter<int>("mine_num", 10);
  RCLCPP_INFO(this->get_logger(), "Mine Number: %d", mine_num);
  for (int i = 0; i < mine_num; i++)
  {
    miner_interfaces::msg::Ore ore;
    ore.id = i;
    ore.type = (dis(gen) > 0.5) ? "gold" : "silver";
    do
    {
      ore.pose.position.x = dis(gen) * 10;
      ore.pose.position.y = dis(gen) * 10;
      ore.pose.position.z = dis(gen) * 10;
    } while (ore.pose.position.x * ore.pose.position.x + ore.pose.position.y * ore.pose.position.y +
                 ore.pose.position.z * ore.pose.position.z >
             100);
    ore.value = (ore.type == "gold") ? 80.8 : 40.4;
    RCLCPP_INFO(this->get_logger(), "Mine %d Position: (%f, %f, %f)", ore.id, ore.pose.position.x, ore.pose.position.y,
                ore.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Mine %d Type: %s", ore.id, ore.type.c_str());
    RCLCPP_INFO(this->get_logger(), "Mine %d Value: %f", ore.id, ore.value);
    ores_msg_.ores.push_back(ore);
  }
}

void MinePublisherNode::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", ores_msg_.header.frame_id.c_str());
  publisher_->publish(ores_msg_);
}
}  // namespace mine_publisher

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mine_publisher::MinePublisherNode)