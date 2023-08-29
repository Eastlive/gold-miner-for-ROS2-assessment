// Copyright 2023 Chen Tingxu
// MIT License

#ifndef MINE_PUBLISHER__MINE_PUBLISHER_NODE_HPP_
#define MINE_PUBLISHER__MINE_PUBLISHER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "miner_interfaces/msg/ores.hpp"

namespace mine_publisher
{
class MinePublisherNode : public rclcpp::Node
{
public:
  explicit MinePublisherNode(const rclcpp::NodeOptions& options);

private:
  void timer_callback();

  void init_mine_msg();

  miner_interfaces::msg::Ores ores_msg_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<miner_interfaces::msg::Ores>::SharedPtr publisher_;
};
}  // namespace mine_publisher

#endif  // MINE_PUBLISHER__MINE_PUBLISHER_NODE_HPP_