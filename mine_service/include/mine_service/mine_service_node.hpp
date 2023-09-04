// Copyright (c) 2023 Eastlive
// MIT License

#ifndef MINE_SERVICE__MINE_SERVICE_NODE_HPP_
#define MINE_SERVICE__MINE_SERVICE_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "miner_interfaces/msg/ores.hpp"
#include "miner_interfaces/srv/mine_map.hpp"

namespace mine_service
{
class MineServiceNode : public rclcpp::Node
{
public:
  explicit MineServiceNode(const rclcpp::NodeOptions & options);
  ~MineServiceNode() {}

  miner_interfaces::msg::Ores get_ores_msg() {return ores_msg_;}

private:
  void mine_callback(const miner_interfaces::msg::Ores ores);
  void mine_service(
    const std::shared_ptr<miner_interfaces::srv::MineMap::Request> request,
    const std::shared_ptr<miner_interfaces::srv::MineMap::Response> response);

  rclcpp::Subscription<miner_interfaces::msg::Ores>::SharedPtr subscription_;
  rclcpp::Service<miner_interfaces::srv::MineMap>::SharedPtr service_;

  miner_interfaces::msg::Ores ores_msg_;
};
} // namespace mine_service

#endif // MINE_SERVICE__MINE_SERVICE_NODE_HPP_
