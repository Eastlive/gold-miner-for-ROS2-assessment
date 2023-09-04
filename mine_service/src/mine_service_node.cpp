// Copyright 2023 Chen Tingxu
// MIT License

#include "mine_service/mine_service_node.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "miner_interfaces/msg/ores.hpp"
#include "miner_interfaces/srv/mine_map.hpp"

namespace mine_service
{
MineServiceNode::MineServiceNode(const rclcpp::NodeOptions & options)
: Node("mine_service", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting Mine Service Node");
  subscription_ = this->create_subscription<miner_interfaces::msg::Ores>(
    "mines", 10, std::bind(&MineServiceNode::mine_callback, this, std::placeholders::_1)
  );
  service_ = this->create_service<miner_interfaces::srv::MineMap>(
    "mine_map",
    std::bind(&MineServiceNode::mine_service, this, std::placeholders::_1, std::placeholders::_2)
  );
}

void MineServiceNode::mine_callback(const miner_interfaces::msg::Ores ores)
{
  ores_msg_ = ores;
  RCLCPP_INFO(this->get_logger(), "Received %zu ores", ores_msg_.ores.size());
  subscription_ = nullptr;
}

void MineServiceNode::mine_service(
  const std::shared_ptr<miner_interfaces::srv::MineMap::Request> request,
  const std::shared_ptr<miner_interfaces::srv::MineMap::Response> response)
{
  if (ores_msg_.ores.size() == 0) {
    RCLCPP_INFO(this->get_logger(), "No ores");
    response->ores = ores_msg_;
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Received mine_map request from %d.", request->id);

  bool found = false;
  for (auto it = ores_msg_.ores.begin(); it != ores_msg_.ores.end(); it++) {
    RCLCPP_INFO(this->get_logger(), "Search for ore No.%d.", it->id);

    if (it->id == request->id) {
      found = true;
      RCLCPP_INFO(this->get_logger(), "Response mine_map request to %d.", request->id);
      ores_msg_.ores.erase(it);
      RCLCPP_INFO(this->get_logger(), "Current reminding ore number: %zu.", ores_msg_.ores.size());
      response->ores = ores_msg_;
      break;
    }
  }

  if (!found) {
    RCLCPP_ERROR(this->get_logger(), "No found ore No.%d.", request->id);
  } else {
    RCLCPP_INFO(this->get_logger(), "Found ore No.%d.", request->id);
  }
}
} // namespace mine_service

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mine_service::MineServiceNode)
