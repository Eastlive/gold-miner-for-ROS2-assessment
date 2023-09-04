// Copyright (c) 2023 Eastlive
// MIT License

#include "miner_client/miner_client_node.hpp"

#include <memory>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "miner_interfaces/msg/ores.hpp"
#include "miner_interfaces/srv/mine_map.hpp"

namespace miner_client
{
MinerClientNode::MinerClientNode(const rclcpp::NodeOptions & options)
: Node("miner_client", options)
{
  client_ = this->create_client<miner_interfaces::srv::MineMap>("mine_map");

  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  RCLCPP_INFO(this->get_logger(), "Start mining.");
  send_request(-1);
  while (!received_callback) {
    rclcpp::spin_some(this->get_node_base_interface());
  }
  RCLCPP_INFO(
    this->get_logger(), "Initializer know recent_ores_msg_.ores.size() = %zu",
    recent_ores_msg_.ores.size());

  while (recent_ores_msg_.ores.size() > 0) {
    int32_t aim_ore_id = find_aim_ore(recent_ores_msg_);
    RCLCPP_INFO(this->get_logger(), "Aim ore No.%d.", aim_ore_id);
    send_request(aim_ore_id);
    while (!received_callback) {
      rclcpp::spin_some(this->get_node_base_interface());
    }
    RCLCPP_INFO(
      this->get_logger(), "recent_ores_msg_.ores.size() = %zu",
      recent_ores_msg_.ores.size());

    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  if (recent_ores_msg_.ores.size() == 0) {
    RCLCPP_INFO(this->get_logger(), "Miner finished mining and no ore left!");
  }
}

void MinerClientNode::send_request(const int32_t id)
{
  received_callback = false;
  if (id == -1) {
    RCLCPP_INFO(this->get_logger(), "[send_request] Initial mine map.");
  } else {
    RCLCPP_INFO(this->get_logger(), "[send_request] Digging No.%d ore.", id);
  }
  auto request = std::make_shared<miner_interfaces::srv::MineMap::Request>();
  request->id = id;

  RCLCPP_INFO(this->get_logger(), "[send_request] Send request id.");
  client_->async_send_request(
    request, std::bind(&MinerClientNode::result_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "[send_request] Waiting for response.");
}

void MinerClientNode::result_callback(
  rclcpp::Client<miner_interfaces::srv::MineMap>::SharedFuture result_future)
{
  RCLCPP_INFO(this->get_logger(), "[result_callback] Start result callback.");
  auto status = result_future.wait_for(std::chrono::seconds(1));
  if (status == std::future_status::ready) {
    auto result = result_future.get();
    if (result) {
      recent_ores_msg_ = result->ores;
      RCLCPP_INFO(
        this->get_logger(), "[result_callback] %zu ore(s) left.", recent_ores_msg_.ores.size());
    } else {
      RCLCPP_ERROR(this->get_logger(), "[result_callback] Cannot recieve response.");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "[result_callback] Failed to get response.");
  }

  received_callback = true;
  RCLCPP_INFO(this->get_logger(), "[result_callback] Finish result callback.");
}

int32_t MinerClientNode::find_aim_ore(const miner_interfaces::msg::Ores ore_array)
{
  double min_distance = std::numeric_limits<double>::max();
  int32_t closest_ore_id = -1;

  for (const auto & ore : ore_array.ores) {
    double x = ore.pose.position.x;
    double y = ore.pose.position.y;
    double z = ore.pose.position.z;

    double distance = std::sqrt(x * x + y * y + z * z);

    if (distance < min_distance) {
      min_distance = distance;
      closest_ore_id = ore.id;
    }
  }

  return closest_ore_id;
}

}  // namespace miner_client

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(miner_client::MinerClientNode)
