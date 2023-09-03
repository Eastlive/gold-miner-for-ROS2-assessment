// Copyright 2023 Chen Tingxu
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
MinerClientNode::MinerClientNode(const rclcpp::NodeOptions& options) : Node("miner_client", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting Miner Client Node");
  subscription_ = this->create_subscription<miner_interfaces::msg::Ores>(
      "mines", 10, std::bind(&MinerClientNode::mine_callback, this, std::placeholders::_1));

  recent_ores_msg_ = origin_ores_msg_;

  client_ = this->create_client<miner_interfaces::srv::MineMap>("mine_map");

  while (recent_ores_msg_.ores.size())
  {
    auto id = find_aim_ore(recent_ores_msg_);
    if (id == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "No found closest ore.");
      break;
    }
    RCLCPP_INFO(this->get_logger(), "Excavate No.%d ore.", id);
    send_request(id);
  }

  if(!recent_ores_msg_.ores.size())
  {
    RCLCPP_INFO(this->get_logger(), "Miner completed.");
  }
}

void MinerClientNode::mine_callback(const miner_interfaces::msg::Ores ores)
{
  RCLCPP_INFO(this->get_logger(), "Received ores.");
  origin_ores_msg_ = ores;
}

int32_t MinerClientNode::find_aim_ore(const miner_interfaces::msg::Ores ore_array)
{
  double min_distance = std::numeric_limits<double>::max();
  int32_t closest_ore_id = -1;

  for (const auto& ore : ore_array.ores)
  {
    double x = ore.pose.position.x;
    double y = ore.pose.position.y;
    double z = ore.pose.position.z;

    double distance = std::sqrt(x * x + y * y + z * z);

    if (distance < min_distance)
    {
      min_distance = distance;
      closest_ore_id = ore.id;
    }
  }

  return closest_ore_id;
}

void MinerClientNode::send_request(const int32_t id)
{
  while (!client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while watting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<miner_interfaces::srv::MineMap::Request>();
  request->id = id;

  client_->async_send_request(request, std::bind(&MinerClientNode::result_callback, this, std::placeholders::_1));
}

void MinerClientNode::result_callback(rclcpp::Client<miner_interfaces::srv::MineMap>::SharedFuture result_future)
{
    auto result = result_future.get();

    if(result)
    {
        recent_ores_msg_ = result->ores;
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "Cannot recieve response.");
    }
}
}  // namespace miner_client

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(miner_client::MinerClientNode)