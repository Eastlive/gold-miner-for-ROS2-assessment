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
MineServiceNode::MineServiceNode() : Node("mine_service")
{
    subscription_ = this->create_subscription<miner_interfaces::msg::Ores>(
        "mines", 10, std::bind(&MineServiceNode::mine_callback, this, std::placeholders::_1)
    );
    service_ = this->create_service<miner_interfaces::srv::MineMap>(
        "mine_map", std::bind(&MineServiceNode::mine_service, this, std::placeholders::_1, std::placeholders::_2)
    );
}

void MineServiceNode::mine_callback(const miner_interfaces::msg::Ores ores)
{
    ores_msg_ = ores;
}

void MineServiceNode::mine_service(const std::shared_ptr<miner_interfaces::srv::MineMap::Request> request,
const std::shared_ptr<miner_interfaces::srv::MineMap::Response> respense)
{
    ;
}
} // namespace mine_service

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mine_service::MineServiceNode)