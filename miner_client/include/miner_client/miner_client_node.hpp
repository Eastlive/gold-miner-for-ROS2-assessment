// Copyright Chen Tingxu
// License MIT

#ifndef MINER_CLIENT__MINER_CLIENT_NODE_HPP_
#define MINER_CLIENT__MINER_CLIENT_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "miner_interfaces/msg/ores.hpp"
#include "miner_interfaces/srv/mine_map.hpp"

namespace miner_client
{
class MinerClientNode : public rclcpp::Node
{
public:
  explicit MinerClientNode(const rclcpp::NodeOptions & option);
  ~MinerClientNode() {}

private:
  void result_callback(rclcpp::Client<miner_interfaces::srv::MineMap>::SharedFuture result_future);

  rclcpp::Client<miner_interfaces::srv::MineMap>::SharedPtr client_;
};
}

#endif // MINER_CLIENT_MINE_CLIENT_NODE_HPP_