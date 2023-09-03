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
  void mine_callback(const miner_interfaces::msg::Ores ores);
  void check_and_proceed();
  int32_t find_aim_ore(const miner_interfaces::msg::Ores ore_array);
  void send_request(const int32_t id);
  void result_callback(rclcpp::Client<miner_interfaces::srv::MineMap>::SharedFuture result_future);

  rclcpp::Subscription<miner_interfaces::msg::Ores>::SharedPtr subscription_;
  rclcpp::Client<miner_interfaces::srv::MineMap>::SharedPtr client_;

  bool received_message_ = false;
  rclcpp::TimerBase::SharedPtr timer_;

  miner_interfaces::msg::Ores origin_ores_msg_;
  miner_interfaces::msg::Ores recent_ores_msg_;
};
}

#endif // MINER_CLIENT_MINE_CLIENT_NODE_HPP_
