// Copyright Chen Tingxu
// MIT License

#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "miner_interfaces/msg/ores.hpp"
#include "miner_interfaces/srv/mine_map.hpp"
#include "mine_service/mine_service_node.hpp"

using namespace std::chrono_literals;

TEST(MineServiceNode, TestMineCallback)
{
  rclcpp::init(0, nullptr);

  auto node_options = rclcpp::NodeOptions();
  auto test_node = std::make_shared<mine_service::MineServiceNode>(node_options);
  auto ores_pub = test_node->create_publisher<miner_interfaces::msg::Ores>("mines", 10);

  auto spin_thread = std::thread([&]() {rclcpp::spin(test_node);});
  miner_interfaces::msg::Ore ore_msg;
  ore_msg.id = 1;
  ore_msg.type = "gold";
  ore_msg.pose.position.x = 0;
  ore_msg.pose.position.y = 0;
  ore_msg.pose.position.z = 0;
  ore_msg.value = 80.8;

  miner_interfaces::msg::Ores ores_msg;
  ores_msg.ores.push_back(ore_msg);

  ores_pub->publish(ores_msg);

  std::this_thread::sleep_for(500ms);  // Allow time for the message to be received

  // Assertionsg
  EXPECT_EQ(test_node->get_ores_msg().ores[0].id, ore_msg.id);
  EXPECT_EQ(test_node->get_ores_msg().ores[0].type, ore_msg.type);
  EXPECT_EQ(test_node->get_ores_msg().ores[0].pose.position.x, ore_msg.pose.position.x);
  EXPECT_EQ(test_node->get_ores_msg().ores[0].pose.position.y, ore_msg.pose.position.y);
  EXPECT_EQ(test_node->get_ores_msg().ores[0].pose.position.z, ore_msg.pose.position.z);
  EXPECT_EQ(test_node->get_ores_msg().ores[0].value, ore_msg.value);

  rclcpp::shutdown();
  spin_thread.join();
  SUCCEED();
}

TEST(MineServiceNode, TestMineService)
{
  rclcpp::init(0, nullptr);

  auto node_options = rclcpp::NodeOptions();
  auto test_node = std::make_shared<mine_service::MineServiceNode>(node_options);
  auto client = test_node->create_client<miner_interfaces::srv::MineMap>("mine_map");

  auto spin_thread = std::thread([&]() {rclcpp::spin(test_node);});
  std::this_thread::sleep_for(500ms);  // Allow time for the service to be ready

  auto request = std::make_shared<miner_interfaces::srv::MineMap::Request>();
  request->id = 1;

  auto result_future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
      test_node,
      result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    // Assertions
    bool id_found = false;
    for (auto ore : response->ores.ores) {
      if (ore.id == request->id) {
        id_found = true;
        break;
      }
    }
    EXPECT_FALSE(id_found);
  } else {
    ADD_FAILURE() << "Failed to call service";
  }

  rclcpp::shutdown();
  spin_thread.join();
  SUCCEED();
}
