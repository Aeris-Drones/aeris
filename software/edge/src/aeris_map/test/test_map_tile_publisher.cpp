#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <gtest/gtest.h>

#include "aeris_msgs/msg/map_tile.hpp"
#include "aeris_msgs/srv/get_map_tile_bytes.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#define AERIS_MAP_TILE_PUBLISHER_NO_MAIN
#include "../src/map_tile_publisher.cpp"

namespace
{

class RosRuntimeFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class TileObserver final : public rclcpp::Node
{
public:
  TileObserver()
  : Node("map_tile_publisher_test_observer")
  {
    tile_sub_ = create_subscription<aeris_msgs::msg::MapTile>(
      "/map/tiles",
      rclcpp::QoS(10),
      [this](const aeris_msgs::msg::MapTile::SharedPtr message) {
        tile_ids_.push_back(message->tile_id);
      });

    occupancy_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(10));

    tile_client_ = create_client<aeris_msgs::srv::GetMapTileBytes>("/map/get_tile_bytes");
  }

  std::vector<std::string> tile_ids_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
  rclcpp::Client<aeris_msgs::srv::GetMapTileBytes>::SharedPtr tile_client_;

private:
  rclcpp::Subscription<aeris_msgs::msg::MapTile>::SharedPtr tile_sub_;
};

nav_msgs::msg::OccupancyGrid build_occupancy_grid(double origin_x_m, double origin_y_m)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 1.0F;
  grid.info.width = 1U;
  grid.info.height = 1U;
  grid.info.origin.position.x = origin_x_m;
  grid.info.origin.position.y = origin_y_m;
  grid.data = {100};
  return grid;
}

template<typename PredicateT>
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  PredicateT predicate,
  const std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  executor.spin_some();
  return predicate();
}

}  // namespace

TEST_F(RosRuntimeFixture, RejectsRuntimeSwitchToUnimplementedBackend)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("slam_mode", "vio"),
    rclcpp::Parameter("map_source", "occupancy"),
    rclcpp::Parameter("mbtiles_path", "/tmp/aeris/test_runtime_mode_switch.mbtiles"),
  });

  auto node = std::make_shared<MapTilePublisher>(options);

  const auto result = node->set_parameter(rclcpp::Parameter("slam_mode", "liosam"));
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("not-implemented"), std::string::npos);
}

TEST_F(RosRuntimeFixture, ServesTileBytesFromMbtilesAfterCacheEviction)
{
  const auto mbtiles_path =
    (std::filesystem::temp_directory_path() / "aeris_map_tile_cache_eviction.mbtiles").string();
  std::error_code ec;
  std::filesystem::remove(mbtiles_path, ec);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("slam_mode", "vio"),
    rclcpp::Parameter("map_source", "occupancy"),
    rclcpp::Parameter("max_cached_tiles", 1),
    rclcpp::Parameter("publish_on_change_only", false),
    rclcpp::Parameter("republish_interval_sec", 0.0),
    rclcpp::Parameter("mbtiles_path", mbtiles_path),
  });

  auto map_node = std::make_shared<MapTilePublisher>(options);
  auto observer = std::make_shared<TileObserver>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(map_node);
  executor.add_node(observer);

  ASSERT_TRUE(spin_until(
      executor,
      [&]() {return observer->tile_client_->service_is_ready();},
      std::chrono::seconds(2)));

  observer->occupancy_pub_->publish(build_occupancy_grid(0.0, 0.0));
  ASSERT_TRUE(spin_until(
      executor,
      [&]() {return !observer->tile_ids_.empty();},
      std::chrono::seconds(3)));
  const std::string first_tile_id = observer->tile_ids_.front();

  observer->occupancy_pub_->publish(build_occupancy_grid(5000.0, 5000.0));
  ASSERT_TRUE(spin_until(
      executor,
      [&]() {
        return std::unordered_set<std::string>(
          observer->tile_ids_.begin(), observer->tile_ids_.end()).size() >= 2U;
      },
      std::chrono::seconds(3)));

  auto request = std::make_shared<aeris_msgs::srv::GetMapTileBytes::Request>();
  request->tile_id = first_tile_id;
  auto response_future = observer->tile_client_->async_send_request(request);

  ASSERT_EQ(
    executor.spin_until_future_complete(response_future, std::chrono::seconds(3)),
    rclcpp::FutureReturnCode::SUCCESS);

  const auto response = response_future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->found);
  EXPECT_FALSE(response->data.empty());
  EXPECT_EQ(response->byte_size, static_cast<uint32_t>(response->data.size()));
  EXPECT_FALSE(response->hash_sha256.empty());

  executor.remove_node(observer);
  executor.remove_node(map_node);
}

