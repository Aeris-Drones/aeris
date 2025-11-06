#include <array>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "aeris_msgs/msg/map_tile.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr char kDefaultTopic[] = "map/tiles";
constexpr std::array<const char *, 4> kDefaultLayers = {"base", "thermal", "gas", "acoustic"};
}  // namespace

class MapTilePublisher : public rclcpp::Node
{
public:
  MapTilePublisher()
  : rclcpp::Node("aeris_map_tile_publisher"), publish_count_(0)
  {
    const auto queue_depth = static_cast<size_t>(
      this->declare_parameter<int>("queue_depth", 10));
    const auto topic = this->declare_parameter<std::string>("topic", kDefaultTopic);
    const auto interval_ms = this->declare_parameter<int>("publish_interval_ms", 2000);

    publisher_ = this->create_publisher<aeris_msgs::msg::MapTile>(topic, queue_depth);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(interval_ms),
      std::bind(&MapTilePublisher::publish_tile, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing fake map tiles on '%s' every %d ms (depth=%zu).",
      topic.c_str(), interval_ms, queue_depth);
  }

private:
  void publish_tile()
  {
    aeris_msgs::msg::MapTile tile_msg;
    tile_msg.tile_id = generate_tile_id();
    tile_msg.format = "mbtiles-1.3";
    tile_msg.layer_ids.assign(kDefaultLayers.begin(), kDefaultLayers.end());
    tile_msg.hash_sha256 = generate_fake_hash();
    tile_msg.byte_size = 32768 + (publish_count_ % 10) * 1024;

    publisher_->publish(tile_msg);
    ++publish_count_;

    RCLCPP_DEBUG(
      this->get_logger(),
      "Published fake tile id=%s size=%u.",
      tile_msg.tile_id.c_str(), tile_msg.byte_size);
  }

  std::string generate_tile_id() const
  {
    std::ostringstream oss;
    oss << "0/0/" << publish_count_;
    return oss.str();
  }

  std::string generate_fake_hash() const
  {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (int i = 0; i < 8; ++i) {
      oss << std::setw(8) << (publish_count_ + i);
    }
    return oss.str();
  }

  rclcpp::Publisher<aeris_msgs::msg::MapTile>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t publish_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapTilePublisher>());
  rclcpp::shutdown();
  return 0;
}
