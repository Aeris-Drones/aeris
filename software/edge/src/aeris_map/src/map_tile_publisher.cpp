#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <png.h>
#include <sqlite3.h>

#include "aeris_msgs/msg/map_tile.hpp"
#include "aeris_msgs/srv/get_map_tile_bytes.hpp"
#include "aeris_map/tile_contract.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr char kTileTopic[] = "/map/tiles";
constexpr char kOccupancyTopic[] = "/map";
constexpr char kGetTileService[] = "/map/get_tile_bytes";
constexpr int kDefaultTileSizePx = 256;
constexpr int kDefaultZoom = 18;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMetersPerDegreeLat = 111320.0;

struct TilePayload
{
  std::vector<uint8_t> data;
  std::string hash_sha256;
  uint32_t byte_size{0};
  builtin_interfaces::msg::Time published_at;
};

struct MemoryWriter
{
  std::vector<uint8_t> * out;
};

void png_write_callback(png_structp png_ptr, png_bytep data, png_size_t length)
{
  auto * writer = static_cast<MemoryWriter *>(png_get_io_ptr(png_ptr));
  writer->out->insert(writer->out->end(), data, data + length);
}

void png_flush_callback(png_structp)
{
}

std::pair<int, int> slippy_xy_from_lat_lon(double lat_deg, double lon_deg, int zoom)
{
  const double lat = std::clamp(lat_deg, -85.05112878, 85.05112878);
  const double lon = std::clamp(lon_deg, -180.0, 180.0);
  const double lat_rad = lat * kPi / 180.0;
  const double n = std::pow(2.0, zoom);

  int xtile = static_cast<int>(std::floor((lon + 180.0) / 360.0 * n));
  int ytile = static_cast<int>(std::floor((1.0 - std::asinh(std::tan(lat_rad)) / kPi) / 2.0 * n));

  const int max_index = static_cast<int>(n) - 1;
  xtile = std::clamp(xtile, 0, max_index);
  ytile = std::clamp(ytile, 0, max_index);
  return {xtile, ytile};
}

std::pair<double, double> local_xy_to_lat_lon(
  double x_m,
  double y_m,
  double origin_lat_deg,
  double origin_lon_deg)
{
  const double lat = origin_lat_deg + (y_m / kMetersPerDegreeLat);
  const double cos_lat = std::cos(origin_lat_deg * kPi / 180.0);
  const double meters_per_degree_lon = std::max(1e-6, kMetersPerDegreeLat * cos_lat);
  const double lon = origin_lon_deg + (x_m / meters_per_degree_lon);
  return {lat, lon};
}

std::array<uint8_t, 4> occupancy_to_rgba(int8_t value)
{
  if (value < 0) {
    return {127, 127, 127, 220};
  }
  const uint8_t clamped = static_cast<uint8_t>(std::clamp<int>(value, 0, 100));
  const uint8_t intensity = static_cast<uint8_t>(255 - (clamped * 2));
  return {intensity, intensity, intensity, 255};
}

std::vector<uint8_t> encode_png_rgba(
  const std::vector<uint8_t> & rgba,
  int width,
  int height)
{
  std::vector<uint8_t> out;
  MemoryWriter writer{&out};

  png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
  if (!png_ptr) {
    throw std::runtime_error("png_create_write_struct failed");
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    png_destroy_write_struct(&png_ptr, nullptr);
    throw std::runtime_error("png_create_info_struct failed");
  }

  if (setjmp(png_jmpbuf(png_ptr))) {
    png_destroy_write_struct(&png_ptr, &info_ptr);
    throw std::runtime_error("PNG encoding failed");
  }

  png_set_write_fn(png_ptr, &writer, png_write_callback, png_flush_callback);
  png_set_IHDR(
    png_ptr,
    info_ptr,
    static_cast<png_uint_32>(width),
    static_cast<png_uint_32>(height),
    8,
    PNG_COLOR_TYPE_RGBA,
    PNG_INTERLACE_NONE,
    PNG_COMPRESSION_TYPE_BASE,
    PNG_FILTER_TYPE_BASE);

  png_write_info(png_ptr, info_ptr);

  std::vector<png_bytep> row_pointers(static_cast<size_t>(height));
  for (int y = 0; y < height; ++y) {
    row_pointers[static_cast<size_t>(y)] = const_cast<png_bytep>(&rgba[static_cast<size_t>(y) * static_cast<size_t>(width) * 4U]);
  }

  png_write_image(png_ptr, row_pointers.data());
  png_write_end(png_ptr, nullptr);
  png_destroy_write_struct(&png_ptr, &info_ptr);

  return out;
}

int tms_row_from_slippy(int z, int y)
{
  return ((1 << z) - 1) - y;
}

}  // namespace

class MapTilePublisher : public rclcpp::Node
{
public:
  MapTilePublisher()
  : rclcpp::Node("aeris_map_tile_publisher")
  {
    queue_depth_ = static_cast<int>(this->declare_parameter<int64_t>("queue_depth", 50));
    tile_topic_ = this->declare_parameter<std::string>("topic", kTileTopic);
    occupancy_topic_ = this->declare_parameter<std::string>("occupancy_topic", kOccupancyTopic);
    tile_service_name_ = this->declare_parameter<std::string>("tile_service_name", kGetTileService);
    const int zoom_param = static_cast<int>(this->declare_parameter<int64_t>("zoom", kDefaultZoom));
    zoom_ = std::clamp(zoom_param, 0, 22);
    const int tile_size_param =
      static_cast<int>(this->declare_parameter<int64_t>("tile_size_px", kDefaultTileSizePx));
    tile_size_px_ = std::max(32, tile_size_param);
    origin_lat_deg_ = this->declare_parameter<double>("origin_lat_deg", 37.7749);
    origin_lon_deg_ = this->declare_parameter<double>("origin_lon_deg", -122.4194);
    publish_on_change_only_ = this->declare_parameter<bool>("publish_on_change_only", true);
    const int max_cached_tiles_param =
      static_cast<int>(this->declare_parameter<int64_t>("max_cached_tiles", 500));
    max_cached_tiles_ = static_cast<size_t>(std::max(10, max_cached_tiles_param));
    mbtiles_path_ = this->declare_parameter<std::string>("mbtiles_path", "/tmp/aeris/map_tiles/live_map.mbtiles");
    layer_ids_ = this->declare_parameter<std::vector<std::string>>(
      "layer_ids",
      std::vector<std::string>{"occupancy", "fetch-service:/map/get_tile_bytes", "mime:image/png"});

    initialize_mbtiles();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_depth_))).reliable().durability_volatile();
    publisher_ = this->create_publisher<aeris_msgs::msg::MapTile>(tile_topic_, qos);

    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      occupancy_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MapTilePublisher::handle_occupancy_grid, this, std::placeholders::_1));

    tile_service_ = this->create_service<aeris_msgs::srv::GetMapTileBytes>(
      tile_service_name_,
      std::bind(
        &MapTilePublisher::handle_get_tile_bytes,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Map tile stream initialized. occupancy_topic=%s publish_topic=%s qos_depth=%d service=%s mbtiles=%s zoom=%d",
      occupancy_topic_.c_str(),
      tile_topic_.c_str(),
      queue_depth_,
      tile_service_name_.c_str(),
      mbtiles_path_.c_str(),
      zoom_);
    RCLCPP_INFO(
      this->get_logger(),
      "QoS rationale: reliable + keep_last(%d) to avoid tile drops while bounding queue memory.",
      queue_depth_);
  }

  ~MapTilePublisher() override
  {
    if (db_ != nullptr) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
  }

private:
  void initialize_mbtiles()
  {
    const std::filesystem::path db_path(mbtiles_path_);
    if (db_path.has_parent_path()) {
      std::filesystem::create_directories(db_path.parent_path());
    }

    if (sqlite3_open(mbtiles_path_.c_str(), &db_) != SQLITE_OK) {
      throw std::runtime_error(
              std::string("Failed to open MBTiles DB: ") + sqlite3_errmsg(db_));
    }

    exec_sql(
      "CREATE TABLE IF NOT EXISTS metadata (name TEXT PRIMARY KEY, value TEXT);");
    exec_sql(
      "CREATE TABLE IF NOT EXISTS tiles ("
      "zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB,"
      "PRIMARY KEY (zoom_level, tile_column, tile_row));");

    upsert_metadata("name", "aeris-live-map");
    upsert_metadata("type", "overlay");
    upsert_metadata("version", "1.3");
    upsert_metadata("format", "png");
    upsert_metadata("minzoom", std::to_string(zoom_));
    upsert_metadata("maxzoom", std::to_string(zoom_));
    upsert_metadata("bounds", "-180.0,-85.0511,180.0,85.0511");
  }

  void exec_sql(const std::string & sql)
  {
    char * err_msg = nullptr;
    const int rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err_msg);
    if (rc != SQLITE_OK) {
      const std::string err = err_msg != nullptr ? err_msg : "unknown sqlite error";
      sqlite3_free(err_msg);
      throw std::runtime_error("SQLite error: " + err + " SQL=" + sql);
    }
  }

  void upsert_metadata(const std::string & key, const std::string & value)
  {
    sqlite3_stmt * stmt = nullptr;
    constexpr const char * kSql =
      "INSERT INTO metadata(name, value) VALUES(?, ?) "
      "ON CONFLICT(name) DO UPDATE SET value=excluded.value;";

    if (sqlite3_prepare_v2(db_, kSql, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare metadata upsert statement");
    }

    sqlite3_bind_text(stmt, 1, key.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, value.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Failed to execute metadata upsert statement");
    }
    sqlite3_finalize(stmt);
  }

  void upsert_tile_blob(int z, int x, int y, const std::vector<uint8_t> & bytes)
  {
    sqlite3_stmt * stmt = nullptr;
    constexpr const char * kSql =
      "INSERT INTO tiles(zoom_level, tile_column, tile_row, tile_data) VALUES(?, ?, ?, ?) "
      "ON CONFLICT(zoom_level, tile_column, tile_row) DO UPDATE SET tile_data=excluded.tile_data;";

    if (sqlite3_prepare_v2(db_, kSql, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare tile upsert statement");
    }

    sqlite3_bind_int(stmt, 1, z);
    sqlite3_bind_int(stmt, 2, x);
    sqlite3_bind_int(stmt, 3, tms_row_from_slippy(z, y));
    sqlite3_bind_blob(stmt, 4, bytes.data(), static_cast<int>(bytes.size()), SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Failed to execute tile upsert statement");
    }
    sqlite3_finalize(stmt);
  }

  std::vector<uint8_t> render_tile_png(const nav_msgs::msg::OccupancyGrid & grid) const
  {
    if (grid.info.width == 0 || grid.info.height == 0 || grid.data.empty()) {
      return {};
    }

    const auto width = static_cast<size_t>(grid.info.width);
    const auto height = static_cast<size_t>(grid.info.height);

    std::vector<uint8_t> rgba(static_cast<size_t>(tile_size_px_) * static_cast<size_t>(tile_size_px_) * 4U, 0U);

    for (int py = 0; py < tile_size_px_; ++py) {
      for (int px = 0; px < tile_size_px_; ++px) {
        const size_t sx = std::min(
          width - 1,
          static_cast<size_t>((static_cast<double>(px) / static_cast<double>(tile_size_px_)) * static_cast<double>(width)));
        const size_t sy = std::min(
          height - 1,
          static_cast<size_t>((static_cast<double>(py) / static_cast<double>(tile_size_px_)) * static_cast<double>(height)));

        const size_t source_index = (height - 1 - sy) * width + sx;
        if (source_index >= grid.data.size()) {
          continue;
        }

        const auto color = occupancy_to_rgba(grid.data[source_index]);
        const size_t out_index = (static_cast<size_t>(py) * static_cast<size_t>(tile_size_px_) + static_cast<size_t>(px)) * 4U;
        rgba[out_index] = color[0];
        rgba[out_index + 1] = color[1];
        rgba[out_index + 2] = color[2];
        rgba[out_index + 3] = color[3];
      }
    }

    return encode_png_rgba(rgba, tile_size_px_, tile_size_px_);
  }

  void handle_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    try {
      const auto bytes = render_tile_png(*msg);
      if (bytes.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Skipping publish: occupancy grid had no usable data");
        return;
      }

      const double center_x_m = msg->info.origin.position.x + (msg->info.resolution * static_cast<double>(msg->info.width) / 2.0);
      const double center_y_m = msg->info.origin.position.y + (msg->info.resolution * static_cast<double>(msg->info.height) / 2.0);
      const auto [center_lat, center_lon] = local_xy_to_lat_lon(center_x_m, center_y_m, origin_lat_deg_, origin_lon_deg_);
      const auto [xtile, ytile] = slippy_xy_from_lat_lon(center_lat, center_lon, zoom_);

      std::ostringstream id;
      id << zoom_ << "/" << xtile << "/" << ytile;
      const std::string tile_id = id.str();
      if (!aeris_map::tile_contract::is_valid_tile_id(tile_id)) {
        RCLCPP_ERROR(this->get_logger(), "Skipping invalid tile id generated from map frame: %s", tile_id.c_str());
        return;
      }

      const std::string hash = aeris_map::tile_contract::sha256_hex(bytes);

      {
        std::scoped_lock<std::mutex> lock(cache_mutex_);
        const auto prev = tile_cache_.find(tile_id);
        if (publish_on_change_only_ && prev != tile_cache_.end() && prev->second.hash_sha256 == hash) {
          return;
        }
      }

      upsert_tile_blob(zoom_, xtile, ytile, bytes);

      const auto descriptor = aeris_map::tile_contract::build_descriptor(
        tile_id,
        layer_ids_,
        hash,
        static_cast<uint32_t>(bytes.size()));

      publisher_->publish(descriptor);

      TilePayload payload;
      payload.data = bytes;
      payload.hash_sha256 = hash;
      payload.byte_size = descriptor.byte_size;
      payload.published_at = this->now();

      {
        std::scoped_lock<std::mutex> lock(cache_mutex_);
        if (tile_cache_.find(tile_id) == tile_cache_.end()) {
          cache_order_.push_back(tile_id);
        }
        tile_cache_[tile_id] = std::move(payload);

        while (cache_order_.size() > max_cached_tiles_) {
          const std::string oldest = cache_order_.front();
          cache_order_.pop_front();
          tile_cache_.erase(oldest);
        }
      }

      RCLCPP_DEBUG(
        this->get_logger(),
        "Published map tile %s size=%u hash=%s",
        tile_id.c_str(),
        descriptor.byte_size,
        descriptor.hash_sha256.c_str());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to process occupancy grid into tile: %s", ex.what());
    }
  }

  void handle_get_tile_bytes(
    const std::shared_ptr<aeris_msgs::srv::GetMapTileBytes::Request> request,
    std::shared_ptr<aeris_msgs::srv::GetMapTileBytes::Response> response)
  {
    if (!aeris_map::tile_contract::is_valid_tile_id(request->tile_id)) {
      response->found = false;
      response->content_type = "image/png";
      response->hash_sha256.clear();
      response->byte_size = 0;
      response->data.clear();
      return;
    }

    std::scoped_lock<std::mutex> lock(cache_mutex_);
    const auto it = tile_cache_.find(request->tile_id);
    if (it == tile_cache_.end()) {
      response->found = false;
      response->content_type = "image/png";
      response->hash_sha256.clear();
      response->byte_size = 0;
      response->data.clear();
      return;
    }

    response->found = true;
    response->content_type = "image/png";
    response->hash_sha256 = it->second.hash_sha256;
    response->byte_size = it->second.byte_size;
    response->published_at = it->second.published_at;
    response->data = it->second.data;
  }

  int queue_depth_{50};
  int zoom_{kDefaultZoom};
  int tile_size_px_{kDefaultTileSizePx};
  double origin_lat_deg_{37.7749};
  double origin_lon_deg_{-122.4194};
  bool publish_on_change_only_{true};
  size_t max_cached_tiles_{500};

  std::string tile_topic_;
  std::string occupancy_topic_;
  std::string tile_service_name_;
  std::string mbtiles_path_;
  std::vector<std::string> layer_ids_;

  sqlite3 * db_{nullptr};

  rclcpp::Publisher<aeris_msgs::msg::MapTile>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Service<aeris_msgs::srv::GetMapTileBytes>::SharedPtr tile_service_;

  std::mutex cache_mutex_;
  std::unordered_map<std::string, TilePayload> tile_cache_;
  std::deque<std::string> cache_order_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MapTilePublisher>());
  } catch (const std::exception & ex) {
    std::fprintf(stderr, "map_tile_publisher fatal error: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
