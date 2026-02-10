#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <png.h>
#include <sqlite3.h>

#include "aeris_map/tile_contract.hpp"
#include "aeris_msgs/msg/map_tile.hpp"
#include "aeris_msgs/srv/get_map_tile_bytes.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
constexpr char kTileTopic[] = "/map/tiles";
constexpr char kOccupancyTopic[] = "/map";
constexpr char kPointCloudTopic[] = "/rtabmap/cloud_map";
constexpr char kGetTileService[] = "/map/get_tile_bytes";

constexpr char kMapSourceOccupancy[] = "occupancy";
constexpr char kMapSourcePointCloud[] = "point_cloud";
constexpr char kMapSourceHybrid[] = "hybrid";

constexpr int kDefaultTileSizePx = 256;
constexpr int kDefaultZoom = 18;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMetersPerDegreeLat = 111320.0;
constexpr double kMaxWebMercatorLatDeg = 85.05112878;

using TileGridKey = std::pair<int, int>;

struct TilePayload
{
  std::vector<uint8_t> data;
  std::string hash_sha256;
  uint32_t byte_size{0};
  builtin_interfaces::msg::Time published_at;
};

struct TileRaster
{
  int x{0};
  int y{0};
  std::vector<uint8_t> rgba;
  bool touched{false};
};

struct SlippyPixel
{
  int xtile{0};
  int ytile{0};
  int px{0};
  int py{0};
};

struct MemoryWriter
{
  std::vector<uint8_t> * out;
};

std::string default_mbtiles_path()
{
  const char * xdg_cache_home = std::getenv("XDG_CACHE_HOME");
  if (xdg_cache_home != nullptr && xdg_cache_home[0] != '\0') {
    return std::string(xdg_cache_home) + "/aeris/map_tiles/live_map.mbtiles";
  }

  const char * home = std::getenv("HOME");
  if (home != nullptr && home[0] != '\0') {
    return std::string(home) + "/.cache/aeris/map_tiles/live_map.mbtiles";
  }

  return "/tmp/aeris/map_tiles/live_map.mbtiles";
}

void png_write_callback(png_structp png_ptr, png_bytep data, png_size_t length)
{
  auto * writer = static_cast<MemoryWriter *>(png_get_io_ptr(png_ptr));
  writer->out->insert(writer->out->end(), data, data + length);
}

void png_flush_callback(png_structp)
{
}

int tms_row_from_slippy(int z, int y)
{
  return ((1 << z) - 1) - y;
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

std::optional<SlippyPixel> slippy_pixel_from_lat_lon(
  double lat_deg,
  double lon_deg,
  int zoom,
  int tile_size_px)
{
  if (zoom < 0 || zoom > 22 || tile_size_px <= 0) {
    return std::nullopt;
  }

  const double clamped_lat = std::clamp(lat_deg, -kMaxWebMercatorLatDeg, kMaxWebMercatorLatDeg);
  const double clamped_lon = std::clamp(lon_deg, -180.0, 180.0);
  const double lat_rad = clamped_lat * kPi / 180.0;
  const double n = std::pow(2.0, zoom);

  const double x_pixel_global = ((clamped_lon + 180.0) / 360.0) * n * static_cast<double>(tile_size_px);
  const double y_pixel_global =
    ((1.0 - std::asinh(std::tan(lat_rad)) / kPi) / 2.0) * n * static_cast<double>(tile_size_px);

  int xtile = static_cast<int>(std::floor(x_pixel_global / static_cast<double>(tile_size_px)));
  int ytile = static_cast<int>(std::floor(y_pixel_global / static_cast<double>(tile_size_px)));

  const int max_index = static_cast<int>(n) - 1;
  xtile = std::clamp(xtile, 0, max_index);
  ytile = std::clamp(ytile, 0, max_index);

  const int pixel_floor_x = static_cast<int>(std::floor(x_pixel_global));
  const int pixel_floor_y = static_cast<int>(std::floor(y_pixel_global));

  int px = pixel_floor_x - (xtile * tile_size_px);
  int py = pixel_floor_y - (ytile * tile_size_px);
  px = std::clamp(px, 0, tile_size_px - 1);
  py = std::clamp(py, 0, tile_size_px - 1);

  return SlippyPixel{xtile, ytile, px, py};
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

std::array<uint8_t, 4> point_to_rgba(float z_m)
{
  const double clamped_z = std::clamp(static_cast<double>(z_m), -10.0, 40.0);
  const double normalized = (clamped_z + 10.0) / 50.0;
  const uint8_t red = static_cast<uint8_t>(60.0 + normalized * 195.0);
  const uint8_t green = static_cast<uint8_t>(255.0 - normalized * 120.0);
  const uint8_t blue = static_cast<uint8_t>(180.0 - normalized * 120.0);
  return {red, green, blue, 255};
}

std::vector<uint8_t> encode_png_rgba(const std::vector<uint8_t> & rgba, int width, int height)
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
    row_pointers[static_cast<size_t>(y)] = const_cast<png_bytep>(
      &rgba[static_cast<size_t>(y) * static_cast<size_t>(width) * 4U]);
  }

  png_write_image(png_ptr, row_pointers.data());
  png_write_end(png_ptr, nullptr);
  png_destroy_write_struct(&png_ptr, &info_ptr);

  return out;
}

std::string normalize_source_mode(std::string mode)
{
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (mode == "occupancy" || mode == "grid") {
    return kMapSourceOccupancy;
  }
  if (mode == "pointcloud" || mode == "point_cloud" || mode == "cloud") {
    return kMapSourcePointCloud;
  }
  if (mode == "hybrid" || mode == "both") {
    return kMapSourceHybrid;
  }

  throw std::invalid_argument(
          "map_source must be one of: occupancy, point_cloud, hybrid");
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
    point_cloud_topic_ =
      this->declare_parameter<std::string>("point_cloud_topic", kPointCloudTopic);
    tile_service_name_ = this->declare_parameter<std::string>("tile_service_name", kGetTileService);

    const int zoom_param = static_cast<int>(this->declare_parameter<int64_t>("zoom", kDefaultZoom));
    zoom_ = std::clamp(zoom_param, 0, 22);

    const int tile_size_param =
      static_cast<int>(this->declare_parameter<int64_t>("tile_size_px", kDefaultTileSizePx));
    tile_size_px_ = std::max(32, tile_size_param);

    origin_lat_deg_ = this->declare_parameter<double>("origin_lat_deg", 37.7749);
    origin_lon_deg_ = this->declare_parameter<double>("origin_lon_deg", -122.4194);

    publish_on_change_only_ = this->declare_parameter<bool>("publish_on_change_only", true);
    republish_interval_sec_ =
      std::max(0.0, this->declare_parameter<double>("republish_interval_sec", 5.0));

    const int max_cached_tiles_param =
      static_cast<int>(this->declare_parameter<int64_t>("max_cached_tiles", 500));
    max_cached_tiles_ = static_cast<size_t>(std::max(10, max_cached_tiles_param));

    mbtiles_path_ =
      this->declare_parameter<std::string>("mbtiles_path", default_mbtiles_path());

    map_source_ = normalize_source_mode(
      this->declare_parameter<std::string>("map_source", kMapSourceOccupancy));

    const std::vector<std::string> default_layer_ids =
      (map_source_ == kMapSourcePointCloud)
      ? std::vector<std::string>{
        "point_cloud", "fetch-service:/map/get_tile_bytes", "mime:image/png"}
      : std::vector<std::string>{
        "occupancy", "fetch-service:/map/get_tile_bytes", "mime:image/png"};

    layer_ids_ = this->declare_parameter<std::vector<std::string>>(
      "layer_ids",
      default_layer_ids);

    initialize_mbtiles();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_depth_)))
                 .reliable()
                 .durability_volatile();
    publisher_ = this->create_publisher<aeris_msgs::msg::MapTile>(tile_topic_, qos);

    const bool use_occupancy =
      (map_source_ == kMapSourceOccupancy) || (map_source_ == kMapSourceHybrid);
    const bool use_point_cloud =
      (map_source_ == kMapSourcePointCloud) || (map_source_ == kMapSourceHybrid);

    if (use_occupancy) {
      occupancy_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        occupancy_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&MapTilePublisher::handle_occupancy_grid, this, std::placeholders::_1));
    }

    if (use_point_cloud) {
      point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&MapTilePublisher::handle_point_cloud, this, std::placeholders::_1));
    }

    tile_service_ = this->create_service<aeris_msgs::srv::GetMapTileBytes>(
      tile_service_name_,
      std::bind(
        &MapTilePublisher::handle_get_tile_bytes,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "Map tile stream initialized. source=%s occupancy_topic=%s point_cloud_topic=%s publish_topic=%s service=%s zoom=%d mbtiles=%s",
      map_source_.c_str(),
      occupancy_topic_.c_str(),
      point_cloud_topic_.c_str(),
      tile_topic_.c_str(),
      tile_service_name_.c_str(),
      zoom_,
      mbtiles_path_.c_str());
    RCLCPP_INFO(
      this->get_logger(),
      "QoS rationale: reliable + keep_last(%d) to avoid tile drops while bounding queue memory.",
      queue_depth_);
    RCLCPP_INFO(
      this->get_logger(),
      "Change policy: publish_on_change_only=%s, republish_interval_sec=%.2f",
      publish_on_change_only_ ? "true" : "false",
      republish_interval_sec_);
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
      const std::string err = db_ != nullptr ? sqlite3_errmsg(db_) : "unknown sqlite open failure";
      if (db_ != nullptr) {
        sqlite3_close(db_);
        db_ = nullptr;
      }
      throw std::runtime_error("Failed to open MBTiles DB: " + err);
    }

    exec_sql("CREATE TABLE IF NOT EXISTS metadata (name TEXT PRIMARY KEY, value TEXT);");
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

  TileRaster & ensure_tile(
    std::map<TileGridKey, TileRaster> & rasters,
    int xtile,
    int ytile) const
  {
    const TileGridKey key{xtile, ytile};
    auto [it, inserted] = rasters.emplace(
      key,
      TileRaster{
        xtile,
        ytile,
        std::vector<uint8_t>(
          static_cast<size_t>(tile_size_px_) * static_cast<size_t>(tile_size_px_) * 4U,
          0U),
        false});

    if (inserted) {
      it->second.x = xtile;
      it->second.y = ytile;
    }

    return it->second;
  }

  void put_pixel(
    TileRaster & tile,
    int px,
    int py,
    const std::array<uint8_t, 4> & rgba) const
  {
    if (px < 0 || py < 0 || px >= tile_size_px_ || py >= tile_size_px_) {
      return;
    }

    const size_t out_index =
      (static_cast<size_t>(py) * static_cast<size_t>(tile_size_px_) + static_cast<size_t>(px)) * 4U;
    tile.rgba[out_index] = rgba[0];
    tile.rgba[out_index + 1] = rgba[1];
    tile.rgba[out_index + 2] = rgba[2];
    tile.rgba[out_index + 3] = rgba[3];
    tile.touched = true;
  }

  std::map<TileGridKey, TileRaster> rasterize_occupancy(
    const nav_msgs::msg::OccupancyGrid & grid) const
  {
    std::map<TileGridKey, TileRaster> rasters;

    if (grid.info.width == 0 || grid.info.height == 0 || grid.data.empty()) {
      return rasters;
    }

    const size_t width = static_cast<size_t>(grid.info.width);
    const size_t height = static_cast<size_t>(grid.info.height);
    const double resolution = static_cast<double>(grid.info.resolution);

    for (size_t gy = 0; gy < height; ++gy) {
      for (size_t gx = 0; gx < width; ++gx) {
        const size_t index = gy * width + gx;
        if (index >= grid.data.size()) {
          continue;
        }

        const double local_x_m =
          grid.info.origin.position.x + ((static_cast<double>(gx) + 0.5) * resolution);
        const double local_y_m =
          grid.info.origin.position.y + ((static_cast<double>(gy) + 0.5) * resolution);

        const auto [lat, lon] =
          local_xy_to_lat_lon(local_x_m, local_y_m, origin_lat_deg_, origin_lon_deg_);
        const auto slippy = slippy_pixel_from_lat_lon(lat, lon, zoom_, tile_size_px_);
        if (!slippy.has_value()) {
          continue;
        }

        auto & tile = ensure_tile(rasters, slippy->xtile, slippy->ytile);
        put_pixel(tile, slippy->px, slippy->py, occupancy_to_rgba(grid.data[index]));
      }
    }

    return rasters;
  }

  std::map<TileGridKey, TileRaster> rasterize_point_cloud(
    const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    std::map<TileGridKey, TileRaster> rasters;

    if (cloud.width == 0 || cloud.height == 0 || cloud.data.empty()) {
      return rasters;
    }

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const float x_m = *iter_x;
        const float y_m = *iter_y;
        const float z_m = *iter_z;

        if (!std::isfinite(x_m) || !std::isfinite(y_m) || !std::isfinite(z_m)) {
          continue;
        }

        const auto [lat, lon] = local_xy_to_lat_lon(
          static_cast<double>(x_m),
          static_cast<double>(y_m),
          origin_lat_deg_,
          origin_lon_deg_);

        const auto slippy = slippy_pixel_from_lat_lon(lat, lon, zoom_, tile_size_px_);
        if (!slippy.has_value()) {
          continue;
        }

        auto & tile = ensure_tile(rasters, slippy->xtile, slippy->ytile);
        put_pixel(tile, slippy->px, slippy->py, point_to_rgba(z_m));
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Point cloud rasterization failed (missing x/y/z fields?): %s",
        ex.what());
      return {};
    }

    return rasters;
  }

  void publish_tile(
    int xtile,
    int ytile,
    const std::vector<uint8_t> & bytes,
    const rclcpp::Time & publish_time)
  {
    if (bytes.empty()) {
      return;
    }

    std::ostringstream id;
    id << zoom_ << "/" << xtile << "/" << ytile;
    const std::string tile_id = id.str();
    if (!aeris_map::tile_contract::is_valid_tile_id(tile_id)) {
      RCLCPP_ERROR(this->get_logger(), "Skipping invalid tile id: %s", tile_id.c_str());
      return;
    }

    const std::string hash = aeris_map::tile_contract::sha256_hex(bytes);

    const auto descriptor = aeris_map::tile_contract::build_descriptor(
      tile_id,
      layer_ids_,
      hash,
      static_cast<uint32_t>(bytes.size()));

    TilePayload payload;
    payload.data = bytes;
    payload.hash_sha256 = hash;
    payload.byte_size = descriptor.byte_size;
    payload.published_at = publish_time;

    {
      std::scoped_lock<std::mutex> lock(cache_mutex_);
      const auto prev = tile_cache_.find(tile_id);
      if (prev != tile_cache_.end() && prev->second.hash_sha256 == hash && publish_on_change_only_) {
        const rclcpp::Time previous_publish_time(prev->second.published_at);
        const double elapsed = (publish_time - previous_publish_time).seconds();
        if (elapsed < republish_interval_sec_) {
          return;
        }
      }

      upsert_tile_blob(zoom_, xtile, ytile, bytes);

      if (prev == tile_cache_.end()) {
        cache_order_.push_back(tile_id);
      }
      tile_cache_[tile_id] = std::move(payload);

      while (cache_order_.size() > max_cached_tiles_) {
        const std::string oldest = cache_order_.front();
        cache_order_.pop_front();
        tile_cache_.erase(oldest);
      }
    }

    publisher_->publish(descriptor);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Published map tile %s size=%u hash=%s",
      tile_id.c_str(),
      descriptor.byte_size,
      descriptor.hash_sha256.c_str());
  }

  void publish_raster_tiles(
    const std::map<TileGridKey, TileRaster> & rasters,
    const char * source_label)
  {
    if (rasters.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No raster tiles generated from %s input", source_label);
      return;
    }

    const rclcpp::Time publish_time = this->now();

    size_t rendered_tiles = 0;
    for (const auto & [_, tile] : rasters) {
      if (!tile.touched) {
        continue;
      }

      const auto bytes = encode_png_rgba(tile.rgba, tile_size_px_, tile_size_px_);
      if (bytes.empty()) {
        continue;
      }

      publish_tile(tile.x, tile.y, bytes, publish_time);
      ++rendered_tiles;
    }

    if (rendered_tiles == 0) {
      RCLCPP_DEBUG(this->get_logger(), "All %s-derived tiles were empty after encoding", source_label);
    }
  }

  void handle_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    try {
      const auto rasters = rasterize_occupancy(*msg);
      publish_raster_tiles(rasters, "occupancy");
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to process occupancy grid into tiles: %s", ex.what());
    }
  }

  void handle_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      const auto rasters = rasterize_point_cloud(*msg);
      publish_raster_tiles(rasters, "point_cloud");
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to process point cloud into tiles: %s", ex.what());
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
  double republish_interval_sec_{5.0};
  size_t max_cached_tiles_{500};

  std::string tile_topic_;
  std::string occupancy_topic_;
  std::string point_cloud_topic_;
  std::string tile_service_name_;
  std::string mbtiles_path_;
  std::string map_source_;
  std::vector<std::string> layer_ids_;

  sqlite3 * db_{nullptr};

  rclcpp::Publisher<aeris_msgs::msg::MapTile>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
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
