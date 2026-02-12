/**
 * @file map_tile_publisher.cpp
 * @brief ROS 2 node for publishing map tiles from SLAM output
 *
 * The MapTilePublisher node converts RTAB-Map SLAM output (occupancy grids
 * and point clouds) into MBTiles format for streaming to ground station
 * operators. It implements the Aeris tile streaming protocol with:
 *
 * - Slippy Map tile coordinate generation
 * - PNG rasterization with configurable zoom levels
 * - SQLite-backed MBTiles storage
 * - Change-detection for bandwidth optimization
 * - ROS 2 topic and service interfaces
 *
 * @copyright Aeris Robotics 2024
 */

#include <algorithm>
#include <array>
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
#include "aeris_map/slam_backend.hpp"
#include "aeris_msgs/msg/map_tile.hpp"
#include "aeris_msgs/srv/get_map_tile_bytes.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

// ROS topic and service names
constexpr char kTileTopic[] = "/map/tiles";
constexpr char kOccupancyTopic[] = "/map";
constexpr char kPointCloudTopic[] = "/rtabmap/cloud_map";
constexpr char kGetTileService[] = "/map/get_tile_bytes";

// Map source mode constants
constexpr char kMapSourceOccupancy[] = "occupancy";
constexpr char kDefaultSlamMode[] = "vio";

// Default tile generation parameters
constexpr int kDefaultTileSizePx = 256;
constexpr int kDefaultZoom = 18;

// Geographic constants for Web Mercator projection
constexpr double kPi = 3.14159265358979323846;
constexpr double kMetersPerDegreeLat = 111320.0;
constexpr double kMaxWebMercatorLatDeg = 85.05112878;

/** @brief Grid key for tile raster lookup (x, y tile coordinates) */
using TileGridKey = std::pair<int, int>;

/** @brief Cached tile payload with metadata for change detection */
struct TilePayload
{
  std::vector<uint8_t> data;              ///< Raw PNG bytes
  std::string hash_sha256;                ///< Content hash for deduplication
  uint32_t byte_size{0};                  ///< Payload size
  builtin_interfaces::msg::Time published_at;  ///< Original publication time
};

/** @brief In-memory tile raster buffer for PNG generation */
struct TileRaster
{
  int x{0};                               ///< Tile X coordinate
  int y{0};                               ///< Tile Y coordinate
  std::vector<uint8_t> rgba;              ///< RGBA pixel data (tile_size^2 * 4)
  bool touched{false};                    ///< True if any pixel was written
};

/** @brief Slippy map pixel coordinate within a tile */
struct SlippyPixel
{
  int xtile{0};                           ///< Tile X index
  int ytile{0};                           ///< Tile Y index
  int px{0};                              ///< Pixel X within tile (0-tile_size)
  int py{0};                              ///< Pixel Y within tile (0-tile_size)
};

/** @brief PNG write callback context for memory output */
struct MemoryWriter
{
  std::vector<uint8_t> * out;             ///< Output buffer
};

/**
 * @brief Returns the default MBTiles database path.
 *
 * Uses XDG_CACHE_HOME if set, otherwise falls back to ~/.cache or /tmp.
 */
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

/** @brief PNG write callback: appends data to memory buffer */
void png_write_callback(png_structp png_ptr, png_bytep data, png_size_t length)
{
  auto * writer = static_cast<MemoryWriter *>(png_get_io_ptr(png_ptr));
  writer->out->insert(writer->out->end(), data, data + length);
}

/** @brief PNG flush callback: no-op for memory output */
void png_flush_callback(png_structp)
{
}

/**
 * @brief Converts Slippy Map Y coordinate to TMS row convention.
 *
 * MBTiles 1.3 specification uses TMS row indexing (origin at bottom-left)
 * while Slippy Map uses origin at top-left.
 */
int tms_row_from_slippy(int z, int y)
{
  return ((1 << z) - 1) - y;
}

/**
 * @brief Parses a tile id string in `z/x/y` form.
 *
 * @return Parsed z/x/y tuple when valid, std::nullopt otherwise.
 */
std::optional<std::array<int, 3>> parse_tile_id(const std::string & tile_id)
{
  int z = 0;
  int x = 0;
  int y = 0;
  char trailing = '\0';
  if (std::sscanf(tile_id.c_str(), "%d/%d/%d%c", &z, &x, &y, &trailing) != 3) {
    return std::nullopt;
  }
  return std::array<int, 3>{z, x, y};
}

/**
 * @brief Converts local Cartesian coordinates to WGS84 lat/lon.
 *
 * Uses equirectangular approximation suitable for small operational areas
 * (within ~10km of origin). Accuracy degrades at high latitudes.
 */
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

/**
 * @brief Converts WGS84 coordinates to Slippy Map tile and pixel coordinates.
 *
 * Implements the standard Web Mercator projection used by OpenStreetMap
 * and most web mapping services.
 *
 * @return SlippyPixel if coordinates are valid, std::nullopt otherwise
 */
std::optional<SlippyPixel> slippy_pixel_from_lat_lon(
  double lat_deg,
  double lon_deg,
  int zoom,
  int tile_size_px)
{
  if (zoom < 0 || zoom > 22 || tile_size_px <= 0) {
    return std::nullopt;
  }

  // Clamp to Web Mercator bounds to avoid projection singularities
  const double clamped_lat = std::clamp(lat_deg, -kMaxWebMercatorLatDeg, kMaxWebMercatorLatDeg);
  const double clamped_lon = std::clamp(lon_deg, -180.0, 180.0);
  const double lat_rad = clamped_lat * kPi / 180.0;
  const double n = std::pow(2.0, zoom);

  // Web Mercator projection formulas
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

/**
 * @brief Converts occupancy grid value to RGBA color.
 *
 * - Unknown (-1): Semi-transparent gray
 * - Occupied (100): Black
 * - Free (0): White
 * - Intermediate values: Grayscale gradient
 */
std::array<uint8_t, 4> occupancy_to_rgba(int8_t value)
{
  if (value < 0) {
    return {127, 127, 127, 220};  // Unknown: gray with transparency
  }
  const uint8_t clamped = static_cast<uint8_t>(std::clamp<int>(value, 0, 100));
  const uint8_t intensity = static_cast<uint8_t>(255 - (clamped * 2));
  return {intensity, intensity, intensity, 255};
}

/**
 * @brief Converts point elevation (Z coordinate) to RGBA color.
 *
 * Maps elevation range -10m to +40m to a color gradient:
 * - Low elevation: Blue/cyan
 * - Mid elevation: Green/yellow
 * - High elevation: Red
 */
std::array<uint8_t, 4> point_to_rgba(float z_m)
{
  const double clamped_z = std::clamp(static_cast<double>(z_m), -10.0, 40.0);
  const double normalized = (clamped_z + 10.0) / 50.0;
  const uint8_t red = static_cast<uint8_t>(60.0 + normalized * 195.0);
  const uint8_t green = static_cast<uint8_t>(255.0 - normalized * 120.0);
  const uint8_t blue = static_cast<uint8_t>(180.0 - normalized * 120.0);
  return {red, green, blue, 255};
}

/**
 * @brief Encodes RGBA pixel data to PNG format.
 *
 * Uses libpng for compression. Returns empty vector on failure.
 *
 * @throws std::runtime_error on PNG encoding failure
 */
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

}  // namespace

/**
 * @brief ROS 2 node for publishing map tiles from SLAM output.
 *
 * Subscribes to occupancy grids and/or point clouds from RTAB-Map,
 * converts them to MBTiles format, and publishes tile descriptors
 * for ground station consumption.
 */
namespace
{
class MapTilePublisher : public rclcpp::Node
{
public:
  explicit MapTilePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("aeris_map_tile_publisher", options)
  {
    // Load parameters with defaults
    queue_depth_ = static_cast<int>(this->declare_parameter<int64_t>("queue_depth", 50));
    tile_topic_ = this->declare_parameter<std::string>("topic", kTileTopic);
    occupancy_topic_ = this->declare_parameter<std::string>("occupancy_topic", kOccupancyTopic);
    point_cloud_topic_ =
      this->declare_parameter<std::string>("point_cloud_topic", kPointCloudTopic);
    tile_service_name_ = this->declare_parameter<std::string>("tile_service_name", kGetTileService);
    slam_mode_ = this->declare_parameter<std::string>("slam_mode", kDefaultSlamMode);

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

    const std::string requested_map_source =
      this->declare_parameter<std::string>("map_source", kMapSourceOccupancy);
    const auto backend_start_result = aeris_map::slam_backend::start_backend(
      slam_mode_,
      requested_map_source,
      occupancy_topic_,
      point_cloud_topic_);
    if (!backend_start_result.status.ready) {
      std::ostringstream error;
      error << "SLAM backend startup failed. mode="
            << aeris_map::slam_backend::normalize_backend_mode(slam_mode_)
            << " code="
            << aeris_map::slam_backend::startup_error_code_name(backend_start_result.status.code)
            << " message=" << backend_start_result.status.message;
      throw std::runtime_error(error.str());
    }
    slam_backend_ = backend_start_result.activation;
    map_source_ = slam_backend_.map_source;

    // Configure layer IDs based on map source
    const std::vector<std::string> default_layer_ids =
      default_layer_ids_for_activation(slam_backend_);

    layer_ids_ = this->declare_parameter<std::vector<std::string>>(
      "layer_ids",
      default_layer_ids);

    initialize_mbtiles();

    // Configure publisher with reliable QoS for tile delivery
    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_depth_)))
                 .reliable()
                 .durability_volatile();
    publisher_ = this->create_publisher<aeris_msgs::msg::MapTile>(tile_topic_, qos);

    // Subscribe to map sources based on backend capability contract.
    configure_backend_subscriptions();

    // Service for on-demand tile byte retrieval
    tile_service_ = this->create_service<aeris_msgs::srv::GetMapTileBytes>(
      tile_service_name_,
      std::bind(
        &MapTilePublisher::handle_get_tile_bytes,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MapTilePublisher::handle_parameter_updates, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Map tile stream initialized. slam_mode=%s source=%s occupancy_topic=%s point_cloud_topic=%s publish_topic=%s service=%s zoom=%d mbtiles=%s",
      slam_backend_.contract.mode.c_str(),
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
    RCLCPP_INFO(
      this->get_logger(),
      "SLAM backend contract: frame_chain=%s outputs=[%s,%s,%s] capabilities=%zu",
      slam_backend_.contract.frame_chain.c_str(),
      slam_backend_.contract.produced_outputs.size() > 0 ? slam_backend_.contract.produced_outputs[0].c_str() : "",
      slam_backend_.contract.produced_outputs.size() > 1 ? slam_backend_.contract.produced_outputs[1].c_str() : "",
      slam_backend_.contract.produced_outputs.size() > 2 ? slam_backend_.contract.produced_outputs[2].c_str() : "",
      slam_backend_.contract.capabilities.size());
  }

  ~MapTilePublisher() override
  {
    if (db_ != nullptr) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
  }

private:
  /**
   * @brief Initializes the MBTiles SQLite database.
   *
   * Creates the database file if it doesn't exist and sets up the
   * required schema (metadata and tiles tables per MBTiles 1.3 spec).
   */
  std::vector<std::string> default_layer_ids_for_activation(
    const aeris_map::slam_backend::BackendActivation & activation) const
  {
    if (activation.use_point_cloud && !activation.use_occupancy) {
      return {"point_cloud", "fetch-service:/map/get_tile_bytes", "mime:image/png"};
    }
    return {"occupancy", "fetch-service:/map/get_tile_bytes", "mime:image/png"};
  }

  void configure_backend_subscriptions()
  {
    if (slam_backend_.use_occupancy) {
      occupancy_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        occupancy_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&MapTilePublisher::handle_occupancy_grid, this, std::placeholders::_1));
    } else {
      occupancy_subscription_.reset();
    }

    if (slam_backend_.use_point_cloud) {
      point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&MapTilePublisher::handle_point_cloud, this, std::placeholders::_1));
    } else {
      point_cloud_subscription_.reset();
    }
  }

  rcl_interfaces::msg::SetParametersResult handle_parameter_updates(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "accepted";

    std::string requested_slam_mode = slam_mode_;
    std::string requested_map_source = map_source_;
    bool backend_config_changed = false;

    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "slam_mode") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "slam_mode must be a string";
          return result;
        }
        requested_slam_mode = parameter.as_string();
        backend_config_changed = true;
      } else if (parameter.get_name() == "map_source") {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          result.successful = false;
          result.reason = "map_source must be a string";
          return result;
        }
        requested_map_source = parameter.as_string();
        backend_config_changed = true;
      }
    }

    if (!backend_config_changed) {
      return result;
    }

    const auto backend_start_result = aeris_map::slam_backend::start_backend(
      requested_slam_mode,
      requested_map_source,
      occupancy_topic_,
      point_cloud_topic_);
    if (!backend_start_result.status.ready) {
      result.successful = false;
      result.reason =
        "SLAM backend update failed: code=" +
        aeris_map::slam_backend::startup_error_code_name(backend_start_result.status.code) +
        " message=" + backend_start_result.status.message;
      return result;
    }

    const auto previous_default_layer_ids = default_layer_ids_for_activation(slam_backend_);
    const bool using_default_layer_ids = layer_ids_ == previous_default_layer_ids;

    slam_mode_ = aeris_map::slam_backend::normalize_backend_mode(requested_slam_mode);
    slam_backend_ = backend_start_result.activation;
    map_source_ = slam_backend_.map_source;
    if (using_default_layer_ids) {
      layer_ids_ = default_layer_ids_for_activation(slam_backend_);
    }
    configure_backend_subscriptions();

    RCLCPP_INFO(
      this->get_logger(),
      "Updated SLAM backend at runtime. slam_mode=%s source=%s occupancy=%s point_cloud=%s",
      slam_backend_.contract.mode.c_str(),
      map_source_.c_str(),
      slam_backend_.use_occupancy ? "enabled" : "disabled",
      slam_backend_.use_point_cloud ? "enabled" : "disabled");

    return result;
  }
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

  /** @brief Executes a SQL statement on the MBTiles database. */
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

  /** @brief Inserts or updates a metadata key-value pair. */
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

  /** @brief Inserts or updates a tile in the MBTiles database. */
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

  /**
   * @brief Retrieves or creates a tile raster buffer.
   *
   * Ensures a TileRaster exists for the given coordinates,
   * initializing with zeroed RGBA data if new.
   */
  std::optional<std::vector<uint8_t>> lookup_tile_blob_from_mbtiles(
    const std::string & tile_id)
  {
    const auto parsed = parse_tile_id(tile_id);
    if (!parsed.has_value()) {
      return std::nullopt;
    }

    const int z = (*parsed)[0];
    const int x = (*parsed)[1];
    const int y = (*parsed)[2];

    sqlite3_stmt * stmt = nullptr;
    constexpr const char * kSql =
      "SELECT tile_data FROM tiles "
      "WHERE zoom_level=? AND tile_column=? AND tile_row=?;";
    if (sqlite3_prepare_v2(db_, kSql, -1, &stmt, nullptr) != SQLITE_OK) {
      throw std::runtime_error("Failed to prepare tile lookup statement");
    }

    sqlite3_bind_int(stmt, 1, z);
    sqlite3_bind_int(stmt, 2, x);
    sqlite3_bind_int(stmt, 3, tms_row_from_slippy(z, y));

    const int rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
      const auto * blob = static_cast<const uint8_t *>(sqlite3_column_blob(stmt, 0));
      const int size = sqlite3_column_bytes(stmt, 0);
      std::vector<uint8_t> bytes;
      if (blob != nullptr && size > 0) {
        bytes.assign(blob, blob + size);
      }
      sqlite3_finalize(stmt);
      return bytes;
    }

    if (rc != SQLITE_DONE) {
      const std::string err = sqlite3_errmsg(db_);
      sqlite3_finalize(stmt);
      throw std::runtime_error("Failed to execute tile lookup statement: " + err);
    }

    sqlite3_finalize(stmt);
    return std::nullopt;
  }

  void upsert_tile_cache_locked(const std::string & tile_id, TilePayload payload)
  {
    const auto prev = tile_cache_.find(tile_id);
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

  /** @brief Writes an RGBA pixel to a tile raster. */
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

  /**
   * @brief Rasterizes an occupancy grid into tile buffers.
   *
   * Iterates through all grid cells, projects each to geographic
   * coordinates, and maps to the appropriate tile and pixel.
   */
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

        // Convert grid cell to local metric coordinates
        const double local_x_m =
          grid.info.origin.position.x + ((static_cast<double>(gx) + 0.5) * resolution);
        const double local_y_m =
          grid.info.origin.position.y + ((static_cast<double>(gy) + 0.5) * resolution);

        // Project to geographic and then to tile coordinates
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

  /**
   * @brief Rasterizes a point cloud into tile buffers.
   *
   * Iterates through all points, projects each to geographic
   * coordinates, and colors by elevation (Z coordinate).
   */
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

  /**
   * @brief Publishes a single tile with change detection and caching.
   *
   * Computes content hash, checks against cache for duplicates,
   * persists to MBTiles, and publishes descriptor if changed.
   */
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
          return;  // Skip unchanged tiles within republish interval
        }
      }

      upsert_tile_blob(zoom_, xtile, ytile, bytes);

      upsert_tile_cache_locked(tile_id, std::move(payload));
    }

    publisher_->publish(descriptor);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Published map tile %s size=%u hash=%s",
      tile_id.c_str(),
      descriptor.byte_size,
      descriptor.hash_sha256.c_str());
  }

  /**
   * @brief Publishes all tiles from a rasterization result.
   *
   * Encodes each touched tile to PNG and publishes with
   * consistent timestamp for batch correlation.
   */
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

  /** @brief ROS callback for occupancy grid messages. */
  void handle_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    try {
      const auto rasters = rasterize_occupancy(*msg);
      publish_raster_tiles(rasters, "occupancy");
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to process occupancy grid into tiles: %s", ex.what());
    }
  }

  /** @brief ROS callback for point cloud messages. */
  void handle_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      const auto rasters = rasterize_point_cloud(*msg);
      publish_raster_tiles(rasters, "point_cloud");
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to process point cloud into tiles: %s", ex.what());
    }
  }

  /** @brief ROS service handler for tile byte retrieval. */
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
    if (it != tile_cache_.end()) {
      response->found = true;
      response->content_type = "image/png";
      response->hash_sha256 = it->second.hash_sha256;
      response->byte_size = it->second.byte_size;
      response->published_at = it->second.published_at;
      response->data = it->second.data;
      return;
    }

    std::optional<std::vector<uint8_t>> db_tile_bytes;
    try {
      db_tile_bytes = lookup_tile_blob_from_mbtiles(request->tile_id);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Tile lookup failed for %s: %s",
        request->tile_id.c_str(),
        ex.what());
      response->found = false;
      response->content_type = "image/png";
      response->hash_sha256.clear();
      response->byte_size = 0;
      response->data.clear();
      return;
    }
    if (!db_tile_bytes.has_value() || db_tile_bytes->empty()) {
      response->found = false;
      response->content_type = "image/png";
      response->hash_sha256.clear();
      response->byte_size = 0;
      response->data.clear();
      return;
    }

    TilePayload payload;
    payload.data = *db_tile_bytes;
    payload.hash_sha256 = aeris_map::tile_contract::sha256_hex(payload.data);
    payload.byte_size = static_cast<uint32_t>(payload.data.size());
    payload.published_at = this->now();
    upsert_tile_cache_locked(request->tile_id, payload);

    response->found = true;
    response->content_type = "image/png";
    response->hash_sha256 = payload.hash_sha256;
    response->byte_size = payload.byte_size;
    response->published_at = payload.published_at;
    response->data = payload.data;
  }

  // Configuration parameters
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
  std::string slam_mode_;
  std::string mbtiles_path_;
  std::string map_source_;
  std::vector<std::string> layer_ids_;
  aeris_map::slam_backend::BackendActivation slam_backend_;

  sqlite3 * db_{nullptr};

  rclcpp::Publisher<aeris_msgs::msg::MapTile>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
  rclcpp::Service<aeris_msgs::srv::GetMapTileBytes>::SharedPtr tile_service_;

  std::mutex cache_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  std::unordered_map<std::string, TilePayload> tile_cache_;
  std::deque<std::string> cache_order_;
};

}  // namespace

#ifndef AERIS_MAP_TILE_PUBLISHER_NO_MAIN
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
#endif
