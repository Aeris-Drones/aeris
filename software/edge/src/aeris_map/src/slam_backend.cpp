#include "aeris_map/slam_backend.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <utility>

namespace
{
constexpr char kMapSourceOccupancy[] = "occupancy";
constexpr char kMapSourcePointCloud[] = "point_cloud";
constexpr char kMapSourceHybrid[] = "hybrid";

constexpr char kModeVio[] = "vio";
constexpr char kModeLiosam[] = "liosam";

constexpr char kOutputMap[] = "/map";
constexpr char kOutputCloudMap[] = "/rtabmap/cloud_map";
constexpr char kOutputTiles[] = "/map/tiles";

std::string to_lower(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

class VioBackend final : public aeris_map::slam_backend::Backend
{
public:
  aeris_map::slam_backend::BackendContract contract() const override
  {
    return {
      kModeVio,
      {
        "/<vehicle>/stereo/left/image_raw",
        "/<vehicle>/stereo/right/image_raw",
        "/<vehicle>/imu/data",
        "/<vehicle>/openvins/odom",
      },
      {kOutputMap, kOutputCloudMap, kOutputTiles},
      "map -> odom -> base_link",
      {
        "occupancy-grid-output",
        "point-cloud-output",
        "deterministic-startup-errors",
        "mode-selection-no-fallback",
      },
    };
  }

  aeris_map::slam_backend::StartupStatus validate(
    const std::string & map_source,
    const std::string & occupancy_topic,
    const std::string & point_cloud_topic) const override
  {
    const std::string normalized_source = aeris_map::slam_backend::normalize_map_source(map_source);
    const bool use_occupancy =
      (normalized_source == kMapSourceOccupancy) || (normalized_source == kMapSourceHybrid);
    const bool use_point_cloud =
      (normalized_source == kMapSourcePointCloud) || (normalized_source == kMapSourceHybrid);

    if (!use_occupancy && !use_point_cloud) {
      return {
        false,
        aeris_map::slam_backend::StartupErrorCode::kUnavailableTopics,
        "No output stream selected for VIO backend",
      };
    }

    if (use_occupancy && occupancy_topic.empty()) {
      return {
        false,
        aeris_map::slam_backend::StartupErrorCode::kInvalidConfig,
        "VIO backend requires a non-empty occupancy topic",
      };
    }

    if (use_point_cloud && point_cloud_topic.empty()) {
      return {
        false,
        aeris_map::slam_backend::StartupErrorCode::kInvalidConfig,
        "VIO backend requires a non-empty point_cloud topic",
      };
    }

    return {
      true,
      aeris_map::slam_backend::StartupErrorCode::kNone,
      "VIO backend ready",
    };
  }

  aeris_map::slam_backend::BackendActivation activate(
    const std::string & map_source,
    const std::string & occupancy_topic,
    const std::string & point_cloud_topic) const override
  {
    const std::string normalized_source = aeris_map::slam_backend::normalize_map_source(map_source);
    const auto status = validate(normalized_source, occupancy_topic, point_cloud_topic);
    if (!status.ready) {
      throw std::invalid_argument(status.message);
    }

    aeris_map::slam_backend::BackendActivation activation;
    activation.contract = contract();
    activation.map_source = normalized_source;
    activation.occupancy_topic = occupancy_topic;
    activation.point_cloud_topic = point_cloud_topic;
    activation.use_occupancy =
      (normalized_source == kMapSourceOccupancy) || (normalized_source == kMapSourceHybrid);
    activation.use_point_cloud =
      (normalized_source == kMapSourcePointCloud) || (normalized_source == kMapSourceHybrid);
    return activation;
  }
};

class LiosamBackend final : public aeris_map::slam_backend::Backend
{
public:
  aeris_map::slam_backend::BackendContract contract() const override
  {
    return {
      kModeLiosam,
      {
        "/<vehicle>/lidar/points",
        "/<vehicle>/imu/data",
      },
      {kOutputMap, kOutputCloudMap, kOutputTiles},
      "map -> odom -> base_link",
      {
        "planned-liosam-adapter",
        "explicit-not-implemented-guardrail",
        "no-silent-fallback",
      },
    };
  }

  aeris_map::slam_backend::StartupStatus validate(
    const std::string & map_source,
    const std::string & occupancy_topic,
    const std::string & point_cloud_topic) const override
  {
    (void)map_source;
    (void)occupancy_topic;
    (void)point_cloud_topic;
    return {
      false,
      aeris_map::slam_backend::StartupErrorCode::kNotImplemented,
      "SLAM mode 'liosam' is registered but not implemented yet; refusing to fall back",
    };
  }

  aeris_map::slam_backend::BackendActivation activate(
    const std::string & map_source,
    const std::string & occupancy_topic,
    const std::string & point_cloud_topic) const override
  {
    (void)map_source;
    (void)occupancy_topic;
    (void)point_cloud_topic;
    throw std::runtime_error(
            "SLAM mode 'liosam' is registered but not implemented yet; refusing to fall back");
  }
};

}  // namespace

namespace aeris_map::slam_backend
{

std::string normalize_backend_mode(std::string mode)
{
  mode = to_lower(std::move(mode));
  std::replace(mode.begin(), mode.end(), '-', '_');
  if (mode.empty()) {
    return kModeVio;
  }
  if (mode == "rtabmap_vio" || mode == "rtabmap") {
    return kModeVio;
  }
  if (mode == "lio_sam") {
    return kModeLiosam;
  }
  return mode;
}

std::string normalize_map_source(std::string map_source)
{
  map_source = to_lower(std::move(map_source));

  if (map_source == "occupancy" || map_source == "grid") {
    return kMapSourceOccupancy;
  }
  if (map_source == "pointcloud" || map_source == "point_cloud" || map_source == "cloud") {
    return kMapSourcePointCloud;
  }
  if (map_source == "hybrid" || map_source == "both") {
    return kMapSourceHybrid;
  }

  throw std::invalid_argument("map_source must be one of: occupancy, point_cloud, hybrid");
}

std::string startup_error_code_name(StartupErrorCode code)
{
  switch (code) {
    case StartupErrorCode::kNone:
      return "none";
    case StartupErrorCode::kInvalidConfig:
      return "invalid-config";
    case StartupErrorCode::kUnavailableTopics:
      return "unavailable-topics";
    case StartupErrorCode::kStartupFailure:
      return "startup-failure";
    case StartupErrorCode::kNotImplemented:
      return "not-implemented";
  }
  return "startup-failure";
}

std::unique_ptr<Backend> make_backend(const std::string & mode)
{
  const std::string normalized_mode = normalize_backend_mode(mode);
  if (normalized_mode == kModeVio) {
    return std::make_unique<VioBackend>();
  }
  if (normalized_mode == kModeLiosam) {
    return std::make_unique<LiosamBackend>();
  }

  throw std::invalid_argument(
          "Unsupported slam_mode '" + normalized_mode + "'. Supported modes: vio, liosam");
}

BackendStartResult start_backend(
  const std::string & mode,
  const std::string & map_source,
  const std::string & occupancy_topic,
  const std::string & point_cloud_topic)
{
  BackendStartResult result;

  try {
    auto backend = make_backend(mode);
    const auto status = backend->validate(map_source, occupancy_topic, point_cloud_topic);
    if (!status.ready) {
      result.status = status;
      return result;
    }

    result.activation = backend->activate(map_source, occupancy_topic, point_cloud_topic);
    result.status = {
      true,
      StartupErrorCode::kNone,
      "backend activation succeeded",
    };
    return result;
  } catch (const std::invalid_argument & ex) {
    result.status = {
      false,
      StartupErrorCode::kInvalidConfig,
      ex.what(),
    };
    return result;
  } catch (const std::exception & ex) {
    result.status = {
      false,
      StartupErrorCode::kStartupFailure,
      ex.what(),
    };
    return result;
  }
}

}  // namespace aeris_map::slam_backend
