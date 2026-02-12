#pragma once

#include <memory>
#include <string>
#include <vector>

namespace aeris_map::slam_backend
{

enum class StartupErrorCode
{
  kNone,
  kInvalidConfig,
  kUnavailableTopics,
  kStartupFailure,
  kNotImplemented,
};

struct StartupStatus
{
  bool ready{false};
  StartupErrorCode code{StartupErrorCode::kStartupFailure};
  std::string message;
};

struct BackendContract
{
  std::string mode;
  std::vector<std::string> expected_input_topics;
  std::vector<std::string> produced_outputs;
  std::string frame_chain;
  std::vector<std::string> capabilities;
};

struct BackendActivation
{
  BackendContract contract;
  std::string map_source;
  std::string occupancy_topic;
  std::string point_cloud_topic;
  bool use_occupancy{false};
  bool use_point_cloud{false};
};

class Backend
{
public:
  virtual ~Backend() = default;

  virtual BackendContract contract() const = 0;

  virtual StartupStatus validate(
    const std::string & map_source,
    const std::string & occupancy_topic,
    const std::string & point_cloud_topic) const = 0;

  virtual BackendActivation activate(
    const std::string & map_source,
    const std::string & occupancy_topic,
    const std::string & point_cloud_topic) const = 0;
};

struct BackendStartResult
{
  StartupStatus status;
  BackendActivation activation;
};

std::string normalize_backend_mode(std::string mode);
std::string normalize_map_source(std::string map_source);
std::string startup_error_code_name(StartupErrorCode code);

std::unique_ptr<Backend> make_backend(const std::string & mode);

BackendStartResult start_backend(
  const std::string & mode,
  const std::string & map_source,
  const std::string & occupancy_topic,
  const std::string & point_cloud_topic);

}  // namespace aeris_map::slam_backend
