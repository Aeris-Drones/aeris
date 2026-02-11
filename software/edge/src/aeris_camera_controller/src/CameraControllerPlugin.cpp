#include "aeris/camera/CameraControllerPlugin.hpp"

/**
 * @file CameraControllerPlugin.cpp
 * @brief Implementation of the cinematic camera controller plugin.
 *
 * This file implements the CameraControllerPlugin class, integrating with
 * Ignition Gazebo's simulation loop and ROS 2 for external camera control.
 * The plugin supports smooth camera transitions, preset viewing modes, and
 * custom animation paths using cubic Bezier interpolation.
 *
 * @copyright Copyright (c) 2024
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/WorldPose.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>

using aeris::camera::CameraControllerPlugin;

namespace {

/** @brief Default transition duration in seconds. */
constexpr double kDefaultDuration = 3.0;

/** @brief Epsilon value for floating-point comparisons. */
constexpr double kEpsilon = 1e-6;

/**
 * @brief Converts a chrono duration to seconds.
 *
 * @param time The duration to convert.
 * @return Duration in seconds as a double.
 */
double ToSeconds(const std::chrono::steady_clock::duration& time) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(time).count();
}

/**
 * @brief Computes a quaternion that orients from one point to another.
 *
 * Creates a rotation quaternion that aligns the negative Z-axis with the
 * direction from the 'from' point to the 'to' point. Handles degenerate
 * cases where the points are coincident or the direction is vertical.
 *
 * @param from Source position vector.
 * @param to Target position vector.
 * @return Quaternion representing the look-at rotation.
 */
ignition::math::Quaterniond LookAt(const ignition::math::Vector3d& from,
                                   const ignition::math::Vector3d& to) {
  ignition::math::Vector3d forward = (to - from);
  if (forward.Length() < kEpsilon) {
    // Degenerate case: points are coincident, return identity.
    return ignition::math::Quaterniond::Identity;
  }
  forward.Normalize();

  // Construct orthonormal basis with forward as the view direction.
  ignition::math::Vector3d world_up{0, 0, 1};
  ignition::math::Vector3d right = forward.Cross(world_up);
  if (right.Length() < kEpsilon) {
    // Forward is parallel to world up, use alternative up vector.
    world_up = ignition::math::Vector3d{0, 1, 0};
    right = forward.Cross(world_up);
  }
  right.Normalize();

  ignition::math::Vector3d up = right.Cross(forward);

  // Build rotation matrix from orthonormal basis vectors.
  ignition::math::Matrix3d rot;
  rot.SetColumn(0, right);
  rot.SetColumn(1, up);
  rot.SetColumn(2, -forward);

  return ignition::math::Quaterniond(rot);
}

}  // namespace

CameraControllerPlugin::CameraControllerPlugin() = default;

CameraControllerPlugin::~CameraControllerPlugin() {
  // Signal the background thread to stop and wait for it to complete.
  ros_running_.store(false);
  if (ros_thread_.joinable()) {
    ros_thread_.join();
  }
  if (node_) {
    node_->get_logger().info("CameraControllerPlugin shutting down ROS interface");
    node_.reset();
  }
}

void CameraControllerPlugin::Configure(const ignition::gazebo::Entity& entity,
                                       const std::shared_ptr<const sdf::Element>& sdf,
                                       ignition::gazebo::EntityComponentManager& ecm,
                                       ignition::gazebo::EventManager&) {
  model_entity_ = entity;

  // Parse optional camera link name from SDF configuration.
  if (sdf && sdf->HasElement("camera_link")) {
    camera_link_name_ = sdf->Get<std::string>("camera_link");
  }

  InitializePresets();
  ResolveCameraEntities(ecm);

  // Initialize ROS 2 if not already running.
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  // Create ROS 2 node and camera control service.
  node_ = std::make_shared<rclcpp::Node>("aeris_cinematic_camera");
  service_ = node_->create_service<aeris_msgs::srv::SetCameraView>(
      "/simulation/set_camera_view",
      std::bind(&CameraControllerPlugin::HandleServiceRequest, this, std::placeholders::_1,
                std::placeholders::_2));

  // Start background thread for ROS 2 event processing.
  ros_running_.store(true);
  ros_thread_ = std::thread(&CameraControllerPlugin::SpinRos, this);

  RCLCPP_INFO(node_->get_logger(), "CameraControllerPlugin ready with link '%s'",
              camera_link_name_.c_str());
}

void CameraControllerPlugin::InitializePresets() {
  presets_.clear();

  // Wide shot: Static overhead view of the entire scene.
  PresetDefinition wide;
  wide.mode = PresetMode::kStatic;
  wide.world_position = ignition::math::Vector3d(0, 0, 100);
  presets_["wide"] = wide;

  // Tracking: Follows target with a trailing offset.
  PresetDefinition tracking;
  tracking.mode = PresetMode::kFollow;
  tracking.follow_offset = ignition::math::Vector3d(-10, 0, 6);
  presets_["tracking"] = tracking;

  // POV: Positioned behind target for first-person perspective.
  PresetDefinition pov;
  pov.mode = PresetMode::kPov;
  pov.pov_distance = 1.5;
  presets_["pov"] = pov;

  // Orbit: Continuously circles around the target.
  PresetDefinition orbit;
  orbit.mode = PresetMode::kOrbit;
  orbit.orbit_radius = 40.0;
  orbit.orbit_height = 30.0;
  presets_["orbit"] = orbit;
}

bool CameraControllerPlugin::ResolveCameraEntities(ignition::gazebo::EntityComponentManager& ecm) {
  // Return cached entity if already resolved.
  if (camera_link_ != ignition::gazebo::kNullEntity) {
    return true;
  }

  // Search for camera link by name within this model.
  auto links = ecm.EntitiesByComponents(
      ignition::gazebo::components::Name(camera_link_name_),
      ignition::gazebo::components::ParentEntity(model_entity_));
  if (links.empty()) {
    return false;
  }

  camera_link_ = links.front();

  // Ensure the camera link has a PoseCmd component for pose updates.
  if (!ecm.Component<ignition::gazebo::components::PoseCmd>(camera_link_)) {
    ecm.CreateComponent(camera_link_, ignition::gazebo::components::PoseCmd());
  }
  return true;
}

void CameraControllerPlugin::HandleServiceRequest(
    const std::shared_ptr<aeris_msgs::srv::SetCameraView::Request> request,
    std::shared_ptr<aeris_msgs::srv::SetCameraView::Response> response) {
  if (!request) {
    response->accepted = false;
    response->message = "Empty request";
    return;
  }

  // Initialize command from request parameters.
  ActiveCommand command;
  command.duration = request->transition_duration > 0.1 ? request->transition_duration : kDefaultDuration;
  command.target_vehicle = request->target_vehicle;
  command.tracking = request->tracking_mode;

  if (request->command_type == "preset") {
    // Validate preset name exists.
    auto preset = presets_.find(request->preset_name);
    if (preset == presets_.end()) {
      response->accepted = false;
      response->message = "Unknown preset: " + request->preset_name;
      return;
    }
    command.type = CommandType::kPreset;
    command.preset_name = request->preset_name;
  } else if (request->command_type == "path") {
    // Validate path has waypoints.
    if (request->path.empty()) {
      response->accepted = false;
      response->message = "Path request missing waypoints";
      return;
    }

    // Convert ROS waypoints to internal format.
    command.type = CommandType::kPath;
    command.path.reserve(request->path.size());
    for (const auto& wp_msg : request->path) {
      Waypoint wp;
      wp.time_from_start = wp_msg.time_from_start.sec + wp_msg.time_from_start.nanosec / 1e9;
      wp.pose = ignition::math::Pose3d(
          ignition::math::Vector3d(wp_msg.pose.position.x, wp_msg.pose.position.y,
                                   wp_msg.pose.position.z),
          ignition::math::Quaterniond(wp_msg.pose.orientation.w, wp_msg.pose.orientation.x,
                                      wp_msg.pose.orientation.y, wp_msg.pose.orientation.z));
      wp.fov_deg = wp_msg.field_of_view_deg;
      command.path.push_back(wp);
    }

    // Sort waypoints by time and set duration from final waypoint.
    std::sort(command.path.begin(), command.path.end(),
              [](const Waypoint& a, const Waypoint& b) {
                return a.time_from_start < b.time_from_start;
              });
    command.duration = command.path.back().time_from_start;
    if (command.duration < kEpsilon) {
      command.duration = kDefaultDuration;
    }
  } else {
    response->accepted = false;
    response->message = "Unsupported command_type: " + request->command_type;
    return;
  }

  // Atomically replace any existing command with the new one.
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    active_command_ = command;
  }

  response->accepted = true;
  response->message = "Camera command accepted";
}

ignition::math::Pose3d CameraControllerPlugin::ComputePresetTarget(
    const PresetDefinition& preset,
    ignition::gazebo::EntityComponentManager& ecm,
    const std::string& target_vehicle) const {
  ignition::math::Vector3d position = preset.world_position;
  ignition::math::Quaterniond orientation = ignition::math::Quaterniond::Identity;

  switch (preset.mode) {
    case PresetMode::kStatic:
      // Static overhead view with fixed pitch angle.
      orientation = ignition::math::Quaterniond(0, -0.6, 0);
      break;

    case PresetMode::kFollow: {
      // Position relative to target, looking at target.
      auto target_pose = QueryEntityPose(target_vehicle, ecm);
      position = target_pose.Pos() + preset.follow_offset;
      orientation = LookAt(position, target_pose.Pos());
      break;
    }

    case PresetMode::kPov: {
      // Position behind target, matching target orientation.
      auto target_pose = QueryEntityPose(target_vehicle, ecm);
      ignition::math::Vector3d forward = target_pose.Rot().RotateVector(ignition::math::Vector3d(1, 0, 0));
      position = target_pose.Pos() - forward * preset.pov_distance;
      orientation = target_pose.Rot();
      break;
    }

    case PresetMode::kOrbit: {
      // Circular orbit around target based on current time.
      auto target_pose = QueryEntityPose(target_vehicle, ecm);
      double now = ToSeconds(std::chrono::steady_clock::now().time_since_epoch());
      double angle = fmod(now, 2 * M_PI);
      position = target_pose.Pos() + ignition::math::Vector3d(
                                         preset.orbit_radius * cos(angle),
                                         preset.orbit_radius * sin(angle),
                                         preset.orbit_height);
      orientation = LookAt(position, target_pose.Pos());
      break;
    }
  }

  return ignition::math::Pose3d(position, orientation);
}

ignition::math::Pose3d CameraControllerPlugin::QueryEntityPose(
    const std::string& name,
    ignition::gazebo::EntityComponentManager& ecm) const {
  auto entity = ignition::gazebo::EntityByName(ecm, name);
  if (entity == ignition::gazebo::kNullEntity) {
    return ignition::math::Pose3d::Zero;
  }

  // Prefer world pose if available, otherwise fall back to relative pose.
  if (auto pose = ecm.Component<ignition::gazebo::components::WorldPose>(entity)) {
    return pose->Data();
  }
  if (auto rel_pose = ecm.Component<ignition::gazebo::components::Pose>(entity)) {
    return rel_pose->Data();
  }
  return ignition::math::Pose3d::Zero;
}

ignition::math::Pose3d CameraControllerPlugin::InterpolatePose(
    const ignition::math::Pose3d& start,
    const ignition::math::Pose3d& end,
    double t) const {
  // Clamp interpolation factor to valid range.
  t = std::clamp(t, 0.0, 1.0);

  // Compute cubic Bezier control points for smooth acceleration/deceleration.
  const auto p0 = start.Pos();
  const auto p3 = end.Pos();
  const auto p1 = p0 + (p3 - p0) * 0.25;  // 25% from start
  const auto p2 = p3 - (p3 - p0) * 0.25;  // 25% from end

  // Evaluate cubic Bezier curve for position.
  const double one_minus_t = 1.0 - t;
  ignition::math::Vector3d pos =
      p0 * std::pow(one_minus_t, 3) +
      3 * p1 * std::pow(one_minus_t, 2) * t +
      3 * p2 * one_minus_t * std::pow(t, 2) +
      p3 * std::pow(t, 3);

  // Spherical linear interpolation for rotation.
  ignition::math::Quaterniond rot = start.Rot().Slerp(t, end.Rot());

  return ignition::math::Pose3d(pos, rot);
}

ignition::math::Pose3d CameraControllerPlugin::SamplePath(const ActiveCommand& command,
                                                          double t) const {
  if (command.path.empty()) {
    return ignition::math::Pose3d::Zero;
  }

  // Convert normalized time to absolute time within path duration.
  double target_time = t * command.duration;

  // Find surrounding waypoints for interpolation.
  const Waypoint* previous = &command.path.front();
  const Waypoint* next = previous;
  for (const auto& wp : command.path) {
    if (wp.time_from_start <= target_time) {
      previous = &wp;
    }
    if (wp.time_from_start >= target_time) {
      next = &wp;
      break;
    }
  }

  // Compute local interpolation factor between waypoints.
  double span = std::max(kEpsilon, next->time_from_start - previous->time_from_start);
  double local_t = std::clamp((target_time - previous->time_from_start) / span, 0.0, 1.0);

  return InterpolatePose(previous->pose, next->pose, local_t);
}

void CameraControllerPlugin::UpdateCameraPose(const ignition::math::Pose3d& pose,
                                              ignition::gazebo::EntityComponentManager& ecm) {
  if (!ResolveCameraEntities(ecm)) {
    return;
  }

  // Update or create PoseCmd component with desired camera pose.
  auto pose_cmd = ecm.Component<ignition::gazebo::components::PoseCmd>(camera_link_);
  if (!pose_cmd) {
    pose_cmd = ecm.CreateComponent(camera_link_, ignition::gazebo::components::PoseCmd());
  }
  pose_cmd->Data() = pose;
}

void CameraControllerPlugin::PreUpdate(const ignition::gazebo::UpdateInfo& info,
                                       ignition::gazebo::EntityComponentManager& ecm) {
  // Skip processing when simulation is paused.
  if (info.paused) {
    return;
  }

  // Copy active command under lock for thread-safe access.
  ActiveCommand command;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!active_command_) {
      return;
    }
    command = *active_command_;
  }

  double sim_time = ToSeconds(info.simTime);

  // Initialize command timing and origin on first execution.
  if (command.start_time <= 0.0) {
    // Capture current camera pose as interpolation starting point.
    ignition::math::Pose3d current = ignition::math::Pose3d::Zero;
    if (auto pose = ecm.Component<ignition::gazebo::components::WorldPose>(camera_link_)) {
      current = pose->Data();
    }

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      active_command_->start_time = sim_time;
      active_command_->origin = current;

      // Pre-compute target for preset modes.
      if (command.type == CommandType::kPreset) {
        auto preset = presets_.find(command.preset_name);
        if (preset != presets_.end()) {
          active_command_->target = ComputePresetTarget(preset->second, ecm, command.target_vehicle);
        }
      }
      command = *active_command_;
    }
  }

  // Compute normalized progress through transition.
  double elapsed = sim_time - command.start_time;
  double normalized = command.duration > 0.0 ? std::clamp(elapsed / command.duration, 0.0, 1.0) : 1.0;

  // Compute desired camera pose based on command type.
  ignition::math::Pose3d desired = command.origin;
  ignition::math::Pose3d target_pose = command.target;
  bool keep_active = false;

  if (command.type == CommandType::kPreset) {
    auto preset = presets_.find(command.preset_name);
    if (preset != presets_.end()) {
      // Recompute target for dynamic modes (follow, orbit) each frame.
      target_pose = ComputePresetTarget(preset->second, ecm, command.target_vehicle);
      desired = InterpolatePose(command.origin, target_pose, normalized);

      // Continue updating if tracking is enabled or mode is non-static.
      keep_active = command.tracking || preset->second.mode != PresetMode::kStatic;
    }
  } else if (command.type == CommandType::kPath) {
    desired = SamplePath(command, normalized);
  }

  UpdateCameraPose(desired, ecm);

  // Handle transition completion.
  if (normalized >= 0.999) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (keep_active && active_command_) {
      // Reset for continuous tracking: new origin at current position,
      // recompute target on next frame.
      active_command_->origin = desired;
      active_command_->start_time = sim_time;
      active_command_->target = target_pose;
    } else {
      // Transition complete, clear active command.
      active_command_.reset();
    }
  }
}

void CameraControllerPlugin::SpinRos() {
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  while (ros_running_.load() && rclcpp::ok()) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  exec.remove_node(node_);
}

// Register the plugin with Ignition Gazebo.
IGNITION_ADD_PLUGIN(
    CameraControllerPlugin,
    ignition::gazebo::System,
    CameraControllerPlugin::ISystemConfigure,
    CameraControllerPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(CameraControllerPlugin, "aeris::camera::CameraControllerPlugin")
