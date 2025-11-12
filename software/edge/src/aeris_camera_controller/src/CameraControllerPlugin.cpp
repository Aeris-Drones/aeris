#include "aeris/camera/CameraControllerPlugin.hpp"

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

namespace
{
constexpr double kDefaultDuration = 3.0;
constexpr double kEpsilon = 1e-6;

double ToSeconds(const std::chrono::steady_clock::duration &time)
{
  return std::chrono::duration_cast<std::chrono::duration<double>>(time).count();
}

ignition::math::Quaterniond LookAt(const ignition::math::Vector3d &from,
                                   const ignition::math::Vector3d &to)
{
  ignition::math::Vector3d forward = (to - from);
  if (forward.Length() < kEpsilon)
  {
    return ignition::math::Quaterniond::Identity;
  }
  forward.Normalize();
  ignition::math::Vector3d world_up{0, 0, 1};
  ignition::math::Vector3d right = forward.Cross(world_up);
  if (right.Length() < kEpsilon)
  {
    world_up = ignition::math::Vector3d{0, 1, 0};
    right = forward.Cross(world_up);
  }
  right.Normalize();
  ignition::math::Vector3d up = right.Cross(forward);
  ignition::math::Matrix3d rot;
  rot.SetColumn(0, right);
  rot.SetColumn(1, up);
  rot.SetColumn(2, -forward);
  return ignition::math::Quaterniond(rot);
}
}  // namespace

CameraControllerPlugin::CameraControllerPlugin() = default;

CameraControllerPlugin::~CameraControllerPlugin()
{
  ros_running_.store(false);
  if (ros_thread_.joinable())
  {
    ros_thread_.join();
  }
  if (node_)
  {
    node_->get_logger().info("CameraControllerPlugin shutting down ROS interface");
    node_.reset();
  }
}

void CameraControllerPlugin::Configure(const ignition::gazebo::Entity &entity,
                                       const std::shared_ptr<const sdf::Element> &sdf,
                                       ignition::gazebo::EntityComponentManager &ecm,
                                       ignition::gazebo::EventManager &)
{
  model_entity_ = entity;
  if (sdf && sdf->HasElement("camera_link"))
  {
    camera_link_name_ = sdf->Get<std::string>("camera_link");
  }
  InitializePresets();
  ResolveCameraEntities(ecm);

  if (!rclcpp::ok())
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  node_ = std::make_shared<rclcpp::Node>("aeris_cinematic_camera");
  service_ = node_->create_service<aeris_msgs::srv::SetCameraView>(
    "/simulation/set_camera_view",
    std::bind(&CameraControllerPlugin::HandleServiceRequest, this, std::placeholders::_1,
              std::placeholders::_2));

  ros_running_.store(true);
  ros_thread_ = std::thread(&CameraControllerPlugin::SpinRos, this);

  RCLCPP_INFO(node_->get_logger(), "CameraControllerPlugin ready with link '%s'",
              camera_link_name_.c_str());
}

void CameraControllerPlugin::InitializePresets()
{
  presets_.clear();

  PresetDefinition wide;
  wide.mode = PresetMode::kStatic;
  wide.world_position = ignition::math::Vector3d(0, 0, 100);
  presets_["wide"] = wide;

  PresetDefinition tracking;
  tracking.mode = PresetMode::kFollow;
  tracking.follow_offset = ignition::math::Vector3d(-10, 0, 6);
  presets_["tracking"] = tracking;

  PresetDefinition pov;
  pov.mode = PresetMode::kPov;
  pov.pov_distance = 1.5;
  presets_["pov"] = pov;

  PresetDefinition orbit;
  orbit.mode = PresetMode::kOrbit;
  orbit.orbit_radius = 40.0;
  orbit.orbit_height = 30.0;
  presets_["orbit"] = orbit;
}

bool CameraControllerPlugin::ResolveCameraEntities(ignition::gazebo::EntityComponentManager &ecm)
{
  if (camera_link_ != ignition::gazebo::kNullEntity)
  {
    return true;
  }

  auto links = ecm.EntitiesByComponents(
    ignition::gazebo::components::Name(camera_link_name_),
    ignition::gazebo::components::ParentEntity(model_entity_));
  if (links.empty())
  {
    return false;
  }

  camera_link_ = links.front();
  if (!ecm.Component<ignition::gazebo::components::PoseCmd>(camera_link_))
  {
    ecm.CreateComponent(camera_link_, ignition::gazebo::components::PoseCmd());
  }
  return true;
}

void CameraControllerPlugin::HandleServiceRequest(
  const std::shared_ptr<aeris_msgs::srv::SetCameraView::Request> request,
  std::shared_ptr<aeris_msgs::srv::SetCameraView::Response> response)
{
  if (!request)
  {
    response->accepted = false;
    response->message = "Empty request";
    return;
  }

  ActiveCommand command;
  command.duration = request->transition_duration > 0.1 ? request->transition_duration : kDefaultDuration;
  command.target_vehicle = request->target_vehicle;
  command.tracking = request->tracking_mode;

  if (request->command_type == "preset")
  {
    auto preset = presets_.find(request->preset_name);
    if (preset == presets_.end())
    {
      response->accepted = false;
      response->message = "Unknown preset: " + request->preset_name;
      return;
    }
    command.type = CommandType::kPreset;
    command.preset_name = request->preset_name;
  }
  else if (request->command_type == "path")
  {
    if (request->path.empty())
    {
      response->accepted = false;
      response->message = "Path request missing waypoints";
      return;
    }
    command.type = CommandType::kPath;
    command.path.reserve(request->path.size());
    for (const auto &wp_msg : request->path)
    {
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
    std::sort(command.path.begin(), command.path.end(),
              [](const Waypoint &a, const Waypoint &b) {
                return a.time_from_start < b.time_from_start;
              });
    command.duration = command.path.back().time_from_start;
    if (command.duration < kEpsilon)
    {
      command.duration = kDefaultDuration;
    }
  }
  else
  {
    response->accepted = false;
    response->message = "Unsupported command_type: " + request->command_type;
    return;
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    active_command_ = command;
  }

  response->accepted = true;
  response->message = "Camera command accepted";
}

ignition::math::Pose3d CameraControllerPlugin::ComputePresetTarget(
  const PresetDefinition &preset,
  ignition::gazebo::EntityComponentManager &ecm,
  const std::string &target_vehicle) const
{
  ignition::math::Vector3d position = preset.world_position;
  ignition::math::Quaterniond orientation = ignition::math::Quaterniond::Identity;

  if (preset.mode == PresetMode::kStatic)
  {
    orientation = ignition::math::Quaterniond(0, -0.6, 0);
  }
  else if (preset.mode == PresetMode::kFollow)
  {
    auto target_pose = QueryEntityPose(target_vehicle, ecm);
    position = target_pose.Pos() + preset.follow_offset;
    orientation = LookAt(position, target_pose.Pos());
  }
  else if (preset.mode == PresetMode::kPov)
  {
    auto target_pose = QueryEntityPose(target_vehicle, ecm);
    ignition::math::Vector3d forward = target_pose.Rot().RotateVector(ignition::math::Vector3d(1, 0, 0));
    position = target_pose.Pos() - forward * preset.pov_distance;
    orientation = target_pose.Rot();
  }
  else if (preset.mode == PresetMode::kOrbit)
  {
    auto target_pose = QueryEntityPose(target_vehicle, ecm);
    double now = ToSeconds(std::chrono::steady_clock::now().time_since_epoch());
    double angle = fmod(now, 2 * M_PI);
    position = target_pose.Pos() + ignition::math::Vector3d(
      preset.orbit_radius * cos(angle), preset.orbit_radius * sin(angle), preset.orbit_height);
    orientation = LookAt(position, target_pose.Pos());
  }

  return ignition::math::Pose3d(position, orientation);
}

ignition::math::Pose3d CameraControllerPlugin::QueryEntityPose(
  const std::string &name,
  ignition::gazebo::EntityComponentManager &ecm) const
{
  auto entity = ignition::gazebo::EntityByName(ecm, name);
  if (entity == ignition::gazebo::kNullEntity)
  {
    return ignition::math::Pose3d::Zero;
  }
  if (auto pose = ecm.Component<ignition::gazebo::components::WorldPose>(entity))
  {
    return pose->Data();
  }
  if (auto rel_pose = ecm.Component<ignition::gazebo::components::Pose>(entity))
  {
    return rel_pose->Data();
  }
  return ignition::math::Pose3d::Zero;
}

ignition::math::Pose3d CameraControllerPlugin::InterpolatePose(
  const ignition::math::Pose3d &start,
  const ignition::math::Pose3d &end,
  double t) const
{
  t = std::clamp(t, 0.0, 1.0);
  const auto p0 = start.Pos();
  const auto p3 = end.Pos();
  const auto p1 = p0 + (p3 - p0) * 0.25;
  const auto p2 = p3 - (p3 - p0) * 0.25;
  const double one_minus_t = 1.0 - t;
  ignition::math::Vector3d pos =
    p0 * std::pow(one_minus_t, 3) +
    3 * p1 * std::pow(one_minus_t, 2) * t +
    3 * p2 * one_minus_t * std::pow(t, 2) +
    p3 * std::pow(t, 3);
  ignition::math::Quaterniond rot = start.Rot().Slerp(t, end.Rot());
  return ignition::math::Pose3d(pos, rot);
}

ignition::math::Pose3d CameraControllerPlugin::SamplePath(const ActiveCommand &command,
                                                          double t) const
{
  if (command.path.empty())
  {
    return ignition::math::Pose3d::Zero;
  }
  double target_time = t * command.duration;
  const Waypoint *previous = &command.path.front();
  const Waypoint *next = previous;
  for (const auto &wp : command.path)
  {
    if (wp.time_from_start <= target_time)
    {
      previous = &wp;
    }
    if (wp.time_from_start >= target_time)
    {
      next = &wp;
      break;
    }
  }
  double span = std::max(kEpsilon, next->time_from_start - previous->time_from_start);
  double local_t = std::clamp((target_time - previous->time_from_start) / span, 0.0, 1.0);
  return InterpolatePose(previous->pose, next->pose, local_t);
}

void CameraControllerPlugin::UpdateCameraPose(const ignition::math::Pose3d &pose,
                                              ignition::gazebo::EntityComponentManager &ecm)
{
  if (!ResolveCameraEntities(ecm))
  {
    return;
  }
  auto pose_cmd = ecm.Component<ignition::gazebo::components::PoseCmd>(camera_link_);
  if (!pose_cmd)
  {
    pose_cmd = ecm.CreateComponent(camera_link_, ignition::gazebo::components::PoseCmd());
  }
  pose_cmd->Data() = pose;
}

void CameraControllerPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &info,
                                       ignition::gazebo::EntityComponentManager &ecm)
{
  if (info.paused)
  {
    return;
  }

  ActiveCommand command;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!active_command_)
    {
      return;
    }
    command = *active_command_;
  }

  double sim_time = ToSeconds(info.simTime);
  if (command.start_time <= 0.0)
  {
    ignition::math::Pose3d current = ignition::math::Pose3d::Zero;
    if (auto pose = ecm.Component<ignition::gazebo::components::WorldPose>(camera_link_))
    {
      current = pose->Data();
    }
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      active_command_->start_time = sim_time;
      active_command_->origin = current;
      if (command.type == CommandType::kPreset)
      {
        auto preset = presets_.find(command.preset_name);
        if (preset != presets_.end())
        {
          active_command_->target = ComputePresetTarget(preset->second, ecm, command.target_vehicle);
        }
      }
      command = *active_command_;
    }
  }

  double elapsed = sim_time - command.start_time;
  double normalized = command.duration > 0.0 ? std::clamp(elapsed / command.duration, 0.0, 1.0) : 1.0;

  ignition::math::Pose3d desired = command.origin;
  ignition::math::Pose3d target_pose = command.target;
  bool keep_active = false;

  if (command.type == CommandType::kPreset)
  {
    auto preset = presets_.find(command.preset_name);
    if (preset != presets_.end())
    {
      target_pose = ComputePresetTarget(preset->second, ecm, command.target_vehicle);
      desired = InterpolatePose(command.origin, target_pose, normalized);
      keep_active = command.tracking || preset->second.mode != PresetMode::kStatic;
    }
  }
  else if (command.type == CommandType::kPath)
  {
    desired = SamplePath(command, normalized);
  }

  UpdateCameraPose(desired, ecm);

  if (normalized >= 0.999)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (keep_active && active_command_)
    {
      active_command_->origin = desired;
      active_command_->start_time = sim_time;
      active_command_->target = target_pose;
    }
    else
    {
      active_command_.reset();
    }
  }
}

void CameraControllerPlugin::SpinRos()
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  while (ros_running_.load() && rclcpp::ok())
  {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  exec.remove_node(node_);
}

IGNITION_ADD_PLUGIN(
  CameraControllerPlugin,
  ignition::gazebo::System,
  CameraControllerPlugin::ISystemConfigure,
  CameraControllerPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(CameraControllerPlugin, "aeris::camera::CameraControllerPlugin")
