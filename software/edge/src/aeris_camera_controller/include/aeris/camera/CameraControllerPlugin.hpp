#pragma once

// Gazebo plugin providing cinematic camera control via ROS 2 services.
// Supports preset views (wide, tracking, POV, orbit) and custom animation paths
// for automated mission recording and operator presentation modes.

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/UpdateInfo.hh>
#include <ignition/math/Pose3.hh>

#include "aeris_msgs/srv/set_camera_view.hpp"

namespace aeris
{
namespace camera
{

class CameraControllerPlugin : public ignition::gazebo::System,
                               public ignition::gazebo::ISystemConfigure,
                               public ignition::gazebo::ISystemPreUpdate
{
public:
  CameraControllerPlugin();
  ~CameraControllerPlugin() override;

  void Configure(const ignition::gazebo::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 ignition::gazebo::EntityComponentManager &ecm,
                 ignition::gazebo::EventManager &eventMgr) override;

  void PreUpdate(const ignition::gazebo::UpdateInfo &info,
                 ignition::gazebo::EntityComponentManager &ecm) override;

private:
  enum class CommandType
  {
    kNone,
    kPreset,
    kPath
  };

  struct Waypoint
  {
    double time_from_start{0.0};
    ignition::math::Pose3d pose;
    double fov_deg{45.0};
  };

  struct ActiveCommand
  {
    CommandType type{CommandType::kNone};
    std::string preset_name;
    std::string target_vehicle;
    double start_time{0.0};
    double duration{3.0};
    bool tracking{false};
    std::vector<Waypoint> path;
    ignition::math::Pose3d origin;
    ignition::math::Pose3d target;
  };

  enum class PresetMode
  {
    kStatic,
    kFollow,
    kPov,
    kOrbit
  };

  struct PresetDefinition
  {
    PresetMode mode{PresetMode::kStatic};
    ignition::math::Vector3d world_position{0, 0, 60};
    ignition::math::Vector3d follow_offset{0, -15, 8};
    double orbit_radius{40.0};
    double orbit_height{30.0};
    double pov_distance{2.0};
  };

  void InitializePresets();
  bool ResolveCameraEntities(ignition::gazebo::EntityComponentManager &ecm);

  void HandleServiceRequest(
    const std::shared_ptr<aeris_msgs::srv::SetCameraView::Request> request,
    std::shared_ptr<aeris_msgs::srv::SetCameraView::Response> response);

  ignition::math::Pose3d ComputePresetTarget(
    const PresetDefinition &preset,
    ignition::gazebo::EntityComponentManager &ecm,
    const std::string &target_vehicle) const;

  ignition::math::Pose3d QueryEntityPose(const std::string &name,
                     ignition::gazebo::EntityComponentManager &ecm) const;

  ignition::math::Pose3d InterpolatePose(
    const ignition::math::Pose3d &start,
    const ignition::math::Pose3d &end,
    double t) const;

  ignition::math::Pose3d SamplePath(const ActiveCommand &command,
                                    double t) const;

  void UpdateCameraPose(const ignition::math::Pose3d &pose,
                        ignition::gazebo::EntityComponentManager &ecm);

  void SpinRos();

  std::optional<ActiveCommand> active_command_;
  std::map<std::string, PresetDefinition> presets_;
  ignition::gazebo::Entity model_entity_{ignition::gazebo::kNullEntity};
  ignition::gazebo::Entity camera_link_{ignition::gazebo::kNullEntity};
  std::string camera_link_name_ = "camera_link";

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<aeris_msgs::srv::SetCameraView>::SharedPtr service_;
  std::thread ros_thread_;
  std::atomic_bool ros_running_{false};
  mutable std::mutex command_mutex_;
};

}  // namespace camera
}  // namespace aeris

#endif  // AERIS_CAMERA_CONTROLLER_PLUGIN_HPP
