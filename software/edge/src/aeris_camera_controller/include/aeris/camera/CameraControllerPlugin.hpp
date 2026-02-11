#pragma once

/**
 * @file CameraControllerPlugin.hpp
 * @brief Gazebo plugin providing cinematic camera control via ROS 2 services.
 *
 * This plugin integrates with Ignition Gazebo's simulation loop to provide
 * automated camera control for mission recording and operator presentation modes.
 * It supports preset camera views (wide, tracking, POV, orbit) and custom
 * animation paths defined by waypoint sequences.
 *
 * @copyright Copyright (c) 2024
 */

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
#include <ignition/math/Pose3d.hh>

#include "aeris_msgs/srv/set_camera_view.hpp"

namespace aeris {
namespace camera {

/**
 * @brief Gazebo system plugin for cinematic camera control.
 *
 * CameraControllerPlugin provides automated camera positioning and animation
 * within Ignition Gazebo simulations. It exposes ROS 2 services for external
 * control and supports multiple camera modes including preset views and
 * custom animation paths.
 *
 * The plugin implements the ISystemConfigure and ISystemPreUpdate interfaces
 * to integrate with Gazebo's simulation lifecycle.
 */
class CameraControllerPlugin : public ignition::gazebo::System,
                               public ignition::gazebo::ISystemConfigure,
                               public ignition::gazebo::ISystemPreUpdate {
 public:
  /**
   * @brief Constructs a new CameraControllerPlugin instance.
   */
  CameraControllerPlugin();

  /**
   * @brief Destroys the CameraControllerPlugin instance.
   *
   * Shuts down the ROS 2 interface and joins the background spinner thread.
   */
  ~CameraControllerPlugin() override;

  /**
   * @brief Configures the plugin during simulation startup.
   *
   * Called once when the plugin is loaded. Initializes the ROS 2 node,
   * creates the camera control service, and resolves camera entity references.
   *
   * @param entity The model entity this plugin is attached to.
   * @param sdf Pointer to the SDF element containing plugin configuration.
   * @param ecm Reference to the EntityComponentManager.
   * @param eventMgr Reference to the EventManager.
   * @throws None Configuration errors are logged but do not throw.
   */
  void Configure(const ignition::gazebo::Entity& entity,
                 const std::shared_ptr<const sdf::Element>& sdf,
                 ignition::gazebo::EntityComponentManager& ecm,
                 ignition::gazebo::EventManager& eventMgr) override;

  /**
   * @brief Updates camera state before each simulation step.
   *
   * Called every simulation iteration to process active camera commands,
   * compute interpolated poses, and update the camera link position.
   *
   * @param info Update timing information including simulation time.
   * @param ecm Reference to the EntityComponentManager.
   */
  void PreUpdate(const ignition::gazebo::UpdateInfo& info,
                 ignition::gazebo::EntityComponentManager& ecm) override;

 private:
  /**
   * @brief Enumeration of supported camera command types.
   */
  enum class CommandType {
    kNone,    ///< No active command.
    kPreset,  ///< Preset camera mode command.
    kPath     ///< Custom animation path command.
  };

  /**
   * @brief Represents a waypoint in an animation path.
   *
   * Waypoints define camera position, orientation, and field of view
   * at specific times along an animation path.
   */
  struct Waypoint {
    /** @brief Time from path start in seconds. */
    double time_from_start{0.0};
    /** @brief Camera pose at this waypoint. */
    ignition::math::Pose3d pose;
    /** @brief Field of view in degrees. */
    double fov_deg{45.0};
  };

  /**
   * @brief Represents an active camera command being executed.
   */
  struct ActiveCommand {
    /** @brief Type of camera command. */
    CommandType type{CommandType::kNone};
    /** @brief Name of the preset mode (for preset commands). */
    std::string preset_name;
    /** @brief Name of the target vehicle to track. */
    std::string target_vehicle;
    /** @brief Simulation time when command started (seconds). */
    double start_time{0.0};
    /** @brief Total duration of the transition (seconds). */
    double duration{3.0};
    /** @brief Whether to continue tracking after transition completes. */
    bool tracking{false};
    /** @brief Sequence of waypoints (for path commands). */
    std::vector<Waypoint> path;
    /** @brief Starting camera pose for interpolation. */
    ignition::math::Pose3d origin;
    /** @brief Target camera pose for interpolation. */
    ignition::math::Pose3d target;
  };

  /**
   * @brief Enumeration of preset camera behavior modes.
   */
  enum class PresetMode {
    kStatic,  ///< Fixed position camera.
    kFollow,  ///< Camera follows target with offset.
    kPov,     ///< Point-of-view camera positioned behind target.
    kOrbit    ///< Orbiting camera around target.
  };

  /**
   * @brief Configuration parameters for a preset camera mode.
   *
   * Defines the behavior and positioning parameters for preset camera views.
   * All position and offset values are in meters.
   */
  struct PresetDefinition {
    /** @brief Behavior mode for this preset. */
    PresetMode mode{PresetMode::kStatic};
    /** @brief World-space position for static mode (meters). */
    ignition::math::Vector3d world_position{0, 0, 60};
    /** @brief Relative offset for follow mode (meters). */
    ignition::math::Vector3d follow_offset{0, -15, 8};
    /** @brief Orbit radius for orbit mode (meters). */
    double orbit_radius{40.0};
    /** @brief Orbit height for orbit mode (meters). */
    double orbit_height{30.0};
    /** @brief Distance behind target for POV mode (meters). */
    double pov_distance{2.0};
  };

  /**
   * @brief Initializes built-in camera preset definitions.
   *
   * Configures the "wide", "tracking", "pov", and "orbit" preset modes
   * with default parameters.
   */
  void InitializePresets();

  /**
   * @brief Resolves camera link entity references in the simulation.
   *
   * Searches the EntityComponentManager for the camera link by name
   * and caches the entity reference for pose updates.
   *
   * @param ecm Reference to the EntityComponentManager.
   * @return true if the camera link entity was found, false otherwise.
   */
  bool ResolveCameraEntities(ignition::gazebo::EntityComponentManager& ecm);

  /**
   * @brief Handles incoming ROS 2 service requests for camera control.
   *
   * Processes SetCameraView service requests to activate preset modes
   * or custom animation paths.
   *
   * @param request Pointer to the service request message.
   * @param response Pointer to the service response message.
   */
  void HandleServiceRequest(
      const std::shared_ptr<aeris_msgs::srv::SetCameraView::Request> request,
      std::shared_ptr<aeris_msgs::srv::SetCameraView::Response> response);

  /**
   * @brief Computes the target camera pose for a preset mode.
   *
   * Calculates the desired camera position and orientation based on
   * the preset configuration and current target vehicle pose.
   *
   * @param preset The preset definition to apply.
   * @param ecm Reference to the EntityComponentManager.
   * @param target_vehicle Name of the target vehicle entity.
   * @return The computed target camera pose.
   */
  ignition::math::Pose3d ComputePresetTarget(
      const PresetDefinition& preset,
      ignition::gazebo::EntityComponentManager& ecm,
      const std::string& target_vehicle) const;

  /**
   * @brief Queries the current world pose of a named entity.
   *
   * Searches for an entity by name and retrieves its world-space
   * transformation. Returns zero pose if entity not found.
   *
   * @param name Name of the entity to query.
   * @param ecm Reference to the EntityComponentManager.
   * @return The entity's world pose, or zero pose if not found.
   */
  ignition::math::Pose3d QueryEntityPose(
      const std::string& name,
      ignition::gazebo::EntityComponentManager& ecm) const;

  /**
   * @brief Interpolates between two poses using cubic Bezier curves.
   *
   * Performs smooth interpolation between start and end poses using
   * cubic Bezier interpolation for position and spherical linear
   * interpolation (slerp) for rotation.
   *
   * @param start Starting pose.
   * @param end Ending pose.
   * @param t Interpolation factor in range [0.0, 1.0].
   * @return The interpolated pose.
   */
  ignition::math::Pose3d InterpolatePose(
      const ignition::math::Pose3d& start,
      const ignition::math::Pose3d& end,
      double t) const;

  /**
   * @brief Samples a point along an animation path.
   *
   * Evaluates the animation path at the specified normalized time,
   * interpolating between surrounding waypoints.
   *
   * @param command The active path command containing waypoints.
   * @param t Normalized time in range [0.0, 1.0].
   * @return The computed camera pose at time t.
   */
  ignition::math::Pose3d SamplePath(const ActiveCommand& command,
                                    double t) const;

  /**
   * @brief Updates the camera link pose in the simulation.
   *
   * Applies the computed pose to the camera link entity via the
   * EntityComponentManager.
   *
   * @param pose The desired camera pose.
   * @param ecm Reference to the EntityComponentManager.
   */
  void UpdateCameraPose(const ignition::math::Pose3d& pose,
                        ignition::gazebo::EntityComponentManager& ecm);

  /**
   * @brief Background thread function for ROS 2 event processing.
   *
   * Spins the ROS 2 executor to process incoming service requests.
   * Runs until ros_running_ is set to false.
   */
  void SpinRos();

  /** @brief Currently active camera command, if any. */
  std::optional<ActiveCommand> active_command_;

  /** @brief Map of preset names to their definitions. */
  std::map<std::string, PresetDefinition> presets_;

  /** @brief Entity handle for the model this plugin is attached to. */
  ignition::gazebo::Entity model_entity_{ignition::gazebo::kNullEntity};

  /** @brief Entity handle for the camera link. */
  ignition::gazebo::Entity camera_link_{ignition::gazebo::kNullEntity};

  /** @brief Name of the camera link in SDF (default: "camera_link"). */
  std::string camera_link_name_ = "camera_link";

  /** @brief ROS 2 node handle. */
  rclcpp::Node::SharedPtr node_;

  /** @brief ROS 2 service for camera control requests. */
  rclcpp::Service<aeris_msgs::srv::SetCameraView>::SharedPtr service_;

  /** @brief Background thread for ROS 2 spinning. */
  std::thread ros_thread_;

  /** @brief Flag to control background thread execution. */
  std::atomic_bool ros_running_{false};

  /** @brief Mutex protecting access to active_command_. */
  mutable std::mutex command_mutex_;
};

}  // namespace camera
}  // namespace aeris

#endif  // AERIS_CAMERA_CONTROLLER_PLUGIN_HPP
