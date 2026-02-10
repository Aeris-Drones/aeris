# Multi-Drone SITL

## Launcher

- Launch entry: `software/sim/launch/multi_drone_sim.launch.py`
- Vehicle config: `software/sim/config/multi_drone.yaml`

## Behavior

- Starts Gazebo world with ROS bridge
- Starts PX4 SITL processes for configured vehicles
- Uses `SCOUT_MODEL_NAME` to map Scout stereo/IMU topics to ROS names expected by VIO + SLAM

## Validation

```bash
python3 software/sim/tools/run_multi_drone_sitl.py --config software/sim/config/multi_drone.yaml --execute
```
