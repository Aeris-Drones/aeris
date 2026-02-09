# Cinematic Camera Controls

The cinematic camera model (`software/sim/models/cinematic_camera/model.sdf`) exposes the camera control service contract through:

- `aeris_msgs/msg/CameraWaypoint.msg`
- `aeris_msgs/srv/SetCameraView.srv`

Use these helpers:

```bash
python3 software/sim/tools/camera_path_builder.py --output software/sim/config/camera_path_example.json
python3 software/sim/tools/send_camera_view.py --preset tracking --target scout1
```
