# Disaster Scene World Specification

This document captures layout, lighting, and asset details for the Aeris demo disaster block created in Story 1.2.

## Goals
- Present a collapsed urban block with visual cues (damaged buildings, debris piles, rescue staging area).
- Provide cinematic "golden hour" lighting (warm key + cool fill) to match PRD expectations.
- Keep assets simple (SDF primitives) but structured so model replacements can drop in later.

## Scene Layout Overview

```
| West Alley | Building A (collapsed) | Central Plaza (debris) | Building B (leaning) | Rescue Zone |
```

- **Building A**: Two stacked box volumes, top rotated 20° to simulate collapse, exposed rebar beams.
- **Central Plaza**: Ground plane with rubble cubes, barriers, and a signal flare stand.
- **Building B**: Tall structure leaning inward 10°, partial upper section missing.
- **Rescue Zone**: Tents plus staging lights and generator crates.

## Lighting
- Key light: warm (1.0, 0.85, 0.6) directional at 35° elevation, azimuth 50°.
- Fill light: cool (0.5, 0.6, 0.9), opposite azimuth, lower intensity.
- Ambient: low to keep contrast.

## Usage
```bash
WORLD_PATH=software/sim/worlds/disaster_scene.sdf ./software/sim/tools/run_basic_sim.sh
```

Spawn helper output shows bridge status. Use `gazebo` GUI to move camera for cinematic views.

## Future Enhancements
- Replace primitive debris with GLTF kit-bashed assets.
- Add particle emitters (smoke) and animated emergency lights.
- Populate ROS 2 markers for mission overlays.