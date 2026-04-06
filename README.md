# Dual UR5 Gazebo Simulation (Dockerized)

This repo now contains a ROS Noetic + Gazebo setup for:
- Two UR5 robot arms
- A table, bowl, cup, and microwave
- One YAML file to customize all measured translations/rotations
- Docker + browser GUI (noVNC), so it runs on macOS without native Linux setup

## Quick Start

```bash
docker compose up --build
```

Then open:

- [http://localhost:6080](http://localhost:6080)

Gazebo starts inside the container desktop session.

## Customize Scene Measurements

Edit:

- `ros_ws/src/dual_ur5_gazebo/config/scene.yaml`

You can set:

- `units`: `m`, `cm`, or `mm`
- Two UR5 base poses (`x`, `y`, `z`, `roll`, `pitch`, `yaw` or `*_deg`)
- Object poses for table/bowl/cup/microwave
- Optional `ur5_xacro` absolute path if your distro uses a non-standard UR5 file location

Example pose block:

```yaml
pose:
  x: 650
  y: 350
  z: 0
  yaw_deg: -90
```

If `units: mm`, this is interpreted as 650 mm, 350 mm, etc.

If robot spawn fails due xacro path discovery, add:

```yaml
ur5_xacro: /opt/ros/noetic/share/ur_description/urdf/ur.urdf.xacro
```

## File Layout

- `docker/`: container runtime scripts + Dockerfile
- `docker-compose.yml`: one-command launch
- `ros_ws/src/dual_ur5_gazebo/`: ROS package (launch/config/models/scripts)

## Notes

- This is built for ROS Noetic (Ubuntu/Linux container) and intended for Docker Desktop on macOS.
- Gazebo model placement uses the world frame.
