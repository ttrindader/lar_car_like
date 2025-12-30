# LaR Car-Like Robot (ROS + Gazebo)

A lightweight **car-like (Ackermann) robot** for simulation with **ROS** and **Gazebo**, including:

- URDF/Xacro model with tuned wheel contact (ODE) for stable driving and steering
- `ros_control` transmissions and controller configuration (steer position + rear wheel velocity)
- Optional **(v, steering_angle)** command interface (Ackermann-style) that maps to the controllers

> This repository is intended for research, teaching, and rapid prototyping in simulation.

---

## Repository layout

Typical files you will find here:

```
car_like_description/
  urdf/
    car_like.urdf.xacro
  meshes/
    *.dae
  config/
    controllers.yaml
  launch/
    car_like_gazebo.launch
  scripts/
    car_like_v_angle_controller.py   # optional: (v, δ) → controllers
  blender/  
```

---

## Requirements

- ROS (ROS 1)
- Gazebo (Gazebo Classic) + `gazebo_ros`
- `ros_control` + `gazebo_ros_control`
- `xacro`
- *(Optional)* `ackermann_msgs` (if you use the Ackermann command interface) 
```bash
sudo apt install ros-noetic-ackermann-msgs
```

---

## Installation

Clone into your catkin workspace and build:

```bash
cd ~/catkin_ws/src
git clone https://github.com/ttrindader/lar_car_like
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## Quick start (Gazebo)

Launch the robot in an empty Gazebo world:

```bash
roslaunch car_like_description car_like_gazebo.launch
```

This should:
1) Load the URDF/Xacro into `/robot_description`  
2) Spawn the model in Gazebo  
3) Load and start the controllers under the robot namespace  

---

## Controllers

This project uses:

- **Steering** (front joints): `effort_controllers/JointPositionController`
- **Rear wheel drive**: `effort_controllers/JointVelocityController`
- `joint_state_controller/JointStateController`

Controller parameters live in:

- `config/controllers.yaml`

---

## Commanding the robot

### Option A — Command controllers directly (simple)

Steering command (radians):

```bash
rostopic pub -1 /car_like/front_left_steer_controller/command  std_msgs/Float64 "data: 0.25"
rostopic pub -1 /car_like/front_right_steer_controller/command std_msgs/Float64 "data: 0.25"
```

Rear wheel angular speed command (rad/s):

```bash
rostopic pub -1 /car_like/rear_left_wheel_controller/command  std_msgs/Float64 "data: 15.0"
rostopic pub -1 /car_like/rear_right_wheel_controller/command std_msgs/Float64 "data: 15.0"
```

---

### Option B — (v, steering_angle) interface (recommended)

If you use the helper node, you can command the robot using:

- linear speed `v` (m/s)
- steering angle `δ` (rad)

**Run the node** (example):

```bash
rosrun car_like_description car_like_v_angle_controller.py _ns:=car_like
```

**Publish commands** (Ackermann message):

```bash
rostopic pub -r 20 /car_like/ackermann_cmd ackermann_msgs/AckermannDriveStamped \
"{drive: {speed: 0.6, steering_angle: 0.25}}"
```

> Tip: adding a **watchdog** (timeout → command=0) and **rate limiting** (dv/dt, dδ/dt) makes driving much smoother.

---

## Useful topics

- Command topics:
  - `/car_like/front_left_steer_controller/command` *(std_msgs/Float64)*
  - `/car_like/front_right_steer_controller/command` *(std_msgs/Float64)*
  - `/car_like/rear_left_wheel_controller/command` *(std_msgs/Float64)*
  - `/car_like/rear_right_wheel_controller/command` *(std_msgs/Float64)*
  - `/car_like/ackermann_cmd` *(ackermann_msgs/AckermannDriveStamped, optional)*
  
  
**Odometry (raw + filtered)**:
  - `/car_like/odom` *(nav_msgs/Odometry — raw wheel/steering-based odometry published by `car_like_odometry_node.py`)*
  - `/car_like/odom_filtered` *(nav_msgs/Odometry — EKF output from `robot_localization`)*

- **TF frames (when `ekf.yaml` has `publish_tf: true`)**:
  - `odom -> base_link` *(published by the EKF; set `publish_tf: false` in the raw odom node to avoid TF duplication)*


- Controller state (debug):
  - `/car_like/<controller_name>/state`

- TF / robot state:
  - `/tf`, `/tf_static`
  - `/joint_states`

---

## Tuning notes (common issues)

### “Robot creeps / moves by itself”
- Make sure wheel joints have **damping/friction**, and wheel contact isn’t overly rigid.
- Avoid large integral gain (`i`) in velocity control unless you really need it.
- Ensure there is no stale publisher holding a non-zero command:
  ```bash
  rostopic info /car_like/rear_left_wheel_controller/command
  ```

### “Jerks / stutters when increasing PID or effort”
- Align physics timestep with control rate (`controlPeriod`).
- Reduce saturation (lower `effort`, lower `p`, add a small `d`).
- Apply command ramps (limit dv/dt and dδ/dt).

### “Slides instead of turning”
- Reduce lateral slip (`slip2`) and use anisotropic friction direction (`fdir1`) for car-like behavior.
- Make sure both front steering joints are commanded and have enough effort.

---

## Credits

Developed at **LaR (Robotics Lab, UFBA)**.

---

## License

Choose a license for your repository (MIT / BSD-3-Clause / Apache-2.0) and add it as `LICENSE`.
