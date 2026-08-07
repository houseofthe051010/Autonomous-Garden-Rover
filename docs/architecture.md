# Recommended Autonomous Rover Architecture

## Platform choice

Use ROS 2 Jazzy on 64-bit Ubuntu 24.04 for the Raspberry Pi 5. Jazzy is supported
through May 2029 and Ubuntu 24.04 arm64 is a Tier 1 target. Keep the STM32, GD32,
and ODESC responsible for deterministic PWM, STEP generation, and inner motor
control; Linux and ROS should send goals and consume timestamped feedback.

The combined Thonny client is a hardware commissioning tool. Reuse its tested
protocol classes in ROS nodes, but do not run autonomy by repeatedly calling
interactive shell functions.

## ROS graph

```text
Camera -> vision node ----------------------------+
                                                   v
Encoders -> wheel odometry -> robot_localization -> odom -> Nav2 -> cmd_vel
BNO080 -> sensor_msgs/Imu -----------^                         |
Mag heading/covariance --------------^                         v
GPS/RTK -> NavSatFix -> navsat_transform (later)       drive controller
                                                           |
                                   +-----------------------+----------------+
                                   v                       v                v
                              STM32 UART1             ODESC UART2      GD32 UART3
```

Recommended packages and interfaces:

- `ros2_control`: one rover hardware system exposing wheel velocity commands,
  wheel position/velocity state, actuator faults, and current telemetry.
- A suitable `ros2_controllers` mobile-base controller for the actual steering
  geometry: differential drive, Ackermann, tricycle, or another steering
  controller. Do not choose until the mechanical layout is fixed.
- A BNO080 node publishing `sensor_msgs/Imu` and
  `sensor_msgs/MagneticField`, with measured covariances and correct frame axes.
- Encoder odometry publishing `nav_msgs/Odometry` and wheel joint states.
- `robot_localization` EKF for continuous local `odom -> base_link`, initially
  fusing wheel encoder velocity and IMU angular velocity/orientation.
- `navsat_transform_node` and a second global filter after GPS/RTK is installed.
- Nav2 for costmaps, planning, obstacle avoidance, and waypoint execution.
- `robot_state_publisher` plus a URDF/Xacro model defining `base_link`, IMU,
  camera, GPS antenna, wheel, and tool transforms.
- `rosbag2` logging for IMU, encoders, commands, currents, camera detections,
  GPS, transforms, and faults during every calibration run.

## Vision

Run camera capture and lightweight inference in a separate ROS process from
UART ownership and motor watchdog service. Publish detections using standard
image/detection messages; let a behavior or navigation node convert detections
into goals. Start with a small quantized model and measure end-to-end latency.
Use an accelerator such as a supported Hailo-based Pi AI module if CPU inference
reduces navigation/control responsiveness.

Vision should identify obstacles, rows, plants, or work targets. It should not
replace wheel odometry for short-term motion estimation. Add visual odometry
only if the camera, calibration, lighting, and compute budget support it.

## Localization sequence

1. Calibrate wheel diameter, wheelbase/steering geometry, encoder scale, and
   direction signs. Precise movement is not possible without encoders.
2. Mount the BNO080 rigidly, record its transform, calibrate it in the final
   powered robot, and characterize motor-current magnetic interference.
3. Fuse encoder velocity and IMU yaw rate in a 2D local EKF. Validate straight
   lines, rotations, repeatability, timestamping, and covariance values.
4. Add a perception sensor suitable for collision avoidance. A monocular
   detector alone does not provide dependable obstacle distance.
5. Add GPS/RTK as a global correction through `navsat_transform_node`; retain
   local encoder/IMU odometry so control stays continuous between GPS updates.
6. Add Nav2 only after manual `cmd_vel` control, stops, odometry, TF, and fault
   recovery are reliable.

## Safety and process boundaries

- Exactly one process owns each serial device. A ROS serial node may not run at
  the same time as this Thonny program on the same UART.
- MCU/controller watchdogs stop motion when commands expire. ROS lifecycle and
  diagnostics report link loss, but an independent hardware emergency stop
  must remove motor energy.
- Use monotonic timestamps at UART receipt and synchronize all sensor messages.
- Never perform neural-network inference, disk logging, or web requests inside
  a motor command/watchdog callback.
- Reject stale `cmd_vel`, cap velocity/acceleration, monitor controller faults,
  and default outputs to stopped during startup, shutdown, and reconnect.

## Primary references

- [ROS 2 REP-2000 supported platforms and release dates](https://www.ros.org/reps/rep-2000.html)
- [ROS 2 Jazzy installation on Ubuntu 24.04 arm64](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ros2_control hardware and controller architecture](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
- [Nav2 odometry and encoder guidance](https://docs.nav2.org/setup_guides/odom/setup_odom_gz.html)
- [Nav2 robot_localization sensor fusion guidance](https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html)
- [Nav2 GPS localization architecture](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)
- [ODrive ASCII command protocol](https://docs.odriverobotics.com/v/latest/manual/ascii-protocol.html)
