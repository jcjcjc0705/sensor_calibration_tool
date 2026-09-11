# Sensor Calibration Tool

Scripts that make sensors in **Isaac Sim 5.1** behave like the real hardware
they stand in for: take the real device's numbers, write them onto the
simulated sensor, and check the result over ROS 2.

The scripts are generic -- nothing in them is tied to a robot or a sensor
brand. The values filled in are for the hardware listed under
[Current values](#current-values).

## Requirements

| | |
|---|---|
| Isaac Sim | 5.1, with the ROS 2 bridge enabled |
| ROS 2 | Jazzy |
| Python | `pyserial` for the IMU tools |

**Isaac Sim launch setting.** For complete 2D lidar scans, Isaac must wait for
each rendered frame before stepping on. Either start it with

```bash
./isaac-sim.sh --/app/hydraEngine/waitIdle=true
```

or add this line under `[settings.app]` in
`isaac_sim_5.1/apps/isaacsim.exp.full.kit` so every launch (including the App
Selector) uses it:

```toml
hydraEngine.waitIdle = true
```

**ROS 2 Python.** Scripts that `import rclpy` must run on the interpreter ROS 2
was built against (`/usr/bin/python3` on Ubuntu 24.04), not a conda Python.

## What's included

```
real2sim/     run inside Isaac Sim (Window > Script Editor)
  imu_calibration.py         IMU
  lidar_2d_calibration.py    2D rotating lidar
  camera_calibration.py      RGB camera
  tof_camera_calibration.py  depth (ToF) camera
  ir_range_calibration.py    single-point IR range sensor
  gps_calibration.py         absolute position (GPS / indoor positioning)

imu_cmp10a/   run from a terminal with the IMU on USB
  measure_imu_cmp10a.py      measure the real IMU, print its apply_imu block
  configure_imu_cmp10a.py    UI to configure the IMU module
```

### real2sim scripts

| Script | Isaac prim | What it sets |
|---|---|---|
| `imu_calibration.py` | Isaac IMU Sensor | output rate, accelerometer / gyroscope / orientation filter widths, mount position and rotation |
| `lidar_2d_calibration.py` | RTX Lidar (`OmniLidar`) | range, dark-target range, scan rate, angular step, rotation direction, range accuracy, wavelength, field of view |
| `camera_calibration.py` | Camera | field of view, resolution, clipping range; resolution also written to the render product node |
| `tof_camera_calibration.py` | Camera | same as above, for a depth camera |
| `ir_range_calibration.py` | LightBeam Sensor | range and beam direction on the sensor; `min_range`, `max_range`, `radiation_type`, `field_of_view`, `variance` on the ROS 2 publisher; update rate |
| `gps_calibration.py` | Xform | update rate; checks the point is attached to a rigid body and prints where it is mounted |

Each script prints every value it changed, and every warning if a prim or
node is not where the script expects it.

## Using the real2sim scripts

1. **Set up the sensor in Isaac.** Put the sensor prim under the robot link
   that carries the rigid body so it moves with the robot, and build its
   Action Graph (see below).
2. **Copy its path.** In the Stage panel, right-click the sensor ->
   *Copy Prim Path*, and paste it into `prim_path` in the script. Do the same
   for any graph node the script asks for (`render_product_node`,
   `publisher_node`, `gate_node`).
3. **Fill in the real device's numbers** from its datasheet, or from the
   measurement tools below.
4. **Run it.** Copy the whole file into **Window > Script Editor** and run.
5. **Press Stop, then Play**, then **save the scene** (Ctrl+S). Everything the
   scripts write is stored in the scene file.

For a second sensor of the same kind, copy the `apply_...(...)` block and
change the path and values. Nothing is auto-discovered; a script only touches
the prims and nodes you name.

### Action Graphs

The graph holds only what does not depend on the sensor model: the nodes,
their wiring, topic names and frame ids. Everything from the datasheet is set
by the script.

| Sensor | Graph | Set in the graph |
|---|---|---|
| IMU | On Playback Tick -> Isaac Read IMU -> ROS2 Publish Imu; Isaac Read Simulation Time -> timeStamp | topic, frame id |
| 2D lidar | On Playback Tick -> Isaac Create Render Product (lidar) -> ROS2 RTX Lidar Helper, type `laser_scan` | topic, frame id |
| RGB camera | On Playback Tick -> Isaac Create Render Product (camera) -> ROS2 Camera Helper, type `rgb`, and ROS2 Camera Info Helper | topics, frame id |
| ToF camera | as RGB, Camera Helper type `depth` | topics, frame id |
| IR range | see below | message type, topic, frame id |
| GPS | see below | message type, topic, frame id, orientation |

Give each camera its own topic names (for example `rgb/camera_info` and
`tof/camera_info`).

**IR range** -- one read / index / publisher chain per sensor, one gate and one
time splitter shared by all:

```
On Playback Tick -> Isaac Simulation Gate -> Isaac Read LightBeam Sensor
    .Linear Depth Data -> Get Array Index (index 0) -> ROS2 Publisher .range
Isaac Read Simulation Time -> Isaac Time Splitter
    -> ROS2 Publisher .header:stamp:sec / .header:stamp:nanosec
```

On the ROS2 Publisher set `messagePackage = sensor_msgs`,
`messageName = Range` first -- its `range`, `min_range`, ... inputs appear
only after that -- then `topicName` and `header:frame_id`.

**GPS** -- create an Xform under the robot body (e.g. `GPS_Antenna`) and move
it to where the antenna sits. The script never moves it.

```
On Playback Tick -> Isaac Simulation Gate -> ROS2 Publisher
Isaac Read World Pose (prim = the antenna Xform) .Translation
    -> Break 3-Vector -> ROS2 Publisher .pose:position:x / y / z
Isaac Read Simulation Time -> Isaac Time Splitter
    -> ROS2 Publisher .header:stamp:sec / .header:stamp:nanosec
```

On the ROS2 Publisher set `messagePackage = geometry_msgs`,
`messageName = PoseStamped`, `topicName`, `header:frame_id = world` and
`pose:orientation:w = 1`.

### Update rates

The IR and GPS graphs would otherwise publish on every simulation tick, so
their scripts throttle them with the Simulation Gate: `step = tick_hz / rate_hz`,
written to `gate_node`. With 60 Hz ticks a 26 Hz IR sensor becomes 30 Hz and a
10 Hz GPS stays 10 Hz; the script prints the rate you actually get.

### Cameras: `match`

Isaac renders square pixels, so the vertical field of view follows the
resolution's aspect ratio.

- `match="fov"` keeps both angles and adjusts the number of rows
  (a 100 x 100 sensor over 70 x 60 deg is rendered at 100 x 82).
- `match="pixels"` keeps the resolution and prints how far the vertical angle
  is off.

## IMU tools (CMP10A)

For WitMotion-protocol IMUs such as the Yahboom CMP10A. The serial port needs
membership of the `dialout` group:

```bash
sudo usermod -aG dialout $USER && newgrp dialout
```

**Measure** -- place the IMU flat and still, then:

```bash
/usr/bin/python3 imu_cmp10a/measure_imu_cmp10a.py
```

It finds the port and baud rate, records for a few seconds, and prints a
ready-to-paste `apply_imu(...)` block (output rate, filter widths) along with
the mean and standard deviation of each axis.

| Option | Default | |
|---|---|---|
| `--port` | auto | e.g. `/dev/ttyUSB0` |
| `--baud` | auto | e.g. `230400` |
| `--seconds` | 8 | recording length |
| `--bandwidth` | 188 | the module's filter bandwidth in Hz, used for the filter widths |

**Configure** -- a window for the module's settings:

```bash
python3 imu_cmp10a/configure_imu_cmp10a.py            # --scale 1.5 for a larger UI
```

Output rate, baud rate, bandwidth, installation orientation, fusion
algorithm, gyro still-threshold, LED and output packets. *Recommended* fills in
settings for a legged robot; nothing is written until *Apply and Save*. Also
has accelerometer calibration, heading reset and factory reset.

## Checking the result over ROS 2

```bash
ros2 topic hz   /scan                 # rate
ros2 topic echo /scan --once          # angle_increment, scan_time, range_min / range_max
ros2 topic echo /rgb/camera_info --once
ros2 topic echo /ir_range/leg1 --once # min_range, max_range, radiation_type
```

A camera's published intrinsics should match the field of view you asked for:

```
hfov = 2 * atan(width  / (2 * fx))
vfov = 2 * atan(height / (2 * fy))
```

## Current values

| Sensor | Device | Values in the scripts |
|---|---|---|
| IMU | Yahboom CMP10A | 200 Hz, filter width 1 (module set to 200 Hz, 230400 baud, 188 Hz bandwidth) |
| 2D lidar | YDLIDAR T-mini Plus, 12 m | 0.05-12 m (4 m on dark targets), 6 Hz, 0.54 deg, clockwise |
| Depth camera | DFRobot SEN0581 (ToF) | 70 x 60 deg, 0.15-1.5 m, rendered at 100 x 82 |
| RGB camera | Intel RealSense D455, colour stream | 1280 x 720, 90 x 58.7 deg |
| IR range | Sharp GP2Y0A02YK0F (x2) | 0.20-1.50 m, infrared, 26 Hz (30 Hz at 60 Hz ticks) |
| Position | GPS / indoor positioning tag | 10 Hz |

## Licence

MIT
