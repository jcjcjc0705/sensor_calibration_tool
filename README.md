# Sensor Calibration Tool

Scripts for making sensors in **Isaac Sim 5.1** behave like the real hardware
they stand in for. Measure the real device, write its numbers onto the
simulated one, then verify over ROS 2.

Built for a hexapod robot project, but nothing here is specific to it.

## Requirements

| | |
|---|---|
| Isaac Sim | 5.1 |
| ROS 2 | Jazzy (for the verification steps) |
| Python | `pyserial` for the IMU tools |

The ROS 2 tools must run on the interpreter ROS 2 was built against
(`/usr/bin/python3` on Ubuntu 24.04). A conda environment on the PATH will
shadow it and `import rclpy` will fail.

## Layout

```
real2sim/     paste into Isaac Sim's Script Editor
  imu_calibration.py         output rate, filter widths, mount pose
  lidar_2d_calibration.py    range, scan rate, angular resolution
  camera_calibration.py      field of view, resolution, clipping range
  tof_camera_calibration.py  depth camera variant of the above

imu_cmp10a/   run from a terminal with the IMU connected over USB
  measure_imu_cmp10a.py      measure the real device, print the parameters
  configure_imu_cmp10a.py    UI for output rate, baud, bandwidth, packets
```

## Workflow

1. **Measure the real device.** For the CMP10A IMU, `measure_imu_cmp10a.py`
   auto-detects the port and baud rate, records for a few seconds and prints a
   ready-to-paste block. For other sensors, take the numbers from the
   datasheet, or read them off the driver's own ROS topic:

   ```bash
   ros2 topic echo /scan --once        # angle_increment, scan_time, range_min/max
   ros2 topic echo /camera_info --once # width, height, k[0], k[4]
   ```

2. **Apply them in Isaac.** Right-click the sensor prim in the Stage panel ->
   *Copy Prim Path*, paste it into the script, fill in the numbers, then copy
   the whole file into **Window > Script Editor** and run it. Each script
   prints exactly what it changed.

3. **Press Stop, then Play.** Nothing takes effect until the pipeline is
   rebuilt.

4. **Verify over ROS 2.** The intrinsics a camera publishes should match what
   you asked for:

   ```
   fx = width  / (2 * tan(hfov/2))
   fy = height / (2 * tan(vfov/2))
   ```

Every script targets a prim by path, so adding a second sensor means copying
the call block and changing the path. Nothing is auto-discovered, so nothing
is changed behind your back.

## Things that cost us time

**RTX lidar specs live in USD attributes, not JSON profiles.** Isaac 5.1 keeps
them as `omni:sensor:Core:*` on the prim. The JSON files under
`data/lidar_configs/` and the `sensorModelConfig` attribute belong to the
deprecated camera-based lidar; setting them succeeds silently and changes
nothing.

**Sensors must be children of the prim that carries the rigid body.** A sensor
under a plain mesh will not follow the body, and an IMU with no rigid body
ancestor returns zeros.

**Rigid bodies cannot nest.** Vendor camera assets often ship their own
`RigidBodyAPI` so they can be dropped into a scene standalone. Parent one under
a robot link and PhysX reports

```
Incompatible size of velocity tensor ... expected 6, received 12
```

and the articulation silently stops accepting commands. Remove the asset's
rigid body.

**Geometry cannot nest either.** A link whose root is a `Cube` cannot contain a
mesh. Make the link an `Xform` carrying the rigid body, with the geometry as a
child -- the same shape as a URDF `<link>`, and what lets you attach anything
later.

**Stop and Play after any structural change.** Sensors bind to their rigid body
when the simulation starts. Reparent a prim or remove a rigid body mid-run and
the sensor returns uninitialised memory -- large, constant, identical across
axes.

**Isaac renders square pixels.** The vertical field of view follows the render
product's aspect ratio, so a sensor with a square pixel grid and a non-square
field of view cannot be reproduced exactly. `match="fov"` keeps the angles
right and changes the row count; `match="pixels"` keeps the grid and accepts a
small angular error. Angles decide what the robot can see, so prefer `"fov"`
unless the resolution itself is a hard requirement.

**Give each camera its own topic namespace.** Two `ROS2 Camera Info Helper`
nodes both default to `camera_info`, and downstream code then receives
intrinsics that alternate between two different cameras.

**Set mass on the collision shapes, not the rigid body.** PhysX aggregates
per-shape mass into the body's centre of mass and inertia tensor, which is how
you place a heavy motor at one end of a light limb. Mass set on the body
itself overrides that and spreads it by volume.

## Licence

MIT
