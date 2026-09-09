# =============================================================================
#  IMU calibration script
#
#  How to use:
#    1. In the Isaac Sim Stage panel, right-click the IMU -> Copy Prim Path
#    2. Paste it into prim_path below
#    3. Fill in the parameters measured from the real device
#    4. For a second IMU, copy the whole apply_imu(...) block and paste it
#       below with a different path and parameters
#    5. Copy this entire file into Window > Script Editor and run it
#
#  Omit any parameter you do not want to change (just delete that line).
# =============================================================================

import math

import omni.usd
from pxr import Gf, UsdGeom

def apply_imu(prim_path, rate_hz=None, acc_filter=None, gyr_filter=None,
              ori_filter=None, mount_xyz_m=None, mount_rpy_deg=None):
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[FAIL] prim not found: {prim_path}")
        return
    if prim.GetTypeName() != "IsaacImuSensor":
        print(f"[FAIL] {prim_path} is not an IMU (type is {prim.GetTypeName()})")
        return

    done = []

    def set_attr(name, value, cast):
        a = prim.GetAttribute(name)
        if a.IsValid():
            a.Set(cast(value))
            done.append(f"{name} = {cast(value)}")
        else:
            print(f"[WARN] {prim_path} has no attribute {name}")

    if rate_hz is not None:
        set_attr("sensorPeriod", 1.0 / rate_hz, float)
    if acc_filter is not None:
        set_attr("linearAccelerationFilterWidth", acc_filter, int)
    if gyr_filter is not None:
        set_attr("angularVelocityFilterWidth", gyr_filter, int)
    if ori_filter is not None:
        set_attr("orientationFilterWidth", ori_filter, int)

    if mount_xyz_m is not None:
        a = prim.GetAttribute("xformOp:translate")
        if not a.IsValid():
            a = UsdGeom.Xformable(prim).AddTranslateOp().GetAttr()
        a.Set(Gf.Vec3d(*mount_xyz_m))
        done.append(f"mount position = {tuple(mount_xyz_m)} m")

    if mount_rpy_deg is not None:
        roll, pitch, yaw = (math.radians(v) / 2.0 for v in mount_rpy_deg)
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        q = Gf.Quatd(cr * cp * cy + sr * sp * sy,
                     Gf.Vec3d(sr * cp * cy - cr * sp * sy,
                              cr * sp * cy + sr * cp * sy,
                              cr * cp * sy - sr * sp * cy))
        a = prim.GetAttribute("xformOp:orient")
        if not a.IsValid():
            a = UsdGeom.Xformable(prim).AddOrientOp().GetAttr()
        a.Set(Gf.Quatf(q) if isinstance(a.Get(), Gf.Quatf) else q)
        done.append(f"mount rotation = {tuple(mount_rpy_deg)} deg")

    print(f"[OK] {prim_path}")
    for d in done:
        print(f"        {d}")

# --------------------------------- IMU 1 -------------------------------------
apply_imu(
    prim_path     = "/World/Body/Imu_Sensor",   # right-click -> Copy Prim Path
    rate_hz       = 200.0,                      # real device output rate (Hz)
    acc_filter    = 1,                          # accelerometer filter width
    gyr_filter    = 1,                          # gyroscope filter width
    ori_filter    = 1,                          # orientation filter width, keep 1-3
    mount_xyz_m   = (0.0, 0.0, 0.5),            # mount position (x, y, z) in meters
    mount_rpy_deg = (0.0, 0.0, 0.0),            # mount rotation (roll, pitch, yaw) in degrees
)


# --------------------------------- IMU 2 -------------------------------------
# For a second IMU, copy the block above, change the path and parameters,
# then remove the leading "#" from each line.
#
# apply_imu(
#     prim_path     = "/World/Body/Imu_2",
#     rate_hz       = 200.0,
#     acc_filter    = 5,
#     gyr_filter    = 5,
#     ori_filter    = 1,
#     mount_xyz_m   = (0.1, 0.0, 0.05),
#     mount_rpy_deg = (0.0, 0.0, 90.0),
# )
