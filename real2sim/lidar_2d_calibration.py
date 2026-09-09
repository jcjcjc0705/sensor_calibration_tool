# =============================================================================
#  2D lidar calibration script
#
#  How to use:
#    1. In the Isaac Sim Stage panel, right-click the lidar -> Copy Prim Path
#    2. Paste it into prim_path below
#    3. Fill in the parameters measured from the real device.
#       Run "ros2 topic echo /scan --once" on the real lidar and convert:
#           scan_rate_hz        = 1 / scan_time
#           angle_increment_deg = angle_increment (rad) * 180 / pi
#           range_min_m         = range_min
#           range_max_m         = range_max
#    4. For a second lidar, copy the whole apply_lidar_2d(...) block and paste
#       it below with a different path and parameters
#    5. Copy this entire file into Window > Script Editor and run it
#    6. Press Stop then Play so the scan pipeline picks up the new settings
# =============================================================================

import omni.usd

C = "omni:sensor:Core:"
E = "omni:sensor:Core:emitterState:s001:"

def _set(prim, name, value, log):
    a = prim.GetAttribute(name)
    if not a.IsValid():
        log.append(f"    (missing attribute {name})")
        return
    cur = a.Get()
    if isinstance(cur, bool):
        value = bool(value)
    elif isinstance(cur, int):
        value = int(round(value))
    elif isinstance(cur, float):
        value = float(value)
    a.Set(value)
    log.append(f"    {name.replace(C, ''):32} = {value}")


def apply_lidar_2d(prim_path, range_min_m, range_max_m, scan_rate_hz,
                   angle_increment_deg, rotation="CW", range_accuracy_m=0.03,
                   wavelength_nm=905.0, fov_deg=360.0):
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[FAIL] prim not found: {prim_path}")
        return
    if prim.GetTypeName() != "OmniLidar":
        print(f"[FAIL] {prim_path} is not a lidar (type is {prim.GetTypeName()})")
        return
    if rotation not in ("CW", "CCW"):
        print(f"[FAIL] rotation must be CW or CCW, got {rotation!r}")
        return

    points_per_rev = fov_deg / angle_increment_deg
    report_rate = int(round(points_per_rev * scan_rate_hz))

    log = []
    _set(prim, C + "scanType", "ROTARY", log)
    _set(prim, C + "nearRangeM", range_min_m, log)
    _set(prim, C + "farRangeM", range_max_m, log)
    _set(prim, C + "minReflectionRangeM", range_max_m, log)
    _set(prim, C + "scanRateBaseHz", scan_rate_hz, log)
    _set(prim, C + "reportRateBaseHz", report_rate, log)
    _set(prim, C + "rotationDirection", rotation, log)
    _set(prim, C + "rangeAccuracyM", range_accuracy_m, log)
    _set(prim, C + "waveLengthNm", wavelength_nm, log)
    _set(prim, C + "validStartAzimuthDeg", 0.0, log)
    _set(prim, C + "validEndAzimuthDeg", fov_deg, log)
    _set(prim, C + "startAzimuthOffsetDeg", 0.0, log)
    _set(prim, C + "maxReturns", 1, log)
    _set(prim, C + "numberOfEmitters", 1, log)
    _set(prim, C + "numberOfChannels", 1, log)
    _set(prim, E + "azimuthDeg", [0.0], log)
    _set(prim, E + "elevationDeg", [0.0], log)
    _set(prim, E + "channelId", [1], log)
    _set(prim, E + "fireTimeNs", [0], log)

    print(f"[OK] {prim_path}")
    print(f"        range            {range_min_m} - {range_max_m} m")
    print(f"        scan rate        {scan_rate_hz} Hz")
    print(f"        points per rev   {points_per_rev:.1f}  ({angle_increment_deg} deg step)")
    print(f"        sample rate      {report_rate} points/sec")
    print(f"        rotation         {rotation}")
    for line in log:
        print(line)
    print("        Press Stop then Play for this to take effect.")

# -------------------------------- LIDAR 1 ------------------------------------
apply_lidar_2d(
    prim_path           = "/World/Body/Example_Rotary_2D",  # Copy Prim Path here
    range_min_m         = 0.05,          # range_min
    range_max_m         = 12.0,          # range_max
    scan_rate_hz        = 10.0,          # 1 / scan_time
    angle_increment_deg = 0.8,           # angle_increment converted to degrees
    rotation            = "CCW",         # "CW" clockwise / "CCW" counter-clockwise
    range_accuracy_m    = 0.015,         # ranging accuracy
    wavelength_nm       = 905.0,         # laser wavelength
    fov_deg             = 360.0,         # scan field of view
)


# -------------------------------- LIDAR 2 ------------------------------------
# For a second lidar, copy the block above, change the path and parameters,
# then remove the leading "#" from each line.
#
# apply_lidar_2d(
#     prim_path           = "/World/Body/Lidar_2",
#     range_min_m         = 0.05,
#     range_max_m         = 12.0,
#     scan_rate_hz        = 10.0,
#     angle_increment_deg = 0.8,
# )
