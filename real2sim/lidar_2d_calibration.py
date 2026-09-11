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
                   wavelength_nm=905.0, fov_deg=360.0,
                   dark_range_m=None, dark_reflectance=0.1):
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
    # datasheets usually give two ranges: one for bright targets and a shorter
    # one for dark ones.  minReflectance / minReflectionRangeM model the latter.
    _set(prim, C + "minReflectance", dark_reflectance, log)
    _set(prim, C + "minReflectionRangeM",
         dark_range_m if dark_range_m is not None else range_max_m, log)
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
    # YDLIDAR T-mini Plus (sold by Yahboom), 12 m version.  Numbers from the
    # official datasheet.  Sampling is fixed at 4000 points/s, so scan rate and
    # angular resolution trade off:
    #    6 Hz -> 0.54 deg (factory default)    8 Hz -> 0.72 deg
    #   10 Hz -> 0.90 deg                      12 Hz -> 1.08 deg
    # Keep scan_rate_hz equal to the "frequency" set in the real ROS driver.
    range_min_m         = 0.05,          # 0.05 m
    range_max_m         = 12.0,          # 12 m at 80 % reflectivity
    dark_range_m        = 4.0,           # only 4 m at 10 % reflectivity
    dark_reflectance    = 0.1,
    scan_rate_hz        = 6.0,           # factory default
    angle_increment_deg = 0.54,          # 360 / (4000 / 6)
    rotation            = "CW",          # clockwise seen from above
    range_accuracy_m    = 0.02,          # 20 mm
    wavelength_nm       = 905.0,         # 905 nm
    fov_deg             = 360.0,
)


# -------------------------------- LIDAR 2 ------------------------------------
# For a second lidar, fill in the path and parameters below, then remove the
# leading "#" from each line.  Every parameter is listed; the ones marked
# (optional) can be deleted to keep their default.
#
# apply_lidar_2d(
#     prim_path           = "/World/Body/Lidar_2",  # Copy Prim Path here
#     range_min_m         = 0.05,        # closest distance reported
#     range_max_m         = 12.0,        # furthest distance, bright target
#     dark_range_m        = 4.0,         # (optional) furthest distance, dark target
#     dark_reflectance    = 0.1,         # (optional) reflectivity dark_range_m is for
#     scan_rate_hz        = 6.0,         # revolutions per second
#     angle_increment_deg = 0.54,        # 360 / (sample_rate / scan_rate_hz)
#     rotation            = "CW",        # (optional) "CW" or "CCW", seen from above
#     range_accuracy_m    = 0.02,        # (optional) ranging accuracy
#     wavelength_nm       = 905.0,       # (optional) laser wavelength
#     fov_deg             = 360.0,       # (optional) scan field of view
# )
