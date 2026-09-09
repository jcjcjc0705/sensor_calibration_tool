#!/usr/bin/env python3
"""Measure a real CMP10A IMU and print the parameters for real2sim/imu_calibration.py

Plug the IMU into USB, place it FLAT AND STILL in its final mounting orientation,
then run:

    /usr/bin/python3 measure_imu_cmp10a.py

It auto-detects the serial port and baud rate, records for a few seconds, and
prints a ready-to-paste apply_imu(...) block.

Requires membership of the 'dialout' group:
    sudo usermod -aG dialout $USER && newgrp dialout
"""

import argparse
import glob
import math
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing.  Install it with:  pip install pyserial")

# WitMotion-style frame: 0x55 <type> D0..D7 SUM   (11 bytes)
HEADER = 0x55
FRAME_LEN = 11
T_ACC, T_GYR, T_ANG, T_QUAT = 0x51, 0x52, 0x53, 0x59

G = 9.8                      # the protocol defines acceleration in units of g
ACC_SCALE = 16.0 * G / 32768.0
GYR_SCALE = 2000.0 / 32768.0          # deg/s
ANG_SCALE = 180.0 / 32768.0           # deg
BAUDS = [921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600]


def s16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def parse(buf):
    """Yield (type, payload) for every checksum-valid frame; return leftover bytes."""
    out, i = [], 0
    while i + FRAME_LEN <= len(buf):
        if buf[i] != HEADER:
            i += 1
            continue
        frame = buf[i:i + FRAME_LEN]
        if (sum(frame[:10]) & 0xFF) == frame[10]:
            out.append((frame[1], frame[2:10]))
            i += FRAME_LEN
        else:
            i += 1
    return out, buf[i:]


def find_ports():
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No /dev/ttyUSB* or /dev/ttyACM* found.  Is the IMU plugged in?")
    return ports


def try_baud(port, baud, seconds=0.6):
    """Return how many valid frames arrive at this baud rate."""
    try:
        with serial.Serial(port, baud, timeout=0.1) as ser:
            ser.reset_input_buffer()
            buf, t0 = b"", time.time()
            while time.time() - t0 < seconds:
                buf += ser.read(4096)
            frames, _ = parse(buf)
            return len(frames)
    except (serial.SerialException, OSError) as e:
        if "Permission denied" in str(e):
            sys.exit(f"Permission denied on {port}.\n"
                     f"Run:  sudo usermod -aG dialout $USER && newgrp dialout")
        return 0


def autodetect(port_hint=None, baud_hint=None):
    ports = [port_hint] if port_hint else find_ports()
    print("Detecting port and baud rate ...")
    best = (0, None, None)
    for port in ports:
        for baud in ([baud_hint] if baud_hint else BAUDS):
            n = try_baud(port, baud)
            if n:
                print(f"    {port} @ {baud:>7} -> {n} valid frames")
            if n > best[0]:
                best = (n, port, baud)
            if baud_hint:
                break
    if not best[1]:
        sys.exit("No valid frames on any port/baud.  Check cabling and power.")
    print(f"\n  Using {best[1]} @ {best[2]}\n")
    return best[1], best[2]


def record(port, baud, seconds):
    print(f"Recording {seconds:.0f} s.  Keep the IMU completely still ...")
    acc, gyr, ang, t_acc = [], [], [], []
    with serial.Serial(port, baud, timeout=0.1) as ser:
        ser.reset_input_buffer()
        buf, t0 = b"", time.time()
        while time.time() - t0 < seconds:
            buf += ser.read(4096)
            frames, buf = parse(buf)
            now = time.time()
            for kind, d in frames:
                if kind == T_ACC:
                    acc.append([s16(d[0], d[1]) * ACC_SCALE,
                                s16(d[2], d[3]) * ACC_SCALE,
                                s16(d[4], d[5]) * ACC_SCALE])
                    t_acc.append(now)
                elif kind == T_GYR:
                    gyr.append([math.radians(s16(d[0], d[1]) * GYR_SCALE),
                                math.radians(s16(d[2], d[3]) * GYR_SCALE),
                                math.radians(s16(d[4], d[5]) * GYR_SCALE)])
                elif kind == T_ANG:
                    ang.append([s16(d[0], d[1]) * ANG_SCALE,
                                s16(d[2], d[3]) * ANG_SCALE,
                                s16(d[4], d[5]) * ANG_SCALE])
    return acc, gyr, ang, t_acc


def col(rows, i):
    return [r[i] for r in rows]


def stats(rows, label, unit):
    print(f"  {label}")
    for i, ax in enumerate("XYZ"):
        c = col(rows, i)
        m = statistics.fmean(c)
        s = statistics.pstdev(c) if len(c) > 1 else 0.0
        print(f"      {ax}   mean {m:+10.5f}   std {s:10.6f}   {unit}")
    return [statistics.pstdev(col(rows, i)) if len(rows) > 1 else 0.0 for i in range(3)]


def suggest_filter(rate_hz, bandwidth_hz):
    """Isaac's FilterWidth is a moving average; it only adds lag, it does not
    remove noise (simulated IMUs have none).  So match the lag of the real
    module's internal low-pass instead.

        first-order low-pass delay  ~ 1 / (2*pi*B)
        moving average of N delay   = (N-1)/2 * 1/rate

    Solving for N gives N = 1 + rate / (pi * B).
    """
    if bandwidth_hz <= 0:
        return 1
    return max(1, min(20, int(round(1 + rate_hz / (math.pi * bandwidth_hz)))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="e.g. /dev/ttyUSB0 (default: auto)")
    ap.add_argument("--baud", type=int, help="e.g. 921600 (default: auto)")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--bandwidth", type=float, default=188.0,
                    help="the module's BANDWIDTH register in Hz (default 188). "
                         "Used to derive the Isaac filter widths.")
    a = ap.parse_args()

    port, baud = autodetect(a.port, a.baud)
    acc, gyr, ang, t_acc = record(port, baud, a.seconds)

    if len(acc) < 10:
        sys.exit(f"Only {len(acc)} acceleration frames.  Is acceleration output enabled?")

    rate = (len(t_acc) - 1) / (t_acc[-1] - t_acc[0])

    print("\n" + "=" * 66)
    print(" MEASURED")
    print("=" * 66)
    print(f"  output rate        {rate:8.1f} Hz     ({len(acc)} frames)")
    acc_std = stats(acc, "acceleration", "m/s^2")
    gyr_std = stats(gyr, "angular velocity", "rad/s") if gyr else [0, 0, 0]

    mx, my, mz = (statistics.fmean(col(acc, i)) for i in range(3))
    mag = math.hypot(math.hypot(mx, my), mz)
    print(f"\n  gravity magnitude  {mag:8.4f} m/s^2   (should be close to 9.8)")
    if abs(mag - G) > 0.5:
        print("      WARNING: far from 9.8 -- the IMU was moving, or it needs calibration.")

    # Tilt of the sensor relative to level, from the measured gravity direction.
    roll = math.degrees(math.atan2(my, mz))
    pitch = math.degrees(math.atan2(-mx, math.hypot(my, mz)))
    up = "XYZ"[max(range(3), key=lambda i: abs([mx, my, mz][i]))]
    up_sign = "+" if [mx, my, mz]["XYZ".index(up)] > 0 else "-"
    print(f"  up axis            {up_sign}{up}      (axis reading ~1 g)")
    print(f"  measured tilt      roll {roll:+.2f} deg   pitch {pitch:+.2f} deg")
    if ang:
        print(f"  device's own angle roll {statistics.fmean(col(ang,0)):+.2f} "
              f"pitch {statistics.fmean(col(ang,1)):+.2f} "
              f"yaw {statistics.fmean(col(ang,2)):+.2f} deg")

    # snap to the nearest configured rate when we are within 5 %
    nominal = min([0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200],
                  key=lambda r: abs(r - rate))
    rate_out = nominal if abs(rate - nominal) / nominal < 0.05 else rate

    af = gf = suggest_filter(rate_out, a.bandwidth)

    print("\n" + "=" * 66)
    print(" PASTE INTO real2sim/imu_calibration.py")
    print("=" * 66)
    print(f"""
apply_imu(
    prim_path     = "/World/YOUR/Imu_Sensor",   # <- Copy Prim Path from Isaac
    rate_hz       = {rate_out:.1f},
    acc_filter    = {af},
    gyr_filter    = {gf},
    ori_filter    = 1,
    mount_xyz_m   = (0.0, 0.0, 0.0),            # <- from your CAD, in meters
    mount_rpy_deg = ({roll:.2f}, {pitch:.2f}, 0.0),   # <- yaw must be set by you
)
""")
    print("  Notes")
    print(f"    * measured {rate:.1f} Hz, reported as {rate_out:g} Hz "
          f"(nearest configured rate).")
    print(f"    * filter widths come from BANDWIDTH={a.bandwidth:g} Hz.  They model the")
    print("      module's internal lag, NOT its noise -- a simulated IMU has no")
    print("      noise for a filter to remove.  Pass --bandwidth if you changed it.")
    print("    * roll and pitch come from gravity and are only valid if the IMU")
    print("      was mounted the way it will be on the robot.  Yaw cannot be")
    print("      measured this way -- set it from how the IMU faces on the body.")
    print(f"    * noise belongs in imu_realism.py, not here:")
    print(f"          accel_noise={max(acc_std):.5f}   gyro_noise={max(gyr_std):.5f}")
    if max(gyr_std) == 0.0:
        print("      The gyro reads exactly zero because the module zeroes it when it")
        print("      decides it is still.  Its real noise cannot be measured this way.")


if __name__ == "__main__":
    main()
