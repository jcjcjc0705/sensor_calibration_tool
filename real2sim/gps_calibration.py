# =============================================================================
#  Absolute position ("GPS") calibration script
#
#  A position fix is simulated as a point stuck to the robot -- the antenna, or
#  the tag of an indoor positioning system -- whose world position is
#  published as geometry_msgs/PoseStamped.
#
#  How to use:
#    1. In Isaac Sim, select the robot body (the prim with the rigid body),
#       Create > Xform, rename it (e.g. GPS_Antenna)
#    2. Move it to where the antenna sits on the robot: type its Translate in
#       the Property panel, or drag it in the viewport (W key)
#    3. Right-click it -> Copy Prim Path, paste it into prim_path below, and
#       the path of your graph's Isaac Simulation Gate node
#    4. Set how often the real receiver updates
#    5. Copy this entire file into Window > Script Editor and run it
#    6. Press Stop then Play
# =============================================================================

import omni.usd
from pxr import UsdGeom, UsdPhysics


def _set_node_input(node_path, name, value):
    """Write one input on an OmniGraph node.  Returns True on success.

    Written on the node's USD prim, the way the Property panel does it, so the
    value is saved with the scene (og.Controller.set is lost on save/reopen).
    """
    node = omni.usd.get_context().get_stage().GetPrimAtPath(node_path)
    if not node.IsValid():
        print(f"        [WARN] node not found: {node_path}")
        return False
    attr = node.GetAttribute(name)
    if not attr.IsValid():
        print(f"        [WARN] {node_path} has no {name}")
        return False
    attr.Set(value)
    return True


def apply_gps(prim_path, rate_hz=None, tick_hz=60.0, gate_node=None):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[FAIL] prim not found: {prim_path}")
        return
    if prim.GetTypeName() != "Xform":
        print(f"[FAIL] {prim_path} should be a plain Xform (type is {prim.GetTypeName()})")
        return

    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        print(f"[FAIL] {prim_path} is the robot body itself.  The GPS point must be a"
              f" separate Xform: create one under the body and point prim_path at it.")
        return

    body = prim.GetParent()
    while body.IsValid() and not body.HasAPI(UsdPhysics.RigidBodyAPI):
        body = body.GetParent()
    if not body.IsValid():
        print(f"[WARN] {prim_path} has no rigid-body ancestor, so it will not move with"
              f" the robot.  Put it under the robot body.")

    print(f"[OK] {prim_path}")
    print(f"        attached to      {body.GetPath() if body.IsValid() else '(nothing)'}")
    if body.IsValid():
        m = (UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0) *
             UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0).GetInverse())
        p = m.ExtractTranslation()
        print(f"        mount position   ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) m  in the body's frame"
              f"  (set it in Isaac Sim)")

    if rate_hz:
        step = max(1, int(round(tick_hz / rate_hz)))
        print(f"        update rate      {tick_hz / step:.1f} Hz  (gate step {step} at"
              f" {tick_hz:g} Hz ticks, asked for {rate_hz:g} Hz)")
        if gate_node:
            if _set_node_input(gate_node, "inputs:step", step):
                print(f"        gate step written to {gate_node}")
        else:
            print(f"        gate step not written -- set Step={step} on the gate, or pass gate_node=")
    print("        Press Stop then Play for this to take effect.")


# ------------------------------ GPS 1 ---------------------------------------
apply_gps(
    prim_path   = "/World/Body/GPS_Antenna",   # Copy Prim Path here
    rate_hz     = 10.0,                        # GPS 1-10 Hz, UWB 10-100 Hz, mocap 100+ Hz
    tick_hz     = 60.0,                        # your simulation tick rate
    gate_node   = "/World/GPSGraph/isaac_simulation_gate",
)


# ------------------------------ GPS 2 ---------------------------------------
# For a second receiver or tag, fill in the path and parameters below, then
# remove the leading "#" from each line.
#
# apply_gps(
#     prim_path   = "/World/Body/GPS_Antenna_01",
#     rate_hz     = 10.0,        # (optional)
#     tick_hz     = 60.0,        # (optional)
#     gate_node   = None,        # (optional)
# )
