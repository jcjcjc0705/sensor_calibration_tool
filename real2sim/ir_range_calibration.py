# =============================================================================
#  Single-point IR range sensor calibration script
#
#  How to use:
#    1. In Isaac Sim: Create > Sensors > LightBeam Sensor > Generic, put it
#       where you want it, then in the Stage panel right-click it
#       -> Copy Prim Path
#    2. Paste it into prim_path below, and the paths of your graph's
#       ROS2 Publisher and Isaac Simulation Gate nodes
#    3. Fill in the parameters from the real device's datasheet
#    4. For another sensor, copy the whole apply_ir_range(...) block and paste
#       it below with a different path and parameters
#    5. Copy this entire file into Window > Script Editor and run it
#    6. Press Stop then Play
# =============================================================================

import omni.usd
from pxr import Gf


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
        print(f"        [WARN] {node_path} has no {name}"
              f"  (set messagePackage / messageName on the node first)")
        return False
    attr.Set(value)
    return True


def apply_ir_range(prim_path, range_min_m, range_max_m, forward_axis=(1.0, 0.0, 0.0),
                   rate_hz=None, tick_hz=60.0, gate_node=None,
                   publisher_node=None, radiation_type=1, field_of_view_rad=None,
                   variance=None):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[FAIL] prim not found: {prim_path}")
        return
    if prim.GetTypeName() != "IsaacLightBeamSensor":
        print(f"[FAIL] {prim_path} is not a LightBeam sensor (type is {prim.GetTypeName()})")
        return

    prim.GetAttribute("numRays").Set(1)
    prim.GetAttribute("curtainLength").Set(0.0)
    prim.GetAttribute("minRange").Set(float(range_min_m))
    prim.GetAttribute("maxRange").Set(float(range_max_m))
    prim.GetAttribute("forwardAxis").Set(Gf.Vec3f(*forward_axis))

    print(f"[OK] {prim_path}")
    print(f"        range            {range_min_m} - {range_max_m} m")
    print(f"        beam direction   {tuple(forward_axis)}  (sensor's local frame)")

    if publisher_node:
        fields = [("inputs:min_range", float(range_min_m)),
                  ("inputs:max_range", float(range_max_m)),
                  ("inputs:radiation_type", int(radiation_type))]
        if field_of_view_rad is not None:
            fields.append(("inputs:field_of_view", float(field_of_view_rad)))
        if variance is not None:
            fields.append(("inputs:variance", float(variance)))
        done = [f"{n.split(':', 1)[1]}={v}" for n, v in fields
                if _set_node_input(publisher_node, n, v)]
        if done:
            print(f"        publisher        {', '.join(done)}")
    else:
        print("        publisher        not set -- pass publisher_node= to write"
              " min_range / max_range / radiation_type")

    if rate_hz:
        step = max(1, int(round(tick_hz / rate_hz)))
        print(f"        update rate      {tick_hz / step:.1f} Hz  (gate step {step} at"
              f" {tick_hz:g} Hz ticks, asked for {rate_hz:g} Hz)")
        if gate_node:
            if _set_node_input(gate_node, "inputs:step", step):
                print(f"        gate step written to {gate_node}")
        else:
            print(f"        gate step not written here -- needs Step={step}; set gate_node"
                  f" in one block, or by hand if no block does")
    print("        Press Stop then Play for this to take effect.")


# --------------------------- IR RANGE 1 (Leg1) -------------------------------
apply_ir_range(
    prim_path      = "/World/Leg1/LightBeam_Sensor",  # Copy Prim Path here
    # Sharp GP2Y0A02YK0F, official datasheet (sheet E4-A00101EN)
    range_min_m    = 0.20,          # 20 cm
    range_max_m    = 1.50,          # 150 cm
    forward_axis   = (0.0, 0.0, -1.0),
    rate_hz        = 26.0,          # one measurement every 38.3 ms +- 9.6 ms
    tick_hz        = 60.0,          # your simulation tick rate
    gate_node      = "/World/IrGraph/isaac_simulation_gate",          # e.g. "/World/IrGraph/isaac_simulation_gate"
    publisher_node = "/World/IrGraph/ros2_publisher",          # e.g. "/World/IrGraph/ros2_publisher"
    radiation_type = 1,             # 1 = infrared, 0 = ultrasound
    # field_of_view_rad and variance are not on the GP2Y0A02YK0F datasheet
)


# --------------------------- IR RANGE 2 (Leg2) -------------------------------
# For another sensor, fill in the path and parameters below.  The ones marked
# (optional) can be deleted to keep their default.  The gate is usually shared
# by all sensors, so gate_node only needs to be set in one block.
apply_ir_range(
    prim_path         = "/World/Leg2/LightBeam_Sensor",
    range_min_m       = 0.20,
    range_max_m       = 1.50,
    forward_axis      = (0.0, 0.0, -1.0),   # (optional) beam direction
    rate_hz           = 26.0,               # (optional) real update rate
    tick_hz           = 60.0,               # (optional) simulation tick rate
    gate_node         = "/World/IrGraph/isaac_simulation_gate",               # (optional) gate node to set
    publisher_node    = "/World/IrGraph/ros2_publisher_01",               # (optional) e.g. "/World/IrGraph/ros2_publisher_01"
    radiation_type    = 1,                  # (optional) 1 = infrared, 0 = ultrasound
    field_of_view_rad = None,               # (optional) beam width, radians
    variance          = None,               # (optional) range variance, m^2
)
