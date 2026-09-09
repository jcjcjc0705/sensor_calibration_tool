# =============================================================================
#  3D ToF depth camera calibration script
#
#  How to use:
#    1. In Isaac Sim: Create > Camera, then in the Stage panel right-click it
#       -> Copy Prim Path
#    2. Paste it into prim_path below
#    3. Fill in the parameters from the real device's datasheet
#    4. For a second camera, copy the whole apply_tof(...) block and paste it
#       below with a different path and parameters
#    5. Copy this entire file into Window > Script Editor and run it
#    6. Set the resolution it prints on your "Isaac Create Render Product" node
#       (Width / Height), then press Stop and Play
# =============================================================================

import math

import omni.usd
from pxr import Gf

def apply_tof(prim_path, width_px, height_px, hfov_deg, vfov_deg,
              range_min_m, range_max_m, match="fov", render_product_node=None):
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[FAIL] prim not found: {prim_path}")
        return
    if prim.GetTypeName() != "Camera":
        print(f"[FAIL] {prim_path} is not a Camera (type is {prim.GetTypeName()})")
        return
    if match not in ("fov", "pixels"):
        print(f"[FAIL] match must be 'fov' or 'pixels', got {match!r}")
        return

    th = math.tan(math.radians(hfov_deg) / 2.0)
    tv = math.tan(math.radians(vfov_deg) / 2.0)

    if match == "fov":
        rp_w = int(width_px)
        rp_h = max(1, int(round(width_px * tv / th)))
    else:
        rp_w, rp_h = int(width_px), int(height_px)

    ha = prim.GetAttribute("horizontalAperture").Get() or 20.955
    fl = ha / (2.0 * th)
    prim.GetAttribute("focalLength").Set(float(fl))
    prim.GetAttribute("verticalAperture").Set(float(ha * rp_h / rp_w))
    prim.GetAttribute("clippingRange").Set(Gf.Vec2f(float(range_min_m), float(range_max_m)))

    fx = rp_w / (2.0 * th)
    v_actual = 2 * math.degrees(math.atan(rp_h / (2 * fx)))

    print(f"[OK] {prim_path}")
    print(f"        h field of view  {hfov_deg:.2f} deg")
    print(f"        v field of view  {v_actual:.2f} deg   (asked for {vfov_deg})")
    print(f"        range            {range_min_m} - {range_max_m} m  (clippingRange)")
    print(f"        focalLength      {fl:.4f}")
    print(f"        render at        {rp_w} x {rp_h}")
    print(f"        camera_info      fx = fy = {fx:.2f}   cx = {rp_w/2:.1f}   cy = {rp_h/2:.1f}")
    if (rp_w, rp_h) != (int(width_px), int(height_px)):
        print(f"        note: the real device is {int(width_px)} x {int(height_px)}; the row")
        print(f"              count differs so the vertical field of view is right")
    if abs(v_actual - vfov_deg) > 1.0:
        print(f"        WARNING: vertical field of view is off by "
              f"{abs(v_actual - vfov_deg):.1f} deg -- use match='fov'")

    if render_product_node:
        node = stage.GetPrimAtPath(render_product_node)
        if not node.IsValid():
            print(f"        [WARN] render product node not found: {render_product_node}")
        else:
            for name, value in (("inputs:width", rp_w), ("inputs:height", rp_h)):
                a = node.GetAttribute(name)
                if a.IsValid():
                    a.Set(int(value))
                else:
                    print(f"        [WARN] node has no {name}")
            print(f"        resolution written to {render_product_node}")
    else:
        print(f"        set Width={rp_w} Height={rp_h} by hand on your")
        print(f"        Isaac Create Render Product node, or pass render_product_node=")
    print("        Press Stop then Play for this to take effect.")

# ------------------------------- ToF CAMERA 1 --------------------------------
apply_tof(
    prim_path           = "/World/Body/ToF",   # Copy Prim Path here
    width_px            = 100,                 # real device columns
    height_px           = 100,                 # real device rows
    hfov_deg            = 70.0,                # horizontal field of view
    vfov_deg            = 60.0,                # vertical field of view
    range_min_m         = 0.15,                # closest the device reports
    range_max_m         = 1.5,                 # furthest the device reports
    match               = "fov",               # "fov" keeps the angles right,
                                               # "pixels" keeps the pixel grid
    # optional: set the resolution on the graph node too
    render_product_node = "/World/TofGraph/isaac_create_render_product",
)


# ------------------------------- ToF CAMERA 2 --------------------------------
# For a second camera, copy the block above, change the path and parameters,
# then remove the leading "#" from each line.
#
# apply_tof(
#     prim_path   = "/World/Body/ToF_rear",
#     width_px    = 100,
#     height_px   = 100,
#     hfov_deg    = 70.0,
#     vfov_deg    = 60.0,
#     range_min_m = 0.15,
#     range_max_m = 1.5,
#     match       = "fov",
#    # optional: set the resolution on the graph node too
#    render_product_node = "/World/TofGraph/isaac_create_render_product",
# )
