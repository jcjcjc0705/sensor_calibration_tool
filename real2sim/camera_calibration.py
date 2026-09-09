# =============================================================================
#  Depth / RGB camera calibration script
#
#  How to use:
#    1. In Isaac Sim: Create > Camera, put it where you want it, then in the
#       Stage panel right-click it -> Copy Prim Path
#    2. Paste it into prim_path below
#    3. Fill in the parameters from the real device's datasheet
#    4. For another camera, copy the whole apply_camera(...) block and paste it
#       below with a different path and parameters
#    5. Copy this entire file into Window > Script Editor and run it
#    6. Set the resolution it prints on your "Isaac Create Render Product" node
#       (Width / Height), or pass render_product_node= to have it done for you
#    7. Press Stop then Play
# =============================================================================

import math

import omni.usd
from pxr import Gf


def apply_camera(prim_path, width_px, height_px, hfov_deg, vfov_deg,
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
        rp_w, rp_h = int(width_px), max(1, int(round(width_px * tv / th)))
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
    print(f"        field of view    {hfov_deg:.2f} x {v_actual:.2f} deg"
          f"   (asked for {hfov_deg} x {vfov_deg})")
    print(f"        range            {range_min_m} - {range_max_m} m  (clippingRange)")
    print(f"        render at        {rp_w} x {rp_h}")
    print(f"        camera_info      fx = fy = {fx:.2f}   cx = {rp_w/2:.1f}   cy = {rp_h/2:.1f}")
    if (rp_w, rp_h) != (int(width_px), int(height_px)):
        print(f"        note: real device is {int(width_px)} x {int(height_px)}; row count")
        print(f"              changed so the vertical field of view is right")
    if abs(v_actual - vfov_deg) > 1.0:
        print(f"        WARNING: vertical field of view off by "
              f"{abs(v_actual - vfov_deg):.1f} deg -- use match='fov' to fix")

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
        print(f"        set Width={rp_w} Height={rp_h} on your render product node")
    print("        Press Stop then Play for this to take effect.")

# ---------------------------- D455 colour stream -----------------------------
apply_camera(
    prim_path           = "/World/Body/Camera",
    width_px            = 1280,
    height_px           = 720,
    hfov_deg            = 90.0,          # colour imager is wider than depth
    vfov_deg            = 58.7,
    range_min_m         = 0.01,          # colour has no range limit
    range_max_m         = 1000.0,
    match               = "pixels",
    render_product_node = "/World/CameraGraph/isaac_create_render_product",          # e.g. "/World/D455ColorGraph/isaac_create_render_product"
)


# ------------------------------- ToF SEN0581 ---------------------------------
# For another camera, copy a block above, change the path and parameters,
# then remove the leading "#" from each line.
#
# apply_camera(
#     prim_path   = "/World/Body/ToF",
#     width_px    = 100,
#     height_px   = 100,
#     hfov_deg    = 70.0,
#     vfov_deg    = 60.0,
#     range_min_m = 0.15,
#     range_max_m = 1.5,
#     match       = "fov",
# )
