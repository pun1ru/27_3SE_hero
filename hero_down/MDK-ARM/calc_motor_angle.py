import math

def solve_wheel_center(param, theta_rad):
    """Replicate JointSolveWheelCenterJacobian forward kinematics (no Jacobian)."""
    mx, mz, l_oa, l_ab, l_mb, l_ow, branch_sign = param
    bx = mx + l_mb * math.cos(theta_rad)
    bz = mz + l_mb * math.sin(theta_rad)
    d = math.sqrt(bx*bx + bz*bz)
    eps = 1e-7
    if d < eps:
        return None, None
    oa2 = l_oa * l_oa
    ab2 = l_ab * l_ab
    a = (oa2 - ab2 + d*d) / (2.0 * d)
    h2 = oa2 - a*a
    if h2 < -1e-8:
        return None, None
    if h2 < 0:
        h2 = 0.0
    h = branch_sign * math.sqrt(h2)
    ux = bx / d
    uz = bz / d
    nx = -uz
    nz = ux
    ax = a * ux + h * nx
    az = a * uz + h * nz
    scale = l_ow / l_oa
    return scale * ax, scale * az

# ===== Parameters from jointControl.c =====
front_p = (-0.04234, 0.03067, 0.05000, 0.05000, 0.06000, 0.11650, 1.0)
rear_p  = (-0.13254, 0.02368, 0.07500, 0.10524, 0.06000, 0.13900, 1.0)

ox_f, oy_f, oz_f = 0.24345, 0.17475, -0.16122
ox_r, oy_r, oz_r = -0.10970, 0.16475, -0.25150
wheel_r = 0.080

# Mechanical limits (motor angles in rad, from code)
front_min_rad, front_max_rad = -2.70, -1.70   # ~[-154.7°, -97.4°]
rear_rb_min_rad,  rear_rb_max_rad  = -0.63, 1.80    # [-36.1°, 103.1°]
rear_lb_min_rad,  rear_lb_max_rad  = -0.56, 1.82    # [-32.1°, 104.3°]

# Target conditions
target_cx_sum = -(ox_f + ox_r)   # -0.13375
target_cz_diff = oz_f - oz_r     # 0.09028

print("=" * 65)
print("四连杆机构前后电机角度求解")
print("=" * 65)
print(f"目标: cx_f + cx_r = {target_cx_sum:.6f} m")
print(f"目标: cz_r - cz_f = {target_cz_diff:.6f} m")
print()
print(f"机械限位: 前腿 [{front_min_rad:.4f}, {front_max_rad:.4f}] rad")
print(f"                    [{math.degrees(front_min_rad):.1f}°, {math.degrees(front_max_rad):.1f}°]")
print(f"          后腿(RB) [{rear_rb_min_rad:.4f}, {rear_rb_max_rad:.4f}] rad")
print(f"                    [{math.degrees(rear_rb_min_rad):.1f}°, {math.degrees(rear_rb_max_rad):.1f}°]")
print(f"          后腿(LB) [{rear_lb_min_rad:.4f}, {rear_lb_max_rad:.4f}] rad")
print(f"                    [{math.degrees(rear_lb_min_rad):.1f}°, {math.degrees(rear_lb_max_rad):.1f}°]")
print()

# ---- 1. Global best (no limit constraint) ----
best_global = None
best_global_err = float('inf')
for tf_d in range(-180, 181):
    tf = math.radians(tf_d)
    cxf, czf = solve_wheel_center(front_p, tf)
    if cxf is None: continue
    for tr_d in range(-180, 181):
        tr = math.radians(tr_d)
        cxr, czr = solve_wheel_center(rear_p, tr)
        if cxr is None: continue
        ex = abs(cxf + cxr - target_cx_sum)
        ez = abs(czr - czf - target_cz_diff)
        err = ex + ez
        if err < best_global_err:
            best_global_err = err
            best_global = (tf_d, tr_d, cxf, czf, cxr, czr, ex, ez)

# ---- 2. Best within mechanical limits (0.1° resolution) ----
best_limited = None
best_limited_err = float('inf')
for tf_d in [d * 0.1 - 180.0 for d in range(3601)]:
    tf = math.radians(tf_d)
    if tf < front_min_rad or tf > front_max_rad:
        continue
    cxf, czf = solve_wheel_center(front_p, tf)
    if cxf is None: continue
    for tr_d in [d * 0.1 - 180.0 for d in range(3601)]:
        tr = math.radians(tr_d)
        if tr < rear_rb_min_rad or tr > rear_lb_max_rad:
            continue
        cxr, czr = solve_wheel_center(rear_p, tr)
        if cxr is None: continue
        ex = abs(cxf + cxr - target_cx_sum)
        ez = abs(czr - czf - target_cz_diff)
        err = ex + ez
        if err < best_limited_err:
            best_limited_err = err
            best_limited = (tf_d, tr_d, cxf, czf, cxr, czr, ex, ez)

# ---- Print global best ----
tf, tr, cxf, czf, cxr, czr, ex, ez = best_global
print("【全局最优解（无机械限位约束）】")
print(f"  θ_front = {tf:.2f}° ({math.radians(tf):.6f} rad)")
print(f"  θ_rear  = {tr:.2f}° ({math.radians(tr):.6f} rad)")
in_limit_f = front_min_rad <= math.radians(tf) <= front_max_rad
in_limit_r = rear_rb_min_rad <= math.radians(tr) <= rear_lb_max_rad
print(f"  前腿在限位内: {'是' if in_limit_f else '✗ 否'} [{math.degrees(front_min_rad):.1f}°, {math.degrees(front_max_rad):.1f}°]")
print(f"  后腿在限位内: {'是' if in_limit_r else '✗ 否'} [{math.degrees(rear_rb_min_rad):.1f}°, {math.degrees(rear_lb_max_rad):.1f}°]")
print(f"  cx_f={cxf:.6f}, cz_f={czf:.6f}")
print(f"  cx_r={cxr:.6f}, cz_r={czr:.6f}")
print(f"  cx_f+cx_r = {cxf+cxr:.6f} (目标 {target_cx_sum:.6f})")
print(f"  cz_r-cz_f = {czr-czf:.6f} (目标 {target_cz_diff:.6f})")
print()

# ---- Print limited best ----
if best_limited:
    tf, tr, cxf, czf, cxr, czr, ex, ez = best_limited
    print("【机械限位内的最优解】")
    print(f"  θ_front = {tf:.1f}° ({math.radians(tf):.6f} rad)")
    print(f"  θ_rear  = {tr:.1f}° ({math.radians(tr):.6f} rad)")
    print(f"  cx_f={cxf:.6f}, cz_f={czf:.6f}")
    print(f"  cx_r={cxr:.6f}, cz_r={czr:.6f}")
    print(f"  cx_f+cx_r = {cxf+cxr:.6f} (目标 {target_cx_sum:.6f})")
    print(f"  cz_r-cz_f = {czr-czf:.6f} (目标 {target_cz_diff:.6f})")
else:
    print("【机械限位内无解】")

print()
print("=" * 65)
print("【末端位置验证 — 使用全局最优解】")
print("=" * 65)

tf, tr, cxf, czf, cxr, czr, ex, ez = best_global
legs = [
    ("LF(前左)", cxf, czf, ox_f,  0.17475, oz_f),
    ("RF(前右)", cxf, czf, ox_f, -0.17475, oz_f),
    ("RB(后右)", cxr, czr, ox_r, -0.16475, oz_r),
    ("LB(后左)", cxr, czr, ox_r,  0.16475, oz_r),
]
for name, cx, cz, ox, oy, oz in legs:
    x = ox + cx
    y = oy
    z = oz + cz - wheel_r
    print(f"  {name}: x={x:+.6f} m, y={y:+.6f} m, z={z:+.6f} m")

xc = sum(ox + cx for _, cx, cz, ox, oy, oz in legs) / 4
print(f"\n  X_center = {xc:+.6f} m")
print(f"  Y_center = 0.000000 m (左右完全对称)")

if best_limited:
    print()
    print("--- 限位内最优解末端位置 ---")
    tf, tr, cxf, czf, cxr, czr, ex, ez = best_limited
    legs2 = [
        ("LF(前左)", cxf, czf, ox_f,  0.17475, oz_f),
        ("RF(前右)", cxf, czf, ox_f, -0.17475, oz_f),
        ("RB(后右)", cxr, czr, ox_r, -0.16475, oz_r),
        ("LB(后左)", cxr, czr, ox_r,  0.16475, oz_r),
    ]
    for name, cx, cz, ox, oy, oz in legs2:
        x = ox + cx
        z = oz + cz - wheel_r
        print(f"  {name}: x={x:+.6f} m, z={z:+.6f} m")
    xc2 = sum(ox + cx for _, cx, cz, ox, oy, oz in legs2) / 4
    print(f"  X_center = {xc2:+.6f} m")
