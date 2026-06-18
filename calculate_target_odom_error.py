#!/usr/bin/env python3
"""比较下发的导航目标 (Target) 与机器人自认为到达后的里程计位姿 (Odom/actual)。

数据来源: run_nav_roundtrip_capture.py 生成的 nav_roundtrip_captures/<round>/metadata.json
仅评估 A 点 (point_name == "A")。
"""

import argparse
import json
import math
import os
import statistics


def angle_diff_deg(actual: float, target: float) -> float:
    return (actual - target + 180.0) % 360.0 - 180.0


def collect_records(captures_dir: str, point: str):
    records = []
    for name in sorted(os.listdir(captures_dir)):
        sub = os.path.join(captures_dir, name)
        meta_path = os.path.join(sub, "metadata.json")
        if not os.path.isfile(meta_path):
            continue

        with open(meta_path, "r", encoding="utf-8") as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                print(f"[警告] {name}/metadata.json 不是合法 JSON，跳过。")
                continue

        nav = data.get("navigation", {})
        if point and nav.get("point_name") != point:
            continue

        target = nav.get("target", {})
        # 优先取 pose.translation (未截断的真实里程计)，fallback 到 summary.actual
        pose = data.get("pose", {})
        translation = pose.get("translation") or {}
        rpy = pose.get("rotation_rpy_degree") or {}

        actual_x = translation.get("x")
        actual_y = translation.get("y")
        actual_yaw = rpy.get("yaw")
        if actual_x is None or actual_y is None or actual_yaw is None:
            actual = data.get("summary", {}).get("actual", {})
            actual_x = actual.get("x")
            actual_y = actual.get("y")
            actual_yaw = actual.get("yaw_degree")
        if actual_x is None or actual_y is None or actual_yaw is None:
            print(f"[警告] {name} 缺少里程计位姿信息，跳过。")
            continue

        tx = float(target.get("x", 0.0))
        ty = float(target.get("y", 0.0))
        tyaw = float(target.get("yaw_degree", 0.0))

        dx = float(actual_x) - tx
        dy = float(actual_y) - ty
        distance = math.hypot(dx, dy)
        dyaw = angle_diff_deg(float(actual_yaw), tyaw)

        records.append({
            "round": name,
            "round_index": nav.get("round_index"),
            "target_x": tx, "target_y": ty, "target_yaw": tyaw,
            "actual_x": float(actual_x), "actual_y": float(actual_y),
            "actual_yaw": float(actual_yaw),
            "dx": dx, "dy": dy,
            "distance": distance,
            "dyaw": dyaw,
        })

    return records


def fmt_stats(values, unit, fmt="{:7.2f}"):
    if not values:
        return f"  无数据"
    avg = statistics.mean(values)
    std = statistics.pstdev(values) if len(values) > 1 else 0.0
    return (f"  平均 {fmt.format(avg)} {unit}   "
            f"最大 {fmt.format(max(values))} {unit}   "
            f"最小 {fmt.format(min(values))} {unit}   "
            f"σ {fmt.format(std)} {unit}")


def main():
    parser = argparse.ArgumentParser(
        description="计算下发目标 (Target) 与里程计实际位姿 (Odom) 的差值")
    parser.add_argument("--captures_dir", type=str,
                        default="nav_roundtrip_captures",
                        help="录制根目录 (run_nav_roundtrip_capture.py 的 --output-root)")
    parser.add_argument("--point", type=str, default="A",
                        help="只评估该点位 (默认 A)")
    parser.add_argument("--target_x", type=float, default=None,
                        help="可选：固定目标 x，用于过滤同点位的不同 round 目标")
    parser.add_argument("--target_y", type=float, default=None,
                        help="可选：固定目标 y")
    parser.add_argument("--target_tol", type=float, default=0.05,
                        help="--target_x/--target_y 过滤的容差 (m)，默认 0.05")
    args = parser.parse_args()

    captures_dir = os.path.abspath(args.captures_dir)
    if not os.path.isdir(captures_dir):
        print(f"[错误] 录制目录不存在: {captures_dir}")
        return

    records = collect_records(captures_dir, args.point)
    if not records:
        print(f"[提示] 在 {captures_dir} 中找不到 point_name == {args.point} 的记录。")
        return

    if args.target_x is not None and args.target_y is not None:
        before = len(records)
        records = [r for r in records
                   if abs(r["target_x"] - args.target_x) <= args.target_tol
                   and abs(r["target_y"] - args.target_y) <= args.target_tol]
        print(f"按目标 ({args.target_x}, {args.target_y}) ±{args.target_tol} m 过滤: "
              f"{before} -> {len(records)}")

    if not records:
        print("[提示] 过滤后无数据。")
        return

    # 检查目标是否一致；不一致就分组列出
    targets = sorted({(round(r["target_x"], 3), round(r["target_y"], 3),
                       round(r["target_yaw"], 3)) for r in records})
    if len(targets) > 1:
        print("[注意] 检测到多个不同的下发目标，建议用 --target_x/--target_y 过滤同一目标:")
        for tx, ty, tyaw in targets:
            n = sum(1 for r in records
                    if (round(r["target_x"], 3), round(r["target_y"], 3),
                        round(r["target_yaw"], 3)) == (tx, ty, tyaw))
            print(f"   target=({tx}, {ty}, yaw={tyaw}°)  共 {n} 次")
        print()

    print(f"{'round':50s}  {'dx(mm)':>8s} {'dy(mm)':>8s} {'dist(mm)':>9s} {'dyaw(°)':>8s}")
    for r in records:
        print(f"{r['round']:50s}  "
              f"{r['dx']*1000:8.2f} {r['dy']*1000:8.2f} "
              f"{r['distance']*1000:9.2f} {r['dyaw']:8.3f}")

    dxs_mm = [r["dx"] * 1000 for r in records]
    dys_mm = [r["dy"] * 1000 for r in records]
    dists_mm = [r["distance"] * 1000 for r in records]
    abs_dxs_mm = [abs(v) for v in dxs_mm]
    abs_dys_mm = [abs(v) for v in dys_mm]
    abs_dyaw = [abs(r["dyaw"]) for r in records]

    print("\n" + "=" * 72)
    print(" Target vs Odom 误差统计 (A 点) ".center(72))
    print("=" * 72)
    print(f" 有效样本数: {len(records)}")
    print("\n【平面距离误差 √(dx²+dy²)】")
    print(fmt_stats(dists_mm, "mm"))
    print("\n【X 方向偏差 (dx = actual_x - target_x，含正负)】")
    print(fmt_stats(dxs_mm, "mm"))
    print("\n【Y 方向偏差 (dy = actual_y - target_y，含正负)】")
    print(fmt_stats(dys_mm, "mm"))
    print("\n【|X| / |Y| 绝对偏差】")
    print(f"  |X| 平均 {statistics.mean(abs_dxs_mm):7.2f} mm   "
          f"|Y| 平均 {statistics.mean(abs_dys_mm):7.2f} mm")
    print("\n【Yaw 偏差 (actual_yaw - target_yaw，已折叠到 [-180, 180]°)】")
    print(fmt_stats([r["dyaw"] for r in records], "°", "{:7.3f}"))
    print(f"  |Yaw| 平均 {statistics.mean(abs_dyaw):7.3f} °   "
          f"最大 {max(abs_dyaw):7.3f} °")
    print("=" * 72)


if __name__ == "__main__":
    main()
