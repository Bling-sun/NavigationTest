import cv2
import numpy as np
import json
import os
import argparse
import math

# ==================== 配置区域 ====================

# 1. 相机内参矩阵 (fx, fy, cx, cy)
# 基于传入的 K 矩阵参数配置
CAMERA_MATRIX = np.array([
    [606.020874023438, 0.0, 327.752685546875],
    [0.0, 605.950622558594, 245.438659667969],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

# 2. 相机畸变系数 (k1, k2, p1, p2, k3)
# 根据传入的 D 矩阵，当前全为 0 (模型: Inverse Brown Conrady)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)

# 3. 标定板类型
BOARD_TYPE = "aruco"  # 支持 "aruco" 或 "checkerboard"

# --- ArUco / AprilTag 码配置 ---
MARKER_SIZE = 0.12  # ⚠️ 真实边长 (米)。如果打印的码不是 12cm，请务必修改这里！
ARUCO_DICT_ID = cv2.aruco.DICT_APRILTAG_36h11

# --- 棋盘格配置 (当前使用 AprilTag，此处不生效，保留默认即可) ---
CHESSBOARD_SIZE = (9, 6) # 内角点数 (列, 行)
SQUARE_SIZE = 0.02 # 每个格子的边长 (米)

# ======================================================

def detect_pose_from_image(image_path):
    """
    从图像中检测标定板，并使用 solvePnP 求解相机位姿 (rvec, tvec)
    """
    img = cv2.imread(image_path)
    if img is None:
        return None, None
        
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    if BOARD_TYPE == "aruco":
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        aruco_params = cv2.aruco.DetectorParameters()
        
        # 兼容 OpenCV 4.7+ 和更旧版本
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            corners, ids, _ = detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            
        if ids is not None and len(ids) > 0:
            # 假设使用画面中检测到的第一个码
            obj_points = np.array([
                [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
                [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
                [-MARKER_SIZE/2, -MARKER_SIZE/2, 0]
            ], dtype=np.float32)
            success, rvec, tvec = cv2.solvePnP(obj_points, corners[0][0], CAMERA_MATRIX, DIST_COEFFS)
            if success:
                return rvec, tvec
                
    elif BOARD_TYPE == "checkerboard":
        found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
        if found:
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                             (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            obj_points = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
            obj_points[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE
            success, rvec, tvec = cv2.solvePnP(obj_points, corners, CAMERA_MATRIX, DIST_COEFFS)
            if success:
                return rvec, tvec

    return None, None

def calculate_errors(rvec_ref, tvec_ref, rvec_cur, tvec_cur):
    """计算当前图像相对于参考图像的平移和旋转误差"""
    R_ref, _ = cv2.Rodrigues(rvec_ref)
    R_cur, _ = cv2.Rodrigues(rvec_cur)
    
    # 我们解算出来的是 T_board_camera (相机系到标定板系)。
    # 要求相机在标定板坐标系下的坐标：P_cam = -R^T * t
    pos_ref = -R_ref.T @ tvec_ref
    pos_cur = -R_cur.T @ tvec_cur
    
    # 1. 整体平移误差 (欧氏距离)
    translation_error = np.linalg.norm(pos_cur - pos_ref)
    
    # 2. X/Y/Z 各轴的偏移量
    dx = float(abs(pos_cur[0][0] - pos_ref[0][0]))
    dy = float(abs(pos_cur[1][0] - pos_ref[1][0]))
    dz = float(abs(pos_cur[2][0] - pos_ref[2][0]))
    
    # 3. 旋转误差: 计算两个相机系之间的相对旋转角 ( R_diff = R_cur * R_ref^T )
    R_diff = R_cur @ R_ref.T
    angle = np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0))
    rotation_error_deg = np.degrees(angle)
    
    return float(translation_error), dx, dy, dz, float(rotation_error_deg)

def main():
    parser = argparse.ArgumentParser(description="机器人重复定位精度视觉计算工具")
    parser.add_argument("--captures_dir", type=str,
                        default="nav_roundtrip_captures",
                        help="录制根目录 (run_nav_roundtrip_capture.py 的 --output-root)")
    parser.add_argument("--ref", type=str,
                        default="round01_A_x0.56_y1.59_yaw14.15",
                        help="参考位置子目录名 (默认: round01_A_x0.56_y1.59_yaw14.15)")
    parser.add_argument("--point", type=str, default="A",
                        help="只评估该点位的 round (默认 A，仅 A 点贴有标定板)")
    args = parser.parse_args()

    captures_root = os.path.abspath(args.captures_dir)
    if not os.path.isdir(captures_root):
        print(f"[错误] 录制目录不存在: {captures_root}")
        return

    ref_dir = os.path.join(captures_root, args.ref)
    ref_image = os.path.join(ref_dir, "image.jpg")
    print(f"正在提取标准参考位置 ({ref_image}) 的位姿...")
    rvec_ref, tvec_ref = detect_pose_from_image(ref_image)
    if rvec_ref is None:
        print(f"[错误] 无法在 {ref_image} 中检测到标定板！请检查图像路径或配置。")
        return

    # 收集所有候选 round 子目录 (含 metadata.json + image.jpg，且点位匹配)
    round_dirs = []
    for name in sorted(os.listdir(captures_root)):
        sub = os.path.join(captures_root, name)
        if not os.path.isdir(sub):
            continue
        meta_path = os.path.join(sub, "metadata.json")
        img_path = os.path.join(sub, "image.jpg")
        if not (os.path.isfile(meta_path) and os.path.isfile(img_path)):
            continue
        round_dirs.append((name, sub, meta_path, img_path))

    print(f"开始处理录制目录中的记录，共找到 {len(round_dirs)} 个子目录...\n")
    results = []
    for name, sub, meta_path, img_path in round_dirs:
        with open(meta_path, 'r', encoding='utf-8') as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                print(f"[警告] {name}/metadata.json 不是合法 JSON，跳过。")
                continue

        nav = data.get("navigation", {})
        if args.point and nav.get("point_name") != args.point:
            continue
        if name == args.ref:
            continue

        rvec_cur, tvec_cur = detect_pose_from_image(img_path)
        if rvec_cur is None:
            print(f"[警告] {name} 未检测到标定板，已跳过。")
            continue

        t_err, dx, dy, dz, r_err = calculate_errors(rvec_ref, tvec_ref, rvec_cur, tvec_cur)

        summary = data.get("summary", {})
        actual = summary.get("actual", {})

        results.append({
            "round": name,
            "round_index": nav.get("round_index"),
            "t_err": t_err, "dx": dx, "dy": dy, "dz": dz,
            "r_err": r_err,
            "robot_x": actual.get("x", 0),
            "robot_y": actual.get("y", 0),
            "robot_yaw": actual.get("yaw_degree", 0),
        })
        print(f"[{name}] 物理平移误差: {t_err*1000:6.2f} mm  旋转: {r_err:5.2f}°  "
              f"(机器人记录: x={actual.get('x')}, y={actual.get('y')}, yaw={actual.get('yaw_degree')})")

    if not results:
        print("\n[提示] 没有成功计算出任何有效结果，请确认图片是否包含完整标定板。")
        return

    # 统计数据
    t_errs = [r["t_err"] for r in results]
    r_errs = [r["r_err"] for r in results]
    dxs = [r["dx"] for r in results]
    dys = [r["dy"] for r in results]
    dzs = [r["dz"] for r in results]

    print("\n" + "="*50)
    print(" 机器人重复定位精度统计报告 ".center(40))
    print("="*50)
    print(f" 有效测试次数 : {len(results)}")
    print(f"\n【平移误差 (3D物理欧氏距离)】")
    print(f"  平均误差 : {np.mean(t_errs)*1000:6.2f} mm")
    print(f"  最大误差 : {np.max(t_errs)*1000:6.2f} mm")
    print(f"  最小误差 : {np.min(t_errs)*1000:6.2f} mm")
    print(f"  标准差(1σ): {np.std(t_errs)*1000:6.2f} mm")
    
    print(f"\n【各轴绝对误差 (基于标定板坐标系)】")
    print(f"  X轴 (左右) 平均 : {np.mean(dxs)*1000:6.2f} mm")
    print(f"  Y轴 (上下) 平均 : {np.mean(dys)*1000:6.2f} mm")
    print(f"  Z轴 (前后) 平均 : {np.mean(dzs)*1000:6.2f} mm")
    
    print(f"\n【旋转误差 (相对角度差异)】")
    print(f"  平均角度误差 : {np.mean(r_errs):6.3f} 度")
    print(f"  最大角度误差 : {np.max(r_errs):6.3f} 度")
    print(f"  最小角度误差 : {np.min(r_errs):6.3f} 度")
    print(f"  标准差 (1σ)  : {np.std(r_errs):6.3f} 度")
    print("="*50)

if __name__ == "__main__":
    main()
