"""LiDAR 点云转换、RANSAC 拟合和车厢角点检测。"""

import math

import numpy as np

from .config import CONFIG
from .config import *


def normalize_angle(theta):
    """把角度归一化到 [-pi, pi) 区间。"""
    while theta > math.pi:
        theta -= 2 * math.pi
    while theta < -math.pi:
        theta += 2 * math.pi
    return theta

def angle_to_deg(theta):
    """角度弧度 -> 度，统一转换入口。"""
    return math.degrees(normalize_angle(theta))

def extract_radar_corner_points(corners):
    """从 corners 字典中提取雷达坐标系下的四个角点，顺序固定为左上、右上、右下、左下。"""
    required_order = ["left_upper", "right_upper", "right_lower", "left_lower"]
    points = []

    for name in required_order:
        p = corners.get(name)
        if p is None:
            raise ValueError(f"corners 中缺少 {name}")
        points.append(np.asarray(p, dtype=float))

    return np.asarray(points, dtype=float)

def scan_to_xy(data):
    """
    将 LiDAR 数据转换成雷达坐标系 XY

    x = r*cos(theta)
    y = r*sin(theta)

    注意：
    laserOffset 不参与这里的坐标计算。
    因为这里要求的是“雷达坐标系”，即雷达原点为 (0, 0)。
    """
    scan = data

    points = []

    for i, r in enumerate(scan):

        # 无效距离
        if not np.isfinite(r):
            continue

        r = r / 1000.0  # mm -> m
        
        if r < CONFIG["range_min"] or r > CONFIG["range_max"]:
            continue

        theta = CONFIG["angle_start"] + i * CONFIG["increment"]

        x = r * math.cos(theta)
        y = r * math.sin(theta)

        points.append({
            "index": i,
            "angle": theta,
            "range": r,
            "x": x,
            "y": y
        })

    return points

def scan4_to_xy(data):
    scan = data
    points = []
    for i, r in enumerate(scan):
        # 无效距离
        if not np.isfinite(r):
            continue
        r = r / 1000.0  # mm -> m
        if r < CONFIG["range_min4"] or r > CONFIG["range_max4"]:
            continue

        theta = CONFIG["angle_start4"] + i * CONFIG["increment4"]

        x = r * math.cos(theta)
        y = r * math.sin(theta)

        points.append({
            "index": i,
            "angle": theta,
            "range": r,
            "x": x,
            "y": y
        })
    return points

# ============================================================
#    Ax + By + C = 0
#    并且 A^2 + B^2 = 1
# ============================================================
def ransac_line(points,
                threshold=CONFIG["RANSAC_THRESHOLD"],
                iterations=CONFIG["RANSAC_ITERATIONS"],
                min_inliers=CONFIG["MIN_INLIERS"]):
    """
    对二维点进行RANSAC直线拟合。

    参数：
        points       : [(x,y), ...]
        threshold    : 点到直线的最大距离，单位 m
        iterations   : RANSAC迭代次数
        min_inliers  : 最少内点数

    返回：
        A, B, C
        inlier_indices
    """

    if len(points) < 2:
        return None, []

    pts = np.asarray(points, dtype=float)

    best_inliers = []

    n = len(pts)

    for _ in range(iterations):

        # 随机选择两个点
        idx = np.random.choice(n, 2, replace=False)

        p1 = pts[idx[0]]
        p2 = pts[idx[1]]

        x1, y1 = p1
        x2, y2 = p2

        dx = x2 - x1
        dy = y2 - y1

        # 两点不能太近
        if dx * dx + dy * dy < 1e-8:
            continue

        # 两点式直线
        A = y1 - y2
        B = x2 - x1
        C = x1 * y2 - x2 * y1

        norm = math.sqrt(A * A + B * B)

        A /= norm
        B /= norm
        C /= norm

        # 点到直线距离
        distances = np.abs(A * pts[:, 0] +
                           B * pts[:, 1] +
                           C)

        inliers = np.where(distances < threshold)[0]

        if len(inliers) > len(best_inliers):
            best_inliers = inliers

    if len(best_inliers) < min_inliers:
        return None, []

    # ========================================================
    # 用RANSAC得到的内点再做一次最小二乘精确拟合
    # ========================================================
    inlier_pts = pts[best_inliers]

    A_mat = np.column_stack(
        (inlier_pts[:, 0],
         inlier_pts[:, 1],
         np.ones(len(inlier_pts)))
    )

    # 求：
    # A*x + B*y + C = 0
    #
    # 即求 A_mat * [A,B,C] = 0
    _, _, vh = np.linalg.svd(A_mat)

    line = vh[-1]

    A, B, C = line

    norm = math.sqrt(A * A + B * B)

    A /= norm
    B /= norm
    C /= norm

    return (A, B, C), best_inliers.tolist()

def ransac_line_with_angle(points,
                           expected_angle,
                           angle_threshold=math.radians(15),
                           threshold=0.03,
                           iterations=1000,
                           min_inliers=20):

    if len(points) < 2:
        return None, []

    pts = np.asarray(points, dtype=float)

    best_line = None
    best_inliers = []

    n = len(pts)

    for _ in range(iterations):

        idx = np.random.choice(n, 2, replace=False)

        p1 = pts[idx[0]]
        p2 = pts[idx[1]]

        x1, y1 = p1
        x2, y2 = p2

        dx = x2 - x1
        dy = y2 - y1

        if dx * dx + dy * dy < 1e-8:
            continue

        # Ax + By + C = 0
        A = y1 - y2
        B = x2 - x1
        C = x1 * y2 - x2 * y1

        norm = math.sqrt(A * A + B * B)

        A /= norm
        B /= norm
        C /= norm

        # --------------------------------------------
        # 当前候选直线的方向角
        # --------------------------------------------
        line_angle = math.atan2(-A, B)

        # 与期望方向的夹角
        diff = angle_difference(
            line_angle,
            expected_angle
        )

        if diff > angle_threshold:
            continue

        # --------------------------------------------
        # 计算内点
        # --------------------------------------------
        distances = np.abs(
            A * pts[:, 0] +
            B * pts[:, 1] +
            C
        )

        inliers = np.where(
            distances < threshold
        )[0]

        if len(inliers) > len(best_inliers):

            best_inliers = inliers
            best_line = (A, B, C)

    if best_line is None:
        return None, []

    if len(best_inliers) < min_inliers:
        return None, []

    # --------------------------------------------
    # 对RANSAC内点重新精确拟合
    # --------------------------------------------
    inlier_pts = pts[best_inliers]

    A_mat = np.column_stack((
        inlier_pts[:, 0],
        inlier_pts[:, 1],
        np.ones(len(inlier_pts))
    ))

    _, _, vh = np.linalg.svd(A_mat)

    A, B, C = vh[-1]

    norm = math.sqrt(A * A + B * B)

    A /= norm
    B /= norm
    C /= norm

    return (A, B, C), best_inliers.tolist()

# ============================================================
# 点到直线距离
# ============================================================
def point_line_distance(x, y, line):
    """
    line = (A,B,C)

    Ax + By + C = 0
    """

    A, B, C = line

    return abs(A * x + B * y + C) / math.sqrt(A * A + B * B)

# ============================================================
# 两条直线求交点
# ============================================================
def line_intersection(line1, line2):
    """
    两条直线：
        A1*x + B1*y + C1 = 0
        A2*x + B2*y + C2 = 0
    返回：
        (x,y)
    """

    A1, B1, C1 = line1
    A2, B2, C2 = line2

    D = A1 * B2 - A2 * B1

    if abs(D) < 1e-10:
        return None

    x = (B1 * C2 - B2 * C1) / D
    y = (C1 * A2 - C2 * A1) / D

    return np.array([x, y])

# ============================================================
# 计算直线方向角
# ============================================================
def line_angle(line):
    """
    返回直线方向角。

    直线 Ax + By + C = 0
    方向向量可以取：
        (B, -A)
    """

    A, B, C = line

    dx = B
    dy = -A

    return math.atan2(dy, dx)

def point_from_line_distances(carriage_yaw, dist_to_right,
                             dist_to_upper,
                             origin=(0.0, 0.0)):
    """
    根据车厢偏航角、上下边线以及两条垂直距离，计算目标点在地图坐标系的 x, y，
    并返回该点的方向角。该点的方向与车厢偏航角一致。

    参数：
        carriage_yaw    : 车厢偏航角（弧度）
        dist_to_right   : 点到右端面的垂直距离（m）
        dist_to_upper   : 点到上边线的垂直距离（m）
        origin          : 参考原点，默认 (0, 0)
                          可以理解为右上角或车厢体坐标系原点

    返回：
        (x_map, y_map, yaw)
    """
    yaw = float(carriage_yaw)
    dx = float(dist_to_right)
    dy = float(dist_to_upper)

    # 车体局部坐标系约定：+x沿车头前方，+y沿车体左侧，-y沿车体右侧。
    # dx、dy是局部坐标偏移，不是地图坐标的x、y增量。
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    ox, oy = origin
    x_map = ox + dx * cos_y - dy * sin_y
    y_map = oy + dx * sin_y + dy * cos_y

    # return x_map, y_map, yaw + math.pi  # 方向角加 180°
    return x_map, y_map, yaw

def angle_difference(a, b):
    """
    两个角度之间的最小夹角
    """

    d = a - b

    while d > math.pi:
        d -= 2 * math.pi

    while d < -math.pi:
        d += 2 * math.pi

    return abs(d)

def get_front_points(points, front_angle_deg=60):
    """获取前方 sensor 视角内的点，默认 ±60°。"""
    result = []
    for p in points:
        angle_deg = angle_to_deg(p["angle"])
        if -front_angle_deg <= angle_deg <= front_angle_deg:
            result.append(p)
    return result

def find_left_upper_corner(points,
                           upper_line,
                           threshold=0.05):

    candidate_points = []

    for p in points:
        angle_deg = math.degrees(p["angle"])

        if (CONFIG["LEFT_UPPER_ANGLE_MIN_DEG"] <= angle_deg <= CONFIG["LEFT_UPPER_ANGLE_MAX_DEG"]):
            candidate_points.append(p)

    if len(candidate_points) < 2:
        return None

    # 从 最小到最大 排列
    candidate_points.sort(
        key=lambda p: math.degrees(p["angle"]),
        reverse=False
    )

    for i in range(len(candidate_points) - 1):

        current = candidate_points[i]
        next_point = candidate_points[i + 1]

        current_distance = point_line_distance(
            current["x"],
            current["y"],
            upper_line
        )

        next_distance = point_line_distance(
            next_point["x"],
            next_point["y"],
            upper_line
        )

        if current_distance <= threshold and next_distance > threshold:
            return np.array([
                current["x"],
                current["y"]
            ])

    return None

def find_left_lower_corner(points,
                           lower_line,
                           threshold=0.05):

    candidate_points = []

    for p in points:
        angle_deg = math.degrees(p["angle"])

        if (CONFIG["LEFT_LOWER_ANGLE_MIN_DEG"] <= angle_deg <= CONFIG["LEFT_LOWER_ANGLE_MAX_DEG"]):
            candidate_points.append(p)

    if len(candidate_points) < 2:
        return None

    candidate_points.sort(
        key=lambda p: math.degrees(p["angle"]),
        reverse=True
    )

    for i in range(len(candidate_points) - 1):

        current = candidate_points[i]
        next_point = candidate_points[i + 1]

        current_distance = point_line_distance(
            current["x"],
            current["y"],
            lower_line
        )

        next_distance = point_line_distance(
            next_point["x"],
            next_point["y"],
            lower_line
        )

        if current_distance <= threshold and next_distance > threshold:
            return np.array([
                current["x"],
                current["y"]
            ])

    return None

def get_angle_points(points, angle_min_deg, angle_max_deg):
    """
    根据LiDAR扫描角度提取点
    angle_min_deg ~ angle_max_deg
    """

    angle_min = math.radians(angle_min_deg)
    angle_max = math.radians(angle_max_deg)

    result = []

    for p in points:
        theta = p["angle"]

        if angle_min <= theta <= angle_max:
            result.append(p)

    return result

# ============================================================
# 自动寻找上边和下边
# ============================================================
def find_upper_lower_lines(points,
                           line_threshold=CONFIG["RANSAC_THRESHOLD"],
                           iterations=CONFIG["RANSAC_ITERATIONS"]):
    """使用全局配置提取上边线和下边线。"""
    upper_points = get_angle_points(points,
                                    CONFIG["UPPER_LINE_ANGLE_MIN_DEG"],
                                    CONFIG["UPPER_LINE_ANGLE_MAX_DEG"])
    upper_xy = [(p["x"], p["y"]) for p in upper_points]
    upper_line, _ = ransac_line_with_angle(
        upper_xy,
        expected_angle=math.radians(CONFIG["UPPER_LOWER_ANGLE_THRESHOLD"]),
        angle_threshold=math.radians(CONFIG["LINE_ANGLE_DETECT_THRESHOLD"]),
        threshold=CONFIG["RANSAC_THRESHOLD"],
        iterations=CONFIG["RANSAC_ITERATIONS"],
        min_inliers=CONFIG["MIN_INLIERS"],
    )

    if upper_line is None:
        raise RuntimeError("RANSAC没有找到车厢上边线")

    lower_points = get_angle_points(points,
                                    CONFIG["LOWER_LINE_ANGLE_MIN_DEG"],
                                    CONFIG["LOWER_LINE_ANGLE_MAX_DEG"])
    lower_xy = [(p["x"], p["y"]) for p in lower_points]
    lower_line, _ = ransac_line_with_angle(
        lower_xy,
        expected_angle=math.radians(CONFIG["UPPER_LOWER_ANGLE_THRESHOLD"]),
        angle_threshold=math.radians(CONFIG["LINE_ANGLE_DETECT_THRESHOLD"]),
        threshold=CONFIG["RANSAC_THRESHOLD"],
        iterations=CONFIG["RANSAC_ITERATIONS"],
        min_inliers=CONFIG["MIN_INLIERS"],
    )
    if lower_line is None:
        raise RuntimeError("RANSAC没有找到车厢下边线")

    # print("\n车厢上边线：")
    # print(f"A = {upper_line[0]:.6f}, B = {upper_line[1]:.6f}, C = {upper_line[2]:.6f}")
    # print("上边线RANSAC内点数:", len(upper_inliers))

    # print("\n车厢下边线：")
    # print(f"A = {lower_line[0]:.6f}, B = {lower_line[1]:.6f}, C = {lower_line[2]:.6f}")
    # print("下边线RANSAC内点数:", len(lower_inliers))

    return upper_line, lower_line

# ============================================================
# 主算法
# ============================================================
def detect_carriage_corners(data):
    """检测车厢四个角点，统一读取全局 CONFIG。"""
    points = scan_to_xy(data)
    print("有效LiDAR点数:", len(points))

    upper_line, lower_line = find_upper_lower_lines(
        points,
        line_threshold=CONFIG["RANSAC_THRESHOLD"],
        iterations=CONFIG["RANSAC_ITERATIONS"],
    )

    upper_angle = line_angle(upper_line)
    lower_angle = line_angle(lower_line)
    
    # 直线方向具有 180° 等价性，先对角度加倍后求圆周均值，避免
    # upper_angle/lower_angle 跨越 ±pi 或拟合方向相反时直接平均失真。
    mean_line_angle = 0.5 * math.atan2(
        math.sin(2.0 * upper_angle) + math.sin(2.0 * lower_angle),
        math.cos(2.0 * upper_angle) + math.cos(2.0 * lower_angle),
    )
    expected_angle = normalize_angle(mean_line_angle + math.pi / 2.0)
    # right_expected_angle = normalize_angle(upper_angle + math.pi / 2)
    # if abs(normalize_angle(lower_angle + math.pi / 2 - right_expected_angle)) > math.radians(5):
    #     right_expected_angle = normalize_angle(lower_angle + math.pi / 2)

    front_points = get_angle_points(points, CONFIG["FRONT_POINT_RANGE_MIN"], CONFIG["FRONT_POINT_RANGE_MAX"])
    front_xy = [(p["x"], p["y"]) for p in front_points]

    # print(expected_angle/math.pi*180)
    # print(upper_angle/math.pi*180)
    # print(lower_angle/math.pi*180)
    # print(right_expected_angle/math.pi*180)

    right_line, _ = ransac_line_with_angle(
        front_xy,
        expected_angle=expected_angle,
        angle_threshold=math.radians(CONFIG["LINE_ANGLE_DETECT_THRESHOLD"]),
        threshold=CONFIG["RANSAC_THRESHOLD"],
        iterations=CONFIG["RANSAC_ITERATIONS"],
        min_inliers=CONFIG["MIN_INLIERS"],
    )
    if right_line is None:
        raise RuntimeError("RANSAC没有找到右端面")

    right_upper = line_intersection(right_line, upper_line)
    right_lower = line_intersection(right_line, lower_line)
    left_upper = find_left_upper_corner(points, upper_line, threshold=CONFIG["CORNER_THRESHOLD"])
    left_lower = find_left_lower_corner(points, lower_line, threshold=CONFIG["CORNER_THRESHOLD"])

    # print("\n==============================")
    # print("车厢四个角点")
    # print("==============================")
    # print("左上角:", left_upper)
    # print("右上角:", right_upper)
    # print("右下角:", right_lower)
    # print("左下角:", left_lower)

    corners = {
        "left_upper": left_upper,
        "right_upper": right_upper,
        "right_lower": right_lower,
        "left_lower": left_lower,
    }

    return corners, {
        "right_line": right_line,
        "upper_line": upper_line,
        "lower_line": lower_line,
    }

def plot_corners(data, corners, out_path="carriage_corners.png"):
    """绘制点云和四个角点，并保存为 PNG。"""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print("Matplotlib 未安装，无法绘图：", e)
        return

    points = scan_to_xy(data)
    # points = get_angle_points(points, 120, 180)
    xs = [p["x"] for p in points]
    ys = [p["y"] for p in points]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(xs, ys, s=4, c="gray", alpha=0.7, label="LiDAR points")

    corner_names = [
        ("left_upper", "red"),
        ("right_upper", "green"),
        ("right_lower", "blue"),
        ("left_lower", "purple")
    ]

    for name, color in corner_names:
        p = corners.get(name)
        if p is None:
            continue
        ax.scatter([p[0]], [p[1]], s=80, color=color, label=name)
        ax.text(p[0], p[1], name, color=color, fontsize=10, weight="bold")

    ax.set_title("Carriage corners")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.3)
    ax.axis("equal")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=200)
    print(f"可视化图已保存到: {out_path}")
    plt.close(fig)
