import os
import json
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from shapely.geometry import Point, Polygon as ShapelyPolygon, LineString
from sklearn.linear_model import RANSACRegressor
import sys
import math
import time
import select
import subprocess
import threading
import websocket
import rclpy
from std_msgs.msg import String

import matplotlib.pyplot as plt
from shapely.geometry import LineString as ShapelyLineString
# plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'WenQuanYi Micro Hei']
# plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 让终端输出中文不乱码（Windows 下最稳妥）
try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass

try:
    sys.stderr.reconfigure(encoding='utf-8')
except Exception:
    pass

# 机器人顶点（原始数据 Y=±0.3）
robot_vertices = np.array([
    [0.18, -0.5],   # P1 右前角
    [0.18, 0.5],    # P2 左前角
    [-1.70, 0.5],   # P3 左后角
    [-1.70, -0.5]   # P4 右后角
])

# robot_poly = ShapelyPolygon(robot_vertices)

# AGV左右边线（用于计算距离）
left_edge = LineString([robot_vertices[1], robot_vertices[2]])  # P2-P3 左边
right_edge = LineString([robot_vertices[3], robot_vertices[0]])  # P4-P1 右边

# 将激光点从极坐标转换为笛卡尔坐标，并分类
points = []  # 所有点
left_points = []   # 角度为60°~120° (左侧)
right_points = []  # 角度为-120°~-60° (右侧)
# map_points = []
angles = []
distances = []


# ============================================================
# 全局参数配置（集中修改）
# ============================================================
CONFIG = {
    # 基础检测参数
    "RANSAC_THRESHOLD": 0.01, # 点到直线的最大距离，单位 m
    "CORNER_THRESHOLD": 0.05, # 
    "RANSAC_ITERATIONS": 2000,
    "MIN_INLIERS": 80,

    "RANSAC_THRESHOLD4": 0.02, # 点到直线的最大距离，单位 m
    "CORNER_THRESHOLD4": 0.05, # 
    "RANSAC_ITERATIONS4": 500,
    "MIN_INLIERS4": 80,

    # 右端面 / 前方检测范围
    "RIGHT_SIDE_ANGLE_MIN_DEG": -45.0,
    "RIGHT_SIDE_ANGLE_MAX_DEG": 0.0,
    "RIGHT_SIDE_ANGLE_MIN_DEG_ALT": 0.0,
    "RIGHT_SIDE_ANGLE_MAX_DEG_ALT": 45.0,

    # 上下边线角度范围
    "UPPER_LINE_ANGLE_MIN_DEG": 0.0,
    "UPPER_LINE_ANGLE_MAX_DEG": 45.0,
    "LOWER_LINE_ANGLE_MIN_DEG": -45.0,
    "LOWER_LINE_ANGLE_MAX_DEG": 0.0,

    # 上下边线角度限制
    "UPPER_LOWER_ANGLE_THRESHOLD" : 0,
    "LINE_ANGLE_DETECT_THRESHOLD" : 5,

    # 左角点搜索范围
    "LEFT_UPPER_ANGLE_MIN_DEG": 45.0,
    "LEFT_UPPER_ANGLE_MAX_DEG": 180.0,
    "LEFT_LOWER_ANGLE_MIN_DEG": -180.0,
    "LEFT_LOWER_ANGLE_MAX_DEG": -45.0,

    "FRONT_POINT_RANGE_MIN": -45,
    "FRONT_POINT_RANGE_MAX": 45,

    # 车体转换参数
    "R2C_X": 0.94,
    "R2C_Y": 0.0,

    # 车厢参数
    "CarriageAngle": 0.0,
    "CarriageLength": 0.0,
    "CarriageWidth": 0.0,

    # 雷达参数
    "range_min": 0.0,
    "range_max": 0.0,
    "angle_start": 0.0,
    "increment": 0.0,

    # 雷达4参数
    "range_min4": 0.0,
    "range_max4": 0.0,
    "angle_start4": 0.0,
    "increment4": 0.0,
    "readed_param" : False,

    # 定点偏移参数
    "yaw_offset" : 1.0, # 度
    "x_offset" : 1.0,
    "y_offset" : 0.2,

    # 定点参数
    "first_point_delX" : 0.84, # 0.76 + 0.05 + 0.03
    "first_point_delY" : 0.57, # 0.1 / 2 + 0.05 + 0.02

    # 检测左右距离最小距离阈值
    "dangerous_Distance" : 0.05,

    # 旋转自调
    "go_straight_action" : 0,

    # 紧急停止标志
    "emergency_stop" : 0
}

final_points = []  # 用来存十次的结果
# 地图点位坐标
POINT_13 = (5.6456,1.5382,-3.0992)
POINT_5 = (4.06825,-1.96819,-1.5708)
POINT_4 = (7.6434,1.62252,-3.09902)
POINT_21 = (3.02084,-0.583914,0)

# TABLE路线参数（参考原代码）
ACTION_NONE = 6
ROUTE_STRAIGHT = 0
ROUTE_CURVE = 2
POINT_START = 0
POINT_MIDDLE = 1
POINT_END = 3

# 导航安全参数
TABLE_TIMEOUT = 8.0
ROUTE_TIMEOUT = 360.0
STATUS_INTERVAL = 0.5
FORK_HEIGHT = 0.0  # 默认货叉高度
FORK_UP_HEIGHT = 0.05  # 升叉目标高度
FORK_DOWN_HEIGHT = 0.0  # 降叉目标高度

def handle_command_message(message):
    """处理来自ROS字符串话题的控制消息。"""
    print(f"收到ROS控制消息: {message.data}")

rclpy.init()
ros_node = rclpy.create_node("navigation_control_node")
laser_publisher = ros_node.create_publisher(
    String,
    "/agv/navigation_status",
    10,
)
camera_subscription = ros_node.create_subscription(
    String,
    "/agv/camera_command",
    handle_command_message,
    10,
)

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

    # 约定车体局部坐标系中：
    # x 轴沿车体前向，y 轴沿车体左侧
    # 因此只需把局部偏移量 (dx, dy) 按 yaw 旋转到地图坐标系
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

# 过滤开机的第一帧
def read_laser_from_ws_filter(ws,
                      timeout=5.0):
    """从 websocket 读取当前激光雷达 scan 数据，命令：{"cmd": "getLaser"}"""
    payload = {"cmd": "getLaser"}
    ws.send(json.dumps(payload))
    raw = ws.recv()

    response = json.loads(raw)

def read_laser_from_ws(ws,
                      timeout=5.0):
    """从 websocket 读取当前激光雷达 scan 数据，命令：{"cmd": "getLaser"}"""

    payload = {"cmd": "getLaser"}
    ws.send(json.dumps(payload))
    raw = ws.recv()

    response = json.loads(raw)

    for laser_data in response.get("data", []):
        if laser_data.get("rame_id") == "scan_link":
            scan_array = laser_data.get("scan", [])

            CONFIG["angle_start"] = laser_data["angle_start"]
            CONFIG["increment"] = laser_data["increment"]
            CONFIG["range_min"] = laser_data["range_min"]
            CONFIG["range_max"] = laser_data["range_max"]
        
            print("DEBUG angle_start =", CONFIG["angle_start"])
            print("DEBUG increment =", CONFIG["increment"])
            print("DEBUG range_min =", CONFIG["range_min"])
            print("DEBUG range_max =", CONFIG["range_max"])
            print("DEBUG scan_len =", len(scan_array))

            return scan_array

    raise ValueError(f"无法解析 getLaser 返回值：{response}")

def read_laser4_from_ws_filter(timeout=5.0):
    payload = {"channel":"ros","op":"subscribe_scan","topic":"/scan"}

    ws = websocket.create_connection("ws://192.168.31.7:6060", timeout=5)
    try:
        ws.send(json.dumps(payload))
        raw = ws.recv() # 每次开关拿到的第一次数据是{"channel":"ros","level":"info","message":"Connected to ws_ros_bridge","op":"status"}
        raw = ws.recv() # 这次的数据才是正常数据
    finally:
        ws.close()
    # ws.send(json.dumps(payload))
    # raw = ws.recv()
    response = json.loads(raw)

def read_laser4_from_ws(timeout=5.0):
    payload = {"channel":"ros","op":"subscribe_scan","topic":"/scan"}

    ws = websocket.create_connection("ws://192.168.31.7:6060", timeout=5)
    try:
        ws.send(json.dumps(payload))
        raw = ws.recv() # 每次开关拿到的第一次数据是{"channel":"ros","level":"info","message":"Connected to ws_ros_bridge","op":"status"}
        raw = ws.recv() # 这次的数据才是正常数据
    finally:
        ws.close()

    # ws.send(json.dumps(payload))
    # raw = ws.recv()
    # raw = ws.recv()

    response = json.loads(raw)

    for laser_data in response.get("msg", []):
        if laser_data.get("rame_id") == "/scan4_link":
            scan_array = laser_data.get("scan", [])
            if(CONFIG["readed_param"] == False):
                CONFIG["angle_start4"] = laser_data["angle_start"]
                CONFIG["increment4"] = laser_data["increment"]
                CONFIG["range_min4"] = laser_data["range_min"]
                CONFIG["range_max4"] = laser_data["range_max"]
                CONFIG["readed_param"] = True

                # print("DEBUG angle_start =", CONFIG["angle_start"])
                # print("DEBUG increment =", CONFIG["increment"])
                # print("DEBUG range_min =", CONFIG["range_min"])
                # print("DEBUG range_max =", CONFIG["range_max"])
                # print("DEBUG scan_len =", len(scan_array))

            return scan_array

    raise ValueError(f"无法解析 getLaser4 返回值：{response}")

def read_car_pose_from_ws(ws,
                           timeout=5.0):
    """
    向 ws://192.168.31.7:6060 发送读取命令，获取当前雷达 x, y, yaw。

    发送：
    {
      "cmd": "table",
      "command": "get",
      "reads": [{"address": "100", "len": 12, "type": "float", "cmd": "read"}]
    }

    返回示例：
    {
      "cmd": "table_reads_result",
      "code": 0,
      "data": [{"address": "100", "value": [-0.29573303, 0.39239883, -3.1283371]}]
    }

    其中 value 的前三个值分别为：x, y, yaw (弧度制)
    """

    payload = {
        "cmd": "table",
        "command": "get",
        "reads": [
            {
                "address": "100",
                "len": 12,
                "type": "float",
                "cmd": "read"
            }
        ]
    }
    ws.send(json.dumps(payload))
    raw = ws.recv()

    response = json.loads(raw)
    data_list = response.get("data", [])
    if not data_list:
        raise ValueError(f"读取AGV pose 失败，返回为空：{response}")

    values = data_list[0].get("value")
    if not isinstance(values, (list, tuple)) or len(values) < 3:
        raise ValueError(f"返回的 value 格式不正确：{values}")

    x, y, yaw = [float(v) for v in values[:3]]
    return {
        "x": x,
        "y": y,
        "yaw": yaw,
        "raw": response,
    }

def transform_points(points, add_x=0.0, add_y=0.0, origin=(0.0, 0.0), angle=0.0):
    """
    将坐标系1中的点变换到坐标系2中。
    
    参数：
    points : array_like, shape (N, 2)
        原始点坐标
    add_x : float
        x坐标加上的值（默认0.94）
    add_y : float
        y坐标加上的值（默认0.0）
    origin : tuple
        坐标系1原点在坐标系2中的位置 (Ox, Oy)
    angle : float
        坐标系1相对于坐标系2的旋转角（弧度，逆时针为正）
    
    返回：
    transformed : ndarray, shape (N, 2)
        在坐标系2中的点坐标
    """
    points = np.asarray(points, dtype=float)
    
    # 第一步：x坐标加上 add_x add_y
    shifted = points.copy()
    shifted[:, 0] += add_x
    shifted[:, 1] += add_y
    
    # 第二步：构建旋转矩阵
    cos_a = np.cos(angle)
    sin_a = np.sin(angle)
    R = np.array([
        [cos_a, -sin_a],
        [sin_a,  cos_a]
    ])
    
    # 第三步：变换
    Ox, Oy = origin
    transformed = shifted @ R.T + np.array([Ox, Oy])
    
    return transformed

def clac_Middle_points_with_yaws(final_point, yaw_offset=0.0, y_offset=0.2, x_offset=1.5, CarriageLen=6.0):

    # 解析终点坐标和角度
    final_x, final_y, final_yaw = final_point
    
    # 计算总偏移距离
    total_offset = CarriageLen + x_offset
    
    # 将角度偏移从度转换为弧度
    yaw_offset_rad = yaw_offset * math.pi / 180.0
    
    # 计算旋转矩阵的余弦和正弦值
    cos_yaw = math.cos(final_yaw)
    sin_yaw = math.sin(final_yaw)
    
    # POINT_16: 最大偏移，与final_point角度相同
    # 原始偏移: (total_offset, y_offset)，应用旋转矩阵
    dx_16 = total_offset * cos_yaw - y_offset * sin_yaw
    dy_16 = total_offset * sin_yaw + y_offset * cos_yaw
    POINT_16 = (final_x + dx_16, 
                final_y + dy_16, 
                final_yaw)
    
    # POINT_17: 3/4偏移，角度加yaw_offset
    # 原始偏移: (total_offset * 3/4, y_offset * 3/4)，应用旋转矩阵
    offset_17_x = total_offset * 3.0 / 4.0
    offset_17_y = y_offset * 3.0 / 4.0
    dx_17 = offset_17_x * cos_yaw - offset_17_y * sin_yaw
    dy_17 = offset_17_x * sin_yaw + offset_17_y * cos_yaw
    POINT_17 = (final_x + dx_17, 
                final_y + dy_17, 
                final_yaw + yaw_offset_rad)
    
    # POINT_18: 1/2偏移，与final_point角度相同
    # 原始偏移: (total_offset * 1/4, y_offset * 2/5)，应用旋转矩阵
    # offset_18_x = total_offset * 1.0 / 5.0
    offset_18_x = 1.5
    offset_18_y = y_offset * 2.0 / 5.0
    dx_18 = offset_18_x * cos_yaw - offset_18_y * sin_yaw
    dy_18 = offset_18_x * sin_yaw + offset_18_y * cos_yaw
    POINT_18 = (final_x + dx_18, 
                final_y + dy_18, 
                final_yaw)
    
    # 返回元组，按照POINT_18, POINT_17, POINT_16的顺序
    return POINT_18, POINT_17, POINT_16

def receive_table_write(ws, timeout=TABLE_TIMEOUT):
    """等待TABLE写入结果。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        ws.settimeout(max(0.1, deadline - time.time()))
        try:
            reply = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue

        # print("收到:", json.dumps(reply, ensure_ascii=False))
        if reply.get("cmd") == "table_writes_result":
            if reply.get("code") != 0:
                raise RuntimeError("TABLE写入失败：%s" % reply)
            return
        if reply.get("type") == "table_set_result":
            if reply.get("status") != 0:
                raise RuntimeError("TABLE写入失败：%s" % reply)
            return
    raise TimeoutError("等待TABLE写入响应超时")

def table_set(ws, writes):
    """发送一组TABLE写入。"""
    command = {"cmd": "table", "command": "set", "writes": writes}
    # print("发送:", json.dumps(command, ensure_ascii=False))
    ws.send(json.dumps(command))
    receive_table_write(ws)

def set_fork_height(ws, target_height):
    table_set(
        ws,
        [
            {
                "address": "48",
                "len": 4,
                "type": "float",
                "cmd": "write",
                "data": [target_height],
            }
        ],
    )

def wait_for_fork(ws, seconds, target):
    deadline = time.time() + seconds
    last_remaining = None
    read_request = [
        {"address": "28", "len": 4, "type": "float", "cmd": "read"}
    ]
    # 容忍差值（米）——当读数接近目标高度时认为到位
    tol = 0.01

    while time.time() < deadline:
        remaining = max(0, math.ceil(deadline - time.time()))
        if remaining != last_remaining:
            last_remaining = remaining

        # 先检查安全位
        try:
            status = get_status(ws)
            check_hard_safety(status)
        except Exception:
            # 若获取状态失败，继续尝试读取TABLE以免长时间阻塞
            pass

        # 读取TABLE 0x48，短超时以快速轮询
        try:
            data = None
            ws.send(json.dumps({"cmd": "table", "command": "get", "reads": read_request}))
            # 等待最多1秒的TABLE读取响应（局部超时）
            read_deadline = time.time() + min(1.0, max(0.1, deadline - time.time()))
            while time.time() < read_deadline:
                ws.settimeout(max(0.05, read_deadline - time.time()))
                try:
                    reply = json.loads(ws.recv())
                except websocket.WebSocketTimeoutException:
                    continue
                if reply.get("cmd") == "table_reads_result":
                    if reply.get("code") != 0:
                        raise RuntimeError("TABLE读取失败：%s" % reply)
                    data = reply.get("data")
                    break
                # 其他消息（例如 agvStatus_result/table_set_result 等）忽略并打印
                print("收到消息（非table_reads_result）：%s" % json.dumps(reply, ensure_ascii=False))

            if data:
                # data 结构为 [{"address": "48", "value": [val]}]
                try:
                    val = float(data[0].get("value", [0])[0])
                except Exception:
                    val = None
                print("当前货叉高度：%s m" % ("%.3f" % val if val is not None else "未知"))
                # 判断是否到位
                if val is not None and abs(val - target) <= tol:
                    print("检测到货叉已到位（%.3f m），结束等待。" % val)
                    return
        except Exception as exc:
            print("轮询TABLE读取时发生：%s（继续重试直到超时）" % exc)

        # 休眠一小段时间再继续轮询
        time.sleep(min(STATUS_INTERVAL, max(0.05, deadline - time.time())))

def node_writes(node_id, pose, route_type, point_type, index, fork_height=FORK_HEIGHT):
    """生成一个路线节点。"""
    return [
        {"address": "24c", "len": 32, "type": "int32", "data": [node_id]},
        {
            "address": "260",
            "len": 16,
            "type": "float",
            "data": [pose[0], pose[1], pose[2], fork_height],
        },
        {"address": "257", "len": 1, "type": "int8", "data": [ACTION_NONE]},
        {"address": "256", "len": 1, "type": "int8", "data": [point_type]},
        {"address": "253", "len": 1, "type": "int8", "data": [route_type]},
        {"address": "290", "len": 16, "type": "string", "data": [""]},
        {"address": "251", "len": 1, "type": "int8", "data": [index]},
    ]

def build_route(nodes, fork_height=FORK_HEIGHT):
    """构建多节点路线。"""
    writes = []
    last_index = len(nodes) - 1
    for index, (node_id, pose, route_type) in enumerate(nodes):
        if index == 0:
            point_type = POINT_START
        elif index == last_index:
            point_type = POINT_END
        else:
            point_type = POINT_MIDDLE
        writes += node_writes(
            node_id,
            pose,
            route_type,
            point_type,
            index,
            fork_height,
        )
    return writes

def read_table_u8(ws, address, timeout=1.0):
    """读取TABLE寄存器值，并返回uint8结果。"""
    request = [{"address": address, "len": 1, "type": "uint8", "cmd": "read"}]
    ws.send(json.dumps({"cmd": "table", "command": "get", "reads": request}))
    deadline = time.time() + timeout
    while time.time() < deadline:
        ws.settimeout(max(0.05, deadline - time.time()))
        try:
            reply = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        if reply.get("cmd") == "table_reads_result":
            if reply.get("code") != 0:
                raise RuntimeError("TABLE读取失败：%s" % reply)
            data = reply.get("data", [])
            if not data:
                raise RuntimeError("TABLE读取返回空数据：%s" % reply)
            value = data[0].get("value", [])
            if not value:
                raise RuntimeError("TABLE读取返回空value：%s" % reply)
            return int(value[0])
    raise TimeoutError("等待TABLE 0x%s读取结果超时。" % address)

def write_table_u8(ws, address, value, timeout=1.0):
    request = [{"address": address,"len": 1,"type": "int8","cmd": "write","data": [value]}]
    ws.send(json.dumps({"cmd": "table","command": "set","writes": request}))
    deadline = time.time() + timeout
    while time.time() < deadline:
        ws.settimeout(max(0.05, deadline - time.time())) # 设置ws读取的超时时间
        try:
            reply = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        if reply.get("cmd") == "table_writes_result":
            if reply.get("code") != 0:
                raise RuntimeError("TABLE写入失败：%s" % reply)
            return
    raise TimeoutError("等待TABLE 0x%s读取结果超时。" % address)

def write_table_float(ws, address, value, timeout=1.0):
    request = [{"address": address,"len": 4,"type": "float","cmd": "write","data": [value]}]
    ws.send(json.dumps({"cmd": "table","command": "set","writes": request}))
    deadline = time.time() + timeout
    while time.time() < deadline:
        ws.settimeout(max(0.05, deadline - time.time()))
        try:
            reply = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        if reply.get("cmd") == "table_writes_result":
            if reply.get("code") != 0:
                raise RuntimeError("TABLE写入失败：%s" % reply)
            return
    raise TimeoutError("等待TABLE 0x%s读取结果超时。" % address)

def send_emergency_stop(ws, timeout=5.0):
    """
    发送软急停命令，使AGV紧急停止。
    
    参数:
    ws_url: websocket地址，默认 ws://192.168.31.7:6060
    timeout: 连接和响应超时时间（秒）
    
    返回:
    dict: {
        "success": bool,
        "message": str,
        "raw_response": dict  # 原始响应
    }
    
    发送的JSON命令:
    {
        "cmd": "softScram",
        "data": true
    }
    """
    
    payload = {
        "cmd": "softScram",
        "data": True
    }
    
    # print("发送急停命令:", json.dumps(payload, ensure_ascii=False))
    
    try:
        # 发送急停命令
        ws.send(json.dumps(payload))
        
        # 等待响应
        deadline = time.time() + timeout
        while time.time() < deadline:
            ws.settimeout(max(0.1, deadline - time.time()))
            try:
                reply = json.loads(ws.recv())
            except websocket.WebSocketTimeoutException:
                continue
            
            # print("收到响应:", json.dumps(reply, ensure_ascii=False))
            
            # 检查软急停响应结果
            if reply.get("cmd") in ("softScram_result"):
                if reply.get("code") == 0:
                    print("✅ 急停命令发送成功，AGV应已紧急停止。")
                    return 
                else:
                    error_msg = f"急停命令发送失败，错误码：{reply.get('code')}"
                    print(f"❌ {error_msg}")
                    return 
        # 超时
        error_msg = "等待急停命令响应超时"
        print(f"❌ {error_msg}")
        return 
    except Exception as exc:
        error_msg = f"发送急停命令异常：{exc}"
        print(f"❌ {error_msg}")
        return 

def clear_emergency_stop(ws, timeout=5.0):
    
    payload = {
        "cmd": "softScram",
        "data": False
    }
    
    try:
        # 发送清除急停命令
        ws.send(json.dumps(payload))
        
        # 等待响应
        deadline = time.time() + timeout
        while time.time() < deadline:
            ws.settimeout(max(0.1, deadline - time.time()))
            try:
                reply = json.loads(ws.recv())
            except websocket.WebSocketTimeoutException:
                continue
            
            # 检查软急停响应结果
            if reply.get("cmd") in ("softScram_result"):
                if reply.get("code") == 0:
                    print("✅ 清除急停命令发送成功。")
                    return
                else:
                    error_msg = f"清除急停命令发送失败，错误码：{reply.get('code')}"
                    print(f"❌ {error_msg}")
                    return
        # 超时
        error_msg = "等待清除急停命令响应超时"
        print(f"❌ {error_msg}")
        return
    except Exception as exc:
        error_msg = f"发送清除急停命令异常：{exc}"
        print(f"❌ {error_msg}")
        return

def check_line_intersection_and_distance(line_a, line_b):
    """
    判断两条线段是否相交，如果不相交则计算两条线段之间的最小距离。
    
    参数：
    line_a : Shapely LineString
        线段A
    line_b : tuple (A, B, C)
        RANSAC 拟合的直线 Ax + By + C = 0（无限直线）
    
    返回：
    intersects : bool
        是否相交
    min_dist : float
        最小距离（如果相交则为 0.0）
    intersection_point : tuple 或 None
        交点坐标（如果相交）
    max_dist : float
        AGV线段两个端点到激光直线的最大垂直距离
    """
    
    A, B, C = line_b
    
    # 将无限直线转换为 Shapely LineString
    # 需要找到一个足够长的线段来表示这条直线
    # 获取 line_a 的范围，然后扩展一些
    x_min, y_min, x_max, y_max = line_a.bounds
    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2
    half_extent = max(x_max - x_min, y_max - y_min) * 10 + 10  # 扩展范围
    
    if abs(B) > 1e-10:
        # y = (-A*x - C) / B
        x1 = center_x - half_extent
        x2 = center_x + half_extent
        y1 = (-A * x1 - C) / B
        y2 = (-A * x2 - C) / B
    else:
        # B ≈ 0，直线是垂直的: A*x + C = 0 -> x = -C/A
        x1 = x2 = -C / A
        y1 = center_y - half_extent
        y2 = center_y + half_extent
    
    line_b_shapely = ShapelyLineString([(x1, y1), (x2, y2)])
    endpoint_distances = [
        point_line_distance(x, y, line_b)
        for x, y in (line_a.coords[0], line_a.coords[-1])
    ]
    max_dist = max(endpoint_distances)
    
    # 判断是否相交
    if line_a.intersects(line_b_shapely):
        intersection = line_a.intersection(line_b_shapely)
        if intersection.geom_type == 'Point':
            return True, 0.0, (intersection.x, intersection.y), max_dist
        elif intersection.geom_type == 'MultiPoint':
            # 多个交点（线段重合部分）
            return True, 0.0, (intersection.geoms[0].x, intersection.geoms[0].y), max_dist
        else:
            return True, 0.0, None, max_dist
    
    # 不相交，计算最小距离
    min_dist = line_a.distance(line_b_shapely)
    return False, min_dist, None, max_dist

def check_agv_edges_with_laser_lines(left_edge, right_edge, left_line, right_line):
    """
    检查 AGV 左右边线与激光拟合直线的关系。
    
    参数：
    left_edge : Shapely LineString
        AGV 左边线段
    right_edge : Shapely LineString
        AGV 右边线段
    left_line : tuple (A, B, C) 或 None
        左侧激光拟合直线
    right_line : tuple (A, B, C) 或 None
        右侧激光拟合直线
    
    返回：
    results : dict
        包含相交和距离信息
    """
    results = {
        'left': {
            'exists': left_line is not None,
            'intersects': False,    # 相交
            'min_distance': None,
            'max_distance': None,
            'nearest_corner': None,
            'intersection_point': None
        },
        'right': {
            'exists': right_line is not None,
            'intersects': False,
            'min_distance': None,
            'max_distance': None,
            'nearest_corner': None,
            'intersection_point': None
        }
    }
    
    # 检查左侧
    if left_line is not None:
        intersects, min_dist, inter_pt, max_dist = check_line_intersection_and_distance(
            left_edge, left_line
        )
        results['left']['intersects'] = intersects
        results['left']['min_distance'] = min_dist
        results['left']['max_distance'] = max_dist
        left_endpoint_distances = [
            point_line_distance(x, y, left_line)
            for x, y in (left_edge.coords[0], left_edge.coords[-1])
        ]
        results['left']['nearest_corner'] = (
            'left_front'
            if left_endpoint_distances[0] <= left_endpoint_distances[1]
            else 'left_rear'
        )
        results['left']['intersection_point'] = inter_pt
        
        # if intersects:
        #     print(f"左侧：AGV左边线与激光直线相交，交点: {inter_pt}")
        # else:
        #     print(f"左侧：AGV左边线与激光直线不相交，最小距离: {min_dist:.4f} m")
    
    # 检查右侧
    if right_line is not None:
        intersects, min_dist, inter_pt, max_dist = check_line_intersection_and_distance(
            right_edge, right_line
        )
        results['right']['intersects'] = intersects
        results['right']['min_distance'] = min_dist
        results['right']['max_distance'] = max_dist
        right_endpoint_distances = [
            point_line_distance(x, y, right_line)
            for x, y in (right_edge.coords[0], right_edge.coords[-1])
        ]
        results['right']['nearest_corner'] = (
            'right_rear'
            if right_endpoint_distances[0] <= right_endpoint_distances[1]
            else 'right_front'
        )
        results['right']['intersection_point'] = inter_pt
        
        # if intersects:
        #     print(f"右侧：AGV右边线与激光直线相交，交点: {inter_pt}")
        # else:
        #     print(f"右侧：AGV右边线与激光直线不相交，最小距离: {min_dist:.4f} m")
    
    return results

def calc_LR_Distance(left_flag=False, right_flag=False):
    laser_data = read_laser4_from_ws()

    points = scan4_to_xy(laser_data)
    # plot_points(points, title="激光点图", save_path="laser_points.png")

    left_points = []
    right_points = []
    for item in points:
        angle_deg = math.degrees(item["angle"])
        
        if (45 <= angle_deg <= 115) and left_flag:
            left_points.append([item["x"], item["y"]])
        elif -125 <= angle_deg <= -45 and right_flag:
            right_points.append([item["x"], item["y"]])

    left_line = None
    right_line = None
    if left_flag:
        left_line, left_inliers = ransac_line(left_points, CONFIG["RANSAC_THRESHOLD4"], CONFIG["RANSAC_ITERATIONS4"], CONFIG["MIN_INLIERS4"])
    if right_flag:
        right_line, right_inliers = ransac_line(right_points, CONFIG["RANSAC_THRESHOLD4"], CONFIG["RANSAC_ITERATIONS4"], CONFIG["MIN_INLIERS4"])

    results = check_agv_edges_with_laser_lines(
        left_edge=left_edge,
        right_edge=right_edge,
        left_line=left_line,
        right_line=right_line
    )

    return results

def wait_until_stopped(ws, timeout=ROUTE_TIMEOUT, left_flag=False, right_flag=False):
    """等待AGV结束行驶并停车，使用TABLE 0x211判断到达终点。"""
    deadline = time.time() + timeout
    route_started = False

    while time.time() < deadline:
        # start_time = time.time()
        
        try:
            motion_state = read_table_u8(ws, "211")
        except Exception as exc:
            print("读取0x211失败：%s" % exc)
            motion_state = None

        if motion_state not in (None, 0):
            route_started = True

        if route_started and motion_state == 0:
            print("车辆已到达并停车。")
            return

        results = calc_LR_Distance(left_flag, right_flag)

        if results['right']['exists']:
            if results['right']['intersects']:
                print("右侧：AGV右边线与激光直线相交，交点: {}".format(results['right']['intersection_point']))
                # 发送急停标志0x150 uint8 为 1，AGV紧急停止
                result = send_emergency_stop(ws)
                time.sleep(0.1)
                # 先清除路线任务
                write_table_u8(ws, "1f", 1)
                time.sleep(0.1)
                CONFIG["emergency_stop"] = 1
                return
            else:
                if results['right']['min_distance'] is not None:
                    print("右侧：AGV右边线与激光直线最小距离: {:.4f} m".format(results['right']['min_distance']))
                    if results['right']['min_distance'] < CONFIG["dangerous_Distance"]:
                        print("右侧：AGV右边线与激光直线靠的很近，最小距离: {:.4f} m".format(results['right']['min_distance']))
                        # 发送急停标志0x150 uint8 为 1，AGV紧急停止
                        result = send_emergency_stop(ws)
                        time.sleep(0.1)
                        # 先清除路线任务
                        write_table_u8(ws, "1f", 1)
                        time.sleep(0.1)
                        CONFIG["emergency_stop"] = 1
                        # # 再清除软急停
                        # clear_emergency_stop(ws)
                        # time.sleep(0.1)
                        # # 直线行驶进入终点标志
                        # CONFIG["go_straight_action"] = 1
                        # print("go_straight_action = 1")
                        return
        elif results['left']['exists']:
            if results['left']['intersects']:
                print("左侧：AGV左边线与激光直线相交，交点: {}".format(results['left']['intersection_point']))
                # 发送急停标志0x150 uint8 为 1，AGV紧急停止
                result = send_emergency_stop(ws)
                return
            else:
                if results['left']['min_distance'] is not None:
                    if results['left']['min_distance'] < CONFIG["dangerous_Distance"]:
                        print("左侧：AGV左边线与激光直线靠的很近，最小距离: {:.4f} m".format(results['left']['min_distance']))
                        # 发送急停标志0x150 uint8 为 1，AGV紧急停止
                        result = send_emergency_stop(ws)
                        return
        # end_time = time.time()
        # elapsed_time = end_time - start_time
        # print(f"本次循环耗时: {elapsed_time:.3f} 秒")
    raise TimeoutError("%.0f秒内未停车。" % (timeout))

def wait_until_stopped2(ws, timeout=ROUTE_TIMEOUT, left_flag=False, right_flag=False):
    """等待AGV结束行驶并停车，使用TABLE 0x211判断到达终点。"""
    deadline = time.time() + timeout
    route_started = False

    while time.time() < deadline:
        # start_time = time.time()
        
        try:
            motion_state = read_table_u8(ws, "211")
        except Exception as exc:
            print("读取0x211失败：%s" % exc)
            motion_state = None

        if motion_state not in (None, 0):
            route_started = True

        if route_started and motion_state == 0:
            print("车辆已到达并停车。")
            return

        # end_time = time.time()
        # elapsed_time = end_time - start_time
        # print(f"本次循环耗时: {elapsed_time:.3f} 秒")
    raise TimeoutError("%.0f秒内未停车。" % (timeout))

def read_float_from_table(ws, address, timeout=1.0):
    """读取TABLE中的float值"""
    payload = {
        "cmd": "table",
        "command": "get",
        "reads": [
            {
                "address": address,
                "len": 4,
                "type": "float",
                "cmd": "read"
            }
        ]
    }
    
    ws.send(json.dumps(payload))
    deadline = time.time() + timeout
    
    while time.time() < deadline:
        ws.settimeout(max(0.1, deadline - time.time()))
        try:
            reply = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        
        if reply.get("cmd") == "table_reads_result":
            if reply.get("code") != 0:
                raise RuntimeError(f"TABLE读取失败：{reply}")
            data = reply.get("data", [])
            if not data:
                raise RuntimeError(f"TABLE读取返回空数据：{reply}")
            value = data[0].get("value", [])
            if not value:
                raise RuntimeError(f"TABLE读取返回空value：{reply}")
            return float(value[0])
    
    raise TimeoutError(f"等待TABLE 0x{address}读取结果超时")

def write_float_to_table(ws, address, value, timeout=1.0):
    """写入float值到TABLE"""
    payload = {
        "cmd": "table",
        "command": "set",
        "writes": [
            {
                "address": address,
                "len": 12,
                "type": "float",
                "cmd": "write",
                "data": value
            }
        ]
    }
    
    ws.send(json.dumps(payload))
    deadline = time.time() + timeout
    
    while time.time() < deadline:
        ws.settimeout(max(0.1, deadline - time.time()))
        try:
            reply = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        
        # 检查各种可能的响应格式
        if reply.get("cmd") == "table_writes_result":
            if reply.get("code") != 0:
                raise RuntimeError(f"TABLE写入失败：{reply}")
            return
        if reply.get("type") == "table_set_result":
            if reply.get("status") != 0:
                raise RuntimeError(f"TABLE写入失败：{reply}")
            return
    
    raise TimeoutError(f"等待TABLE 0x{address}写入响应超时")
# 频率建议为10Hz
def remoteControl(ws2, x_vel, W_theta, theta_vel):
    """写入float值到TABLE"""
    payload = {
    "packet": {
        "cmd": "region",
        "region": "cmd_vel",
        "index": 1
    },
    "msg": {
        "xvel": x_vel,
        "wheelTheta": W_theta,
        "thetavel": theta_vel,
       "isRemote":True
    }
    }
    ws2.send(json.dumps(payload))

def stop_rotation(ws, ws2):
        """停止旋转"""
        try:
            send_emergency_stop(ws)
            # 再清除软急停
            time.sleep(0.1)
            clear_emergency_stop(ws)
            time.sleep(0.1)
            # 将速度设为0, 角度回到正常值
            remoteControl(ws2, 0.0, 0.0, 0.3)
            # 短暂延时确保停止
            time.sleep(0.1)  
            # 再发送一次速度为0（确保停止）
            remoteControl(ws2, 0.0, 0.0, 0.3)
            time.sleep(0.1)  
            print("已停止旋转")
        except Exception as exc:
            print(f"停止旋转时出错: {exc}")

def rotate_until_parallel(ws, 
                          timeout=30.0, 
                          speed=0.25,
                          distance_diff_threshold=0.02,
                          max_iterations=100,
                          detection_side='right'):

    ws2 = websocket.create_connection("ws://192.168.31.7:1202", timeout=TABLE_TIMEOUT)
    
    # 1. 首先将舵轮转到最大角度（1.57弧度 ≈ 90度）
    max_steering_angle = 1.57  # 最大舵轮角度（弧度）
    # print(f"将舵轮转到最大角度: {max_steering_angle} rad ({math.degrees(max_steering_angle):.1f}°)")
    
    try:
        # time.sleep(2)
        # 写入舵轮角度
        # remoteControl(ws2, 0.0, max_steering_angle, 0.3)

        # 等待舵轮转动到位
        print("等待舵轮转动到位...")
        while True:
            remoteControl(ws2, 0.0, max_steering_angle, 0.3)
            # 难道这里要等待0.1s左右再读？
            time.sleep(0.1) # 同时0.1s读一次

            current_steering_angle = read_float_from_table(ws, "24")
            if abs(max_steering_angle - current_steering_angle)  < 0.05:
                break
        
        deadline = time.time() + timeout
        best_diff = float('inf')
        current_direction = 1  # 初始方向为正（逆时针）
        detect_direction = False
        curr_time = time.time()
        last_time = None
        while time.time() < deadline:
            # 计算距离
            if detection_side == 'right':
                results = calc_LR_Distance(left_flag=False, right_flag=True)
            else:
                results = calc_LR_Distance(left_flag=True, right_flag=False)
            
            # 获取侧面数据
            side_data = results.get(detection_side, {})
            
            if side_data.get('exists'):
                min_dist = side_data.get('min_distance')
                max_dist = side_data.get('max_distance')
                
                if detect_direction == False:
                    detect_direction = True
                    nearest_corner = side_data.get('nearest_corner')
                    # 根据最近角点判断旋转方向
                    if detection_side == 'right':
                        # 右侧检测
                        if nearest_corner == 'right_front':
                            # 右前角离的近，速度为正（逆时针旋转）
                            current_direction = 1
                            direction_str = "逆时针"
                            reason = "右前角离车厢壁更近"
                        elif nearest_corner == 'right_rear':
                            # 右后角离的近，速度为负（顺时针旋转）
                            current_direction = -1
                            direction_str = "顺时针"
                            reason = "右后角离车厢壁更近"
                        else:
                            direction_str = "未知"
                            reason = "无法判断最近角点"
                    else:
                        # 左侧检测（方向相反）
                        if nearest_corner == 'left_front':
                            # 左前角离的近，速度为负（顺时针旋转）
                            current_direction = -1
                            direction_str = "顺时针"
                            reason = "左前角离车厢壁更近"
                        elif nearest_corner == 'left_rear':
                            # 左后角离的近，速度为正（逆时针旋转）
                            current_direction = 1
                            direction_str = "逆时针"
                            reason = "左后角离车厢壁更近"
                        else:
                            direction_str = "未知"
                            reason = "无法判断最近角点"
                
                if min_dist is not None and max_dist is not None:

                    distance_diff = abs(max_dist - min_dist)

                    # print(distance_diff)
                    
                    # 根据方向设置实际速度
                    actual_speed = speed * current_direction
                    
                    # 检查是否平行（距离差小于阈值）
                    if distance_diff < distance_diff_threshold:

                        remoteControl(ws2, 0.0, max_steering_angle, 0.0)
                        time.sleep(0.1)
                        # 停止旋转
                        stop_rotation(ws, ws2)

                        print(f"\n✅ AGV已与车厢壁平行！")
                        print(f"距离差: {distance_diff:.4f}m")
                        
                        return
                    
                    # 记录最佳结果
                    if distance_diff < best_diff:
                        best_diff = distance_diff
            else:
                actual_speed = speed * current_direction
            
            # print("第一次发送")
            # 持续发送速度（速度需要一直发）
            # lase_time = curr_time
            # curr_time = time.time()
            # print(f"本次发送耗时: {(curr_time-lase_time):.3f} 秒")
            remoteControl(ws2, actual_speed, max_steering_angle, 0) # 循环运行本身需要0.15s左右
        
        # 超时或达到最大迭代次数
        stop_rotation(ws, ws2)
        print(f"\n⚠️ 未能找到完全平行的位置")
        print(f"最佳距离差: {best_diff:.4f}m")
        return 
        
    except Exception as exc:
        print(f"\n❌ 旋转过程出错: {exc}")
        try:
            stop_rotation(ws)
        except:
            pass
        return

def start_route(ws, route_writes):
    """写入并启动路线。"""
    # print("\n准备下发路线。")
    # ws = websocket.create_connection("ws://192.168.31.7:6060", timeout=5)
    table_set(ws, route_writes)
    table_set(
        ws,
        [{"address": "250", "len": 1, "type": "int8", "data": [1]}],
    )
    print("路线已启动。")

def go_straight_to_dest(ws, dest_point, current_point):
    """
    将目标点垂直映射到当前点的yaw方向直线上
    
    参数:
        dest_point: 目标点B，格式为 (x, y, yaw) 或 [x, y, yaw]
        current_point: 当前点A，格式为 (x, y, yaw) 或 [x, y, yaw]
    
    返回:
        projected_point: 投影后的目标点，格式为 (x, y, yaw)
                        其中yaw为当前点A的方向
    """

    # 向量 AB = (x1-x, y1-y)，即从当前点指向目标点的向量
    # 方向向量 d = (cos(yaw), sin(yaw))，即当前点的单位方向向量
    # t = AB · d，表示向量AB在方向d上的投影长度

    # 解包坐标和方向
    x, y, yaw = current_point[0], current_point[1], current_point[2]
    x1, y1, yaw1 = dest_point[0], dest_point[1], dest_point[2]
    
    # 计算投影参数t
    # t是目标点在当前点方向上的投影距离
    t = (x1 - x) * math.cos(yaw) + (y1 - y) * math.sin(yaw)
    
    # 计算投影点坐标
    x_proj = x + t * math.cos(yaw)
    y_proj = y + t * math.sin(yaw)
    
    # 投影后的方向设置为当前点的方向
    yaw_proj = yaw
    
    # 返回投影后的点
    point_20 =  (x_proj, y_proj, yaw_proj)
    point_19 = current_point
    nodes = [
        (19, point_19, ROUTE_STRAIGHT),
        (20, point_20, ROUTE_CURVE),
        ]
    # 构建路线
    route_writes = build_route(nodes)
    start_route(ws, route_writes)
    # 等待到达终点
    wait_until_stopped2(ws, 3600, left_flag=False, right_flag=True)

def drive_to_point_with_right_wall(ws, goal_x, target_distance=0.08,
                                   timeout=120.0, x_tolerance=0.1,
                                   distance_tolerance=0.015,
                                   steering_speed=1.0):
    """低速闭环行驶到目标 x，并将 AGV 右侧与墙的距离保持在目标值。"""
    ws2 = websocket.create_connection("ws://192.168.31.7:1202", timeout=TABLE_TIMEOUT)
    deadline = time.monotonic() + timeout
    stable_count = 0

    # 正舵角为逆时针，负舵角为顺时针；右侧距离过大时向右修正。
    steering_gain = 4.0
    max_steering_angle = 0.157
    max_forward_speed = 0.2
    min_crawl_speed = 0.05
    kp = 0.3
    pass_reached = False
    x_reached = False
    try:
        while time.monotonic() < deadline:
            pose = read_car_pose_from_ws(ws)
            x_error = float(goal_x) - pose["x"]
            # print(x_error)
            results = calc_LR_Distance(left_flag=False, right_flag=True)
            right_data = results["right"]
            measured_distance = right_data.get("min_distance")

            distance_error = float(measured_distance) - target_distance
            if (abs(x_error) <= x_tolerance):
                x_reached = True
                print("x_reached = True")
            distance_reached = abs(distance_error) <= distance_tolerance

            # if x_reached and distance_reached:
            if x_reached or pass_reached:
                # print("pass_reached = True")
                pass_reached = True
                if read_table_u8(ws, "55") == 1:
                    print("已碰到碰撞开关")
                    remoteControl(ws2, 0.0, 0.0, steering_speed)
                    return True
                else:
                    remoteControl(ws2, -0.2, 0.0, steering_speed)
                    continue
                # stable_count += 1
                # remoteControl(ws2, 0.0, 0.0, steering_speed)
                # if stable_count >= 3:
                #     print("已到达目标 x 位置，且右侧距离墙壁 %.3f m。" % measured_distance)
                #     return True
                # time.sleep(0.05)
                # continue

            if not right_data.get("exists") or measured_distance is None:
                remoteControl(ws2, 0.0, 0.0, steering_speed)
                stable_count = 0
                time.sleep(0.05)
                continue

            stable_count = 0
            # xvel 是车体前进速度，使用当前航向将地图 x 误差投影到车体前向。
            forward_error = x_error * math.cos(pose["yaw"])
            forward_speed = max(-max_forward_speed,
                                min(max_forward_speed, kp * forward_error))
            if abs(forward_speed) < min_crawl_speed and (not x_reached or not distance_reached):
                crawl_direction = forward_error if abs(forward_error) > 1e-6 else 1.0
                # steering_velocity = math.copysign(steering_speed, crawl_direction)
                forward_speed = math.copysign(min_crawl_speed, crawl_direction)
            
            if (abs(measured_distance) < target_distance):
                if ((right_data.get("max_distance") - right_data.get("min_distance")) < 0.015):
                    distance_error = 0.0
                else:
                    if (right_data.get("nearest_corner") == "right_front"):
                        distance_error = -distance_error

            steering_angle = max(-max_steering_angle,
                                 min(max_steering_angle,
                                     -steering_gain * distance_error))

            remoteControl(ws2, forward_speed, steering_angle, steering_speed)

        raise TimeoutError("闭环行驶在 %.1f 秒内未达到目标。" % timeout)
    finally:
        remoteControl(ws2, 0.0, 0.0, 0.0)
        ws2.close()

def drive_to_point_with_right_wall_return(ws, goal_x, target_distance=0.08,
                                   timeout=120.0, x_tolerance=0.2,
                                   distance_tolerance=0.015,
                                   steering_speed=1.0):
    """低速闭环行驶到目标 x，并将 AGV 右侧与墙的距离保持在目标值。"""
    ws2 = websocket.create_connection("ws://192.168.31.7:1202", timeout=TABLE_TIMEOUT)
    deadline = time.monotonic() + timeout
    stable_count = 0

    # 正舵角为逆时针，负舵角为顺时针；右侧距离过大时向右修正。
    steering_gain = 4.0
    max_steering_angle = 0.157
    max_forward_speed = 0.5
    min_crawl_speed = 0.05
    kp = 0.3

    try:
        while time.monotonic() < deadline:
            pose = read_car_pose_from_ws(ws)
            x_error = float(goal_x) - pose["x"]
            print(x_error)
            results = calc_LR_Distance(left_flag=False, right_flag=True)
            right_data = results["right"]
            measured_distance = right_data.get("min_distance")
            adjust_distance = right_data.get("max_distance")

            if not right_data.get("exists") or measured_distance is None:
                remoteControl(ws2, 0.0, 0.0, steering_speed)
                stable_count = 0
                time.sleep(0.05)
                continue

            distance_error = float(measured_distance) - target_distance
            x_reached = abs(x_error) <= x_tolerance
            distance_reached = abs(distance_error) <= distance_tolerance

            # if x_reached and distance_reached:
            if x_reached:
                stable_count += 1
                remoteControl(ws2, 0.0, 0.0, steering_speed)
                if stable_count >= 3:
                    print("已到达目标 x 位置，且右侧距离墙壁 %.3f m。" % measured_distance)
                    return True
                time.sleep(0.05)
                continue

            stable_count = 0
            # xvel 是车体前进速度，使用当前航向将地图 x 误差投影到车体前向。
            forward_error = x_error * math.cos(pose["yaw"])
            forward_speed = max(-max_forward_speed,
                                min(max_forward_speed, kp * forward_error))
            if abs(forward_speed) < min_crawl_speed and (not x_reached or not distance_reached):
                crawl_direction = forward_error if abs(forward_error) > 1e-6 else 1.0
                # steering_velocity = math.copysign(steering_speed, crawl_direction)
                forward_speed = math.copysign(min_crawl_speed, crawl_direction)
            
            if (abs(adjust_distance) > target_distance):
                if ((right_data.get("max_distance") - right_data.get("min_distance")) < 0.015):
                    distance_error = 0.0
                else:
                    if (right_data.get("nearest_corner") == "right_rear"):
                        distance_error = -distance_error

            steering_angle = max(-max_steering_angle,
                                 min(max_steering_angle,
                                     -steering_gain * distance_error))

            remoteControl(ws2, forward_speed, steering_angle, steering_speed)

        raise TimeoutError("闭环行驶在 %.1f 秒内未达到目标。" % timeout)
    finally:
        remoteControl(ws2, 0.0, 0.0, 0.0)
        ws2.close()

def set_AGV_vel(ws, fast_or_slow):
    if fast_or_slow == "fast":
        # 最大速度设置
        write_table_float(ws, "220", 1.5)
        # 舵轮在转向最大角时的最大线速度
        write_table_float(ws, "224", 1.0)
        # 基本加速度
        write_table_float(ws, "600", 1.0)
        # 基本减速度
        write_table_float(ws, "604", 1.0)
        # 舵轮加速度
        write_table_float(ws, "608", 1.0)
        # 最大减速距离
        write_table_float(ws, "60c", 1.0)
        # 1级 (减速距离,减速速度) 终点
        write_table_float(ws, "610", 1.0)
        write_table_float(ws, "614", 0.5)
        # 2级 (减速距离,减速速度) 终点
        write_table_float(ws, "618", 0.3)
        write_table_float(ws, "61c", 0.05)
    else:
        # 最大速度设置
        write_table_float(ws, "220", 0.5)
        # 舵轮在转向最大角时的最大线速度
        write_table_float(ws, "224", 0.5)
        # 基本加速度
        write_table_float(ws, "600", 3.0)
        # 基本减速度
        write_table_float(ws, "604", 1.0)
        # 舵轮加速度
        write_table_float(ws, "608", 0.3)
        # 最大减速距离
        write_table_float(ws, "60c", 2.5)
        # 1级 (减速距离,减速速度) 终点
        write_table_float(ws, "610", 1.8)
        write_table_float(ws, "614", 0.3)
        # 2级 (减速距离,减速速度) 终点
        write_table_float(ws, "618", 0.3)
        write_table_float(ws, "61c", 0.05)

def move_from_current_to_points(
    ws,
    point_20=None,
    point_18=None,
    point_17=None,
    point_16=None,
    timeout=ROUTE_TIMEOUT):
    # 检查必须的点位
    if point_20 is None or point_18 is None or point_17 is None or point_16 is None:
        raise ValueError("必须提供 point_20, point_18, point_17, point_16 坐标")
    
    # 读取当前坐标
    # print("正在读取当前车辆坐标...")
    current_pose_data = read_car_pose_from_ws(ws)
    current_pose = (
        current_pose_data["x"],
        current_pose_data["y"],
        current_pose_data["yaw"]
    )
    # print(f"当前位置: x={current_pose[0]:.4f}, y={current_pose[1]:.4f}, yaw={current_pose[2]:.4f} rad")
    
    # 构建路线节点
    # 节点ID使用序号，从1开始
    nodes1 = [
        (1, current_pose, ROUTE_STRAIGHT), # 起点：当前位置（直线）
        (13, POINT_13, ROUTE_STRAIGHT),       # 13点（直线到达）
        (4, POINT_4, ROUTE_STRAIGHT),       # 4点（直线到达）
    ]
    
    # 构建路线
    route_writes1 = build_route(nodes1)
    
    # set_AGV_vel(ws, "slow")
    try:
        # 更改速度参数
        set_AGV_vel(ws, "fast")

        # 启动路线
        start_route(ws, route_writes1)
        # 等待到达终点
        wait_until_stopped(ws, timeout, left_flag=False, right_flag=True)

        if CONFIG["emergency_stop"] == 1:
            print("AGV已经紧急停车，等待人工处理。")
            return

        # set_AGV_vel(ws, "slow")
        # 告知camera已到达
        msg = String()
        msg.data = "ok"
        laser_publisher.publish(msg)

        # 等待camera取货
        print("等待收到ROS控制消息...")
        rclpy.spin_once(ros_node, timeout_sec=None)
        print("收到ROS控制消息，开始执行路线。")
        
        # 读取当前坐标
        current_pose_data = read_car_pose_from_ws(ws)
        current_pose = (
            current_pose_data["x"],
            current_pose_data["y"],
            current_pose_data["yaw"]
        )

        nodes2 = [
            (1, current_pose, ROUTE_STRAIGHT)
            (4, POINT_4, ROUTE_STRAIGHT),
            (13, POINT_13, ROUTE_CURVE),       # 13点（直线到达）
            (5, POINT_5, ROUTE_CURVE),         # 5点（直线+圆弧）
            (21, POINT_21, ROUTE_STRAIGHT),
            (16, point_16, ROUTE_STRAIGHT),       # 16点（直线）
            (17, point_17, ROUTE_STRAIGHT),       # 17点（直线+圆弧）
            (18, point_18, ROUTE_STRAIGHT),       # 18点（直线+圆弧）
        ]
        route_writes2 = build_route(nodes2)

        # set_AGV_vel(ws, "fast")
        # 启动路线
        start_route(ws, route_writes2)
        # 等待到达终点
        wait_until_stopped(ws, timeout, left_flag=False, right_flag=True)

        if CONFIG["emergency_stop"] == 1:
            print("AGV已经紧急停车，等待人工处理。")

        # 测量判断函数
        i = 0
        adjust_Distance = 0.0
        while True:
            results = calc_LR_Distance(left_flag=False, right_flag=True)
            if results['right']['exists']:
                if (results['right']['min_distance'] is not None) and (results['right']['max_distance'] is not None):
                    adjust_Distance += ((results['right']['min_distance'] + results['right']['max_distance']) / 2.0)
                    i += 1
                    if i == 3:
                        adjust_Distance = adjust_Distance / 3.0
                        break

        # 更改速度参数
        set_AGV_vel(ws, "slow")

        drive_to_point_with_right_wall(ws, point_20[0])

        return adjust_Distance
        
    except Exception as exc:
        print(f"路线执行失败：{exc}")
        return 

if __name__ == "__main__":
    # # timeout_sec=0：立即返回，不等待。
    # # timeout_sec=None（默认）：阻塞直到有至少一个回调准备就绪并执行完。
    # # timeout_sec=0.1：最多等待 0.1 秒，超时若无回调则返回。
    # print("等待收到ROS控制消息...")
    # rclpy.spin_once(ros_node, timeout_sec=None)

    # # 告知camera已到达
    # msg = String()
    # msg.data = "ok"
    # laser_publisher.publish(msg)

    # # 等待camera取货
    # print("等待收到ROS控制消息...")
    # rclpy.spin_once(ros_node, timeout_sec=None)
    # print("收到ROS控制消息，开始执行路线。")
    # while True:
    #     pass

    websocket_url = sys.argv[1] if len(sys.argv) > 1 else "ws://192.168.31.7:6060"
    ws = websocket.create_connection(websocket_url, timeout=TABLE_TIMEOUT)

    read_laser4_from_ws_filter()
    CarriageAngleArr = []
    CarriageLengthArr= []
    CarriageWidthArr = []
    read_laser_from_ws_filter(ws)
    for _ in range(10):
        data = read_laser_from_ws(ws)
        corners, _ = detect_carriage_corners(data)

        radar_points = extract_radar_corner_points(corners)

        try:
            pose = read_car_pose_from_ws(ws)
            # 注意建图时，Map坐标系改变时，transform_points该函数需修改
            map_points = transform_points(
                radar_points, 
                add_x=CONFIG["R2C_X"], 
                add_y=CONFIG["R2C_Y"], 
                origin=(pose["x"], pose["y"]), 
                angle=pose["yaw"])
        except Exception as exc:
            print(f"无法读取车辆位姿，跳过地图坐标转换: {exc}")

        upLineAngle = np.arctan2(map_points[0][1] - map_points[1][1], map_points[0][0] - map_points[1][0])
        lowLineAngle = np.arctan2(map_points[3][1] - map_points[2][1], map_points[3][0] - map_points[2][0])
        
        # 直线方向具有 180° 等价性，先对角度加倍后求圆周均值，避免
        # upper_angle/lower_angle 跨越 ±pi 或拟合方向相反时直接平均失真。
        carriage_angle = math.atan2(
            math.sin(upLineAngle) + math.sin(lowLineAngle),
            math.cos(upLineAngle) + math.cos(lowLineAngle),
        )
        print(f"车厢偏航角: {math.degrees(carriage_angle):.6f}°")
        CarriageAngleArr.append(carriage_angle)
        CarriageLengthArr.append(max(np.sqrt((map_points[0][0] - map_points[1][0]) ** 2 + (map_points[0][1] - map_points[1][1]) ** 2), 
                                    np.sqrt((map_points[3][0] - map_points[2][0]) ** 2 + (map_points[3][1] - map_points[2][1]) ** 2)))
        CarriageWidthArr.append(np.sqrt((map_points[2][0] - map_points[1][0]) ** 2 + (map_points[2][1] - map_points[1][1]) ** 2))

        final_point = point_from_line_distances(carriage_angle, CONFIG["first_point_delX"], CONFIG["first_point_delY"], map_points[1])
        final_points.append(final_point)
        # print(f"最终点坐标: {final_point}")

    if final_points:
        avg_final_point = np.array([
            np.mean([point[0] for point in final_points]),
            np.mean([point[1] for point in final_points]),
            math.atan2(
                sum(math.sin(point[2]) for point in final_points),
                sum(math.cos(point[2]) for point in final_points),
            ),
        ])
        # print(f"平均 final_point: {avg_final_point}")
    else:
        print("没有成功计算出 final_point")

    CONFIG["CarriageAngle"] = math.atan2(
        sum(math.sin(angle) for angle in CarriageAngleArr),
        sum(math.cos(angle) for angle in CarriageAngleArr),
    )
    CONFIG["CarriageLength"] = np.mean(CarriageLengthArr)
    CONFIG["CarriageWidth"] = np.mean(CarriageWidthArr)
    
    # 注意建图时，Map坐标系改变时，clac_Middle_points_with_yaws该函数需修改
    [POINT_18, POINT_17, POINT_16] = clac_Middle_points_with_yaws(avg_final_point, yaw_offset=CONFIG["yaw_offset"], y_offset=CONFIG["y_offset"], x_offset=CONFIG["x_offset"], CarriageLen=CONFIG["CarriageLength"])
    # 定义18、17、16点的坐标（需要根据实际情况赋值）
    POINT_20 = tuple(avg_final_point)

    print(f"车厢长度: {CONFIG['CarriageLength']:.3f} m")
    print(f"车厢宽度: {CONFIG['CarriageWidth']:.3f} m")
    # print(f"车厢偏航角: {(CONFIG['CarriageAngle']):.6f}")
    print(f"车厢偏航角: {math.degrees(CONFIG['CarriageAngle']):.6f}°")
    print(f"POINT_20:({POINT_20[0]:.4f},{POINT_20[1]:.4f},{POINT_20[2]:.4f})")
    print(f"POINT_18:({POINT_18[0]:.4f},{POINT_18[1]:.4f},{POINT_18[2]:.4f})")
    print(f"POINT_17:({POINT_17[0]:.4f},{POINT_17[1]:.4f},{POINT_17[2]:.4f})")
    print(f"POINT_16:({POINT_16[0]:.4f},{POINT_16[1]:.4f},{POINT_16[2]:.4f})")

    input(
            "\n确认路线无人无障碍、能安全托起货架，"
            "现场人员手持急停后按回车开始计时；Ctrl+C取消：\n"
        )
    # print("已确认，5秒后开始执行路线。")
    # time.sleep(5)

    y_Distance = move_from_current_to_points(
        ws=ws,
        point_20=POINT_20,
        point_18=POINT_18,
        point_17=POINT_17,
        point_16=POINT_16,
        timeout=3600.0
    )

    # 缓慢前进直至INPUT5触发再放下
    # while True:
    #     if read_table_u8(ws, "55") == 1:
    #         print("Down")
    #         break

    # 放下货物顺便读取此时AGV的位置作为全局精确位置参考
    set_fork_height(ws, 0.0)
    # 虽然是通过碰撞判断x是否到达，但是前几个点的x坐标需根据第一个点来规划
    pose = read_car_pose_from_ws(ws)
    POINT_20 = (pose["x"], pose["y"], pose["yaw"])
    wait_for_fork(ws, 10, 0.0)

    # 回到POINT_18
    if y_Distance is not None:
        print(f"回程目标点y坐标: {y_Distance:.4f}")
        drive_to_point_with_right_wall_return(ws, POINT_18[0], y_Distance)
    else:
        print("未获取到y_Distance，无法回程。")
        

    # 固定路线18，17，16，5，13
    nodes = [
        (18, POINT_18, ROUTE_STRAIGHT),
        (17, POINT_17, ROUTE_STRAIGHT),       
        (16, POINT_16, ROUTE_CURVE),         
        (5, POINT_5, ROUTE_CURVE),       
        (13, POINT_13, ROUTE_STRAIGHT),      
        (4, POINT_4, ROUTE_STRAIGHT) 
    ]
    
    # 构建路线
    route_writes = build_route(nodes)
    # 更改速度参数
    set_AGV_vel(ws, "fast")
    # 启动路线
    start_route(ws, route_writes)
    # 等待到达终点
    wait_until_stopped(ws, 3600.0, left_flag=False, right_flag=True)


    
