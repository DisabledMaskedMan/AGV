"""侧向雷达安全检测和 AGV 边界距离计算。"""

import math

from shapely.geometry import LineString as ShapelyLineString

from .config import CONFIG, LEFT_EDGE, RIGHT_EDGE
from .lidar import point_line_distance, ransac_line, scan4_to_xy
from .table_api import read_laser4_from_ws


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
        left_edge=LEFT_EDGE,
        right_edge=RIGHT_EDGE,
        left_line=left_line,
        right_line=right_line
    )

    return results
