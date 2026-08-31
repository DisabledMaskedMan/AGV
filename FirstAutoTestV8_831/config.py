"""FirstAutoTestV8 的全部配置参数和固定点位。"""
import numpy as np
from shapely.geometry import LineString

CONFIG = {
    "RANSAC_THRESHOLD": 0.01, "CORNER_THRESHOLD": 0.05,
    "RANSAC_ITERATIONS": 2000, "MIN_INLIERS": 80,
    "RANSAC_THRESHOLD4": 0.02, "RANSAC_ITERATIONS4": 500,
    "MIN_INLIERS4": 80,
    "UPPER_LINE_ANGLE_MIN_DEG": 0.0, "UPPER_LINE_ANGLE_MAX_DEG": 45.0,
    "LOWER_LINE_ANGLE_MIN_DEG": -45.0, "LOWER_LINE_ANGLE_MAX_DEG": 0.0,
    "UPPER_LOWER_ANGLE_THRESHOLD": 0.0, "LINE_ANGLE_DETECT_THRESHOLD": 5.0,
    "LEFT_UPPER_ANGLE_MIN_DEG": 45.0, "LEFT_UPPER_ANGLE_MAX_DEG": 180.0,
    "LEFT_LOWER_ANGLE_MIN_DEG": -180.0, "LEFT_LOWER_ANGLE_MAX_DEG": -45.0,
    "FRONT_POINT_RANGE_MIN": -45.0, "FRONT_POINT_RANGE_MAX": 45.0,
    "R2C_X": 0.94, "R2C_Y": 0.0,
    "CarriageAngle": 0.0, "CarriageLength": 0.0, "CarriageWidth": 0.0,
    "range_min": 0.0, "range_max": 0.0, "angle_start": 0.0, "increment": 0.0,
    "range_min4": 0.0, "range_max4": 0.0, "angle_start4": 0.0,
    "increment4": 0.0, "readed_param": False,
    "yaw_offset": 0, "x_offset": 2.0, "y_offset": 0.2,
    "first_point_delX": 0.84, "first_point_delY": 0.57,
    "dangerous_Distance": 0.05, "go_straight_action": 0,
    "emergency_stop": 0,
    # 最终进库时AGV右侧边线与车厢右墙的目标距离（单位：m）。
    # 这是所有箱子共用的右墙安全距离，不是箱子之间的横向间距。
    "right_wall_target_distance": 0.08,
    # 每个箱子完成并返回POINT_4后，下一步开始前的确认方式：
    # "manual"：终端按回车确认，适合现场人工操作（默认）。
    # "camera"：发布"ok"，等待相机通过/agv/camera_command回传任意消息。
    # 需要切换方式时，只修改下面这一行的值，不要填写其他字符串。
    "point_4_confirmation_mode": "manual",
    # 第2个箱子的POINT_16局部横向位置（单位：m）。
    # 以首箱精确POINT_20的横向基准计算，不是地图坐标的固定x/y增量。
    # 当前设置为1.9m，使车辆从入口进入时距车厢右侧约0.2m。
    "second_box_point16_lateral_offset": 1.9,
    # 单箱测试选择：
    # 0：执行完整三箱流程
    # 1：只执行第一箱
    # 2：独立执行第二箱
    # 3：独立执行第三箱
    # 独立执行第2/3箱时，启动前车辆应停在车厢前扫描位置，
    # 货箱应已由人工放到货叉上并托起。
    "standalone_box_index": 0,
}

ROBOT_VERTICES = np.array([[0.18, -0.5], [0.18, 0.5], [-1.70, 0.5], [-1.70, -0.5]], dtype=float)
LEFT_EDGE = LineString([ROBOT_VERTICES[1], ROBOT_VERTICES[2]])
RIGHT_EDGE = LineString([ROBOT_VERTICES[3], ROBOT_VERTICES[0]])

POINT_13 = (5.6456, 1.5382, -3.0992)
POINT_5 = (5.77757, -2.22915, -1.5708)
POINT_4 = (7.6434, 1.62252, -3.09902)
POINT_21 = (3.02084, -0.583914, 0.0)

ACTION_NONE = 6
ROUTE_STRAIGHT = 0
# TABLE路线类型2：直线+圆弧。
ROUTE_CURVE = 2
ROUTE_LINE_CURVE = 2
POINT_START = 0
POINT_MIDDLE = 1
POINT_END = 3
TABLE_TIMEOUT = 8.0
ROUTE_TIMEOUT = 360.0
STATUS_INTERVAL = 0.5
FORK_HEIGHT = 0.0
FORK_UP_HEIGHT = 0.05
FORK_DOWN_HEIGHT = 0.0

# 货箱横向偏移以第二次准确运行的现场方向为准。
# 正 right_offset 表示从首箱向现场验证的右侧偏移；
# 2.1 m和1.05 m只用于生成POINT_20路线点，不能加到右墙距离上。
THREE_BOX_RIGHT_OFFSETS = (0.0, 2.1, 1.05)
