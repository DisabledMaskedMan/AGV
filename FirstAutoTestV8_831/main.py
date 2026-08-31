"""FirstAutoTestV8 的程序入口。"""

import math
import sys
from dataclasses import dataclass

import numpy as np
import websocket

from .config import CONFIG, TABLE_TIMEOUT, THREE_BOX_RIGHT_OFFSETS
from .lidar import (
    detect_carriage_corners, extract_radar_corner_points,
    point_from_line_distances,
)
from .table_api import (
    read_car_pose_from_ws,
    read_laser4_from_ws_filter, read_laser_from_ws, read_laser_from_ws_filter,
    transform_points,
)
from .workflow import (
    build_box_tasks,
    get_standalone_box_index,
    run_multi_box_workflow,
    run_one_box_cycle,
    run_standalone_box_workflow,
)
from . import runtime


def handle_command_message(message):
    print(f"收到ROS控制消息: {message.data}")

#*******run函数功能*******#
#初始化雷达数据，扫车厢10次，检测角点，提取角点，读当前位姿，
#转换地图坐标，计算车厢角度...，计算20点，取平均，计算车厢内部路线。

#****run内部的move_from_current_to_points****#
#从当前点(车库门口)移动到20点
#读当前坐标，构建第一段路线（取货），等待拍照消息回传，取货，第二次路线，
#从取货点到达18点。最后一段路线慢慢减速进去20点(pid)。

@dataclass(frozen=True)
class CarriageScanResult:
    """一次车厢扫描得到的路线计算输入。"""

    point_20: tuple
    carriage_length: float
    carriage_width: float
    carriage_angle: float
    scan_pose: tuple


def scan_carriage(ws):
    """扫描车厢并返回基础POINT_20、尺寸、角度和扫描结束位姿。"""
    final_points = []
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
    
    # print(read_car_pose_from_ws(ws))
    # while True:
    #     pass

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
    
    if not final_points:
        raise RuntimeError("没有成功计算出 final_point")

    scan_pose_data = read_car_pose_from_ws(ws)
    if scan_pose_data is None:
        raise RuntimeError("扫描结束时无法读取车辆位姿")
    scan_pose = (
        scan_pose_data["x"],
        scan_pose_data["y"],
        scan_pose_data["yaw"],
    )
    return CarriageScanResult(
        point_20=tuple(avg_final_point),
        carriage_length=float(CONFIG["CarriageLength"]),
        carriage_width=float(CONFIG["CarriageWidth"]),
        carriage_angle=float(CONFIG["CarriageAngle"]),
        scan_pose=scan_pose,
    )


def _print_scan_result(scan_result):
    print(f"车厢长度: {scan_result.carriage_length:.3f} m")
    print(f"车厢宽度: {scan_result.carriage_width:.3f} m")
    print(f"车厢偏航角: {math.degrees(scan_result.carriage_angle):.6f}°")
    print(
        f"POINT_20:({scan_result.point_20[0]:.4f},"
        f"{scan_result.point_20[1]:.4f},{scan_result.point_20[2]:.4f})"
    )


def run_selected_workflow(ws, tasks, box_index, scan_pose, timeout=3600.0):
    """按配置选择完整流程、第一箱流程或独立第二/第三箱流程。"""
    if box_index == 0:
        return run_multi_box_workflow(ws, tasks, timeout=timeout)
    if box_index == 1:
        return run_one_box_cycle(ws, tasks[0], timeout=timeout)
    if box_index in (2, 3):
        return run_standalone_box_workflow(
            ws, tasks[box_index - 1], scan_pose, timeout=timeout
        )
    raise ValueError("standalone_box_index 必须为 0、1、2 或 3")


def _run(websocket_url):
    box_index = get_standalone_box_index()
    ws = websocket.create_connection(websocket_url, timeout=TABLE_TIMEOUT)
    runtime.websocket_connection = ws
    scan_result = scan_carriage(ws)
    _print_scan_result(scan_result)

    tasks = build_box_tasks(
        base_point_20=scan_result.point_20,
        right_offsets=THREE_BOX_RIGHT_OFFSETS,
        carriage_length=scan_result.carriage_length,
    )
    input(
        "\n确认路线无人无障碍、能安全托起货架，"
        "现场人员手持急停后按回车开始计时；Ctrl+C取消：\n"
    )
    if box_index in (2, 3):
        task = tasks[box_index - 1]
        print(f"独立执行第{box_index}个箱子，使用扫描结束位姿返回POINT_5。")
        for name, point in (
            ("POINT_16", task.point_16),
            ("POINT_17", task.point_17),
            ("POINT_18", task.point_18),
            ("POINT_20", task.point_20),
        ):
            print(f"{name}:({point[0]:.4f},{point[1]:.4f},{point[2]:.4f})")
    return run_selected_workflow(
        ws, tasks, box_index, scan_result.scan_pose, timeout=3600.0
    )



def main(argv=None):
    """初始化 ROS2 并运行一次完整的 AGV 流程。"""
    args = sys.argv[1:] if argv is None else list(argv)
    websocket_url = args[0] if args else "ws://192.168.31.7:6060"
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    runtime.ros_node = rclpy.create_node("navigation_control_node")
    runtime.laser_publisher = runtime.ros_node.create_publisher(
        String, "/agv/navigation_status", 10
    )
    runtime.camera_subscription = runtime.ros_node.create_subscription(
        String, "/agv/camera_command", handle_command_message, 10
    )
    try:
        _run(websocket_url)#建立程序和AGV接口之间的socket通信
    finally:
        if runtime.websocket_connection is not None:
            runtime.websocket_connection.close()
            runtime.websocket_connection = None
        if runtime.ros_node is not None:
            runtime.ros_node.destroy_node()
            runtime.ros_node = None
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    
