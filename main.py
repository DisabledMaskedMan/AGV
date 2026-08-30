"""FirstAutoTestV8 的程序入口。"""

import json
import math
import sys

import numpy as np
import websocket

from .config import CONFIG, POINT_4, POINT_5, ROUTE_CURVE, ROUTE_STRAIGHT, TABLE_TIMEOUT
from .lidar import (
    detect_carriage_corners, extract_radar_corner_points,
    point_from_line_distances,
)
from .route_control import (
    build_route, drive_to_point_with_right_wall_return, set_AGV_vel,
    start_route, wait_until_stopped,
)
from .table_api import (
    clac_Middle_points_with_yaws, read_car_pose_from_ws,
    read_laser4_from_ws_filter, read_laser_from_ws, read_laser_from_ws_filter,
    set_fork_height, wait_for_fork, transform_points,
)
from .workflow import move_from_current_to_points
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

#***************3**************#
#读取位姿，更新20点，降货叉高度，回到18点
def _run(websocket_url):
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
    
    ws = websocket.create_connection(websocket_url, timeout=TABLE_TIMEOUT)#
    runtime.websocket_connection = ws

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

    if CONFIG["emergency_stop"] == 1:
        raise RuntimeError("AGV已经紧急停车，等待人工处理。")

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
    
