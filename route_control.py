"""路线启动、到站等待、遥控和贴墙闭环控制。"""

import json
import math
import time

import numpy as np
import websocket

from .config import (
    ACTION_NONE, CONFIG, FORK_HEIGHT, POINT_END, POINT_MIDDLE, POINT_START,
    ROUTE_TIMEOUT, TABLE_TIMEOUT,
)
from .lidar import point_line_distance
from .safety import calc_LR_Distance
from .table_api import (
    clear_emergency_stop, read_car_pose_from_ws, read_float_from_table,
    read_table_u8, table_set,
    send_emergency_stop, write_float_to_table, write_table_float, write_table_u8,
)


def node_writes(node_id, pose, route_type, point_type, index, fork_height=FORK_HEIGHT):
    """生成一个路线节点。"""
    return [
        {"address": "24c", "len": 32, "type": "int32", "data": [node_id]},
        {"address": "260", "len": 16, "type": "float",
         "data": [pose[0], pose[1], pose[2], fork_height]},
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
        point_type = (
            POINT_START if index == 0
            else POINT_END if index == last_index
            else POINT_MIDDLE
        )
        writes += node_writes(node_id, pose, route_type, point_type, index, fork_height)
    return writes


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
            if pose is None:
                continue
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
            if pose is None:
                continue
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
        write_table_float(ws, "220", 1.0)
        # 舵轮在转向最大角时的最大线速度
        write_table_float(ws, "224", 0.25)
        # 基本加速度
        write_table_float(ws, "600", 0.5)
        # 基本减速度
        write_table_float(ws, "604", 0.2)
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
        write_table_float(ws, "224", 0.25)
        # 基本加速度
        write_table_float(ws, "600", 3.0)
        # 基本减速度
        write_table_float(ws, "604", 0.05)
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
