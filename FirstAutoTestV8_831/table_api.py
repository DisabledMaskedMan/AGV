"""WebSocket 雷达、位姿、TABLE、急停和货叉接口。"""

import json
import math
import time

import numpy as np
import websocket

from .config import (
    ACTION_NONE, CONFIG, FORK_HEIGHT, POINT_END, POINT_MIDDLE, POINT_START,
    STATUS_INTERVAL, TABLE_TIMEOUT,
)

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

    if response.get("code") != 0:
        return None

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

def clac_Middle_points_with_yaws(
    final_point,
    yaw_offset=0.0,
    y_offset=0.2,
    x_offset=2.0,
    CarriageLen=6.0,
    lateral_reference_point=None,
    point18_lateral_offset=None,
    point17_lateral_offset=None,
    point16_lateral_offset=None,
):
    """根据POINT_20生成车厢内路线点。

    局部坐标约定为：+x沿车厢前进方向，+y沿车体左侧；
    因此这里的正y_offset表示向车体左侧偏移。
    """

    # 解析终点坐标和角度
    final_x, final_y, final_yaw = final_point
    
    # 计算总偏移距离
    total_offset = CarriageLen + x_offset
    
    # 将角度偏移从度转换为弧度
    yaw_offset_rad = yaw_offset * math.pi / 180.0
    
    # 计算旋转矩阵的余弦和正弦值
    cos_yaw = math.cos(final_yaw)
    sin_yaw = math.sin(final_yaw)

    if lateral_reference_point is None:
        lateral_origin_x = final_x
        lateral_origin_y = final_y
    else:
        lateral_origin_x = lateral_reference_point[0]
        lateral_origin_y = lateral_reference_point[1]
    
    legacy_point18_lateral = y_offset * 2.0 / 5.0
    point16_lateral = (
        y_offset if point16_lateral_offset is None else point16_lateral_offset
    )
    point18_lateral = (
        legacy_point18_lateral
        if point18_lateral_offset is None else point18_lateral_offset
    )
    if point17_lateral_offset is None:
        if point18_lateral_offset is None and point16_lateral_offset is None:
            point17_lateral = y_offset * 3.0 / 4.0
        else:
            point17_lateral = point18_lateral + (
                point16_lateral - point18_lateral
            ) * 3.0 / 4.0
    else:
        point17_lateral = point17_lateral_offset

    # POINT_16: 车厢外侧约2m的入口点。自定义箱位时，横向位置跟随目标箱位。
    dx_16 = total_offset * cos_yaw - point16_lateral * sin_yaw
    dy_16 = total_offset * sin_yaw + point16_lateral * cos_yaw
    POINT_16 = (lateral_origin_x + dx_16,
                lateral_origin_y + dy_16,
                final_yaw)
    
    # POINT_17: 3/4的局部+x和局部+y偏移。
    offset_17_x = total_offset * 3.0 / 4.0
    offset_17_y = point17_lateral
    dx_17 = offset_17_x * cos_yaw - offset_17_y * sin_yaw
    dy_17 = offset_17_x * sin_yaw + offset_17_y * cos_yaw
    POINT_17 = (lateral_origin_x + dx_17,
                lateral_origin_y + dy_17,
                final_yaw)
    
    # POINT_18: 距离POINT_20约1m的进库预对准点。
    offset_18_x = 1.0
    offset_18_y = point18_lateral
    dx_18 = offset_18_x * cos_yaw - offset_18_y * sin_yaw
    dy_18 = offset_18_x * sin_yaw + offset_18_y * cos_yaw
    POINT_18 = (lateral_origin_x + dx_18,
                lateral_origin_y + dy_18,
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


def read_float_from_table(ws, address, timeout=1.0):
    """读取 TABLE 中的 float 值。"""
    payload = {
        "cmd": "table",
        "command": "get",
        "reads": [{"address": address, "len": 4, "type": "float", "cmd": "read"}],
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
            if not data or not data[0].get("value"):
                raise RuntimeError(f"TABLE读取返回空数据：{reply}")
            return float(data[0]["value"][0])
    raise TimeoutError(f"等待TABLE 0x{address}读取结果超时")


def write_float_to_table(ws, address, value, timeout=1.0):
    """写入 float 值到 TABLE。"""
    payload = {
        "cmd": "table",
        "command": "set",
        "writes": [{
            "address": address, "len": 12, "type": "float",
            "cmd": "write", "data": value,
        }],
    }
    ws.send(json.dumps(payload))
    deadline = time.time() + timeout
    while time.time() < deadline:
        ws.settimeout(max(0.1, deadline - time.time()))
        try:
            reply = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        if reply.get("cmd") == "table_writes_result":
            if reply.get("code") != 0:
                raise RuntimeError(f"TABLE写入失败：{reply}")
            return
        if reply.get("type") == "table_set_result":
            if reply.get("status") != 0:
                raise RuntimeError(f"TABLE写入失败：{reply}")
            return
    raise TimeoutError(f"等待TABLE 0x{address}写入响应超时")

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
