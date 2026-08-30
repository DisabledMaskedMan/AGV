"""一次完整的取货、运输、放货和返回流程。"""

from dataclasses import dataclass
from typing import Iterable

from .config import CONFIG, POINT_4, POINT_5, ROUTE_CURVE, ROUTE_STRAIGHT, ROUTE_TIMEOUT
from .route_control import (
    build_route, drive_to_point_with_right_wall,
    drive_to_point_with_right_wall_return, set_AGV_vel, start_route,
    wait_until_stopped,
)
from .safety import calc_LR_Distance
from .table_api import read_car_pose_from_ws, set_fork_height, wait_for_fork
from . import runtime


@dataclass(frozen=True)
class BoxTask:
    """一个货箱对应的动态路线点位。"""

    point_20: tuple
    point_18: tuple
    point_17: tuple
    point_16: tuple


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
        (2, current_pose, ROUTE_STRAIGHT), # 起点：当前位置（直线）
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
        # # 告知camera已到达
        # runtime.publish_navigation_status("ok")

        # # 等待camera取货
        # print("等待收到ROS控制消息...")
        # runtime.spin_once()
        # print("收到ROS控制消息，开始执行路线。")
        
        # set_fork_height(ws, 0.05)
        # wait_for_fork(ws, 10, 0.05)

        # 读取当前坐标
        current_pose_data = read_car_pose_from_ws(ws)
        current_pose = (
            current_pose_data["x"],
            current_pose_data["y"],
            current_pose_data["yaw"]
        )

        nodes2 = [
            (1, current_pose, ROUTE_STRAIGHT),
            (4, POINT_4, ROUTE_CURVE),
            (5, POINT_5, ROUTE_CURVE),         # 5点（直线+圆弧）
            # (21, POINT_21, ROUTE_STRAIGHT),
            (16, point_16, ROUTE_STRAIGHT),       # 16点（直线）
            (17, point_17, ROUTE_STRAIGHT),       # 17点（直线+圆弧）
            (18, point_18, ROUTE_STRAIGHT),       # 18点（直线+圆弧）
        ]
        route_writes2 = build_route(nodes2)

        set_AGV_vel(ws, "fast")
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


def run_one_box_cycle(ws, task: BoxTask, timeout=ROUTE_TIMEOUT):
    """执行一个货箱的取货、运输和靠墙定位流程。"""
    return move_from_current_to_points(
        ws=ws,
        point_20=task.point_20,
        point_18=task.point_18,
        point_17=task.point_17,
        point_16=task.point_16,
        timeout=timeout,
    )


def run_multi_box_workflow(
    ws, tasks: Iterable[BoxTask], timeout=ROUTE_TIMEOUT
):
    """按顺序执行多个货箱任务并返回每个任务的右墙距离结果。"""
    results = []
    for index, task in enumerate(tasks, start=1):
        print(f"开始执行第 {index} 个货箱任务")
        if CONFIG["emergency_stop"] == 1:
            raise RuntimeError("AGV已经紧急停车，等待人工处理。")
        results.append(run_one_box_cycle(ws, task, timeout=timeout))
    return results
