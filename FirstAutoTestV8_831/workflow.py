"""一次完整的取货、运输、放货和返回流程。"""

from dataclasses import dataclass
import math
from typing import Iterable, Optional

from .config import (
    CONFIG, POINT_4, POINT_5, ROUTE_CURVE, ROUTE_LINE_CURVE,
    ROUTE_STRAIGHT, ROUTE_TIMEOUT,
)
from .route_control import (
    build_route, drive_to_point_with_right_wall,
    set_AGV_vel, start_route,
    wait_until_stopped, wait_until_stopped2,
)
from .lidar import normalize_angle
from .safety import calc_LR_Distance
from .table_api import (
    clac_Middle_points_with_yaws, read_car_pose_from_ws, set_fork_height,
    wait_for_fork,
)
from . import runtime


@dataclass(frozen=True)
class BoxTask:
    """一个货箱对应的动态路线点位。"""

    point_20: tuple
    point_18: tuple
    point_17: tuple
    point_16: tuple
    right_offset: float = 0.0
    point18_lateral_offset: Optional[float] = None
    point16_lateral_offset: Optional[float] = None
    box_index: int = 1
    approach_mode: str = "right_wall_pid"
    point16_to_17_route_type: int = ROUTE_STRAIGHT
    point17_to_18_route_type: int = ROUTE_STRAIGHT
    wait_mode: str = "radar"
    wait_left_flag: bool = True
    wait_right_flag: bool = False


def offset_point_to_right(point, distance):
    """按第二次准确运行的现场方向向右偏移地图点，保持原yaw不变。"""
    x, y, yaw = point
    return (
        x - distance * math.sin(yaw),
        y + distance * math.cos(yaw),
        yaw,
    )


def heading_towards(start_point, end_point):
    """计算从一个路线点驶向下一个路线点的地图航向角。"""
    dx = end_point[0] - start_point[0]
    dy = end_point[1] - start_point[1]
    if math.isclose(dx, 0.0, abs_tol=1e-12) and math.isclose(
        dy, 0.0, abs_tol=1e-12
    ):
        raise ValueError("两个路线点不能重合，无法计算航向角")
    return math.atan2(dy, dx)


def reversed_heading_towards(start_point, end_point):
    """计算路线方向反向180度后的AGV车头航向。"""
    return normalize_angle(heading_towards(start_point, end_point) + math.pi)


def build_box_task(base_point_20, right_offset, carriage_length, box_index=1):
    """由精确POINT_20生成指定箱位的全部动态点。"""
    point_20 = offset_point_to_right(base_point_20, right_offset)
    if box_index == 1:
        point18_lateral_offset = None
        point16_lateral_offset = None
        approach_mode = "right_wall_pid"
        point16_to_17_route_type = ROUTE_STRAIGHT
        point17_to_18_route_type = ROUTE_STRAIGHT
        wait_mode = "radar"
        wait_left_flag = True
        wait_right_flag = False
    elif box_index == 2:
        point18_lateral_offset = right_offset
        point16_lateral_offset = CONFIG["second_box_point16_lateral_offset"]
        approach_mode = "straight"
        point16_to_17_route_type = ROUTE_LINE_CURVE
        point17_to_18_route_type = ROUTE_LINE_CURVE
        wait_mode = "radar"
        wait_left_flag = False
        wait_right_flag = True
    else:
        point18_lateral_offset = right_offset
        point16_lateral_offset = right_offset
        approach_mode = "straight"
        point16_to_17_route_type = ROUTE_STRAIGHT
        point17_to_18_route_type = ROUTE_STRAIGHT
        wait_mode = "no_radar"
        wait_left_flag = False
        wait_right_flag = False

    point_18, point_17, point_16 = clac_Middle_points_with_yaws(
        point_20,
        yaw_offset=CONFIG["yaw_offset"],
        y_offset=CONFIG["y_offset"],
        x_offset=CONFIG["x_offset"],
        CarriageLen=carriage_length,
        lateral_reference_point=base_point_20,
        point18_lateral_offset=point18_lateral_offset,
        point16_lateral_offset=point16_lateral_offset,
    )
    if box_index in (2, 3):
        point_16 = (
            *point_16[:2],
            reversed_heading_towards(point_16, point_17),
        )
    if box_index == 2:
        point_17 = (
            *point_17[:2],
            reversed_heading_towards(point_17, point_18),
        )
    return BoxTask(
        point_20=point_20,
        point_18=point_18,
        point_17=point_17,
        point_16=point_16,
        right_offset=right_offset,
        point18_lateral_offset=point18_lateral_offset,
        point16_lateral_offset=point16_lateral_offset,
        box_index=box_index,
        approach_mode=approach_mode,
        point16_to_17_route_type=point16_to_17_route_type,
        point17_to_18_route_type=point17_to_18_route_type,
        wait_mode=wait_mode,
        wait_left_flag=wait_left_flag,
        wait_right_flag=wait_right_flag,
    )


def build_box_tasks(base_point_20, right_offsets, carriage_length):
    """按给定顺序生成多个现场右侧箱位任务。"""
    return [
        build_box_task(base_point_20, offset, carriage_length, box_index=index)
        for index, offset in enumerate(right_offsets, start=1)
    ]


def get_point_4_confirmation_mode():
    """读取并校验POINT_4确认模式。"""
    mode = CONFIG["point_4_confirmation_mode"]
    if mode not in ("manual", "camera"):
        raise ValueError(
            "CONFIG['point_4_confirmation_mode'] 必须为 'manual' 或 'camera'"
        )
    return mode


def get_standalone_box_index():
    """读取并校验单箱测试选择。"""
    box_index = CONFIG["standalone_box_index"]
    if box_index not in (0, 1, 2, 3):
        raise ValueError(
            "CONFIG['standalone_box_index'] 必须为 0、1、2 或 3"
        )
    return box_index


def wait_for_point_4_confirmation(box_index):
    """货箱返回POINT_4后，等待人工或相机确认。"""
    mode = get_point_4_confirmation_mode()
    if mode == "manual":
        input(
            f"\n第{box_index}个箱子已完成并返回4点，"
            "确认可以继续后按回车；Ctrl+C取消：\n"
        )
        return

    if mode == "camera":
        print(f"第{box_index}个箱子已完成并返回4点，等待相机确认...")
        runtime.publish_navigation_status("ok")
        runtime.spin_once()
        print("收到相机确认，继续执行。")
        return


def wait_for_route_stop(
    ws, timeout, wait_mode="radar", left_flag=False, right_flag=False
):
    """按当前箱子的安全模式等待路线结束。"""
    if wait_mode == "no_radar":
        return wait_until_stopped2(ws, timeout)
    if wait_mode != "radar":
        raise ValueError("wait_mode 必须为 'radar' 或 'no_radar'")
    return wait_until_stopped(ws, timeout, left_flag, right_flag)


def build_standalone_approach_route(scan_pose):
    """构建扫描结束位姿经POINT_4到POINT_5的独立测试路线。"""
    return build_route([
        (2, scan_pose, ROUTE_STRAIGHT),
        (4, POINT_4, ROUTE_STRAIGHT),
        (5, POINT_5, ROUTE_LINE_CURVE),
    ])


def build_point4_to_point5_route(current_pose):
    """构建当前位姿经过POINT_4到POINT_5的路线。"""
    return build_route([
        (2, current_pose, ROUTE_STRAIGHT),
        (4, POINT_4, ROUTE_LINE_CURVE),
        (5, POINT_5, ROUTE_LINE_CURVE),
    ])


def build_current_to_point5_route(current_pose):
    """兼容旧调用，构建当前位姿经过POINT_4到POINT_5的路线。"""
    return build_point4_to_point5_route(current_pose)


def build_loading_route(task: BoxTask):
    """构建一次性下发的POINT_5到POINT_20进库路线。"""
    return build_route([
        (5, POINT_5, ROUTE_LINE_CURVE),
        (16, task.point_16, task.point16_to_17_route_type),
        (17, task.point_17, task.point17_to_18_route_type),
        (18, task.point_18, ROUTE_STRAIGHT),
        (20, task.point_20, ROUTE_STRAIGHT),
    ])


def build_loading_to_point18_route(task: BoxTask):
    """构建第一箱从POINT_5到POINT_18的路线。"""
    return build_route([
        (5, POINT_5, ROUTE_LINE_CURVE),
        (16, task.point_16, task.point16_to_17_route_type),
        (17, task.point_17, ROUTE_STRAIGHT),
        (18, task.point_18, ROUTE_STRAIGHT),
    ])


def build_return_to_point5_route(task: BoxTask, current_pose):
    """构建放货后从当前POINT_20连续返回POINT_5的路线。"""
    return build_route([
        (20, current_pose, ROUTE_STRAIGHT),
        (18, task.point_18, ROUTE_STRAIGHT),
        (17, task.point_17, ROUTE_STRAIGHT),
        (16, task.point_16, ROUTE_LINE_CURVE),
        (5, POINT_5, ROUTE_LINE_CURVE),
    ])


def build_point5_to_point4_route(current_pose):
    """构建从POINT_5返回POINT_4的路线。"""
    return build_route([
        (5, current_pose, ROUTE_LINE_CURVE),
        (4, POINT_4, ROUTE_LINE_CURVE),
    ])


def build_standalone_loading_route(task: BoxTask):
    """构建独立测试一次性下发的POINT_5到POINT_20进库路线。"""
    return build_loading_route(task)


def _return_box_to_point_4(ws, task, timeout, y_distance=None):
    """降叉后返回POINT_5停车，再返回POINT_4。"""
    current_pose_data = read_car_pose_from_ws(ws)
    current_pose = (
        current_pose_data["x"],
        current_pose_data["y"],
        current_pose_data["yaw"],
    )
    route_writes = build_return_to_point5_route(task, current_pose)
    set_AGV_vel(ws, "fast")
    start_route(ws, route_writes)
    wait_for_route_stop(
        ws,
        timeout,
        task.wait_mode,
        task.wait_left_flag,
        task.wait_right_flag,
    )

    current_pose_data = read_car_pose_from_ws(ws)
    current_pose = (
        current_pose_data["x"],
        current_pose_data["y"],
        current_pose_data["yaw"],
    )
    set_AGV_vel(ws, "fast")
    start_route(ws, build_point5_to_point4_route(current_pose))
    wait_for_route_stop(
        ws,
        timeout,
        task.wait_mode,
        task.wait_left_flag,
        task.wait_right_flag,
    )
    return y_distance


def run_standalone_box_workflow(ws, task: BoxTask, scan_pose, timeout=ROUTE_TIMEOUT):
    """独立执行第二或第三箱的扫描、进库、放货和返回流程。"""
    if task.box_index not in (2, 3):
        raise ValueError("独立流程只支持第二箱或第三箱")

    set_AGV_vel(ws, "fast")
    start_route(ws, build_standalone_approach_route(scan_pose))
    wait_for_route_stop(
        ws,
        timeout,
        task.wait_mode,
        task.wait_left_flag,
        task.wait_right_flag,
    )

    set_AGV_vel(ws, "fast")
    start_route(ws, build_standalone_loading_route(task))
    wait_for_route_stop(
        ws,
        timeout,
        task.wait_mode,
        task.wait_left_flag,
        task.wait_right_flag,
    )

    if CONFIG["emergency_stop"] == 1:
        raise RuntimeError("AGV已经紧急停车，等待人工处理。")

    set_fork_height(ws, 0.0)
    wait_for_fork(ws, 10, 0.0)
    y_distance = _return_box_to_point_4(ws, task, timeout)
    wait_for_point_4_confirmation(task.box_index)
    return y_distance

def move_from_current_to_points(
    ws,
    point_20=None,
    point_18=None,
    point_17=None,
    point_16=None,
    timeout=ROUTE_TIMEOUT,
    target_distance=None,
    adjust_distance=None,
    measure_adjust_distance=True,
    approach_mode="right_wall_pid",
    point16_to_17_route_type=ROUTE_STRAIGHT,
    point17_to_18_route_type=ROUTE_STRAIGHT,
    wait_mode="radar",
    wait_left_flag=True,
    wait_right_flag=False,
    task_index=1):
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
    
    try:
        set_AGV_vel(ws, "fast")

        if task_index == 1:
            route_to_pickup = build_route([
                (2, current_pose, ROUTE_STRAIGHT),
                (4, POINT_4, ROUTE_STRAIGHT),
            ])
            start_route(ws, route_to_pickup)
            wait_for_route_stop(
                ws, timeout, wait_mode, wait_left_flag, wait_right_flag
            )
            wait_for_point_4_confirmation(task_index)
            current_pose_data = read_car_pose_from_ws(ws)
            current_pose = (
                current_pose_data["x"],
                current_pose_data["y"],
                current_pose_data["yaw"],
            )
            start_route(ws, build_current_to_point5_route(current_pose))
            wait_for_route_stop(
                ws, timeout, wait_mode, wait_left_flag, wait_right_flag
            )
        else:
            start_route(ws, build_current_to_point5_route(current_pose))
            wait_for_route_stop(
                ws, timeout, wait_mode, wait_left_flag, wait_right_flag
            )

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

        if task_index == 1:
            loading_task = BoxTask(
                point_20, point_18, point_17, point_16,
                point16_to_17_route_type=point16_to_17_route_type,
                point17_to_18_route_type=point17_to_18_route_type,
            )
            start_route(ws, build_loading_to_point18_route(loading_task))
        else:
            loading_task = BoxTask(
                point_20, point_18, point_17, point_16,
                point16_to_17_route_type=point16_to_17_route_type,
                point17_to_18_route_type=point17_to_18_route_type,
            )
            start_route(ws, build_loading_route(loading_task))
        wait_for_route_stop(
            ws, timeout, wait_mode, wait_left_flag, wait_right_flag
        )

        if CONFIG["emergency_stop"] == 1:
            print("AGV已经紧急停车，等待人工处理。")

        if measure_adjust_distance:
            # 只在第一个货箱测量一次，后续货箱复用这个回程基准。
            i = 0
            adjust_distance = 0.0
            while True:
                results = calc_LR_Distance(left_flag=False, right_flag=True)
                if results['right']['exists']:
                    if (results['right']['min_distance'] is not None) and (results['right']['max_distance'] is not None):
                        adjust_distance += ((results['right']['min_distance'] + results['right']['max_distance']) / 2.0)
                        i += 1
                        if i == 3:
                            adjust_distance = adjust_distance / 3.0
                            break

        if task_index == 1:
            set_AGV_vel(ws, "slow")
            if target_distance is None:
                target_distance = CONFIG["right_wall_target_distance"]
            drive_to_point_with_right_wall(
                ws,
                point_20[0],
                target_distance=target_distance,
            )

        return adjust_distance
        
    except Exception as exc:
        print(f"路线执行失败：{exc}")
        return 


def _run_one_box_cycle(
    ws, task: BoxTask, timeout=ROUTE_TIMEOUT, task_index=1,
    adjust_distance=None,
):
    """执行一个货箱周期，并返回首箱测得的回程基准和放箱后的实际位置。"""
    get_point_4_confirmation_mode()
    y_distance = move_from_current_to_points(
        ws=ws,
        point_20=task.point_20,
        point_18=task.point_18,
        point_17=task.point_17,
        point_16=task.point_16,
        timeout=timeout,
        target_distance=CONFIG["right_wall_target_distance"],
        adjust_distance=adjust_distance,
        measure_adjust_distance=(task_index == 1),
        approach_mode=task.approach_mode,
        point16_to_17_route_type=task.point16_to_17_route_type,
        point17_to_18_route_type=task.point17_to_18_route_type,
        wait_mode=task.wait_mode,
        wait_left_flag=task.wait_left_flag,
        wait_right_flag=task.wait_right_flag,
        task_index=task_index,
    )

    if CONFIG["emergency_stop"] == 1:
        raise RuntimeError("AGV已经紧急停车，等待人工处理。")

    set_fork_height(ws, 0.0)
    wait_for_fork(ws, 10, 0.0)
    pose = read_car_pose_from_ws(ws)
    actual_point_20 = (pose["x"], pose["y"], pose["yaw"])

    _return_box_to_point_4(ws, task, timeout, y_distance)
    wait_for_point_4_confirmation(task_index)

    return y_distance, actual_point_20


def run_one_box_cycle(ws, task: BoxTask, timeout=ROUTE_TIMEOUT):
    """执行一个货箱的取货、运输、放货和返回流程。"""
    y_distance, _ = _run_one_box_cycle(ws, task, timeout=timeout)
    return y_distance


def run_multi_box_workflow(
    ws, tasks: Iterable[BoxTask], timeout=ROUTE_TIMEOUT
):
    """首箱放下后，以实际 POINT_20 和首箱 adjust_distance 执行后续箱位。"""
    get_point_4_confirmation_mode()
    results = []
    precise_base_point_20 = None
    first_adjust_distance = None
    for index, planned_task in enumerate(tasks, start=1):
        task = planned_task
        if precise_base_point_20 is not None:
            task = build_box_task(
                precise_base_point_20,
                planned_task.right_offset,
                CONFIG["CarriageLength"],
                box_index=planned_task.box_index,
            )

        print(f"开始执行第 {index} 个货箱任务")
        print(
            f"第{index}个箱子 POINT_20:"
            f"({task.point_20[0]:.4f},{task.point_20[1]:.4f},"
            f"{task.point_20[2]:.4f})"
        )
        if index == 2:
            print(
                "第二箱动态点位："
                f"POINT_16=({task.point_16[0]:.4f},"
                f"{task.point_16[1]:.4f},{task.point_16[2]:.4f}), "
                f"POINT_17=({task.point_17[0]:.4f},"
                f"{task.point_17[1]:.4f},{task.point_17[2]:.4f}), "
                f"POINT_18=({task.point_18[0]:.4f},"
                f"{task.point_18[1]:.4f},{task.point_18[2]:.4f}), "
                f"POINT_20=({task.point_20[0]:.4f},"
                f"{task.point_20[1]:.4f},{task.point_20[2]:.4f})"
            )
        if CONFIG["emergency_stop"] == 1:
            raise RuntimeError("AGV已经紧急停车，等待人工处理。")
        y_distance, actual_point_20 = _run_one_box_cycle(
            ws,
            task,
            timeout=timeout,
            task_index=index,
            adjust_distance=first_adjust_distance,
        )
        results.append(y_distance)
        if first_adjust_distance is None:
            first_adjust_distance = y_distance
        if precise_base_point_20 is None:
            precise_base_point_20 = actual_point_20
            print(
                "第1个箱子放下后的精确 POINT_20:"
                f"({actual_point_20[0]:.4f},{actual_point_20[1]:.4f},"
                f"{actual_point_20[2]:.4f})"
            )
    return results
