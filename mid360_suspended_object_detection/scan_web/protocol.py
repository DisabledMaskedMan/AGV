import math
from dataclasses import dataclass


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError(f"扫描字段 {name} 必须是有限数值")
    return float(value)


@dataclass(frozen=True)
class ScanPacket:
    frame: str
    name: str
    ranges: tuple
    angle_start: float
    increment: float
    range_min: float
    range_max: float
    laser_offset: tuple | None

    def points(self):
        result = []
        for index, distance in enumerate(self.ranges):
            if distance is None:
                continue
            angle = self.angle_start + index * self.increment
            result.append((distance * math.cos(angle), distance * math.sin(angle)))
        return result


def parse_scan(response, frame="scan_link"):
    if not isinstance(response, dict):
        return None
    if response.get("channel") != "ros" or response.get("op") != "publish" or response.get("topic") != "/scan":
        return None
    items = response.get("msg")
    if not isinstance(items, list):
        return None
    for item in items:
        if not isinstance(item, dict) or str(item.get("rame_id", "")).lstrip("/") != frame.lstrip("/"):
            continue
        try:
            timeout = finite_number(item.get("timeoutSec", 0), "timeoutSec")
            if timeout < 0:
                raise ValueError("扫描字段 timeoutSec 不能为负数")
            if timeout > 0:
                raise ValueError(f"雷达报告超时: timeoutSec={timeout:g}")
            start = finite_number(item["angle_start"], "angle_start")
            increment = finite_number(item["increment"], "increment")
            minimum = finite_number(item["range_min"], "range_min")
            maximum = finite_number(item["range_max"], "range_max")
            if increment == 0 or minimum < 0 or maximum <= minimum:
                raise ValueError("扫描角增量或量程范围无效")
            scan = item["scan"]
            if not isinstance(scan, list) or not scan:
                raise ValueError("目标扫描为空或不是数组")
            if not math.isfinite(start + (len(scan) - 1) * increment):
                raise ValueError("扫描角度溢出")
            ranges = []
            for value in scan:
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    ranges.append(None)
                    continue
                distance = value * 0.001
                ranges.append(distance if math.isfinite(distance) and distance > 0
                              and minimum <= distance <= maximum else None)
            offset = item.get("laserOffset")
            if offset is not None:
                if not isinstance(offset, list) or len(offset) != 3:
                    raise ValueError("laserOffset 必须为 [x, y, yaw]")
                offset = tuple(finite_number(value, "laserOffset") for value in offset)
            return ScanPacket(str(item["rame_id"]), str(item.get("name", "")), tuple(ranges),
                              start, increment, minimum, maximum, offset)
        except KeyError as error:
            raise ValueError(f"目标扫描缺少字段: {error.args[0]}") from error
    return None
