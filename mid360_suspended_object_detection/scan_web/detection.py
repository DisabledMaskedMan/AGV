import math
import os
import tempfile
from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path

import yaml


@dataclass(frozen=True)
class DetectionConfig:
    min_x: float = 0.3
    max_x: float = 5.0
    min_y: float = -1.0
    max_y: float = 1.0
    cluster_distance: float = 0.12
    min_cluster_points: int = 8
    max_cluster_points: int = 100000
    min_width: float = 0.08
    max_width: float = 2.0
    min_depth: float = 0.08
    max_depth: float = 3.0
    cloud_timeout: float = 1.0

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{field.name} 必须是有限数值")
        for name in ("min_cluster_points", "max_cluster_points"):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f"{name} 必须是整数")
        if self.min_x >= self.max_x or self.min_y >= self.max_y:
            raise ValueError("ROI 最小值必须小于最大值")
        if self.cluster_distance <= 0 or self.cloud_timeout <= 0:
            raise ValueError("聚类距离和超时必须大于零")
        if self.min_cluster_points < 1 or self.max_cluster_points < self.min_cluster_points:
            raise ValueError("聚类点数范围无效")
        if self.min_width < 0 or self.max_width < self.min_width:
            raise ValueError("宽度范围无效")
        if self.min_depth < 0 or self.max_depth < self.min_depth:
            raise ValueError("深度范围无效")

    def updated(self, changes):
        if not isinstance(changes, dict):
            raise ValueError("参数必须是 JSON 对象")
        unknown = changes.keys() - asdict(self).keys()
        if unknown:
            raise ValueError("未知参数: " + ", ".join(sorted(unknown)))
        return replace(self, **changes)


def load_config(path):
    with Path(path).open(encoding="utf-8") as source:
        document = yaml.safe_load(source)
    parameters = document["mid360_suspended_object_detection"]["ros__parameters"]
    names = {field.name for field in fields(DetectionConfig)}
    return DetectionConfig(**{name: value for name, value in parameters.items() if name in names})


def save_config(path, config):
    target = Path(path).resolve(strict=True)
    with target.open(encoding="utf-8") as source:
        document = yaml.safe_load(source)
    parameters = document["mid360_suspended_object_detection"]["ros__parameters"]
    if not isinstance(parameters, dict):
        raise ValueError("配置缺少 ros__parameters")
    parameters.update(asdict(config))
    content = yaml.safe_dump(document, allow_unicode=True, sort_keys=False)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=target.parent,
                                         prefix=".scan-web-", delete=False) as output:
            temporary = Path(output.name)
            os.chmod(temporary, target.stat().st_mode & 0o777)
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, target)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def detect(points, config):
    roi = [
        point for point in points
        if config.min_x <= point[0] <= config.max_x and config.min_y <= point[1] <= config.max_y
    ]
    parents = list(range(len(roi)))
    ranks = [0] * len(roi)

    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    grid = {}
    distance_squared = config.cluster_distance ** 2
    for index, (point_x, point_y) in enumerate(roi):
        cell_x = math.floor(point_x / config.cluster_distance)
        cell_y = math.floor(point_y / config.cluster_distance)
        for delta_x in (-1, 0, 1):
            for delta_y in (-1, 0, 1):
                for neighbor in grid.get((cell_x + delta_x, cell_y + delta_y), ()):
                    other_x, other_y = roi[neighbor]
                    if (point_x - other_x) ** 2 + (point_y - other_y) ** 2 > distance_squared:
                        continue
                    first_root, second_root = root(index), root(neighbor)
                    if first_root == second_root:
                        continue
                    if ranks[first_root] < ranks[second_root]:
                        first_root, second_root = second_root, first_root
                    parents[second_root] = first_root
                    if ranks[first_root] == ranks[second_root]:
                        ranks[first_root] += 1
        grid.setdefault((cell_x, cell_y), []).append(index)

    groups = {}
    for index, point in enumerate(roi):
        groups.setdefault(root(index), []).append(point)
    objects = []
    for group in groups.values():
        if not config.min_cluster_points <= len(group) <= config.max_cluster_points:
            continue
        min_x = min(point[0] for point in group)
        max_x = max(point[0] for point in group)
        min_y = min(point[1] for point in group)
        max_y = max(point[1] for point in group)
        width, depth = max_y - min_y, max_x - min_x
        if not (config.min_width <= width <= config.max_width
                and config.min_depth <= depth <= config.max_depth):
            continue
        objects.append({
            "min_x": min_x, "max_x": max_x, "min_y": min_y, "max_y": max_y,
            "width": width, "depth": depth, "count": len(group),
            "distance": min(math.hypot(*point) for point in group),
            "center_x": (min_x + max_x) / 2, "center_y": (min_y + max_y) / 2,
        })
    objects.sort(key=lambda box: box["distance"])
    return {"roi_count": len(roi), "objects": objects}


def largest_obstacle(objects):
    return min(objects, key=lambda box: (-box["width"] * box["depth"], box["distance"],
        box["center_x"], box["center_y"], box["min_x"], box["max_x"], box["min_y"], box["max_y"], box["count"]), default=None)
