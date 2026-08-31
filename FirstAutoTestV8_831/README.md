# FirstAutoTestV8 模块说明

`../FirstAutoTestV8.py` 是兼容入口，实际实现位于本目录。

模块职责：

- `config.py`：参数、固定点位、AGV 外形
- `lidar.py`：雷达读取、坐标转换、RANSAC、车厢角点检测
- `table_api.py`：WebSocket TABLE 读写、位姿、急停、货叉
- `route_control.py`：路线构建、启动、等待到站、右墙闭环
- `workflow.py`：一次完整的取货/放货/返回流程
- `main.py`：程序入口和 ROS2 初始化
- `runtime.py`：ROS2 运行时句柄
- `safety.py`：侧向雷达安全检测公共接口

启动方式：

```bash
python FirstAutoTestV8.py [websocket_url]
python -m agv_base_driver.FirstAutoTestV8.main [websocket_url]
```

建议先在仿真或断开驱动输出的环境中验证，再连接真实 AGV。

扩展多个货箱时，在 `workflow.py` 中使用 `BoxTask` 描述每个货箱的点位，使用
`run_multi_box_workflow(ws, tasks)` 按顺序执行。横向偏移按照第二次准确运行的现场右侧方向配置；
当前三箱配置使用 `THREE_BOX_RIGHT_OFFSETS = (0.0, 2.1, 1.05)`，
这些偏移只用于以首箱精确 `POINT_20` 生成后续箱位，不作为右墙距离。
首箱进库时仍按原流程测量 `adjust_distance`；降叉后的返回路线现在统一使用普通 TABLE 路线，
不再使用返回 PID。
地图坐标的 `y` 不能直接当作车辆左右方向。`main.py` 只负责生成任务列表和启动流程。

完整三箱流程中，第一箱保留原有 `POINT_18 -> POINT_20` 右墙 PID。
第二箱及后续箱子使用一次性进库路线 `POINT_5 -> POINT_16 -> POINT_17 -> POINT_18 -> POINT_20`。
所有箱子降叉完成后都使用普通路线 `POINT_20 -> POINT_18 -> POINT_17 -> POINT_16 -> POINT_5`，
在 `POINT_5` 停车后再单独发送 `POINT_5 -> POINT_4`，不使用返回 PID。
第一箱第一次到达 `POINT_4` 后先执行相机或人工装货确认，再读取当前位姿，
经过 `POINT_4` 到 `POINT_5` 并停车，随后发送 `POINT_5 -> POINT_16 -> POINT_17 -> POINT_18`，
最后保留 `POINT_18 -> POINT_20` 右墙 PID。
当前位姿到 `POINT_5` 的路线节点为 `[当前位姿, POINT_4, POINT_5]`，
其中 `POINT_4` 和 `POINT_5` 使用 TABLE 的路线类型 `2`（直线+圆弧）。
返回路线 `POINT_5 -> POINT_4` 同样使用路线类型 `2`。

第二箱的 `POINT_16 -> POINT_17` 和 `POINT_17 -> POINT_18` 均使用路线类型 `2`；
第二箱 `POINT_17` 的航向按 `POINT_17 -> POINT_18` 的路线方向加 180 度计算，
`POINT_18` 与 `POINT_20` 使用相同的横向位置。

## 单箱测试

在 `config.py` 修改 `CONFIG["standalone_box_index"]`：

- `0`：完整三箱流程（默认）
- `1`：只执行第一箱完整流程
- `2`：独立执行第二箱
- `3`：独立执行第三箱

独立执行第二或第三箱时，启动前车辆停在车厢前扫描位置，货箱由人工放到
货叉上并托起。程序扫描车厢后先按 `扫描起点 -> POINT_4 -> POINT_5` 行驶并在
`POINT_5` 停车，再一次性下发 `POINT_5 -> POINT_16 -> POINT_17 -> POINT_18 -> POINT_20`。
降叉后分两段返回：`POINT_20 -> POINT_18 -> POINT_17 -> POINT_16 -> POINT_5`，
停车后再执行 `POINT_5 -> POINT_4`。启动命令保持不变：

```bash
python FirstAutoTestV8.py
```

也可以使用模块方式启动：

```bash
python -m agv_base_driver.FirstAutoTestV8.main
```
