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
`run_multi_box_workflow(ws, tasks)` 按顺序执行。`main.py` 只负责生成任务列表和启
动流程。
