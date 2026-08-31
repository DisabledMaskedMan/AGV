"""FirstAutoTestV8 模块包。

具体职责分别位于 ``config``、``lidar``、``table_api``、``safety``、
``route_control`` 和 ``workflow``；程序入口位于 ``main``。导入本包不会
初始化 ROS、连接 WebSocket 或向 AGV 下发命令。
"""

__all__ = ["config", "lidar", "table_api", "safety", "route_control", "workflow", "main"]
