"""运行时依赖，由入口初始化，其他模块只读取这里的句柄。"""

ros_node = None
laser_publisher = None
camera_subscription = None
websocket_connection = None


def publish_navigation_status(value):
    """向导航状态话题发布消息；ROS 依赖只在实际运行时加载。"""
    if laser_publisher is None:
        return
    from std_msgs.msg import String

    message = String()
    message.data = value
    laser_publisher.publish(message)


def spin_once():
    """处理一次 ROS 回调。"""
    import rclpy

    rclpy.spin_once(ros_node, timeout_sec=None)
