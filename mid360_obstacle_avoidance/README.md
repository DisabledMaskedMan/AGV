# MID360 点云避障（ROS 2 Jazzy / C++）

快速使用与算法原理见 [工程说明](工程说明.md)（启动方式放在最前，含 RK3576 工程目录、编译、停车规则与在线调参说明）。

独立功能包，不修改 Livox 驱动。默认只启动 MID360 驱动和 C++ 检测节点，不启动图形窗口；可通过 `enable_roi_editor:=true` 开启 RViz2 点云显示和可拖动 ROI 编辑。

## 编译与启动

启动命令：
source /opt/ros/jazzy/setup.bash
source ~/ws_livox/install/setup.bash

ros2 launch mid360_obstacle_avoidance mid360_obstacle.launch.py \
  enable_web_editor:=true \
  enable_roi_editor:=false \
  params_file:=/home/neardi/ws_livox/src/mid360_obstacle_avoidance/config/obstacle.yaml \
  lidar_config:=/home/neardi/ws_livox/src/mid360_obstacle_avoidance/config/MID360_board.json

工作空间已有编译安装的 `livox_ros_driver2` 时：

```bash
cd /home/neardi/ws_livox
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select mid360_obstacle_avoidance --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch mid360_obstacle_avoidance mid360_obstacle.launch.py
```

构建时显式使用系统 Python，避免当前 Miniforge 环境的 Python 与 ROS 2 Jazzy 的 Python 3.12 不兼容。

如果驱动已经运行（必须输出 `livox_ros_driver2/msg/CustomMsg`，即 `xfer_format=1`），只启动检测节点：

```bash
ros2 launch mid360_obstacle_avoidance mid360_obstacle.launch.py start_driver:=false
```

也可直接运行 C++ 节点（使用代码中的默认参数）：

```bash
ros2 run mid360_obstacle_avoidance mid360_obstacle_node
```

默认使用驱动包的 `config/MID360_config.json`，当前主机 IP 为 `192.168.1.50`、雷达 IP 为 `192.168.1.143`。修改网络配置后需按驱动包原有方式重新构建安装；也可以直接传入配置文件的绝对路径：

```bash
ros2 launch mid360_obstacle_avoidance mid360_obstacle.launch.py \
  lidar_config:=/absolute/path/MID360_config.json
```

驱动默认合并点云，订阅 `/livox/lidar`。对接已经运行的多话题驱动时可指定输入话题，例如：

```bash
ros2 launch mid360_obstacle_avoidance mid360_obstacle.launch.py \
  start_driver:=false input_topic:=/livox/lidar_192_168_1_143
```

## RViz2 点云与 ROI 编辑

在板卡上安装依赖后编译本包；例如 ROS 2 Jazzy：

```bash
sudo apt install ros-jazzy-rviz2 ros-jazzy-interactive-markers \
  ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs ros-jazzy-visualization-msgs
ros2 launch mid360_obstacle_avoidance mid360_obstacle.launch.py \
  enable_roi_editor:=true
```

默认 `enable_roi_editor=false`。关闭时不创建 PointCloud2/ROI Marker 发布器或交互服务器，也不复制可视化点云。开关在启动时确定。RViz 需要板卡具有可用的图形会话。驱动已运行时可以同时传入 `start_driver:=false`。

RViz 自动加载 `rviz/mid360_roi_editor.rviz`，Fixed Frame 跟随 launch 的 `frame_id`：

1. 白色点云显示输入帧中所有有限且非全零点，包括 ROI 外的点；保留输入 Header 的时间戳，使用 `expected_frame` 坐标系，不做 TF 变换。消息校验失败的帧不发布可视化点云。
2. 半透明青色长方体是当前检测 ROI。选中工具栏的 **Interact**，拖动六个面中心的球形控制柄；红色控制 X、绿色控制 Y、蓝色控制 Z，对应 `roi_min_x/max_x`、`roi_min_y/max_y`、`roi_min_z/max_z`。每个柄只改变对应轴的一个边界。
3. 拖动立即修改内存 ROI，下一帧点云使用新范围。修改时清除旧障碍结果，等待下一帧重新判定；未超时时不因修改请求停车。每轴至少保留 `0.01 m`，边界不能交叉，`roi_min_x` 不能小于零；无效反馈被拒绝。
4. 右键中央黄色球，在菜单点击 **Save ROI to YAML** 才会保存。**Reload ROI from startup values** 恢复本次启动时的六个边界，不从磁盘重读，也不自动保存。

保存目标是本次 launch 的 `params_file`，默认是安装目录中的 `config/obstacle.yaml`。建议直接指定可写的标定文件：

```bash
ros2 launch mid360_obstacle_avoidance mid360_obstacle.launch.py \
  enable_roi_editor:=true params_file:=/absolute/path/obstacle.yaml
```

保存只修改当前节点 `ros__parameters` 下的六个 ROI 数值，保留聚类、超时、话题等其他参数、注释和换行格式；先写同目录临时文件，再替换原文件。使用 `--symlink-install` 时跟随配置文件符号链接更新源文件。目标文件必须已经存在，文件及所在目录必须可写；失败会输出 ROS 错误日志，不删除原文件。若安装目录只读，先复制配置到可写位置，再传入 `params_file`。写回支持本包采用的块式 YAML；缺少目标区段/ROI 键、重复键或不支持的 ROI 标量写法会拒绝保存。

节点参数仍是启动时只读；`ros2 param get` 显示启动值，当前编辑值以 detector_config / roi_state 及 RViz 标记为准。保存后重启可加载新值。launch 自动将实际配置路径传入 `roi_config_file`，默认 YAML 不存此路径。直接运行节点开启编辑资源时必须显式提供绝对路径，RViz 需另行启动：

```bash
ros2 run mid360_obstacle_avoidance mid360_obstacle_node --ros-args \
  --params-file /absolute/path/obstacle.yaml \
  -p enable_roi_editor:=true -p roi_config_file:=/absolute/path/obstacle.yaml
```

## 检测启停控制

默认 `detection_enabled_on_start: false`，启动后等待控制指令。状态持续到下一条指令或节点重启，不设置控制指令超时；重复相同指令不会清除当前结果。

```bash
# 开始检测（等待新帧；超过 cloud_timeout 无有效点云才请求停车）
ros2 topic pub --once /mid360_obstacle_avoidance/enable_detection std_msgs/msg/Bool '{data: true}'
# 停止检测，同时停止本节点的停车干预
ros2 topic pub --once /mid360_obstacle_avoidance/enable_detection std_msgs/msg/Bool '{data: false}'
# 查看实际启停状态，和 startup 参数的初始值区分
ros2 topic echo /mid360_obstacle_avoidance/detection_enabled std_msgs/msg/Bool --qos-durability transient_local
```

控制话题为 reliable、volatile、深度 1；重启不会重放之前的启动指令。实际状态话题为 reliable、transient-local、深度 1，节点启动、切换以及 10 Hz 心跳均发布，晚加入的网页能获取当前状态。需要启动即检测时，在 YAML 中显式设置 `detection_enabled_on_start: true`；直接运行可使用 `-p detection_enabled_on_start:=true`。

**停止检测意味着不请求停车**：即使点云无效或超时，`stop_required` 仍为 false。此时继续校验输入、输出可视化点云和心跳，ROI 编辑与保存不受影响，但跳过 ROI 聚类；障碍标志为 false、点数为 0、距离为 NaN。`data_valid` 仍独立描述点云健康情况，不能仅靠 `stop_required=false` 判断已检测安全。

重新开始时清除旧结果，等待下一帧有效点云，等待本身不请求停车。检测开启后仅在检测到合格障碍物或有效点云超时时请求停车。启停及 ROI 修改不伪造输入无效；输入 Header 保持最后收到的点云。网页分别显示检测启停、停车输出和点云健康，连接或状态心跳过期显示未知，不提供网页启停按钮。

## 检测规则与坐标

本版本对前方矩形 ROI 内的点进行空间聚类及尺寸过滤，作为避障输入，不直接发送底盘速度、转向或 PLC 指令，也不做路径规划。

- 输入坐标要求 **X 向前、Y 向左、Z 向上**，单位米。ROI 直接使用输入点云坐标；不执行 TF、地面分割或姿态补偿。
- 源码默认 ROI 为 `X ∈ [0.2, 1.7]`、`Y ∈ [-0.4, 0.8]`、`Z ∈ [-0.1, 0.5]`，包含边界。板卡 YAML 保存实测标定值，可能与源码默认值不同；当前值以 `config/obstacle.yaml` 为准。若需检测 2 m 处目标，需要扩大 ROI。
- 先忽略 NaN、Inf 和全零点，再筛选 ROI。**只有 ROI 内的有效点参与聚类**，ROI 外的点不能连接两个群。
- 两点三维欧氏距离不超过 `cluster_distance_threshold` 时相邻，通过空间哈希网格和广度优先搜索组成连通点群。相邻关系可传递：A 邻近 B、B 邻近 C 即属于同群，不要求 A 邻近 C；与点在输入数组中的顺序无关。网格边长等于距离阈值，只搜索本格及周围 26 格，并以实际距离确认连接。无需 PCL。
- 每群分别检查点数和轴对齐包围盒对角线。当前默认至少 10 个点且对角线至少 0.1 m 才接受；多个不合格群不会合计凑够点数。包围盒跨度为 `ΔX=maxX-minX`（Y、Z 同理），对角线为 `sqrt(ΔX²+ΔY²+ΔZ²)`。
- `min_cluster_volume > 0` 时，还必须满足 `ΔX×ΔY×ΔZ >= min_cluster_volume`。这是**点群包围盒体积，不是物体实际体积**；默认关闭，以避免把平面、薄板或杆件过滤掉。所有距离、点数和尺寸阈值均包含等号。几何计算使用实际 float32 输入坐标，不添加人为比较容差。
- **检测开启时，整个 ROI 都是停车区域**：只要至少一个群合格，`obstacle_detected=true`、`stop_required=true`。例如位于 2 m 的合格群，只要在配置的 ROI 内也会请求停车。采用单帧判断，没有跨帧累积、确认延迟或停车释放滞回。
- 检测开启且完成当前帧判定后，`nearest_distance` 为所有合格群中最小的 **X 坐标**，以雷达原点为基准，不是欧氏距离或车头净空距离。被过滤的点不会影响距离；没有合格群时为 `+inf`。
- `obstacle_points` 为所有合格群的点数总和；兼容字段 `stop_zone_points` 与它相同。
- 输入存在有限且非全零的回波即可视为有效，即使所有点都在 ROI 外或被点群过滤。此时 `data_valid=true`，没有合格群时不请求停车。噪声过滤不代表输入无效，也不证明前方覆盖充分。
- 停车规则为 `detection_enabled && (obstacle_detected || cloud_timed_out)`。从开启时开始使用单调时钟计时，有效点云以接收时刻刷新参考时间。空帧、全部点无效、数量或坐标系错误会清除旧结果、置 `data_valid=false`、距离为 `NaN`，但未超时不停车；持续异常最终超时停车。重复 true、ROI 或算法修改均不刷新计时。修改 `cloud_timeout` 按已有间隔重新判断；有效帧恢复后重新决定停车或释放。
- 检测状态与启停状态按每帧发布，并以 10 Hz 心跳发布；超时使用单调时钟检测，超时停车请求最多再增加约 100 ms 检查周期。输出使用 reliable + transient-local QoS；输入使用 best-effort、深度 1。

### 聚类参数

在 `config/obstacle.yaml` 的 `ros__parameters` 下配置，所有参数保持启动时只读：

| 参数 | 源码默认值 | 单位 / 含义 |
| --- | --- | --- |
| `cluster_distance_threshold` | `0.05` | m，相邻点最大三维距离，必须有限且大于 0 |
| `min_obstacle_points` | `10` | 每群最少点数，整数范围 1～4294967295 |
| `min_cluster_diagonal` | `0.1` | m，包围盒最小对角线，必须有限且非负 |
| `min_cluster_volume` | `0.0` | m³，可选包围盒最小体积，有限且非负；0 关闭 |

板卡当前 YAML 的相邻距离已标定为 `0.1 m`，和上表的源码默认值不同；本地 config 已同步板卡标定值。

参数只是调试起点，需要用实测噪声与最小目标障碍物标定。密集噪声仍可能通过；相邻距离过大会把点串接成大群，过小会把稀疏目标拆散。减小尺寸阈值可以保留更小的障碍物，但也会降低噪声过滤能力。本版本不增加地面分割、姿态补偿或时间滤波。

**升级兼容性**：原 `stop_distance` 参数已移除，请从自定义 YAML、启动命令和外部配置中删除它，并通过 ROI 控制停车范围。`min_obstacle_points` 从“整个 ROI 的点数”改为“每个群的点数”；话题名及消息字段布局不变，但点数与距离只统计合格群。

根据雷达安装高度、倾角、车体宽度和车头到雷达的偏移调整 `config/obstacle.yaml`。地面与车体应排除在 ROI 外，且 `roi_min_x` 内存在未检测区域。默认值仅供调试，需要实车标定。下游控制器还应监测检测节点的状态心跳是否中断。

`expected_frame` 默认 `livox_frame`。启动参数 `frame_id` 会同时设置驱动 frame 标签与检测节点期望名称，**改名字不会旋转或平移坐标**。所有 ROS 参数为启动时只读；编辑模式单独维护可变 ROI 和算法配置；通过服务提交，ROS 启动参数不动态改写。手动调整 YAML 后需重启节点。可使用 `params_file:=/absolute/path/obstacle.yaml` 指定参数文件；通过 launch 启动时，`input_topic`、`expected_frame` 和 `enable_roi_editor` 分别以同名 launch 参数（坐标系为 `frame_id`）为准。

## 输出接口

```bash
ros2 topic echo /mid360_obstacle_avoidance/status
ros2 topic echo /mid360_obstacle_avoidance/stop_required
```

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/mid360_obstacle_avoidance/status` | `mid360_obstacle_avoidance/msg/ObstacleStatus` | 输入帧 Header、数据有效性、障碍物标志、停车请求、合格群最近 X、合格群总点数及兼容计数字段 |
| `/mid360_obstacle_avoidance/stop_required` | `std_msgs/msg/Bool` | `true` 表示请求停车，`false` 表示本节点未请求停车 |
| `/mid360_obstacle_avoidance/detection_enabled` | `std_msgs/msg/Bool` | 实际检测启停状态，reliable + transient-local、深度 1、10 Hz 心跳 |
| `/mid360_obstacle_avoidance/pointcloud` | `sensor_msgs/msg/PointCloud2` | 仅编辑模式：完整有效点云，float32 XYZ，best-effort、深度 1 |
| `/mid360_obstacle_avoidance/roi_marker` | `visualization_msgs/msg/Marker` | 仅编辑模式：当前 ROI，reliable + transient-local |
| `/mid360_obstacle_avoidance/roi_editor/*` | 交互标记协议 | 仅编辑模式：六个控制柄及保存/恢复菜单 |

状态的 Header 保留最近收到的输入帧时间戳与坐标系；启动前时间戳为零。超时时 Header 不变，必须结合 `data_valid` 判断，不能把无效数据解释为道路畅通。

## 测试

```bash
cd /home/neardi/ws_livox
source install/setup.bash
colcon test --packages-select mid360_obstacle_avoidance --event-handlers console_direct+
colcon test-result --test-result-base build/mid360_obstacle_avoidance --verbose
```

测试包括：

- 独立 C++ 几何测试：相邻距离/点数/对角线/体积边界、链式连接、重复坐标、负坐标网格、输入顺序、多群独立过滤，以及与全点对参考算法的随机比对。
- 独立 C++ ROI 测试：六个边界拖动、交叉钳制、最小尺寸、非法坐标、限定节点的六键写回、其他参数/注释/换行保留，以及文件不存在、只读和替换失败时保护原文件。
- ROS 集成测试：显式指定 ROI、聚类参数和启动检测状态，覆盖分散噪声、尺寸不足、2 m 处停车、ROI 边界、ROI 外点不能桥接、异常点、无效帧、开启后无有效点云超时停车、心跳、超时、恢复及非法参数。
- 启停与 Web 测试：源码默认暂停、true/false 切换、重复指令、新帧门控、暂停时无效/超时不停车、晚加入状态、重启不重放历史启动指令，以及暂停期间的点云二进制传输、ROI 编辑/保存和 WebSocket 启停状态。
- 编辑器 ROS 集成测试（无需 RViz 窗口）：关闭模式无额外资源、PointCloud2 的 Header/字段及 ROI 外有效点、六个控制柄实时生效、跨边界钳制、异常反馈、显式保存和启动值恢复。

无需雷达硬件或桌面环境。安装 Node.js 的开发机可运行 `node test/test_web_state.mjs` 验证网页的 9 种状态渲染；板卡运行网页不需要 Node.js。没有 ROS 环境时，可单独运行纯 C++ 测试：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Isrc \
  test/test_cluster_detection.cpp -o /tmp/test_cluster_detection
/tmp/test_cluster_detection
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Isrc \
  test/test_roi_editor.cpp -o /tmp/test_roi_editor
/tmp/test_roi_editor
```

当前 Windows 环境不要求启动 ROS 2/RViz2。ROS 节点的完整编译、集成测试和图形交互留到具备依赖的板卡上执行。板卡验收依次检查：实时点云及坐标系、六个边界拖动后的检测变化、菜单保存、重启加载、普通模式无可视化资源；最后使用真实噪声和目标障碍物标定 ROI、聚类距离、点数及尺寸阈值。

纯 C++ 测试会打印合成密集点群的处理耗时，仅供检查算法退化，不能替代实测点云和目标硬件的性能验证。部署时用代表性点云测试稀疏/密集场景、噪声和最大输入点数，测量整帧处理耗时及最大值，确认满足 10 Hz 输入对应的 100 ms 周期，并留出 ROS 通信和调度余量。


## Web 点云与 ROI 编辑（无需板卡桌面）

浏览器采用离线 Three.js 0.180.0，许可证位于 `web/vendor/LICENSE`；板卡无需 Node.js 或 CDN。
安装依赖 `sudo apt install python3-aiohttp python3-numpy`。Windows 复制到 Linux 后，symlink-install 前执行 `chmod +x scripts/mid360_web_editor.py`。

```bash
source /opt/ros/jazzy/setup.bash
source /home/neardi/ws_livox/install/setup.bash
ros2 launch mid360_obstacle_avoidance mid360_obstacle.launch.py   enable_web_editor:=true enable_roi_editor:=false   params_file:=/home/neardi/ws_livox/src/mid360_obstacle_avoidance/config/obstacle.yaml   lidar_config:=/home/neardi/ws_livox/src/mid360_obstacle_avoidance/config/MID360_board.json
```

在 VS Code Remote-SSH 的“端口”面板添加板卡 **8080**，打开面板提供的本地地址。
默认 `web_host=127.0.0.1`、`web_port=8080`，只监听板卡回环，不配置开机自启。
Web 界面适用于可信 SSH 隧道，没有公网认证，不要直接暴露到公网。

- 界面显示全部有限非零点云（含 ROI 外点）、半透明 ROI、坐标系、六个边界、点云时间戳、接收年龄和停车状态；不进行 TF 变换。
- 左键旋转、右键平移、滚轮缩放。拖动六个彩色轴柄或输入数值，每次仅修改对应边界；每轴至少 0.01 m，X 下界不小于 0。检测立即采用返回的实际范围。
- 拖动不自动保存。“保存 ROI 到 YAML”等待最后一次拖动响应后，调用 C++ 现有原子写回，目标为当前 params_file，仅修改六个 ROI 键，保留其他参数和注释。缺失、只读或替换失败会明确报错并保留原文件。
- “恢复启动值”恢复本次启动时的范围，不自动保存。只读 ROS 参数仍表示启动值，当前边界以 roi_state 为准。ROI 修改后清除旧结果，下一有效点云重新判定；不延后超时、不因修改本身停车。
- 独立 Python rclpy + aiohttp 进程使用 WebSocket 二进制 little-endian float32 XYZ。仅保留最新帧，慢连接跳帧或断开，不阻塞检测线程；页面明确显示断线/超时，旧点云淡出。
- `enable_web_editor` 和 `enable_roi_editor` 可独立开启或共存。两者均 false 时没有编辑器发布器、服务、Web 进程或 RViz。

任一编辑模式开启时增加以下接口，前缀为 `/mid360_obstacle_avoidance/`：

| 接口 | 类型 | 语义 |
| --- | --- | --- |
| roi_state | msg/RoiState | transient-local，当前实际内存边界、frame、revision、cloud_timeout |
| set_roi_boundary | srv/SetRoiBoundary | 修改一个 roi_min/max_x/y/z，返回实际 ROI 和结果 |
| save_roi | std_srvs/srv/Trigger | 显式保存当前 ROI |
| reset_roi | std_srvs/srv/Trigger | 恢复启动值，不落盘 |

专用 `config/MID360_board.json` 将 LiDAR `192.168.1.143` 的接收地址设为 eth0 `192.168.1.5`，保留原驱动配置和网卡设置。
该板卡实测雷达广播的设备类型为 `35`，对应已安装 Livox SDK 的 `Mid360s`；专用 JSON 使用区分大小写的 `Mid360s` 区段。普通 MID360 的类型为 `9`，应使用匹配该设备的 `MID360` 区段，不能仅凭产品外观或初始化成功日志判断型号匹配。更换雷达时应重新核对广播设备类型与 SDK 支持情况。
如果网页没有点云，先检查 `ros2 topic info /livox/lidar -v` 的 Publisher count，再检查 `ros2 topic hz /livox/lidar`。仅有订阅者也会使话题出现在列表中；`Init lds lidar success!` 只代表驱动初始化。若网口已有雷达 UDP 包、ROS 发布者仍为 0，需要检查驱动设备发现及型号配置，而不是调整 ROI。修改 JSON 后需要重启驱动。
`detector_log_level:=mid360_obstacle_avoidance:=debug` 可输出单帧 `cloud_processing_us`（微秒，含 C++ 过滤、聚类、转换和发布，不含异步 Web 编码及网络），平时保持 info。

Web 集成测试覆盖完整有效点云二进制传输、行/点填充和大小端、六面修改及钳制、非法输入、显式保存、只读文件保护、恢复与独立节点重启读取已保存配置。
本次板卡部署验收记录见 [docs/DEPLOYMENT_VALIDATION.md](docs/DEPLOYMENT_VALIDATION.md)。

## Web 在线调参与话题面板

保持原网页启动方式，转发板卡 8080 后访问，无需 Node.js、CDN 或图形桌面。网页显示中文名称、英文参数名及单位：

| 中文名称（英文参数名） | 单位及范围 |
| --- | --- |
| X 最小/最大边界（roi_min_x / roi_max_x） | m，X 最小值 ≥ 0 |
| Y 最小/最大边界（roi_min_y / roi_max_y） | m |
| Z 最小/最大边界（roi_min_z / roi_max_z） | m，每轴长度至少 0.01 |
| 相邻点最大距离（cluster_distance_threshold） | m，有限且 > 0，须兼容当前 ROI 网格范围 |
| 每群最少点数（min_obstacle_points） | 点，整数 1～4294967295 |
| 最小点群对角线（min_cluster_diagonal） | m，有限且 ≥ 0 |
| 最小包围盒体积（min_cluster_volume） | m³，有限且 ≥ 0；0 关闭 |
| 点云超时（cloud_timeout） | s，有限且 > 0 |

ROI 拖动或数值修改立即生效、不自动落盘。“保存 ROI 到 YAML”仅保存六边界；“恢复启动值”仅恢复本次启动 ROI，不恢复算法参数、不自动保存。

五个算法输入框为待应用修改，下方显示当前生效值；状态刷新不会覆盖正在编辑的内容。点击“应用并保存参数”时，C++ 整组校验五个候选值，与服务执行时实际六边界一起原子保存到当前 params_file；成功后一次提交内存，每帧使用一致快照。失败时算法保持原值，此前拖动 ROI 保持当前状态。只修改十一项，保留其他参数、注释、权限和换行；并发修改、缺键/重复键或不可写均拒绝保存。

ROS 启动参数继续只读，`ros2 param get` 仍显示启动值；服务修改内部配置。修改 C++/消息接口需要编译和重启；网页调参本身无需编译或重启。

| 接口（前缀 /mid360_obstacle_avoidance/） | 类型 | 说明 |
| --- | --- | --- |
| detector_config | mid360_obstacle_avoidance/msg/DetectorConfig | 实际六边界、五算法值及 revision；reliable / transient-local / depth 1 |
| apply_detector_config | mid360_obstacle_avoidance/srv/ApplyDetectorConfig | 五候选值；响应含成功/错误及实际配置 |

两个编辑模式均关闭时不创建上述编辑器接口或可视化资源。话题名、坐标系、Web/RViz 开关和启动检测开关仍为启动配置；没有网页启停按钮。

话题面板显示 status 全部七字段（含完整 Header）、独立 stop_required / detection_enabled 的实际 Bool、roi_state / detector_config 配置及版本、PointCloud2 帧头/字段/点数/字节摘要、ROI Marker 帧头/姿态/尺寸。每项显示名称、类型、接收时间、年龄及消息数；区分未收到、连接断开、消息过期；配置与标记不按心跳过期。距离区分有限值、+inf 和 NaN。暂停明确显示不请求停车，停车原因区分障碍物和有效点云超时，等待新帧和输入异常单独显示。

入口及 JavaScript 响应使用 Cache-Control: no-cache 重新验证缓存；升级后重启原 launch 并刷新网页。若旧页面尚未更新，使用浏览器强制刷新。
