# MID-360 悬挂物检测（ROS 2 Jazzy / C++）

常用启动步骤、板端目录、编译方式和算法原理见 [工程说明](工程说明.md)。

此包订阅 `livox_ros_driver2/msg/CustomMsg`，在雷达坐标系的前上方 ROI 中进行欧氏聚类，并用点数及包围盒长宽高过滤候选物体。检测到候选物时发布布尔状态和 RViz 包围盒。该实现提供几何候选检测，不包含机器学习语义分类；环境中的横梁、树枝等也可能成为候选，需要按现场调整 ROI 和尺寸。

## 构建与启动

```bash
cd ~/ws_livox
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mid360_suspended_object_detection --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch mid360_suspended_object_detection suspended_object.launch.py
```

要求工作空间已构建并 source `livox_ros_driver2`。驱动应发布 `/livox/lidar` 的 `CustomMsg`（通常 `xfer_format=1`）。检测输入使用 X 向前、Y 向左、Z 向上的雷达坐标；默认 Z 区间为 1.2～3.5 m，必须根据雷达安装高度、俯仰角和地面坐标标定，节点不会进行 TF 或地面姿态补偿。

## 输出

- `/mid360_suspended_object_detection/detected` (`std_msgs/Bool`)：每帧检测状态。
- `/mid360_suspended_object_detection/markers` (`visualization_msgs/MarkerArray`)：候选物体的红色三维包围盒，可在 RViz 添加 MarkerArray 显示。

点云停止超过 `cloud_timeout` 后会清除状态并删除旧标记。无效帧按未检测发布；下游系统如需安全联锁，应同时监控点云是否持续到达。

主要参数见 `config/detection.yaml`：`min_x/max_x`、`min_y/max_y`、`min_z/max_z` 定义 ROI；`cluster_distance` 为聚类距离；`min_cluster_points/max_cluster_points` 约束点数；`min_width/max_width`、`min_height/max_height`、`min_depth/max_depth` 限定包围盒尺寸（宽为 Y、深为 X）。可通过 ROS 参数文件替换配置。

## Windows 无线二维扫描 Web 测试

本机工具使用板卡封装的 WebSocket 接口，不需要安装 ROS 2，也不需要导入外部工程的 `table_api.py`。现有 ROS 2 三维节点继续使用原启动方式。

使用 Python 3.10 或更新版本，在工程目录执行：

```powershell
python -m pip install -r requirements.txt
python -m scan_web
```

浏览器打开 <http://127.0.0.1:8765>。默认通过无线连接 `ws://192.168.20.199:6060`，发送 `{"channel":"ros","op":"subscribe_scan","topic":"/scan"}`；只选择 `rame_id` 为 `scan_link` 或 `/scan_link` 的记录。首次状态消息和其他雷达记录会被跳过。按 `Ctrl+C` 停止服务。

通过 ROS 2 的 `scan_web.launch.py` 启动 C++ 扫描 Web 后端时，会同时发布 `/mid360_suspended_object_detection/largest_obstacle`（可用 launch 参数 `obstacle_topic:=...` 修改）。直接运行 Python `python -m scan_web` 不创建 ROS 节点。消息类型为 `mid360_suspended_object_detection/msg/LargestObstacle`，每 100 ms 发布一次。最大障碍物按包围盒宽×深的面积选择；面积相同时优先最近扫描点更近者，再按中心坐标排序。

消息的 `center.x/y`、`distance_x/y` 是雷达局部坐标下包围盒中心位置，单位米且保留正负号（X 向前、Y 向左）；例如 `distance_x=2.0, distance_y=-0.5` 表示前方 2 m、右侧 0.5 m。`center_distance` 是中心到雷达原点的平面直线距离；`nearest_point` 和 `distance` 给出该候选最近的实际扫描点及其距离。`width` 沿 Y、`depth` 沿 X，`area` 为平方米。二维消息中点的 Z 固定为 0，不能解释成测得高度。`header.stamp` 为发布时间，数据源未提供采集时间戳。

`header.frame_id` 为去掉前导 `/` 的雷达坐标系（默认 `scan_link`），不应用车体外参或 `laserOffset`。`candidate_count` 是通过筛选的候选数，`point_count` 是最大候选的实际扫描点数。`status/error` 与网页后端状态及异常原因一致。`receive_age_s` 为距最近一次有效目标扫描接收的秒数，`result_age_s` 为距分析完成的秒数，不可用时为 -1；它们不是传感器采集时间或端到端延迟。距离、尺寸和年龄使用 `float64`。在线 ROI 更新立即使旧结果失效，等待重新分析；后续扫描分析期间沿用尚未过期的上一已完成结果。

`data_valid=true, detected=false` 表示扫描有效但没有通过筛选的候选；`data_valid=false` 表示等待/分析中、超时、断线或数据异常，这些情况下障碍物字段清零。可用下面命令订阅并查看消息：

```bash
ros2 topic echo /mid360_suspended_object_detection/largest_obstacle
ros2 interface show mid360_suspended_object_detection/msg/LargestObstacle
```

通过 `publish_obstacle:=false` 可关闭该发布器；通过 `obstacle_topic:=/your/topic` 修改话题名。Web 服务仍绑定回环地址并通过 VS Code 转发访问。

有线连接时切换地址：

```powershell
python -m scan_web --ws-url ws://192.168.31.7:6060
```

可选参数：`--frame /scan_link`、`--port 8766`、`--host 127.0.0.1`、`--config config/scan_web.yaml`。默认仅本机访问，不向车辆发送控制或 TABLE 写入命令。

### 页面与坐标

- 当前分析选中的最大障碍物用红框标注“最终障碍物”，其余候选保持黄框；红框最后绘制，候选表同步标记对应行。C++ 与 Python 均由后端选择，不在浏览器重复排序。ROI 更新、无候选、结果失效时清除旧框。
- 候选表下的“ROS 2 发布”面板显示解析及 remap 后的实际话题名、消息类型、发布状态、累计次数、最近成功发布年龄及最新消息摘要；“完整消息”可展开查看全部原始数值和发布时间。正常无障碍显示实际零值，尚未发布显示“—”。
- “后端实际发布频率”按最近 5 秒成功返回的 `publish()` 事件计算，至少两条才有频率；超过 1 秒没有成功发布显示 0 Hz 和停滞状态（有发布错误时显示错误）。该频率不代表订阅者接收频率。发布失败保留最近成功消息，后续周期继续尝试。
- 红框表示当前有效分析，发布面板表示最近一次实际发布，两者可能因 100 ms 发布周期短暂不同。关闭发布显示“ROS 发布已关闭”；Python 与独立非 ROS C++ 显示“不支持 ROS 发布”，不生成模拟消息。网页连接中断时清空实时框、消息和频率，连接恢复后自动更新。
- 俯视图使用 X 向前、Y 向左，单位 m；扫描距离从 mm 换算为 m，角度使用 rad。零值、非有限值以及量程外读数不参与检测。
- 页面显示全部有效扫描点、ROI 和候选包围盒。框内拖动移动 ROI，四角和四边中点共八个控制柄调整区域；框外拖动画布，滚轮或 `＋/−` 缩放。“定位 ROI”和“全部点”主动调整视野。
- 拖动实时更新区域预览与 XY 输入框，松手自动提交四个 ROI 参数，视野保持不变。支持负坐标，拖动最小宽深为 0.05 m；控制柄命中范围按屏幕像素计算。`Esc`、指针取消或捕获丢失恢复原区域；提交失败恢复已确认值并显示原因。轮询不覆盖拖动草稿，提交期间禁止重叠修改。
- 左侧可修改 XY 范围、聚类距离、点数、宽深阈值和接收超时。拖动 ROI 松手只在内存生效，提示“已应用，尚未保存”；点击“应用并保存”保存当前表单全部参数，立即生效且重启后保留。“恢复启动值”只填回本次启动时读取的值，需要再次点击“应用并保存”。
- C++ 和 Python 二维后端默认读取独立的 `config/scan_web.yaml`；原三维节点继续读取 `config/detection.yaml`。使用 `--config 路径`（launch 使用 `config:=/绝对路径/参数.yaml`）时，保存目标是该指定文件。服务解析符号链接后原子替换真实目标，兼容 `--symlink-install`；保存保留其他 YAML 字段的值，但注释和排版可能改变。写入失败不会改变文件或已确认的内存配置，页面显示原因并恢复已确认参数。
- 二维检测忽略 Z、高度、ROS 输入话题及 `expected_frame`。`laserOffset` 仅展示，不应用坐标变换；本次不能由 frame 名独立确认物理传感器身份。若主动用 `--config` 指向三维配置，保存的 XY 等共用参数会影响三维节点下次启动，建议使用默认独立文件。
- 初始 ROI 为 X `[0.3, 5]`、Y `[-1, 1]`。若 ROI 内为零点，先点“全部点”观察，再调整检测区域；零点不等于雷达连接失败。

**本接口只提供二维距离，没有 Z，因此只能检测平面障碍物候选，不能判断物体高度或是否悬挂。** 需要悬挂物检测时，仍需将真实 XYZ 点云接入原三维节点。

### 数据状态与接口

`GET /api/state` 新增 `largest_obstacle`（完整候选包围盒或 `null`）和 `ros_publication`（`supported/enabled/topic/message_type/status/frequency_hz/published_count/last_publish_age_s/error/message`）。`message` 为最近成功发布的完整 ROS 消息；未发布时为 `null`。发布缓存和分析快照各自加锁，不新增订阅器或重复聚类。

持久连接由接收线程持续读取，只保留最新帧；检测频率最多 10 Hz，重复扫描不重复分析。页面每 200 ms 查询最新状态，不积压历史帧。参数修改也会触发重新分析。

目标记录超过 `cloud_timeout` 未到达、记录格式异常或 `timeoutSec > 0` 时，清除候选并显示异常；连接中断后按 1、2、5 秒间隔重试，后续持续失败间隔为 5 秒。连接状态、等待目标扫描、分析中、正常未检出、检出候选和异常分别展示。浏览器与本机服务断开时也会清除显示的结果。

板卡可能重复推送相同内容；相同内容仍会刷新本机接收时间，不会单独作为失效依据。页面分别统计最近 5 秒的接收频率与内容变化频率。源端没有提供采集时间戳，因此本机接收正常不代表上游采集一定新鲜。

| 接口 | 行为 |
| --- | --- |
| `GET /api/state` | 状态、frame、以 m 为单位的 XY 点、ROI 点数、候选框、频率、接收间隔、累计帧数和处理耗时。`source_timestamp` 为 `null`。 |
| `GET /api/config` | 当前二维检测参数。 |
| `POST /api/config` | `application/json` 参数对象，支持部分更新；成功返回完整参数，无效请求返回 400 且保持原参数。 |
| `POST /api/config/save` | 相同参数对象并支持部分更新；合并校验后将全部二维参数原子写入启动时的 YAML，成功再应用到内存并返回完整参数。无效请求返回 400，文件写入失败返回 500，失败不修改已确认配置。 |

### 自动测试与现场验收

```powershell
python -m unittest discover -s tests -v
```

测试覆盖消息解析、单位转换、frame、过滤、二维聚类与包围盒、超时清除、重复帧、重连、并发参数更新和 HTTP 接口。合成扫描通过不代表现场物体已实测检出。真实验收时，调整 ROI 使扫描点进入区域，放置已知障碍物并核对位置、尺寸和候选框；移走后确认候选消失。当前 Windows 环境没有 ROS 2，不能在此编译验证原 C++ 节点。

## RK3576 C++ 后端与 VS Code 端口转发

C++17 后端与 Python 基线共用 `scan_web/static` 网页和相同的 API（包括独立保存接口）。接收、检测和 HTTP 服务分别运行，保留最新扫描、限制分析至 10 Hz；缓存角度投影参数与分析结果的 JSON，重复扫描不重新分析。接口边界仍为二维候选检测。

板端依赖 Boost 1.75 以上的 System/JSON 和 yaml-cpp。Ubuntu 24.04 可安装：

```bash
sudo apt install build-essential cmake libboost-system-dev libboost-json-dev libyaml-cpp-dev
cd /home/neardi/ws_livox
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select mid360_suspended_object_detection --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch mid360_suspended_object_detection scan_web.launch.py
```

默认连接板端有线可达的 `ws://192.168.31.7:6060`、frame `scan_link`，HTTP 仅监听板端 `127.0.0.1:8765`。可用 `ws_url:=ws://192.168.20.199:6060`、`port:=8766`、`frame:=/scan_link` 覆盖 launch 参数。Python 默认无线地址；C++ 默认有线地址。

配置示例：`ros2 launch mid360_suspended_object_detection scan_web.launch.py config:=/home/neardi/my_scan.yaml`。先准备好具有相同 YAML 节点结构的可读写文件；保存不会自动创建缺失配置。网页保存的是板端文件，不会自动复制回电脑的源码目录。程序由你手动启动，`Ctrl+C` 停止；本功能不增加后台服务或开机自启。

无 ROS 环境可独立构建：

```bash
cmake -S . -B build/standalone -DBUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build/standalone -j2
ctest --test-dir build/standalone --output-on-failure
./build/standalone/scan_web_server --host 127.0.0.1 --port 8765 \
  --ws-url ws://192.168.31.7:6060 --frame scan_link
```

可通过 `--config` 和 `--static-dir` 指定资源目录；安装后从包的 share 目录查找资源。原有三维 `suspended_object.launch.py` 入口保留，板端构建会同时验证原节点。

### 在电脑访问板端网页

1. 在 VS Code Remote SSH 中连接 `neardi@192.168.20.199`。
2. 打开底部“端口”面板（菜单“查看 → 终端”打开底部面板后选择“端口”），点击“转发端口”或“添加端口”，输入 `8765` 并回车。
3. 在“本地地址”列点击链接。通常为 `http://127.0.0.1:8765`；本机端口占用时使用 VS Code 实际分配的地址。
4. 右键该行设置端口标签为“MID-360 扫描工作台”；设置中启用 `Remote: Restore Forwarded Ports`，使重新连接时恢复。

包内 `.vscode/settings.json` 提供恢复转发和端口标签配置，只有将**本包目录作为工作区**打开时才生效；若当前打开 `/home/neardi` 或整个 `ws_livox`，应在当前工作区或远程用户设置中配置上述项目。现有其他端口转发可以保留。此方案不需要板端监听所有网卡，也不添加网络转发规则或开机服务。

### 回归与性能测量

```bash
ctest --test-dir /home/neardi/ws_livox/build/mid360_suspended_object_detection --output-on-failure
/usr/bin/python3 -m unittest discover -s tests -v
node tests/test_roi.js
node tests/test_roi_interaction.js
/usr/bin/python3 tools/board_verify.py \
  --binary /home/neardi/ws_livox/build/mid360_suspended_object_detection/scan_web_server \
  --fixture /home/neardi/ws_livox/build/mid360_suspended_object_detection/scan_fixture --check-ros
```

发布链路自动验收要求 ROS 构建目录已安装 `rclpy` 和本包消息：

```bash
source /opt/ros/jazzy/setup.bash
source /home/neardi/ws_livox/install/setup.bash
/usr/bin/python3 tests/ros_topic_check.py --seconds 4 --require-detected
/usr/bin/python3 tests/ros_replay_check.py \
  --binary /home/neardi/ws_livox/build/mid360_suspended_object_detection/scan_web_server \
  --output test-results/ros-obstacle
```

它核验不少于约 10 Hz 发布、最大候选字段、中心与最近点距离、符号坐标及无效/无候选清零。集成测试对雷达录制帧的 `scan_link` 数据合成一个 ROI 内二维簇，经真实 WebSocket 回放并检查 ROS 消息；合成簇不代表现场障碍物检测结果。

独立非 ROS 后端运行 `board_verify.py` 时省略 `--check-ros`，仍检查 Web API、并发更新、断线恢复和 Python/C++ 数值一致性。`ros_replay_check.py` 单独验证前后左右及坐标轴候选、Web/ROS 相同结果、空 ROI 清零、异常和恢复。

验收工具录制一次有效扫描，在默认与扩大 ROI 下分别为两版预热 10 秒、测量 60 秒，页面查询 5 Hz，回放 65 Hz。记录单核 100% 口径 CPU、RSS、分析耗时 P50/P95、分析频率和本机结果年龄。结果年龄从分析完成时开始计算；源时间戳未提供，不能当作传感器采集至显示的端到端延迟。`processing_ms` 测量投影与聚类，CPU 则包含收包、解析和 HTTP 序列化。

工具还验证固定输入 Python/C++ 数值结果（绝对误差 1e-8）、真实协议下超时清除、断线恢复、并发参数更新。Node 测试使用模拟 DOM 验证交互状态，不替代浏览器视觉验收。原始录制与归一化回放分开保存；默认不会接受上游报告超时的扫描。仅诊断回放的 `--allow-reported-timeout-replay` 会显式记录元数据变更，不能用于实时验收。
