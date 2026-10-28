# 板卡部署验收记录

日期：2026-10-24。目标：neardi@192.168.20.199，/home/neardi/ws_livox/src/mid360_obstacle_avoidance。

## 已完成

- 原功能包完整备份：/home/neardi/mid360_backups/mid360_obstacle_avoidance_20261024_073732.tar.gz。
- 同步本地检测配置：ROI X[0.2,1.7]、Y[-0.4,0.8]、Z[-0.1,0.5]；聚类距离0.05 m、10点、对角线0.1 m、体积过滤关闭。原 stop_distance 不再用于检测。
- 安装 python3-aiohttp；Jazzy + /usr/bin/python3，仅构建 mid360_obstacle_avoidance，未修改 FirstAutoTestV8、原 Livox 驱动配置或网卡。
- 独立 C++ 聚类与 ROI/YAML 测试通过；51项 ROS 检测用例和2项 Web 用例通过，最终构建成功。Web 用例含独立 ROS 域重启读取保存的 YAML。
- 通过本地 SSH 转发浏览器实际打开三维界面；X 最大边界轴柄拖动立即生效，网页保存写入指定源 YAML，恢复启动值与再次保存均成功；验收后 ROI 恢复为预定范围。
- Web 单独开启无 RViz 交互服务器；两种编辑模式关闭时无编辑器资源。使用临时启动验收，没有添加开机自启服务。

## 2026-10-24 排查记录（历史）

- 雷达192.168.1.143可达，抓包确认56300/56400等UDP已到达板卡192.168.1.5。
- 现有 Livox 驱动初始化成功，但实际检查期间 /livox/lidar 没有 ROS 发布器或点云消息。页面因此显示等待点云/状态未知，检测保持请求停车。
- 2026-10-24 后续获准继续只读排查：板卡当前使用专用 JSON，真实点云与 IMU UDP 到达 eth0，ROS 原始点云话题仍为 0 发布者。广播设备类型为 35（0x23），已安装 SDK 将其定义为 Mid360s；原专用 JSON 的 MID360 区段对应类型 9，不匹配。已将本功能包专用 JSON 区段修正为 Mid360s，IP、端口与检测参数不变，需重启驱动验证。没有修改 SDK、驱动源码或网卡。
- 由于缺少真实 ROS 点云，尚未完成实时点云显示和 Web 开关前后10 Hz输入下的处理耗时对比；不能把独立合成点群测试的约8.7 ms当作实测整链路结果。

## 使用

按 README 中的命令启动功能包；在 VS Code Remote-SSH 端口面板转发8080。网页代码和Three.js资源全部随包提供，不需要板卡桌面或外网。Mid360s 型号修正后用户已确认雷达与网页正常；实测噪声/障碍物标定与真实输入的10 Hz耗时对比继续由现场验收。

## 2026-10-25 检测启停更新

- 用户确认修正 Mid360s 配置并重启后，雷达和网页功能正常；保留此前排查记录作为历史。
- 板卡功能包更新前完整备份：/home/neardi/mid360_backups/detection_control_before_20261025_023523.tar.gz。本地 config 更新前已归档到 .deployment/local_config_before_*.zip。
- 以板卡整个 config 目录为基准同步本地，保留六个 ROI 标定值、聚类距离 0.1 m、10 点、0.1 m 对角线及 Mid360s 配置，仅新增 detection_enabled_on_start=false。
- 新增 enable_detection 控制订阅和 detection_enabled 实际状态发布；启动暂停、暂停不请求停车、开启等待新有效帧。网页显示启停及独立点云健康，未改消息布局、驱动、SDK、网卡或 FirstAutoTestV8。
- 板卡 Jazzy 构建通过（RelWithDebInfo，系统 /usr/bin/python3）；仅出现 Livox 传递依赖 PCL 的既有 CMake 开发警告，无编译错误。
- 板卡完整回归通过：4 个 CTest 测试组全部通过，其中 54 项 ROS 检测用例、2 项 Web 用例，以及独立 C++ 聚类和 ROI/YAML 测试。测试隔离 ROS 域，未向实际运行域发送启停指令。
- 板卡验证 Python 3.12.3、aiohttp 3.9.1、numpy 1.26.4 和 ROS 消息导入可用；4 个 Python/launch 文件语法检查通过。核对源码/config 校验和一致，ObstacleStatus 保留原有 7 个字段。
- 本地 JavaScript 模块语法检查及网页 9 种状态渲染测试通过。板卡 HTTP/WebSocket 测试覆盖离线资源、暂停时完整点云、ROI 编辑保存、启停状态切换及恢复启动值。
- Fast DDS 的端点发现不公布 history/depth（显示 UNKNOWN/0），测试在其公布时核对深度，并核对可靠性、持久性；历史指令不重放和晚加入状态由实际 ROS 通信验证。
- 更新后重新启动原 launch、刷新网页即可使用；启动默认暂停，需发布 enable_detection=true 开始。没有添加后台常驻或自启服务。

## 2026-10-25 Web 在线调参与停车规则更新

- 更新前完整备份：/home/neardi/mid360_backups/online_params_before_20261025_083824.tar.gz；部署前再次备份：/home/neardi/mid360_backups/online_params_deploy_20261025_085216.tar.gz。本地 config 先归档，再以板卡整个 config 为基准同步；部署前校验标定未变化。
- 保留板卡六个 ROI 标定、0.1 m 聚类距离、10 点、0.1 m 对角线、体积关闭、0.5 s 超时及 Mid360s JSON；未修改驱动、SDK、网卡或 FirstAutoTestV8。
- 新增 DetectorConfig / ApplyDetectorConfig，网页五参数整组保存后应用、十一键写回、双语名称和实际输出话题面板，ROI 拖动继续立即生效。
- 停车规则改为 detection_enabled && (obstacle_detected || cloud_timed_out)。有效点云接收单调计时，持续异常不刷新计时；重复 true/ROI/算法更新不推迟超时。
- 板卡 Jazzy / 系统 Python 最终构建通过（RelWithDebInfo），仅有原 Livox 传递依赖 PCL 的 CMake CMP0144 开发警告。
- 板卡 4 个 CTest 测试组全部通过：56 项 ROS 检测用例、2 项 HTTP/WebSocket 用例，以及独立 C++ 聚类和 ROI/YAML 用例；colcon 汇总 62 tests、0 errors、0 failures、0 skipped。测试独立 ROS 域，未向实际运行域发送控制消息。
- 回归覆盖：暂停不停车、开启等待期不停车、障碍停车/清除、无数据及持续异常超时、有效帧恢复；重复 true、ROI 修改及算法更新不重置计时；超时缩短/延长立即根据已有间隔判断。
- 在线参数覆盖：五参数全部影响判定、只读启动参数与实际配置分离、非法值/网格溢出拒绝、整组保存十一项、只读/缺键/重复键保存失败不提交、当前拖动 ROI 保留、重启加载保存的算法及 ROI；C++ 十一键往返验证保留其他字节。
- Web 覆盖：七个实际话题及完整 status 七字段、独立 Bool、点云/标记帧头与摘要、NaN/+inf 表示、全部有效点含 ROI 外点、暂停下编辑保存、整组调参及错误返回；HTTP 入口、app.js、status.js 均返回 no-cache。
- 本地 JavaScript 模块语法与状态/话题年龄、断线、配置不过期、草稿/焦点保护测试通过；板卡 Python/launch/test 语法与 ROS 消息生成通过。本地旧 MinGW 8.1 的 filesystem 标准库不能完成 ROI 测试编译，该项由板卡 GCC/Jazzy 的成功编译和执行覆盖；本地独立聚类测试通过。
- 2 万点合成密集 ROI 的板卡独立聚类测试约 8.8 ms，仅为合成算法测试，不代表真实点云整链路或 Web 开关耗时。
- 最终验收未重启用户当前 launch、未添加常驻或自启。使用新版需在原终端结束并重启 launch，刷新网页；最终核对 31 个源码/config/资源文件 SHA-256 与本地一致；ROS ObstacleStatus 仍为原七字段，DetectorConfig 为 13 字段。确认 Python 3.12.3、aiohttp 3.9.1、numpy 1.26.4、aarch64 ROS 消息导入、脚本 0755 权限和安装后 status.js 资源可用。
