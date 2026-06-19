# Franky ROS 兼容复刻实施计划

## 1. 目标

在现有 `ros_control` 路径保持默认不变的前提下，使用 `franky` 补齐兼容后端能力，并保持对外 ROS 话题接口兼容：

```bash
roslaunch franka_example_controllers cartesian_impedance_example_controller.launch load_gripper:=true robot_ip:=172.16.0.2
```

约束：
- 不修改 libfranka 版本策略（按你当前环境兼容版本运行）
- 对外尽量保持已有上层程序无需改动
- 原命令默认仍走 `ros_control`；显式 `backend:=franky` 时才切换到 Franky 兼容后端

## 2. 交付范围

当前交付范围包含：

- `script/franky_cartesian_impedance.py`
  - 作为兼容入口保留
  - 内部转发到包内正式节点实现，避免影响现有脚本路径
- `franka_example_controllers/scripts/franky_cartesian_impedance_compat.py`
  - 作为 `roslaunch` 可直接拉起的正式实现
  - 保留双模式：
    - `interface=ros`：ROS 兼容桥接模式
    - `interface=cli`：保留交互模式
  - ROS bridge 使用系统 Python，避免 conda 环境中 `rospy.init_node()` 卡住
  - Franky runtime 使用内部子进程自动选择可导入 `franky` 的 Python，并通过串行 JSON 命令通道执行机器人动作
- `launch_franky_cartesian_impedance.sh`
  - 支持 roslaunch 风格参数透传
  - 支持 `interface:=ros|cli`
- `README.md`
  - 增加 franky 复刻说明
  - 增加 ROS 兼容话题清单（订阅/发布）

## 3. ROS 接口兼容目标

### 3.1 订阅话题（输入）

- `/equilibrium_pose` (`geometry_msgs/PoseStamped`)
  - 行为：驱动末端目标位姿
  - 稳定性：高频输入只保留最新目标，不累计执行过期位姿
  - 兼容性：当 `/interactive_marker` 与外部程序同时发布时，外部程序目标优先，避免 marker 高频刷新覆盖上层命令
- `/equilibrium_configuration` (`std_msgs/Float32MultiArray`)
  - 行为：接收 7 维关节目标（用于空域/关节兼容）
  - 稳定性：与 `/joint_command` 一样按最新目标覆盖处理
- `/joint_command` (`std_msgs/Float64MultiArray`)
  - 行为：接收 7 维关节目标
  - 稳定性：高频输入只保留最新目标，避免队列堆积导致控制滞后
- `/gripper_control` (`std_msgs/Float64MultiArray`)
  - 行为：`[position, speed, force]`
- `/impedance_mode` (`std_msgs/Bool`)
  - 行为：兼容模式切换信号（桥接层记录状态并发布状态话题）
- `/stiffness` (`std_msgs/Float32MultiArray`)
  - 行为：同步到 `/dynamic_reconfigure_compliance_param_node`
  - 行为：尽力映射到 franky 刚度接口，保证现有 `dynamic_reconfigure.client` 调用代码不需要修改

### 3.2 发布话题（输出）

- `/cartesian_pose` (`geometry_msgs/PoseStamped`)
  - 行为：周期发布当前末端位姿
- `/robot_pose` (`geometry_msgs/Pose`)
  - 行为：兼容已有脚本读取位姿
- `/impedance_mode_status` (`std_msgs/Bool`)
  - 行为：发布当前模式状态
- `/current_impedance_mode` (`std_msgs/String`)
  - 行为：发布 `cartesian` / `joint`
- `/joint_states` (`sensor_msgs/JointState`)
  - 行为：发布聚合后的机械臂+夹爪关节状态
- `/franka_state_controller/joint_states` (`sensor_msgs/JointState`)
  - 行为：兼容现有依赖 `franka_state_controller` 命名空间的脚本
- `/franka_state_controller/joint_states_desired` (`sensor_msgs/JointState`)
  - 行为：发布最近一次目标关节状态
- `/franka_state_controller/franka_states` (`franka_msgs/FrankaState`)
  - 行为：兼容现有交互脚本初始化与状态读取
- `/franka_state_controller/F_ext` (`geometry_msgs/WrenchStamped`)
  - 行为：发布外力矩估计；若 franky 底层信息有限则允许近似
- `/force_torque_ext`（可选）
  - 行为：若 franky 当前接口可获得外力矩则发布；否则本期可标记为后续增强

## 4. 代码结构计划

### 4.1 `franka_example_controllers/scripts/franky_cartesian_impedance_compat.py`

新增模块与结构：

- `FrankyCartesianController`
  - 保留：连接机器人、笛卡尔/关节移动、夹爪动作
- `FrankyRosCompatBridge`
  - 新增：
    - ROS `Publisher`/`Subscriber` 初始化
    - 命令队列与串行执行线程（避免并发命令冲突）
    - 话题回调到 franky 调用的映射
    - 位姿周期发布
    - `joint_states` / `franka_states` / 外力矩状态发布
    - `dynamic_reconfigure_compliance_param_node` 兼容
- CLI 模式
  - 保留现有 REPL

参数计划：
- `--robot-ip`
- `--load-gripper`
- `--dynamics`
- `--auto-recover`
- `--interface` (`ros|cli`)
- `--pose-rate-hz`
- `--arm-id`
- `--connect-on-start`（默认 `false`，先上线 ROS 接口，收到命令后再连接机器人）

### 4.2 `launch_franky_cartesian_impedance.sh`

增强点：
- 支持 `interface:=...`、`pose_rate_hz:=...`
- 继续支持 `robot_ip:=...`、`load_gripper:=...`
- 自动选择可导入 `franky` 的 Python 解释器
- 输出当前使用解释器，便于排障
- 顶层 `launch_franka_with_rviz.sh` 默认保持 `BACKEND=ros_control`；需要兼容后端时使用 `BACKEND=franky_compat`

### 4.3 `README.md`

新增/更新章节：
- franky 复刻启动方式
- ROS 兼容话题清单
- 与原 ROS 控制器差异说明
- 典型验证命令（`rostopic pub/echo`、`dynamic_reconfigure.client`）

## 5. 验收标准

### 5.1 启动成功

- 启动命令可运行：

```bash
./launch_franky_cartesian_impedance.sh interface:=ros load_gripper:=true robot_ip:=172.16.0.2
```

### 5.2 话题存在

`rostopic list` 至少包含：
- `/equilibrium_pose`
- `/joint_command`
- `/gripper_control`
- `/cartesian_pose`
- `/robot_pose`

### 5.3 基础链路验证

- 发布 `/equilibrium_pose` 后机器人运动
- 发布 `/gripper_control` 后夹爪响应
- `rostopic echo /cartesian_pose` 持续有数据

### 5.4 稳定性

- 连续发布 30s 不崩溃
- 非法消息长度有防护日志，不导致节点退出

## 6. 风险与处理

- 风险：不同 franky 版本 API 差异
  - 处理：对可选接口使用 `hasattr` 检查并降级
- 风险：并发命令导致控制冲突
  - 处理：一次性命令统一串行执行；高频位姿/关节目标采用“最新值覆盖 + 异步 motion 切换”，避免旧目标堆积
- 风险：外力矩话题不可直接映射
  - 处理：一期先保证核心控制兼容，`/force_torque_ext` 作为增强项

## 7. 回退方案

- 通过环境变量切回原 ROS 控制器路径：

```bash
BACKEND=ros_control ./launch_franka_with_rviz.sh
```

- franky 方案与原方案并存，不覆盖原控制器包

## 8. 审阅确认项

请重点确认以下三点：

1. 话题兼容清单是否满足你现有上层程序
2. `/stiffness` 与 `/force_torque_ext` 的一期策略是否可接受
3. 是否同意先交付“核心控制兼容 + 稳定运行”，再做高级参数/外力矩增强
