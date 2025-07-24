# Franka ROS Integration Guide

## 目录 (Table of Contents)
- [快速启动 (Quick Start)](#快速启动-quick-start)
- [安装 (Installation)](#安装-installation)
- [使用指南 (Usage Guide)](#使用指南-usage-guide)
- [控制器使用 (Controllers)](#控制器使用-controllers)
- [夹爪控制 (Gripper Control)](#夹爪控制-gripper-control)
- [故障排除 (Troubleshooting)](#故障排除-troubleshooting)
- [参考文档 (References)](#参考文档-references)

---

## 快速启动 (Quick Start)

### Franka 机器人启动步骤
- [ ] 进入上位机 ubuntu2，rt系统，Password: [请填入密码]
- [ ] 打开franka控制器
- [ ] 登录franka图形控制界面 (e.g. 171.16.0.3)
- [ ] 激活关节
![alt text](FrankaUI_System_ActivateFCI_Done.png)
- [ ] 启动FCI模式
![alt text](FrankaUI_System_ActivateFCI.png)

### 环境设置
```bash
cd /home/jz2995/frankaJointImpedance_ws/catkin_ws
source devel/setup.bash
```

---

## ROS integration for Franka Robotics research robots

参考官方文档：[Franka Control Interface (FCI) documentation](https://frankarobotics.github.io/docs/getting_started.html)
---

## 安装 (Installation)

### 1. 创建Catkin工作空间
```bash
cd /path/to/desired/folder
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/noetic/setup.sh
catkin_init_workspace src
```

### 2. 克隆franka_ros仓库
```bash
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
# 可选：切换到特定版本
git checkout <version>
```

### 3. 安装依赖并编译
```bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
source devel/setup.sh
```

> **注意**: 如果之前安装过franka ros，可以在其build cmakecache.txt中找到libfranka路径

> **警告**: 如果安装了ros-noetic-libfranka，libfranka可能会从/opt/ros/noetic而不是自定义构建中获取！

---

## 使用指南 (Usage Guide)

### 环境设置
每次使用前需要设置环境：
```bash
source devel/setup.sh
```

---

## 控制器使用 (Controllers)

### 笛卡尔阻抗控制器 (Cartesian Impedance Controller)
```bash
roslaunch franka_example_controllers cartesian_impedance_controller.launch \
    load_gripper:=true robot_ip:=172.16.0.3
```

### 关节阻抗控制器 (Joint Impedance Controller)
```bash
roslaunch franka_example_controllers joint_impedance_unified.launch \
    load_gripper:=true robot_ip:=172.16.0.3 arm_id:="panda"
```

### 双阻抗控制器 (Dual Impedance Controller)
```bash
roslaunch franka_example_controllers dual_impedance_controller.launch \
    load_gripper:=true robot_ip:=172.16.0.3 arm_id:="panda"
```

### 测试命令 (Test Commands)

#### 关节位置命令
```bash
# 基础关节命令
rostopic pub /joint_command std_msgs/Float64MultiArray \
    "data: [0.21937448498853018, 0.1383980017689663, 0.04700368355125891, -2.022921479594973, -0., 1.2, 0.5]"

# 关节阻抗控制器命令
rostopic pub /joint_impedance_example_controller/joint_command std_msgs/Float64MultiArray \
    "data: [1., -0.16, -0.23, -1.96, -0.152, 1.8, 1.98, 0.04, 0.04]"
```

---

## 双阻抗控制器 (Dual Impedance Controller) 详细使用指南

`DualImpedanceController` 是一个ROS控制器，为Franka Emika机器人提供双模式控制，可以在笛卡尔阻抗控制和关节阻抗控制之间切换。

### 基本概述
该控制器提供两种主要模式：
1. **笛卡尔阻抗模式**：使用笛卡尔空间中的阻抗控制来控制末端执行器的位置和方向
2. **关节阻抗模式**：在关节级别使用阻抗控制来控制机器人

### 启动控制器
```bash
roslaunch franka_example_controllers dual_impedance_controller.launch \
    load_gripper:=true robot_ip:=172.16.0.3 arm_id:="panda"
```

### 切换控制模式
控制器监听 `/impedance_mode` 话题以切换模式：
- `true`：笛卡尔阻抗模式
- `false`：关节阻抗模式

```bash
# 切换到笛卡尔阻抗模式
rostopic pub /impedance_mode std_msgs/Bool "data: true" -1

# 切换到关节阻抗模式
rostopic pub /impedance_mode std_msgs/Bool "data: false" -1
```

### 笛卡尔阻抗模式控制

#### 1. 末端执行器姿态控制
```bash
rostopic pub /equilibrium_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'panda_link0'}, pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" -1
```

#### 2. 刚度设置
```bash
# 格式：[x, y, z, rx, ry, rz, nullspace] 刚度值
rostopic pub /stiffness std_msgs/Float32MultiArray \
    "data: [1000.0, 1000.0, 1000.0, 30.0, 30.0, 30.0, 10.0]" -1
```

#### 3. 零空间配置控制
```bash
# 格式：[q1, q2, q3, q4, q5, q6, q7] - 用于零空间控制的关节位置
rostopic pub /equilibrium_configuration std_msgs/Float32MultiArray \
    "data: [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]" -1
```

### 关节阻抗模式控制
```bash
# 格式：[q1, q2, q3, q4, q5, q6, q7] - 目标关节位置
rostopic pub /joint_command std_msgs/Float64MultiArray \
    "data: [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]" -1
```

### 状态监控话题
1. **当前姿态**：`/cartesian_pose`
2. **外部力/力矩**：`/force_torque_ext`
3. **关节力矩对比**：`/torque_comparison`

### 调试和调优建议
1. 从较低的刚度值开始
2. 检查期望的姿态是否可达
3. 监控 `/force_torque_ext` 话题以确保机器人不会施加过大的力
4. 渐进式地调整刚度和阻尼参数
#### configured force thresholds reached
1. 保守步长策略
2. 加速度限制
3. 显著降低PD增益
4.降低力矩限制 tau_limit_
#### 卡顿或者抖动
1. 消除过冲：指数平滑确保渐进
2. 减少抖动：分层减速 + 自适应滤波
3. 平滑到达：渐进式速度衰减
4. 精确定位：严格的收敛检查


---

## some issues may 
只出现话题没有消息，请检查/etc/hosts设置

### franka_gripper 消息类型错误
如果出现类似以下错误：
```
rostopic pub -1 /franka_gripper/homing/goal franka_gripper/HomingActionGoal "{}"
ERROR: invalid message type: franka_gripper/HomingActionGoal.
If this is a valid message type, perhaps you need to type 'rosmake franka_gripper'
```

**解决方案：**
1. 重新编译工作空间：
```bash
cd /path/to/catkin_ws
catkin_make
```

2. 重新source环境：
```bash
source devel/setup.bash
```

3. 验证消息类型可用：
```bash
rosmsg list | grep franka_gripper
```

4. 重新尝试命令：
```bash
# Homing
rostopic pub -1 /franka_gripper/homing/goal franka_gripper/HomingActionGoal "{}"

# Move to specific width
rostopic pub --once /franka_gripper/move/goal franka_gripper/MoveActionGoal "goal: { width: 0.08, speed: 0.1 }"

# Grasp
rostopic pub --once /franka_gripper/grasp/goal franka_gripper/GraspActionGoal "goal: { width: 0.03, epsilon:{ inner: 0.005, outer: 0.005 }, speed: 0.1, force: 5.0}"
```

**根本原因：** 这个问题通常是因为：
- 工作空间没有正确编译
- 环境变量没有正确设置（没有source setup.bash）
- ROS包路径不包含franka_gripper包的构建输出

参考官方文档：https://frankarobotics.github.io/docs/franka_ros.html#franka-gripper

修改`franka_example_controllers/src/joint_impedance_example_controller.cpp`，使这个基于ROS实现的程序完成`deoxys_control/deoxys/franka-interface/src/franka_control_node.cpp`在`deoxys_control/deoxys/config/joint-impedance-controller.yml`配置下的功能，函数默认参数和配置文件保持一致，


## 夹爪控制 (Gripper Control)

### DualImpedanceController 集成夹爪控制

`DualImpedanceController` 现在包含了集成的夹爪控制功能，可以通过 `/gripper_control` 话题接受命令并转换为原生的Franka夹爪动作。

#### 话题接口

##### `/gripper_control` (std_msgs/Float64MultiArray)

**消息格式**: `[position, speed, force]`

- **position**: 目标夹爪宽度
  - `0.0-1.0`: 归一化位置 (0=关闭, 1=完全打开 ~8cm)  
  - `>1.0`: 直接宽度以米为单位 (最大 ~0.08m for Franka)
- **speed**: 运动速度 m/s (默认: 0.1 m/s)
- **force**: 抓取力 牛顿 (可选)
  - `< 0` 或省略: 位置模式 (简单定位)
  - `> 0`: 抓取模式 (关闭时施加力)

#### 使用示例

##### 命令行控制
```bash
# 完全打开夹爪
rostopic pub /gripper_control std_msgs/Float64MultiArray "data: [1.0, 0.1]"

# 关闭夹爪 
rostopic pub /gripper_control std_msgs/Float64MultiArray "data: [0.0, 0.05]"

# 半开夹爪
rostopic pub /gripper_control std_msgs/Float64MultiArray "data: [0.5, 0.1]"

# 设置特定宽度 (3cm)
rostopic pub /gripper_control std_msgs/Float64MultiArray "data: [0.03, 0.1]"

# 用10N力抓取
rostopic pub /gripper_control std_msgs/Float64MultiArray "data: [0.02, 0.05, 10.0]"

# 轻柔抓取用5N力
rostopic pub /gripper_control std_msgs/Float64MultiArray "data: [0.01, 0.03, 5.0]"
```

##### Python控制示例
```python
import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node('gripper_control_example')
pub = rospy.Publisher('/gripper_control', Float64MultiArray, queue_size=1)
rospy.sleep(1)  # Wait for publisher

# Open gripper
msg = Float64MultiArray()
msg.data = [1.0, 0.1]  # fully open, 0.1 m/s
pub.publish(msg)

# Grasp with force
msg.data = [0.02, 0.05, 8.0]  # 2cm width, 0.05 m/s, 8N force
pub.publish(msg)
```

#### 设置要求

1. **启动时启用夹爪**:
   ```bash
   roslaunch franka_example_controllers dual_impedance_controller.launch load_gripper:=true robot_ip:=172.16.0.3 arm_id:="panda"
   ```

2. **验证夹爪话题可用**:
   ```bash
   rostopic list | grep gripper
   ```
   应该显示类似的话题:
   - `/panda_gripper/move/goal`
   - `/panda_gripper/grasp/goal`
   - etc.

#### 实现细节

- 控制器自动将归一化位置 (0-1) 转换为米 (0-0.08m)
- 自动限制宽度到有效范围
- 基于force参数选择move或grasp动作
- 非阻塞命令执行，便于实时控制
- 直接发布到Franka夹爪action goal话题

### 传统夹爪控制方法

### 硬件连接与初始化

#### 1. 硬件重新连接步骤
- **物理检查**: 确保夹爪与机械臂末端法兰盘通过螺丝牢固固定，线缆插头完全插入控制器接口，无松动或弯折
- **供电与通信**: 夹爪需通过Franka控制箱供电。重新插线后，启动控制箱电源，观察夹爪指示灯：
  - **白灯慢闪**: 夹爪正在启动（正常状态）
  - **红灯常亮**: 存在硬件错误（需检查连接或重启系统）

#### 2. 软件激活与校准

##### 方法1：通过Desk界面操作（推荐）
1. **进入Desk控制台**: 登录Franka Desk网页界面（默认IP：robot.franka.de 或 192.168.0.1，用户名/密码通常为franka/franka123）
2. **添加夹爪模型**: 
   - 路径：Setting → End Effector → Migrated Profile
   - 点击右下角 Active，然后在 Joints 旁选择 End-Effector → Power On → RE-INITIALIZE，等待初始化完成
3. **执行Homing校准**:
   - 切换到 Programming 模式，在 Pilot Mode 中选择夹爪控制选项，点击 Homing 按钮
   - **注意**: 校准前夹爪必须无物体阻挡，否则零点位置会错误

### ROS命令行控制

#### 基本命令

##### Homing校准
```bash
rostopic pub -1 /franka_gripper/homing/goal franka_gripper/HomingActionGoal "{}"
```

##### 打开夹爪（Move）
```bash
rostopic pub --once /franka_gripper/move/goal franka_gripper/MoveActionGoal \
"goal:
  width: 0.08
  speed: 0.1"
```

##### 抓取（Grasp）
```bash
rostopic pub --once /franka_gripper/grasp/goal franka_gripper/GraspActionGoal \
"goal:
  width: 0.03
  epsilon:
    inner: 0.005
    outer: 0.005
  speed: 0.1
  force: 5.0"
```

### Python脚本控制

```python
import rospy
from franka_gripper.msg import MoveActionGoal, GraspActionGoal

rospy.init_node('gripper_topic_control')

# 创建发布者
move_pub = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=1)
grasp_pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)

# 等待连接建立
rospy.sleep(0.5)

# 打开夹爪
move_goal = MoveActionGoal()
move_goal.goal.width = 0.08
move_goal.goal.speed = 0.1
move_pub.publish(move_goal)

rospy.sleep(1.0)

# 抓取物体
grasp_goal = GraspActionGoal()
grasp_goal.goal.width = 0.03
grasp_goal.goal.epsilon.inner = 0.005
grasp_goal.goal.epsilon.outer = 0.005
grasp_goal.goal.speed = 0.1
grasp_goal.goal.force = 5.0
grasp_pub.publish(grasp_goal)
```

### 使用总结
- 发布 `franka_gripper/MoveActionGoal` 到 `/franka_gripper/move/goal` 打开夹爪
- 发布 `franka_gripper/GraspActionGoal` 到 `/franka_gripper/grasp/goal` 抓取物体
- `--once` 发布一次即可，Python 可使用 `rospy.Publisher` 实现相同控制效果

---

## 故障排除 (Troubleshooting)

### 常见问题解决

#### 1. 网络通信问题
**问题**: 只出现话题没有消息 or ping 不通

**解决方案**: 检查`/etc/hosts`设置，确保网络配置正确

#### 2. franka_gripper 消息类型错误
**问题**: 出现类似以下错误：
```
rostopic pub -1 /franka_gripper/homing/goal franka_gripper/HomingActionGoal "{}"
ERROR: invalid message type: franka_gripper/HomingActionGoal.
If this is a valid message type, perhaps you need to type 'rosmake franka_gripper'
```

**解决方案**:
1. **重新编译工作空间**:
   ```bash
   cd /path/to/catkin_ws
   catkin_make
   ```

2. **重新source环境**:
   ```bash
   source devel/setup.bash
   ```

3. **验证消息类型可用**:
   ```bash
   rosmsg list | grep franka_gripper
   ```

4. **重新尝试命令**:
   ```bash
   # Homing校准
   rostopic pub -1 /franka_gripper/homing/goal franka_gripper/HomingActionGoal "{}"
   
   # 打开夹爪
   rostopic pub --once /franka_gripper/move/goal franka_gripper/MoveActionGoal \
       "goal: { width: 0.08, speed: 0.1 }"
   
   # 抓取
   rostopic pub --once /franka_gripper/grasp/goal franka_gripper/GraspActionGoal \
       "goal: { width: 0.03, epsilon:{ inner: 0.005, outer: 0.005 }, speed: 0.1, force: 5.0}"
   ```

**根本原因**: 
- 工作空间没有正确编译
- 环境变量没有正确设置（没有source setup.bash）
- ROS包路径不包含franka_gripper包的构建输出

---

## 开发注意事项 (Development Notes)

### 控制器开发
修改`franka_example_controllers/src/joint_impedance_example_controller.cpp`，使这个基于ROS实现的程序完成`deoxys_control/deoxys/franka-interface/src/franka_control_node.cpp`在`deoxys_control/deoxys/config/joint-impedance-controller.yml`配置下的功能，函数默认参数和配置文件保持一致。

---

## 参考文档 (References)

- [官方Franka ROS文档](https://frankarobotics.github.io/docs/franka_ros.html)
- [Franka Gripper文档](https://frankarobotics.github.io/docs/franka_ros.html#franka-gripper)
- [Franka Control Interface (FCI) 文档](https://frankarobotics.github.io/docs/getting_started.html)

---

**最后更新**: 2025年6月
