# Start Franka
- [ ] 进入上位机 ubuntu2，rt系统，Passward: 
- [ ] 打开franka控制器
- [ ] 登录franka图形控制界面（e.g. 171.16.0.3）
- [ ] 激活关节
![alt text](FrankaUI_System_ActivateFCI_Done.png)
- [ ] 启动FCI模式
![alt text](FrankaUI_System_ActivateFCI.png)
# ROS integration for Franka Robotics research robots

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
## install 
After setting up ROS Noetic, create a Catkin workspace in a directory of your choice:
``` shell
cd /path/to/desired/folder
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/noetic/setup.sh
catkin_init_workspace src
```

Then clone the franka_ros repository from GitHub:
``` shell
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
```
By default, this will check out the newest release of franka_ros. If you want to build a particular version of franka_ros instead, check out the corresponding Git tag:
``` shell
git checkout <version>
```
Install any missing dependencies and build the packages:
``` shell
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
source devel/setup.sh
```
Note: If you have installed a franka ros before, you can get libfranka in its build cmakecache.txt
Warning

If you also installed ros-noetic-libfranka, libfranka might be picked up from /opt/ros/noetic instead of from your custom libfranka build!

## use

```
source devel/setup.sh
```
cartesian_impedance_controller
```
roslanch franka_example_controllers cartesian_impedance_controller.launch load_gripper:=true robot_ip:=172.16.0.3 
```
joint_impedance
``` shell
roslaunch franka_example_controllers joint_impedance_unified.launch load_gripper:=true robot_ip:=172.16.0.3 arm_id:="panda"
```

rostopic pub test

``` shell
rostopic pub /joint_command std_msgs/Float64MultiArray "data: [0.21937448498853018, 0.1383980017689663, 0.04700368355125891, -2.022921479594973, -0., 1.2, 0.5]"
```

``` shell
rostopic pub /joint_impedance_example_controller/joint_command std_msgs/Float64MultiArray "data: [1., -0.16, -0.23, -1.96, -0.152, 1.8, 1.98, 0.04, 0.04]"
```
## some issues may 
只出现话题没有消息，请检查/etc/hosts设置


修改`franka_example_controllers/src/joint_impedance_example_controller.cpp`，使这个基于ROS实现的程序完成`deoxys_control/deoxys/franka-interface/src/franka_control_node.cpp`在`deoxys_control/deoxys/config/joint-impedance-controller.yml`配置下的功能，函数默认参数和配置文件保持一致，


## gripper control
在 ROS 中，你可以直接通过 **发布消息到夹爪的 Move/Grasp 目标 topic** 来控制 Franka gripper 而无需 action client。主要的 topic 有：

---

## 🎯 使用 rostopic 发布命令

### 打开夹爪（Move）

```bash
rostopic pub --once /franka_gripper/move/goal franka_gripper/MoveActionGoal \
"goal:
  width: 0.08
  speed: 0.1"
```

### 抓取（Grasp）

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

这种方式可以直接通过 topic 模拟 action，但依旧需要 action message 类型 ([stackoverflow.com][1])。

---

## 🧰 在 Python 脚本中发布 topic 控制

你可以通过 `rospy.Publisher`，发布消息到与 `rostopic pub` 相同的 topic：

```python
import rospy
from franka_gripper.msg import MoveActionGoal, GraspActionGoal

rospy.init_node('gripper_topic_control')

# Publisher for open
move_pub = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=1)
# Publisher for grasp
grasp_pub = rospy.Publisher('/franka_gripper/grasp/goal', GraspActionGoal, queue_size=1)

# 等待 publisher 建立连接
rospy.sleep(0.5)

# 打开
move_goal = MoveActionGoal()
move_goal.goal.width = 0.08
move_goal.goal.speed = 0.1
move_pub.publish(move_goal)

rospy.sleep(1.0)

# 抓取
grasp_goal = GraspActionGoal()
grasp_goal.goal.width = 0.03
grasp_goal.goal.epsilon.inner = 0.005
grasp_goal.goal.epsilon.outer = 0.005
grasp_goal.goal.speed = 0.1
grasp_goal.goal.force = 5.0
grasp_pub.publish(grasp_goal)
```

与命令行一致，只是换成 Python 脚本自动化运行 ([stackoverflow.com][1])。

---

## ✅ 小结

* 发布 `franka_gripper/MoveActionGoal` 到 `/franka_gripper/move/goal` 打开夹爪。
* 发布 `franka_gripper/GraspActionGoal` 到 `/franka_gripper/grasp/goal` 抓取物体。
* `--once` 发布一次即可，Python 可使用 `rospy.Publisher` 实现相同控制效果。

如果你还想通过 Python 设置成循环控制或动态控制闭合力度宽度，也可以更灵活扩展。需要我帮你做一个完整可运行的 ROS 节点示例吗？

## 双阻抗控制器 (DualImpedanceController) 使用指南

`DualImpedanceController` 是一个ROS控制器，为Franka Emika机器人提供双模式控制，可以在笛卡尔阻抗控制和关节阻抗控制之间切换。以下是使用方法：

### 基本概述

该控制器提供两种主要模式：
1. **笛卡尔阻抗模式**：使用笛卡尔空间中的阻抗控制来控制末端执行器的位置和方向
2. **关节阻抗模式**：在关节级别使用阻抗控制来控制机器人

### 设置与启动

YAML文件（例如 `dual_impedance_controller.yaml`）
启动文件（例如 `dual_impedance_control.launch`）来加载和启动控制器：

#### 3. 启动控制器

```bash
roslaunch franka_example_controllers dual_impedance_controller.launch load_gripper:=true robot_ip:=172.16.0.3 arm_id:="panda"
```
#### 切换控制模式

控制器监听 `/impedance_mode` 话题以切换模式：
- `true`：笛卡尔阻抗模式
- `false`：关节阻抗模式

切换模式的命令：

```bash
# 切换到笛卡尔阻抗模式
rostopic pub /impedance_mode std_msgs/Bool "data: true" -1

# 切换到关节阻抗模式
rostopic pub /impedance_mode std_msgs/Bool "data: false" -1
```

#### 笛卡尔阻抗模式

在笛卡尔模式下，您可以控制：

1. **末端执行器姿态**：通过 `/equilibrium_pose` 话题发送目标位置
   ```bash
   rostopic pub /equilibrium_pose geometry_msgs/PoseStamped "{header: {frame_id: 'panda_link0'}, pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" -1
   ```

2. **刚度设置**：通过 `/stiffness` 话题修改刚度值
   ```bash
   # 格式：[x, y, z, rx, ry, rz, nullspace] 刚度值
   rostopic pub /stiffness std_msgs/Float32MultiArray "data: [1000.0, 1000.0, 1000.0, 30.0, 30.0, 30.0, 10.0]" -1
   ```

3. **零空间配置**：通过 `/equilibrium_configuration` 控制零空间行为
   ```bash
   # 格式：[q1, q2, q3, q4, q5, q6, q7] - 用于零空间控制的关节位置
   rostopic pub /equilibrium_configuration std_msgs/Float32MultiArray "data: [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]" -1
   ```

#### 关节阻抗模式

在关节模式下，您可以通过 `joint_command` 话题发送关节位置命令：

```bash
# 格式：[q1, q2, q3, q4, q5, q6, q7] - 目标关节位置
rostopic pub /dual_impedance_controller/joint_command std_msgs/Float64MultiArray "data: [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]" -1
```

#### 监控

1. **当前姿态**：当前末端执行器姿态发布到 `/cartesian_pose`
2. **外部力/力矩**：力和力矩估计发布到 `/force_torque_ext`
3. **关节力矩**：命令和测量的关节力矩发布到 `/torque_comparison`

### 高级功能

- 控制器自动处理关节限制
- 系统应用力矩速率限制以确保平稳过渡
- 您可以调整参数如刚度、阻尼和零空间行为
- 它支持关节模式下的轨迹执行

### 调试和调优

如果您遇到问题或需要调整控制器：

1. 从较低的刚度值开始
2. 检查您期望的姿态是否可达
3. 监控 `/force_torque_ext` 话题以确保机器人不会施加过大的力
4. 渐进式地调整刚度和阻尼参数，直到达到您期望的行为

该控制器设计为在机器人的能力范围内安全运行，但在测试新参数或命令时，请始终监控机器人的行为。

