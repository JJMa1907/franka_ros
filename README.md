
# Franka ROS 使用说明

## 目录
- [网络与环境准备](#网络与环境准备)
- [安装与编译](#安装与编译)
- [快速启动流程](#快速启动流程)
- [环境变量与代理设置](#环境变量与代理设置)
- [控制器与夹爪控制](#控制器与夹爪控制)
- [常见问题与故障排除](#常见问题与故障排除)
- [开发与兼容性](#开发与兼容性)
- [参考文档](#参考文档)

## 网络与环境准备

1. **局域网配置**：所有设备（如上位机、机器人、PC）需在同一子网（如`172.16.0.0/24`），IP不重复，网关指向路由器（如`172.16.0.1`），关闭DHCP，手动分配IP。
2. **物理连接**：所有设备通过网线连接到路由器LAN口，无需外网。
3. **防火墙与hosts**：确保防火墙允许内网通信，`/etc/hosts` 配置正确， 可能影响ROS 话题功能。
4. **验证**：设备间可互ping，能访问各自服务（如Franka控制器Web界面）。

## 安装与编译

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
```

### 3. 安装依赖并编译
```bash
# 安装Python依赖
pip3 install rospkg catkin_pkg empy
# 安装ROS依赖
rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys libfranka
# 编译
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/william/franka/libfranka/build
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=/usr/local/lib/cmake/Franka
source devel/setup.sh
```
**注意**: 若已安装ros-noetic-libfranka，libfranka可能会被优先从/opt/ros/noetic加载。如果之前安装过franka ros，可以在其build cmakecache.txt中找到libfranka/build路径


## 使用指南 (Usage Guide)

### 硬件连接与初始化

#### 1. 硬件重新连接步骤
- **物理检查**: 确保夹爪与机械臂末端法兰盘通过螺丝牢固固定，线缆插头完全插入控制器接口，无松动或弯折
- **供电与通信**: 夹爪需通过Franka控制箱供电。重新插线后，启动控制箱电源，观察夹爪指示灯：
  - **白灯慢闪**: 夹爪正在启动（正常状态）
  - **红灯常亮**: 存在硬件错误（需检查连接或重启系统）

#### 2. 软件激活与校准
##### Franka 机器人启动步骤
- [ ] 进入上位机 ubuntu2，rt系统，Password: [请填入密码]
- [ ] 打开franka控制器
- [ ] 登录franka图形控制界面 (e.g. 171.16.0.3)
- [ ] 激活关节
![alt text](FrankaUI_System_ActivateFCI_Done.png)
- [ ] 启动FCI模式
![alt text](FrankaUI_System_ActivateFCI.png)

##### gripper 设置
1. **进入Desk控制台**: 登录Franka Desk网页界面（默认IP：172.16.0.2，用户名/密码通常为franka/franka123）
2. **添加夹爪模型**: 
   - 路径：Setting → End Effector → Migrated Profile
   - 点击右下角 Active，然后在 Joints 旁选择 End-Effector → Power On → RE-INITIALIZE，等待初始化完成
3. **执行Homing校准**:
   - 切换到 Programming 模式，在 Pilot Mode 中选择夹爪控制选项，点击 Homing 按钮
   - **注意**: 校准前夹爪必须无物体阻挡，否则零点位置会错误

### 笔记本设置

本机ip设置 `172.16.0.5` ，mask`255.255.255.0`，ROS Master: `172.16.0.1.113311` ,
每次使用前都设置好，或者放在bashrc / zshrc 中并source
```shell
export ROS_MASTER_URI=http://172.16.0.2:11311
export ROS_IP=172.16.0.5
export ROS_HOSTNAME=172.16.0.5
```

## 环境变量与代理设置

如需联网安装依赖或访问外部资源，建议配置代理：

**全局代理**（/etc/environment）：
```bash
http_proxy=http://proxy_server:port
https_proxy=http://proxy_server:port
no_proxy=localhost,127.0.0.1,172.16.0.0/24
```

``` bash
export http_proxy=http://proxy_server:port
export https_proxy=http://proxy_server:port
export no_proxy=localhost,127.0.0.1,172.16.0.0/24
```
**临时终端代理**：
```bash
export http_proxy=http://proxy_server:port
export https_proxy=http://proxy_server:port
export no_proxy=localhost,127.0.0.1,172.16.0.0/24
```
**apt/pip/git/rosdep代理**：
详见原文，或参考下方命令：
```bash
pip install --proxy=http://proxy_server:port package_name
git config --global http.proxy http://proxy_server:port
rosdep install ... # 见上
```

## 控制器与夹爪控制

### 控制器启动

``` bash
cd catkin_ws/src/franka_ros
sh ./launch_franka_with_rviz.sh # ./ matters
```

**笛卡尔阻抗控制器**
```bash
roslaunch franka_example_controllers cartesian_impedance_controller.launch load_gripper:=true robot_ip:=172.16.0.3
```
**关节阻抗控制器**
```bash
roslaunch franka_example_controllers joint_impedance_unified.launch load_gripper:=true robot_ip:=172.16.0.3 arm_id:="panda"
```
**双阻抗控制器**
```bash
roslaunch franka_example_controllers dual_impedance_controller.launch load_gripper:=true robot_ip:=172.16.0.3 arm_id:="panda"
```
### 一些常用的ROS topic
``` bash
rostopic echo /cartesian_pose # 获取机器人夹爪中心点坐标
```

### 控制模式切换与命令

**切换模式**
```bash
# 笛卡尔阻抗
rostopic pub /impedance_mode std_msgs/Bool "data: true" -1
# 关节阻抗
rostopic pub /impedance_mode std_msgs/Bool "data: false" -1
```
**末端姿态控制**
```bash
rostopic pub /equilibrium_pose geometry_msgs/PoseStamped 
```
**刚度设置**
```bash
rostopic pub /stiffness std_msgs/Float32MultiArray "data: [1000.0, 1000.0, 1000.0, 30.0, 30.0, 30.0, 10.0]" -1
```
**零空间配置**
```bash
rostopic pub /equilibrium_configuration std_msgs/Float32MultiArray "data: [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]" -1
```
**关节命令**
```bash
rostopic pub /joint_command std_msgs/Float64MultiArray "data: [0.0, 0, 0.0, -1.57, 0.0, 1.57, 0]"
```

### 夹爪控制
####  dual_impedance_controller
**dual_impedance_controller集成控制（推荐）**
dual_impedance_controller 开启后，通过 `/gripper_control` (std_msgs/Float64MultiArray) 话题，格式 `[position, speed, force]`：
- position: 0~1归一化或>1为米
- speed: m/s
- force: N（>0为抓取，<=0为定位）

**示例**
```bash
# 完全打开夹爪
rostopic pub /gripper_control std_msgs/Float64MultiArray "data: [1.0, 0.1]"
# 关闭夹爪
rostopic pub /gripper_control std_msgs/Float64MultiArray "data: [0.0, 0.05]"
# 用10N力抓取
rostopic pub /gripper_control std_msgs/Float64MultiArray "data: [0.02, 0.05, 10.0]"
```
#####  实现细节

- 控制器自动将归一化位置 (0-1) 转换为米 (0-0.08m)
- 自动限制宽度到有效范围
- 基于force参数选择move或grasp动作
- 非阻塞命令执行，便于实时控制
- 直接发布到Franka夹爪action goal话题

#### 传统夹爪控制方法
```bash
# Homing
rostopic pub -1 /franka_gripper/homing/goal franka_gripper/HomingActionGoal "{}"
# Move
rostopic pub --once /franka_gripper/move/goal franka_gripper/MoveActionGoal "goal: { width: 0.08, speed: 0.1 }"
# Grasp
rostopic pub --once /franka_gripper/grasp/goal franka_gripper/GraspActionGoal "goal: { width: 0.03, epsilon:{ inner: 0.005, outer: 0.005 }, speed: 0.1, force: 5.0}"
```

```
rostopic pub /franka_gripper/grasp/goal franka_gripper/GraspveActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  width: 0.003
  epsilon:
    inner: 0.003
    outer: 0.001
  speed: 0.01
  force: 60.0"
```
**Python示例**
```python
import rospy
from std_msgs.msg import Float64MultiArray
rospy.init_node('gripper_control_example')
pub = rospy.Publisher('/gripper_control', Float64MultiArray, queue_size=1)
rospy.sleep(1)
msg = Float64MultiArray()
msg.data = [1.0, 0.1]
pub.publish(msg)
msg.data = [0.02, 0.05, 8.0]
pub.publish(msg)
```

## 常见问题与故障排除

### 1. 只出现话题没有消息 or ping 不通
- 检查 `/etc/hosts` 设置，网络配置。

### 2. franka_gripper 消息类型错误
rostopic pub -1 /franka_gripper/homing/goal franka_gripper/HomingActionGoal "{}"
ERROR: invalid message type: franka_gripper/HomingActionGoal.
```
**解决办法**：
1. 重新编译：`catkin_make`
2. 重新source：`source devel/setup.bash`
3. 检查消息类型：`rosmsg list | grep franka_gripper`
4. 再次尝试命令

### 3. libfranka/CMake/gazebo/protobuf 兼容性
如遇 `libfranka: Incompatible library version` 等，需升级libfranka到0.15.x，修复CMake等依赖。
可用脚本：
```bash
source ~/franka_ros_env.sh
./fix_libfranka_0.15.sh
./test_franka_complete_final.sh
```

## 开发与兼容性

- 控制器开发建议：如需自定义控制器，可参考`franka_example_controllers/src/joint_impedance_example_controller.cpp`，实现与`deoxys_control_node.cpp`和`joint-impedance-controller.yml`等配置一致的功能。
- 版本兼容：libfranka ≥ 0.15.0，见[compatibility](https://frankaemika.github.io/docs/compatibility.html)。


## 参考文档
- [Franka ROS官方文档](https://frankarobotics.github.io/docs/franka_ros.html)
- [Franka Control Interface (FCI)](https://frankarobotics.github.io/docs/getting_started.html)
- [github repo](https://github.com/frankaemika/franka_ros)
- [compatibility](https://frankaemika.github.io/docs/compatibility.html)
- [franka hand 文档](https://download.franka.de/documents/220010_Product%20Manual_Franka%20Hand_1.2_ZH.pdf)


[array([ 0.092, -0.198, -0.02 , -2.473, -0.013,  2.304,  0.848,  0.04 ,  0.04 ]), array([ 0.177,  0.484, -0.185, -1.207,  0.175,  1.981,  2.193,  0.04 ,  0.04 ]), array([ 0.194,  0.747, -0.155, -1.184,  0.184,  2.099,  2.201])]