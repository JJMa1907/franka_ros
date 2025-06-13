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

## License

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

rostopic pub test

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

[1]: https://stackoverflow.com/questions/76947858/publishing-to-a-ros-topic-to-open-robot-hand-through-python-script?utm_source=chatgpt.com "Publishing to a ROS topic to open robot hand through Python script"


[ INFO] [1749818599.599606724]: Received joint command: [0.080, -0.151, -0.015, -2.508, -0.037, 2.326, 0.850]
[ INFO] [1749818599.599642802]: Trajectory active before adding point: false
[ INFO] [1749818599.599656520]: Trajectory started! Set trajectory_active_ = true
[ INFO] [1749818599.599697803]: Trajectory active after adding point: true
[ INFO] [1749818599.599713004]: Added trajectory point: [0.080, -0.151, -0.015, -2.508, -0.037, 2.326, 0.850]
[ INFO] [1749818600.531781839]: Debug: use_external_command_=true, external_command_received_=true, isTrajectoryActive()=false
[ INFO] [1749818600.531826412]: Debug: trajectory_buffer_.size()=1, trajectory_active_=false
[ INFO] [1749818600.531845839]: Debug: No active trajectory, maintaining current position: 0.080
[ INFO] [1749818600.531857009]: Target joints: [0.080, -0.150, -0.015, -2.508, -0.037, 2.326, 0.810]
[ INFO] [1749818600.531880575]: Current joints: [0.080, -0.150, -0.015, -2.508, -0.037, 2.326, 0.810]
[ INFO] [1749818600.531904815]: Joint errors: [-0.000, -0.000, 0.000, -0.000, 0.000, 0.000, -0.000]
[ INFO] [1749818600.531916655]: Calculated torques: [-0.012, -0.011, 0.006, -0.028, -0.001, 0.011, -0.000]
[ INFO] [1749818600.531926854]: Saturated torques: [-0.012, -0.011, 0.006, -0.028, -0.001, 0.011, -0.000]



[ INFO] [1749822330.231676026]: Debug: use_external_command_=true, external_command_received_=true, isTrajectoryActive()=false
[ INFO] [1749822330.231689744]: Debug: trajectory_buffer_.size()=1, trajectory_active_=false
[ INFO] [1749822330.231704597]: isTrajectoryActive() called: trajectory_active_ = false, buffer size = 1
[ INFO] [1749822330.231726486]: Debug: No active trajectory, maintaining current position: 0.080
[ INFO] [1749822330.231740080]: isTrajectoryActive() called: trajectory_active_ = false, buffer size = 1
[ INFO] [1749822330.231754332]: isTrajectoryActive() called: trajectory_active_ = false, buffer size = 1
[ INFO] [1749822330.231767955]: isTrajectoryActive() called: trajectory_active_ = false, buffer size = 1
[ INFO] [1749822330.231782182]: isTrajectoryActive() called: trajectory_active_ = false, buffer size = 1
[ INFO] [1749822330.231796173]: isTrajectoryActive() called: trajectory_active_ = false, buffer size = 1
[ INFO] [1749822330.231810468]: isTrajectoryActive() called: trajectory_active_ = false, buffer size = 1
[ INFO] [1749822330.231829554]: Target joints: [0.080, -0.131, -0.015, -2.508, -0.037, 2.326, 0.810]
[ INFO] [1749822330.231846425]: Current joints: [0.080, -0.131, -0.015, -2.508, -0.037, 2.326, 0.810]
[ INFO] [1749822330.231863633]: Joint errors: [-0.000, -0.000, -0.000, 0.000, -0.000, 0.000, -0.000]
[ INFO] [1749822330.231879795]: Calculated torques: [0.007, 0.001, -0.007, -0.015, -0.005, -0.005, 0.003]
[ INFO] [1749822330.231897302]: Saturated torques: [0.007, 0.001, -0.007, -0.015, -0.005, -0.005, 0.003]
