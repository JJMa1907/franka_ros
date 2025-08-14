#!/usr/bin/env python3
"""
增强版机械臂控制器 - 支持位置和姿态控制
提供完整的6自由度位姿控制接口
"""
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from tf.transformations import quaternion_from_euler, quaternion_slerp, quaternion_matrix
import tf

class ArmController:
    def __init__(self, 
                 position_topic="/cartesian_pose", 
                 command_topic="/equilibrium_pose",
                 interpolation_step=0.02):
        """
        初始化机械臂控制器
        
        参数:
        position_topic: 获取当前位姿的话题
        command_topic: 发送控制命令的话题
        interpolation_step: 位置插值步长(m)
        # control_frame: 控制参考坐标系
        """
        rospy.init_node('arm_controller', anonymous=True)
        
        self.position_topic = position_topic
        self.command_topic = command_topic
        self.step = interpolation_step
        # self.control_frame = control_frame
        
        # 创建TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 创建控制命令发布器
        self.cmd_publisher = rospy.Publisher(
            command_topic, 
            PoseStamped, 
            queue_size=10
        )
        
        # 等待连接建立
        rospy.sleep(1)
        # rospy.loginfo(f"机械臂控制器初始化完成，控制参考系: {control_frame}")

    def get_current_pose(self, timeout=3.0):
        """
        获取当前机械臂末端位姿
        
        返回:
        geometry_msgs/PoseStamped 或 None(超时时)
        """
        try:
            rospy.logdebug("等待当前位姿信息...")
            pose = rospy.wait_for_message(
                self.position_topic, 
                PoseStamped, 
                timeout=timeout
            )
            
            # 转换到控制参考系
            return pose
            
        except (rospy.ROSException, tf.Exception) as e:
            rospy.logwarn(f"获取当前位姿失败: {str(e)}")
            return None

    def move_to_pose(self, position=None, orientation=None, 
                    relative=False, duration=1.0):
        """
        移动机械臂到指定位姿
        
        参数:
        position: 目标位置 (Point 或 (x,y,z)元组)
        orientation: 目标姿态 (Quaternion, (x,y,z,w)元组, 或欧拉角(yaw,pitch,roll))
        relative: 是否为相对当前位置的移动
        duration: 期望移动时间(秒)
        
        返回:
        bool: 是否成功执行
        """
        # 获取当前位姿
        current_pose = self.get_current_pose()
        if current_pose is None:
            return False
            
        # 处理位置参数
        if position is not None:
            if isinstance(position, Point):
                target_position = position
            elif isinstance(position, (tuple, list)) and len(position) == 3:
                target_position = Point(*position)
            else:
                rospy.logerr("无效的位置参数格式")
                return False
            
            # 相对位置计算
            if relative:
                target_position.x += current_pose.pose.position.x
                target_position.y += current_pose.pose.position.y
                target_position.z += current_pose.pose.position.z
        else:
            # 未指定位置则保持当前位置
            target_position = current_pose.pose.position
            
        # 处理姿态参数
        if orientation is not None:
            target_orientation = self._parse_orientation(orientation)
            if target_orientation is None:
                return False
        else:
            # 未指定姿态则保持当前姿态
            target_orientation = current_pose.pose.orientation
        
        rospy.loginfo(f"移动到位姿: "
                     f"位置({target_position.x:.3f}, {target_position.y:.3f}, {target_position.z:.3f})")
        
        # 计算位置距离和姿态差异
        pos_diff = self._position_difference(current_pose.pose.position, target_position)
        orient_diff = self._orientation_difference(
            current_pose.pose.orientation, 
            target_orientation
        )
        
        # 基于距离和角度确定插值点数
        pos_points = max(10, int(np.ceil(pos_diff / self.step)))
        orient_points = max(10, int(np.ceil(orient_diff / 0.15)))  # 5度/步
        
        N = max(pos_points, orient_points, int(duration))
        print("pos_points:", pos_points, "orient_points:", orient_points, "N:", N)
        rospy.loginfo(f"生成 {N} 个路径点，位置距离: {pos_diff:.3f}m, 姿态角度差: {orient_diff:.1f}°")
        
        # 生成位置插值路径
        x_path = np.linspace(current_pose.pose.position.x, target_position.x, N)
        y_path = np.linspace(current_pose.pose.position.y, target_position.y, N)
        z_path = np.linspace(current_pose.pose.position.z, target_position.z, N)
        
        # 生成姿态插值路径
        q_start = [
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w
        ]
        
        q_target = [
            target_orientation.x,
            target_orientation.y,
            target_orientation.z,
            target_orientation.w
        ]
        
        # 发布路径点
        for i in range(N):
            cmd = PoseStamped()
            cmd.header.stamp = rospy.Time.now()
            # cmd.header.frame_id = self.control_frame
            
            # 设置位置
            cmd.pose.position.x = x_path[i]
            cmd.pose.position.y = y_path[i]
            cmd.pose.position.z = z_path[i]
            
            # 设置姿态 (球面线性插值)
            ratio = i / (N - 1) if N > 1 else 1.0
            q_interp = quaternion_slerp(q_start, q_target, ratio)
            
            cmd.pose.orientation.x = q_interp[0]
            cmd.pose.orientation.y = q_interp[1]
            cmd.pose.orientation.z = q_interp[2]
            cmd.pose.orientation.w = q_interp[3]
            
            self.cmd_publisher.publish(cmd)
            
            # 显示进度
            # if i % 10 == 0:
            #     rospy.loginfo(f"路径点 {i+1}/{N}: "
            #                  f"位置({cmd.pose.position.x:.3f}, "
            #                  f"{cmd.pose.position.y:.3f}, "
            #                  f"{cmd.pose.position.z:.3f})")
            
            rospy.sleep(0.05)  # 20Hz
        
        rospy.loginfo("位姿移动完成")
        return True

    def _parse_orientation(self, orientation):
        """解析不同格式的姿态输入"""
        if isinstance(orientation, Quaternion):
            return orientation
            
        elif isinstance(orientation, (tuple, list)):
            if len(orientation) == 4:  # (x,y,z,w)
                return Quaternion(*orientation)
            elif len(orientation) == 3:  # 欧拉角 (yaw, pitch, roll)
                q = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
                return Quaternion(*q)
        
        rospy.logerr("无效的姿态格式，使用Quaternion、元组(4元素)或欧拉角(3元素)")
        return None

    def _position_difference(self, p1, p2):
        """计算两点间距离"""
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dz = p2.z - p1.z
        return np.sqrt(dx**2 + dy**2 + dz**2)

    def _orientation_difference(self, q1, q2):
        """计算两个四元数间的角度差(度)"""
        # 转换为旋转矩阵
        R1 = quaternion_matrix([q1.x, q1.y, q1.z, q1.w])
        R2 = quaternion_matrix([q2.x, q2.y, q2.z, q2.w])
        
        # 计算相对旋转矩阵
        R_rel = np.dot(R1[:3, :3].T, R2[:3, :3])
        
        # 计算旋转角度
        cos_theta = (np.trace(R_rel) - 1) / 2
        cos_theta = np.clip(cos_theta, -1.0, 1.0)  # 避免数值误差
        angle_rad = np.arccos(cos_theta)
        
        return np.degrees(angle_rad)

    def move_to_position(self, x, y, z, relative=False, duration=5.0):
        """
        移动机械臂到指定位置（保持当前姿态）
        简化方法，内部调用move_to_pose
        """
        return self.move_to_pose(
            position=(x, y, z),
            orientation=None,
            relative=relative,
            duration=duration
        )

    def set_orientation(self, orientation, relative=False, duration=3.0):
        """
        设置机械臂末端姿态（保持当前位置）
        
        参数:
        orientation: 目标姿态 (Quaternion, (x,y,z,w)元组, 或欧拉角(yaw,pitch,roll))
        relative: 是否为相对当前姿态的旋转
        duration: 期望旋转时间(秒)
        """
        # 获取当前位姿
        current_pose = self.get_current_pose()
        if current_pose is None:
            return False
            
        return self.move_to_pose(
            position=None,  # 保持当前位置
            orientation=orientation,
            relative=relative,
            duration=duration
        )

    def execute_trajectory(self, waypoints, duration=0.0):
        """
        执行预定义轨迹（支持完整位姿）
        
        参数:
        waypoints: 路径点列表，每个点为dict:
            {
                'position': (x,y,z) 或 Point, 
                'orientation': (x,y,z,w) 或 Quaternion 或 欧拉角(yaw,pitch,roll),
                'duration': 到达该点的期望时间(秒) - 可选
            }
        duration: 全局时间参数(0表示使用每个点的独立时间)
        """
        if not waypoints:
            rospy.logwarn("空轨迹无法执行")
            return False
            
        rospy.loginfo(f"开始执行轨迹，共 {len(waypoints)} 个路径点")
        
        # 获取起始位姿
        current_pose = self.get_current_pose()
        if current_pose is None:
            return False
            
        # 处理轨迹点
        processed_points = []
        for i, wp in enumerate(waypoints):
            # 处理位置
            pos = wp.get('position')
            if pos is None:
                # 未指定位置则使用前一点位置
                if processed_points:
                    position = processed_points[-1]['position']
                else:
                    position = current_pose.pose.position
            else:
                if isinstance(pos, Point):
                    position = pos
                elif isinstance(pos, (tuple, list)) and len(pos) == 3:
                    position = Point(*pos)
                else:
                    rospy.logerr(f"路径点 {i} 位置格式无效")
                    return False
                    
            # 处理姿态
            orient = wp.get('orientation')
            if orient is None:
                # 未指定姿态则使用前一点姿态
                if processed_points:
                    orientation = processed_points[-1]['orientation']
                else:
                    orientation = current_pose.pose.orientation
            else:
                orientation = self._parse_orientation(orient)
                if orientation is None:
                    return False
                    
            # 处理时间
            point_duration = wp.get('duration', 0)
            if point_duration <= 0:
                if duration > 0:
                    point_duration = duration
                else:
                    point_duration = 2.0  # 默认2秒
                    
            processed_points.append({
                'position': position,
                'orientation': orientation,
                'duration': point_duration
            })
        
        # 执行轨迹
        for i, point in enumerate(processed_points):
            if rospy.is_shutdown():
                return False
                
            rospy.loginfo(f"执行路径点 {i+1}/{len(processed_points)}")
            
            success = self.move_to_pose(
                position=point['position'],
                orientation=point['orientation'],
                relative=False,
                duration=point['duration']
            )
            
            if not success:
                rospy.logerr(f"路径点 {i+1} 执行失败")
                return False
                
        rospy.loginfo("轨迹执行完成")
        return True
    
if __name__== '__main__':
    # 创建控制器实例
    arm = ArmController()

    # # 移动到绝对位置
    # arm.move_to_position(0.3, -0.06, 0.31)

    # # 相对当前位置移动
    # arm.move_to_position(0.1, 0, 0, relative=True)  # X方向移动10cm
    # arm.move_to_position(-0.1, 0, 0, relative=True)  # X方向移动10cm

    arm.move_to_pose(position=Point(0.55, 0.35, 0.15),
                     orientation=Quaternion(0.5, 0.5, 0.5, 0.5))