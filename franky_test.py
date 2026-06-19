# from franky import *

# robot = Robot("172.16.0.2")  # Replace this with your robot's IP

# # Let's start slow (this lets the robot use a maximum of 5% of its velocity, acceleration, and jerk limits)
# robot.relative_dynamics_factor = 0.05

# # Move the robot 20cm along the relative X-axis of its end-effector
# motion = CartesianMotion(Affine([0.02, 0.0, 0.0]), ReferenceType.Relative)
# robot.move(motion)

#!/usr/bin/env python3

from franky import Robot, JointMotion

ROBOT_IP = "172.16.0.2"
# 0.0, 0, 0.0, -1.57, 0.0, 1.57, 0
q_goal = [
    0.0,
    0.0,
    0.0,
    -1.57,
    0.0,
    1.57,
    0.0
]

def main():
    robot = Robot(ROBOT_IP)

    robot.recover_from_errors()

    # 第一次真实机器人测试建议 0.02~0.05
    robot.relative_dynamics_factor = 0.03

    print("Moving to target joint position:")
    print(q_goal)

    robot.move(JointMotion(q_goal))

    print("Done.")

if __name__ == "__main__":
    main()