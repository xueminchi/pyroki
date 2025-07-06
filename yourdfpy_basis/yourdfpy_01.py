import os
import yourdfpy

def main():
    urdf_path = "/home/cxm/.cache/robot_descriptions/robot-assets/urdfs/robots/xarm/xarm7_with_gripper.urdf"
    robot = yourdfpy.URDF.load(urdf_path)  # 或者 yourdfpy.Robot.from_urdf()
    robot.show()  # 可视化

if __name__ == "__main__":
    main()
