import os
import yourdfpy

def main():
    # urdf_path = "/home/cxm/.cache/robot_descriptions/robot-assets/urdfs/robots/xarmgripper/xarm7_with_gripper.urdf"
    # robot = yourdfpy.URDF.load(urdf_path)  # 或者 yourdfpy.Robot.from_urdf()
    # robot.show()  # 可视化

    # urdf_path = "/home/cxm/.cache/robot_descriptions/robot-assets/urdfs/robots/xarm7/xarm7_robot.urdf"
    # robot = yourdfpy.URDF.load(urdf_path)  # 或者 yourdfpy.Robot.from_urdf()
    # print('robot joint names:', robot._link_map.keys())  # 观察所有link，方便选target link
    # robot.show()  # 可视化

    urdf_path = "/home/cxm/.cache/robot_descriptions/robot-assets/urdfs/robots/franka_panda/panda.urdf"
    robot = yourdfpy.URDF.load(urdf_path)  # 或者 yourdfpy.Robot.from_urdf()
    print('robot joint names:', robot._link_map.keys())  # 观察所有link，方便选target link
    robot.show()  # 可视化


if __name__ == "__main__":
    main()
