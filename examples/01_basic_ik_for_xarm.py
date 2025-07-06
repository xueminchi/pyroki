"""Basic IK

Simplest Inverse Kinematics Example using PyRoki.
"""

import time

import numpy as np
import pyroki as pk                 # PyRoki: 机器人逆运动学库
import viser                       # viser: 交互式3D可视化工具
from robot_descriptions.loaders.yourdfpy import load_robot_description  # 加载URDF机器人模型
from viser.extras import ViserUrdf # viser提供的URDF可视化工具

import pyroki_snippets as pks      # 你自己写的小工具，比如IK求解函数

def main():
    """Main function for basic IK."""

    # 加载 Panda 机械臂的URDF描述
    urdf = load_robot_description("xArm7_description")

    print("robot joint names:", urdf._link_map.keys())  # 观察所有link，方便选target link
    target_link_name = "panda_hand"  # 目标末端执行器的名字

    # 用URDF创建一个PyRoki的机器人对象
    robot = pk.Robot.from_urdf(urdf)

    # 启动 viser 可视化服务器
    server = viser.ViserServer()
    server.scene.add_grid("/ground", width=2, height=2)  # 地面网格

    # 可视化机器人
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")  # 可视化URDF模型

    # 在场景里加一个交互控件，让你用鼠标拖动设置目标位姿
    ik_target = server.scene.add_transform_controls(
        "/ik_target",                       # 控件名字
        scale=0.2,                          # 控件缩放
        position=(0.61, 0.0, 0.56),         # 初始位置
        wxyz=(0, 0, 1, 0)                   # 初始朝向（四元数）
    )
    # 一个显示“当前耗时”的界面元素
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    while True:
        # 每次循环：求解当前目标位置的逆运动学
        start_time = time.time()
        solution = pks.solve_ik(
            robot=robot,
            target_link_name=target_link_name,
            target_position=np.array(ik_target.position),  # 读取控件的位置
            target_wxyz=np.array(ik_target.wxyz),          # 读取控件的方向
        )

        # 更新界面显示的耗时信息
        elapsed_time = time.time() - start_time
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)  # unit is ms

        # 更新可视化机器人状态
        urdf_vis.update_cfg(solution)

if __name__ == "__main__":
    main()
