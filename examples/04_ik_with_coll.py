"""IK with Collision

Basic Inverse Kinematics with Collision Avoidance using PyRoKi.
"""

import time

import numpy as np
import pyroki as pk
import viser
from pyroki.collision import HalfSpace, RobotCollision, Sphere
from robot_descriptions.loaders.yourdfpy import load_robot_description
from viser.extras import ViserUrdf
    
import pyroki_snippets as pks

def main():
    """带碰撞避免的基础逆运动学主函数"""
    # 加载 Panda 机器人的 URDF 模型
    urdf = load_robot_description("panda_description")
    # 定义目标末端执行器的 link 名称
    target_link_name = "panda_hand"
    # 创建 Robot 对象，用于计算运动学
    robot = pk.Robot.from_urdf(urdf)

    # 从 URDF 创建机器人碰撞模型
    robot_coll = RobotCollision.from_urdf(urdf)

    # 创建一个半空间碰撞体（地面），从原点向上
    plane_coll = HalfSpace.from_point_and_normal(
        np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])
    )
    # 创建一个球形碰撞体，作为障碍物
    sphere_coll = Sphere.from_center_and_radius(
        np.array([0.0, 0.0, 0.0]), np.array([0.05])
    )

    # 设置可视化环境
    server = viser.ViserServer()
    # 添加地面网格
    server.scene.add_grid("/ground", width=2, height=2, cell_size=0.1)
    # 在 viser 中加载 URDF 并显示
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/robot")

    # 创建 IK 目标的交互式控制器
    ik_target_handle = server.scene.add_transform_controls(
        "/ik_target", scale=0.2, position=(0.5, 0.0, 0.5), wxyz=(0, 0, 1, 0)
    )

    # 创建球形障碍物的交互式控制器和网格
    sphere_handle = server.scene.add_transform_controls(
        "/obstacle", scale=0.2, position=(0.4, 0.3, 0.4)
    )
    # 添加球形障碍物的可视化网格
    server.scene.add_mesh_trimesh("/obstacle/mesh", mesh=sphere_coll.to_trimesh())

    # 在 GUI 中添加一个显示计算耗时的数字框
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    # 进入主循环
    while True:
        # 记录开始时间
        start_time = time.time()

        # 根据当前交互控件的位置更新球形碰撞体在世界坐标系中的位置
        sphere_coll_world_current = sphere_coll.transform_from_wxyz_position(
            wxyz=np.array(sphere_handle.wxyz),
            position=np.array(sphere_handle.position),
        )

        # 定义世界中的碰撞体列表（地面 + 球形障碍物）
        world_coll_list = [plane_coll, sphere_coll_world_current]
        # 调用带碰撞避免的 IK 求解函数
        solution = pks.solve_ik_with_collision(
            robot=robot,
            coll=robot_coll,
            world_coll_list=world_coll_list,
            target_link_name=target_link_name,
            target_position=np.array(ik_target_handle.position),
            target_wxyz=np.array(ik_target_handle.wxyz),
        )


if __name__ == "__main__":
    main()
