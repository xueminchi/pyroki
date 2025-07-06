"""轨迹优化

使用PyRoKi进行基础轨迹优化。

机器人越过墙壁，同时避免世界碰撞。
"""

import time
from typing import Literal

import numpy as np
import pyroki as pk
import trimesh
import tyro
import viser
from viser.extras import ViserUrdf
from robot_descriptions.loaders.yourdfpy import load_robot_description

import pyroki_snippets as pks


def main(robot_name: Literal["ur5", "panda"] = "panda"):
    """主函数：轨迹优化演示"""
    # 根据机器人类型加载相应的URDF和配置
    if robot_name == "ur5":
        urdf = load_robot_description("ur5_description")
        down_wxyz = np.array([0.707, 0, 0.707, 0])  # UR5的向下方向四元数
        target_link_name = "ee_link"  # 目标末端执行器链接名称

        # 对于UR5，重要的是在安全配置中初始化机器人；
        # 零配置会使机器人与墙壁障碍物对齐
        default_cfg = np.zeros(6)
        default_cfg[1] = -1.308  # 设置第二个关节的默认角度
        robot = pk.Robot.from_urdf(urdf, default_joint_cfg=default_cfg)

    elif robot_name == "panda":
        urdf = load_robot_description("panda_description")
        target_link_name = "panda_hand"  # Panda机器人的目标链接
        down_wxyz = np.array([0, 0, 1, 0])  # Panda的向下方向四元数
        robot = pk.Robot.from_urdf(urdf)

    else:
        raise ValueError(f"无效的机器人类型: {robot_name}")

    # 从URDF创建机器人碰撞模型
    robot_coll = pk.collision.RobotCollision.from_urdf(urdf)

    # 定义轨迹问题参数：
    # - 时间步数和时间步长
    timesteps, dt = 25, 0.02
    # - 起始和结束位置
    start_pos, end_pos = np.array([0.5, -0.3, 0.2]), np.array([0.5, 0.3, 0.2])

    # 定义障碍物：
    # - 地面（半空间）
    ground_coll = pk.collision.HalfSpace.from_point_and_normal(
        np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])
    )
    # - 墙壁（胶囊体）
    wall_height = 0.4  # 墙壁高度
    wall_width = 0.1   # 墙壁宽度
    wall_length = 0.4  # 墙壁长度
    # 创建墙壁的多个胶囊体段
    wall_intervals = np.arange(start=0.3, stop=wall_length + 0.3, step=0.05)
    translation = np.concatenate(
        [
            wall_intervals.reshape(-1, 1),  # X坐标
            np.full((wall_intervals.shape[0], 1), 0.0),  # Y坐标（固定在0）
            np.full((wall_intervals.shape[0], 1), wall_height / 2),  # Z坐标（墙壁中心高度）
        ],
        axis=1,
    )
    # 创建墙壁的胶囊体碰撞体
    wall_coll = pk.collision.Capsule.from_radius_height(
        position=translation,
        radius=np.full((translation.shape[0], 1), wall_width / 2),  # 半径
        height=np.full((translation.shape[0], 1), wall_height),     # 高度
    )
    # 组合所有世界碰撞体
    world_coll = [ground_coll, wall_coll]

    # 求解轨迹优化问题
    traj = pks.solve_trajopt(
        robot,
        robot_coll,
        world_coll,
        target_link_name,
        start_pos,
        down_wxyz,
        end_pos,
        down_wxyz,
        timesteps,
        dt,
    )
    traj = np.array(traj)  # 转换为numpy数组

    # 可视化设置
    server = viser.ViserServer()
    urdf_vis = ViserUrdf(server, urdf)
    # 添加网格地面
    server.scene.add_grid("/grid", width=2, height=2, cell_size=0.1)
    # 添加墙壁的可视化网格
    server.scene.add_mesh_trimesh(
        "wall_box",
        trimesh.creation.box(
            extents=(wall_length, wall_width, wall_height),
            transform=trimesh.transformations.translation_matrix(
                np.array([0.5, 0.0, wall_height / 2])
            ),
        ),
    )
    # 添加起始和结束位置的坐标系标记
    for name, pos in zip(["start", "end"], [start_pos, end_pos]):
        server.scene.add_frame(
            f"/{name}",
            position=pos,
            wxyz=down_wxyz,
            axes_length=0.05,
            axes_radius=0.01,
        )

    # 添加GUI控件
    slider = server.gui.add_slider(
        "Timestep", min=0, max=timesteps - 1, step=1, initial_value=0
    )
    playing = server.gui.add_checkbox("Playing", initial_value=True)

    # 主循环：动画播放
    while True:
        if playing.value:
            slider.value = (slider.value + 1) % timesteps  # 循环播放

        urdf_vis.update_cfg(traj[slider.value])  # 更新机器人配置
        time.sleep(1.0 / 10.0)  # 控制播放速度


if __name__ == "__main__":
    tyro.cli(main)
