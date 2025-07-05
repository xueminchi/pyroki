"""
Bimanual IK

与 01_basic_ik.py 类似，但这里同时控制两个末端执行器！
"""

import time
import viser  # 可视化工具
from robot_descriptions.loaders.yourdfpy import load_robot_description  # 加载机器人 URDF
import numpy as np

import pyroki as pk  # PyRoki 运动学/逆运动学库
from viser.extras import ViserUrdf  # 用于在 viser 中显示 URDF
import pyroki_snippets as pks  # 封装好的 IK 计算函数


def main():
    """双臂逆运动学的主函数"""

    # 加载 YuMi 机器人的 URDF 模型
    urdf = load_robot_description("yumi_description")
    # 定义两个末端执行器的 link 名称
    target_link_names = ["yumi_link_7_r", "yumi_link_7_l"]

    # 创建 Robot 对象，用于计算运动学
    robot = pk.Robot.from_urdf(urdf)

    # 启动 viser 可视化服务器
    server = viser.ViserServer()
    # 在场景中添加地面网格
    server.scene.add_grid("/ground", width=2, height=2)
    # 在 viser 中加载 URDF 并显示
    urdf_vis = ViserUrdf(server, urdf, root_node_name="/base")

    # 创建两个交互式目标控制器，用于调整左右末端目标位置
    ik_target_0 = server.scene.add_transform_controls(
        "/ik_target_0",                 # 控制器节点路径
        scale=0.2,                      # 控制器大小
        position=(0.41, -0.3, 0.56),    # 初始位置
        wxyz=(0, 0, 1, 0)               # 初始姿态（四元数 wxyz）
    )
    ik_target_1 = server.scene.add_transform_controls(
        "/ik_target_1",
        scale=0.2,
        position=(0.41, 0.3, 0.56),
        wxyz=(0, 0, 1, 0)
    )

    # 在 GUI 中添加一个显示计算耗时的数字框
    timing_handle = server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

    # 进入主循环
    while True:
        # 记录开始时间
        start_time = time.time()

        # 调用封装好的函数，计算同时满足两个目标的 IK 解
        solution = pks.solve_ik_with_multiple_targets(
            robot=robot,
            target_link_names=target_link_names,
            target_positions=np.array([ik_target_0.position, ik_target_1.position]),
            target_wxyzs=np.array([ik_target_0.wxyz, ik_target_1.wxyz]),
        )

        # 计算耗时并更新到 GUI
        elapsed_time = time.time() - start_time
        # 使用指数加权平均平滑显示的耗时
        timing_handle.value = 0.99 * timing_handle.value + 0.01 * (elapsed_time * 1000)

        # 更新 URDF 可视化到新的配置
        urdf_vis.update_cfg(solution)


# 入口
if __name__ == "__main__":
    main()
