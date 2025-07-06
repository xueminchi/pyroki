# PyRoKi 碰撞球体可视化功能

## 概述

本功能允许在PyRoKi的可视化界面中显示机器人的碰撞几何体（如球体、胶囊体等），帮助用户更好地理解碰撞检测的工作原理。

## 功能特性

1. **实时碰撞球体显示**: 在机器人运动过程中实时显示碰撞几何体
2. **GUI控制**: 通过复选框控制是否显示碰撞球体
3. **半透明线框显示**: 碰撞球体以半透明红色线框形式显示，不会遮挡机器人模型
4. **动态更新**: 碰撞球体会根据机器人的当前配置自动更新位置和形状

## 使用方法

### 1. 运行示例

```bash
python examples/04_ik_with_coll.py
```

### 2. 界面操作

- **Show URDF Model**: 控制是否显示机器人URDF模型
- **Show Collision Spheres**: 控制是否显示碰撞球体
- 拖动IK目标控制器来移动机器人末端执行器
- 拖动障碍物控制器来移动球形障碍物

### 3. 代码实现

```python
# 创建碰撞检测模型
robot_coll = RobotCollision.from_urdf(urdf)

# 添加GUI控制
show_collision_spheres_handle = server.gui.add_checkbox("Show Collision Spheres", initial_value=False)

# 在主循环中更新碰撞球体可视化
if show_collision_spheres_handle.value:
    # 获取当前配置下的碰撞几何体
    collision_geom = robot_coll.at_config(robot, jnp.array(solution))
    
    # 转换为trimesh网格
    collision_mesh = collision_geom.to_trimesh()
    
    # 创建或更新可视化
    if collision_sphere_handle is None:
        collision_sphere_handle = server.scene.add_mesh_simple(
            "/collision_spheres", 
            vertices=collision_mesh.vertices,
            faces=collision_mesh.faces,
            color=(255, 100, 100),  # 红色
            wireframe=True,
            opacity=0.3  # 半透明
        )
    else:
        # 更新现有网格
        collision_sphere_handle.vertices = collision_mesh.vertices
        collision_sphere_handle.faces = collision_mesh.faces
```

## 技术细节

### 碰撞几何体类型

PyRoKi支持以下碰撞几何体类型：

1. **Sphere (球体)**: 用于表示球形碰撞体
2. **Capsule (胶囊体)**: 用于表示圆柱形碰撞体
3. **HalfSpace (半空间)**: 用于表示平面障碍物
4. **Heightmap (高度图)**: 用于表示地形

### 可视化方法

- 使用`add_mesh_simple`方法而不是`add_mesh_trimesh`，因为前者支持颜色和线框属性
- 碰撞几何体通过`to_trimesh()`方法转换为trimesh网格
- 使用半透明红色线框显示，便于观察而不遮挡机器人

### 性能考虑

- 碰撞球体可视化会增加计算开销
- 建议只在需要时启用此功能
- 可以通过GUI复选框动态控制显示/隐藏

## 故障排除

### 常见问题

1. **类型错误**: 确保将numpy数组转换为JAX数组传递给`at_config`方法
2. **可视化不显示**: 检查是否正确设置了颜色、线框和透明度属性
3. **性能问题**: 如果性能较慢，可以降低碰撞网格的分辨率

### 调试技巧

```python
# 添加错误处理
try:
    collision_geom = robot_coll.at_config(robot, jnp.array(solution))
    collision_mesh = collision_geom.to_trimesh()
    print(f"Mesh: {collision_mesh.vertices.shape} vertices, {collision_mesh.faces.shape} faces")
except Exception as e:
    print(f"Error: {e}")
```

## 扩展功能

可以进一步扩展此功能：

1. **不同颜色**: 为不同类型的碰撞几何体设置不同颜色
2. **碰撞状态**: 根据碰撞状态改变颜色（如碰撞时变红）
3. **距离显示**: 显示碰撞几何体之间的距离
4. **动画效果**: 添加碰撞时的动画效果

## 总结

碰撞球体可视化功能为PyRoKi用户提供了强大的调试和教学工具，帮助理解碰撞检测算法的工作原理，提高机器人运动规划的可视化效果。 