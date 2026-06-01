# 车辆仿真项目改造踩坑记录

## 1. 报错：WaitForTopic 不存在

**报错信息：**
```
ImportError: cannot import name 'WaitForTopic' from 'launch.actions'
```

**原因：** `WaitForTopic` 是 ROS2 Iron 才加的 API，Humble 没有。

**解决：** 改用 `TimerAction` 延迟启动代替：
```python
# ❌ Humble 不支持
from launch.actions import WaitForTopic
WaitForTopic(topic='/scan', timeout=10.0, actions=[...])

# ✅ Humble 兼容写法
from launch.actions import TimerAction
TimerAction(period=4.0, actions=[...])
```

---

## 2. 报错：gazebo_ros2_control 插件加载失败

**报错信息：**
```
[Err] [Model.cc:1160] Exception occured in the Load function of plugin
with name[gazebo_ros2_control] and filename[libgazebo_ros2_control.so].
This plugin will not run.
```

**根本原因：三个叠加问题**

### 2a. GAZEBO_PLUGIN_PATH 为空

Gazebo 通过 `GAZEBO_PLUGIN_PATH` 环境变量搜索 `.so` 插件，
ROS2 Humble 的 setup.bash **不会自动设置它**，导致 Gazebo 找不到
`/opt/ros/humble/lib/` 下的插件。

**解决：** 在 launch 文件中启动 Gazebo 时注入环境变量：
```python
gazebo_env = dict(os.environ)
gazebo_env['GAZEBO_PLUGIN_PATH'] = '/opt/ros/humble/lib'
ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
    output='screen',
    env=gazebo_env,
)
```

### 2b. URDF 中 `<parameters>` 用了 package:// 或 file://

`gazebo_ros2_control` 插件读取 URDF 中的 `<parameters>` 标签时，
用的是 C++ `fopen()`，不认识 `file://` 前缀，只能接受**纯文件系统路径**。

我们之前在 launch 文件里把 `package://` 统一展开成 `file://`，
mesh 路径没问题，但 `<parameters>` 被展开成 `file:///home/.../xxx.yaml`，
导致插件打开文件失败。

**解决：** URDF 中用 `@package_share@` 占位符（展开后不带 `file://`）：
```xml
<!-- mesh 路径：用 package://，展开为 file:// -->
<mesh filename="package://vehicle_description/meshes/body.stl"/>

<!-- 插件参数：用 @pkg_share@，展开为纯路径 -->
<parameters>@vehicle_bringup_share@/config/steering_controller.yaml</parameters>
```

launch 文件中的展开函数需要分别处理：
```python
def _expand_urdf_paths(urdf_text):
    # 1) package://pkg/... → file:///abs/...   (mesh 渲染用)
    urdf_text = re.sub(r'package://([a-zA-Z_][\w]*)/', _to_file_uri, urdf_text)
    # 2) @pkg_share@/...   → /abs/...           (插件 fopen 用)
    urdf_text = re.sub(r'@([a-zA-Z_][\w]*)_share@/', _to_plain_path, urdf_text)
    return urdf_text
```

### 2c. 配置文件移到了新包，URDF 指向了旧包

重构后 `steering_controller.yaml` 从 `vehicle_description` 移到了
`vehicle_bringup`，但 URDF 还引用旧路径。

**解决：** 更新 URDF 中的包名引用。

---

## 3. Gazebo mesh 路径使用绝对路径而非 package://

**现象：** Gazebo 经常报找不到 mesh 文件，即使 `package://` 路径正确。

**原因：** Gazebo Classic 原生不识别 `package://` URI 方案。
虽然 `gazebo_ros` 理论上注册了 handler，但时常不生效。

**正确做法（工业级）：**
- **源文件**用 `package://` 保持可移植
- **运行时**在 launch 文件中展开为 `file://` 绝对路径
- mesh 用 `file://`（Gazebo 识别）
- 插件参数用纯路径（gazebo_ros2_control 识别）

---

## 4. 硬编码路径在多人协作时不可用

**报错信息：**
```
Unable to find mesh file: /home/lhh/vehicle_ws/install/...
```

**原因：** URDF 中写死了 `file:///home/lhh/...` 绝对路径，
换一台机器或换用户就找不到。

**解决：** 上面第 3 点的展开方案。

---

## 5. 包结构调整后 build 失败

**现象：** 把 launch/config/scripts 移到新包后，原来引用
`vehicle_description` 的路径全失效。

**解决步骤：**
1. 每个新包创建独立的 `package.xml` + `CMakeLists.txt`
2. launch 文件中用 `get_package_share_directory('vehicle_bringup')` 替代原来的 `vehicle_description`
3. script 的 package 引用从 `vehicle_description` 改为 `vehicle_control`

---

## 6. syntax error in launch file：list 内部不能写赋值语句

**报错信息：**
```
SyntaxError: invalid syntax. Maybe you meant '==' or ':=' instead of '='?
```

**原因：** 在 Python list 字面量 `[...]` 内部写了 `x = ...` 赋值：
```python
# ❌ 错误：赋值不能放在 list 里面
actions = [
    gazebo_env = dict(os.environ),   # ← 语法错误
    ExecuteProcess(...),
]
```

**解决：** 把赋值移到 list 外面：
```python
# ✅ 正确
gazebo_env = dict(os.environ)
gazebo_env['GAZEBO_PLUGIN_PATH'] = '/opt/ros/humble/lib'
actions = [
    ExecuteProcess(..., env=gazebo_env),
]
```

---

## 7. pkill 杀不掉 Gazebo 残留进程

**现象：** Ctrl+C 退出后，再次启动报端口占用或行为异常。

**原因：** Gazebo 有 `gzserver`（物理引擎）和 `gzclient`（渲染）两个进程，
`Ctrl+C` 有时只杀掉一个。

**解决：**
```bash
# 一键杀掉所有 Gazebo 相关进程
pkill -9 -f "gzserver|gzclient|gazebo"

# 或者更粗暴的
pkill -9 -f gazebo && sleep 1 && pkill -9 -f gazebo
```

---

## 工业级 URDF 路径处理总结

```
┌─────────────────────────────────────────────────────────────┐
│ 源文件 (可移植)            运行时展开              消费方    │
├─────────────────────────────────────────────────────────────┤
│ package://pkg/meshes/... → file:///install/.../  → Gazebo   │
│ @pkg_share@/config/...   → /install/.../         → Plugin   │
└─────────────────────────────────────────────────────────────┘
```

- **`package://`**：展开为 `file://` 绝对路径，给 Gazebo 渲染 mesh 用
- **`@pkg_share@`**：展开为纯路径（无协议前缀），给 C++ 插件 `fopen()` 用
- **不要**在 URDF 源文件中写死绝对路径（`/home/xxx/...`）
- **不要**对 `<parameters>` 标签使用 `file://` 前缀
