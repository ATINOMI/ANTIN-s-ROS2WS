# mini_nav 贡献指南

最后更新：2026-08-27

## 1. 开始之前

提交修改前先阅读：

1. `src/mini_nav/docs/project_status.md`：当前里程碑、已知限制和下一步目标。
2. `src/mini_nav/docs/architecture.md`：模块职责、接口契约、话题和 TF 关系。
3. 工作区根目录的 `AGENTS.md`：构建、测试、代码风格和提交要求。

`mini_nav` 是学习型项目。每次修改都应让一个接口更清楚、更容易验证，并尽量把复杂性留在正确的模块实现内部。

## 2. 选择正确的模块

| 要修改的内容 | 应放在哪里 | 不应放在哪里 |
|---|---|---|
| 地图数据结构、坐标转换、A* 搜索 | `mini_nav_core` | ROS 节点回调或 launch 文件 |
| ROS 消息、参数、QoS、TF、RViz 交互 | `mini_nav_nodes` | `mini_nav_core` |
| Gazebo、Nav2、AMCL、节点组合和默认资源 | `mini_nav_bringup` | 算法实现 |
| RViz 配置 | `src/mini_nav/rviz/` | `mini_nav_bringup/rviz/` 的重复副本 |
| 项目说明、里程碑、设计决策 | `src/mini_nav/docs/` | 代码注释替代正式文档 |

依赖方向保持为：

```text
mini_nav_bringup → mini_nav_nodes → mini_nav_core
```

不要让 `mini_nav_core` 依赖 ROS 2。不要让项目生产代码依赖 `nav2_learning`、`nav2_stvl_demo` 或 `navigation2_tutorials` 的私有文件。

## 3. 推荐工作流

### 3.1 修改前

- 明确要解决的一个问题和一个可验证结果。
- 找到负责该行为的模块和接口，不要从 launch 文件开始堆参数。
- 检查工作区是否已有未提交修改，保留不属于本次任务的改动。
- 如果修改话题、参数、坐标系或启动入口，先同步考虑 `architecture.md` 和 `project_status.md`。

### 3.2 编码时

C++ 使用 C++17、四空格缩进、同一行左大括号，并保持以下编译警告：

```text
-Wall -Wextra -Wpedantic
```

命名约定：

- 包、节点、话题、参数、launch 文件和 frame 使用 `snake_case`。
- ROS 接口文件使用 PascalCase。
- C++ 类型使用 PascalCase，函数和变量使用 `snake_case`。

ROS 代码要求：

- 参数声明使用明确类型，并验证数值范围。
- QoS 根据数据语义显式设置，不能依赖默认值。
- launch 使用 package-share 查找资源，不写只适用于某台机器的绝对路径。
- 不发布静态 `map -> odom` 来伪造定位。
- 不把路径跟踪、速度控制或恢复行为偷偷塞进 A* 模块。

### 3.3 测试时

核心算法的测试放在对应包的 `test/`，C++ 测试文件命名为 `test_<unit>.cpp`。至少覆盖：

- 正常路径。
- 起点或终点越界。
- 起点或终点位于障碍物。
- 无路径。
- 地图原点、分辨率和坐标转换。
- 地图消息尺寸与数据长度不一致等边界输入。

如果增加新的接口，优先在该接口上增加回归测试，而不是只测试更底层的内部函数。

## 4. 标准验证命令

从工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup
source install/setup.bash

colcon test --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup \
  --event-handlers console_cohesion+
colcon test-result --verbose
```

修改 launch 后至少执行：

```bash
ros2 launch mini_nav_bringup official_localization_astar.launch.py --show-args
ros2 launch mini_nav_bringup mini_localization_astar.launch.py --show-args
```

修改仿真或定位后，再做运行时检查：

```bash
source src/mini_nav/scripts/env_mini_nav.sh
ros2 launch mini_nav_bringup mini_localization_astar.launch.py
```

另开终端并同样加载 `src/mini_nav/scripts/env_mini_nav.sh`，检查：

```bash
ros2 node list
ros2 topic info /map
ros2 topic info /scan
ros2 topic info /amcl_pose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_footprint
```

RViz 验收顺序：

1. Fixed Frame 为 `map`。
2. 使用 `SetInitialPose` 设置 AMCL 初始位姿。
3. 确认机器人模型、激光和粒子云可见。
4. 使用项目的 `SetGoal`，不是官方 Nav2 action 的 `Nav2 Goal`。
5. 确认 `/mini_nav/global_path` 生成路径。

## 5. 文档要求

以下改动必须同步文档：

- 新增或删除启动入口：更新 `architecture.md` 和 `project_status.md`。
- 修改话题、参数、QoS 或 TF：更新 `architecture.md` 的接口契约。
- 完成或改变里程碑：更新 `project_status.md` 的日期、验证结果和已知限制。
- 新增可安装地图、RViz 或参数资源：说明源目录和 package-share 安装位置。

文档应描述当前已经验证的行为；未来计划放在“下一里程碑”，不要写进“当前已完成”。

## 6. Git 和提交

不要提交以下生成物：

- `build/`
- `install/`
- `log/`
- 编辑器缓存
- 临时 RViz、地图或日志备份文件

提交应保持一个逻辑变更，使用简短的祈使句标题，例如：

```text
Add AMCL localization launch entry
```

提交或合并请求说明至少包含：

- 受影响的 ROS 包。
- 主要接口、参数或 launch 变化。
- 执行过的构建和测试命令。
- 如果行为涉及 RViz/Gazebo，附上关键日志或截图说明。
- 已知限制和下一步工作。

## 7. 提交前检查清单

- [ ] 修改放在职责正确的模块中。
- [ ] 公共接口、参数、QoS 和 TF 约束已记录。
- [ ] 新行为有针对性的测试或明确记录为什么暂时无法测试。
- [ ] `colcon build` 通过。
- [ ] `colcon test` 和 `colcon test-result --verbose` 通过。
- [ ] 相关 launch 的 `--show-args` 通过。
- [ ] 没有静态 `map -> odom` 伪造定位。
- [ ] 没有提交 build/install/log/编辑器生成文件。
- [ ] `project_status.md` 已记录真实验证结果和剩余限制。
