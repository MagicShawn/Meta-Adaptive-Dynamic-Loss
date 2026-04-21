# 基于动力学映射的无人机底层控制节点 (UAV Low-Level Control Node) 需求与设计文档

## 1. 概述 (Overview)
本模块是无人机控制堆栈中的核心底层执行器。其主要职责是承接上层规划算法（如轨迹规划、MPC、LQR等）输出的**期望速度向量**和**期望加速度向量**，基于微分平坦度（Differential Flatness）与几何控制原理，将其转换为 MAVROS 兼容的目标姿态（四元数）与归一化推力，并负责管理无人机的 OFFBOARD 模式与底层安全状态。

## 2. 系统架构 (System Architecture)
本节点充当“上层控制算法”与“飞控底层（MAVROS）”之间的桥梁，实现全自由度解耦控制。

* **输入端 (From Upper Controller):** 期望速度 $\mathbf{v}_{des} \in \mathbb{R}^3$，期望加速度 $\mathbf{a}_{des} \in \mathbb{R}^3$。
* **输出端 (To MAVROS):** 目标姿态四元数 $\mathbf{q}_{des}$，归一化推力 $T_{norm} \in [0, 1]$。
* **交互接口:** 基于 `mavros_msgs/AttitudeTarget` 的高频发布。

---

## 3. 核心功能模块 (Core Modules)

### 3.1 模块一：运动学与姿态映射 (Kinematics to Attitude Mapping)
**功能描述：** 根据期望加速度确定推力方向（机体 Z 轴），结合期望速度确定机头朝向（偏航角基准），最终构造正交旋转矩阵并转化为四元数。

**算法步骤与数学推导：**

1.  **计算推力向量（机体 $Z_b$ 轴）：**
    期望推力方向应与期望加速度和克服重力所需的加速度合力方向一致。在 ROS 标准 ENU（东北天）坐标系下，重力向量定义为 $\mathbf{g} = [0, 0, -9.81]^T$。
    $$Z_b = \frac{\mathbf{a}_{des} - \mathbf{g}}{||\mathbf{a}_{des} - \mathbf{g}||}$$

2.  **确定偏航基准（虚拟 $X_c$ 轴）：**
    将期望速度投影到水平面，作为机头的期望朝向。
    $$X_c = \frac{[v_{des,x}, v_{des,y}, 0]^T}{\sqrt{v_{des,x}^2 + v_{des,y}^2}}$$
    *异常处理 (Edge Case)：* 当 $||\mathbf{v}_{des,xy}|| \approx 0$（如原点悬停或纯垂直升降）时，无法提取有效的航向向量。**需求约束：** 此时必须锁存并使用上一个有效周期的偏航基准，或维持当前实际航向。

3.  **构造机体正交基（机体 $X_b, Y_b$ 轴）：**
    利用叉乘构造完整的机体坐标系旋转矩阵。
    $$Y_b = \frac{Z_b \times X_c}{||Z_b \times X_c||}$$
    $$X_b = Y_b \times Z_b$$

4.  **矩阵转四元数：**
    组合旋转矩阵 $R = [X_b, Y_b, Z_b]$，并将其转换为对应的期望四元数 $\mathbf{q}_{des} = [q_w, q_x, q_y, q_z]^T$。

### 3.2 模块二：推力映射 (Thrust Mapping)
**功能描述：** 将期望的物理加速度模长转换为 MAVROS 所需的 $[0, 1]$ 无量纲归一化油门值。

**第一阶段标定方案（悬停油门比例）：**
采用线性比例模型。设无人机在标准重力 $g$ 下悬停时的经验油门值为 $K_{hover}$（需暴露为 ROS 参数进行调参）。
$$T_{norm} = K_{hover} \cdot \frac{||\mathbf{a}_{des} - \mathbf{g}||}{9.81}$$

**接口拓展规范：**
为保证后续向精确动力学模型（含螺旋桨拉力系数与电机反电动势模型）升级，推力计算必须封装为独立的方法接口：
* **输入：** 期望加速度向量、当前真实速度（用于空气阻力补偿）。
* **输出：** 经过安全限幅的归一化推力（严格限制在 `0.0` 到 `1.0` 之间）。

### 3.3 模块三：状态机与安全机制 (State Machine & Failsafe)
**功能描述：** 负责 MAVROS 节点的解锁操作、模式切换以及异常状态下的安全兜底机制。

1.  **状态机转移逻辑：**
    * **INIT:** 节点启动，订阅 `/mavros/state`，等待系统 `connected == true`。
    * **PRE_ARM:** 建立连接后，配置 `AttitudeTarget` 掩码以屏蔽角速度指令。必须以至少 **20Hz** 的频率（推荐与主控循环频率一致）发送安全预设值（如：无旋转四元数、推力 $0.0$ 或极小值），以满足飞控进入 OFFBOARD 模式的看门狗前置条件。
    * **OFFBOARD_ACTIVE:** 调用服务端解锁（Arm）并切换至 OFFBOARD 模式。激活主计算循环，高频发布映射后的姿态与推力。
2.  **指令层看门狗 (Command Watchdog)：**
    * **触发条件：** 超过设定阈值（如 $0.5$ 秒）未收到上层控制器的 $\mathbf{v}_{des}$ 和 $\mathbf{a}_{des}$ 更新。
    * **兜底动作：** 立即拦截下发指令，强制重置目标加速度为 $\mathbf{a}_{des} = [0,0,0]^T$，计算对应的水平姿态与悬停推力（$K_{hover}$），使无人机进入**原地悬停**状态，防止因上层节点崩溃导致无人机失控飞出。

---

## 4. 接口规范 (ROS API Specifications)

### 4.1 订阅的外部话题 (Subscribers)
* `/controller/des_vel` (`geometry_msgs/Vector3Stamped`): 上层规划器输出的期望速度。
* `/controller/des_acc` (`geometry_msgs/Vector3Stamped`): 上层规划器输出的期望加速度。
* `/mavros/state` (`mavros_msgs/State`): 飞控连接状态与当前飞行模式反馈。

### 4.2 发布的话题 (Publishers)
* `/mavros/setpoint_raw/attitude` (`mavros_msgs/AttitudeTarget`): 输出至底层飞控的四元数与推力指令。

### 4.3 调用的服务 (Service Clients)
* `/mavros/cmd/arming` (`mavros_msgs/CommandBool`): 发送解锁 (Arm) / 上锁 (Disarm) 请求。
* `/mavros/set_mode` (`mavros_msgs/SetMode`): 请求切换 `OFFBOARD` 模式。

### 4.4 可动态调整的 ROS 参数 (Parameters)
* `~hover_thrust` (double): 悬停油门基准值，默认 $0.4$。
* `~ctrl_rate` (double): 控制主循环发布频率，默认 $50.0$ Hz。
* `~cmd_timeout` (double): 上层控制指令看门狗超时阈值，默认 $0.5$ 秒。

---

## 5. 任务分配与开发计划 (Task Breakdown)

| 任务编号 | 任务名称 | 具体内容与验收标准 |
| :--- | :--- | :--- |
| **Task 1** | **核心算法与数学库实现** | 编写 C++ 类处理 3.1 节与 3.2 节的数学转换。利用 Eigen 库实现向量投影与叉乘，处理速度趋近零时的偏航锁存异常。完成单元测试验证四元数与推力输出。 |
| **Task 2** | **ROS 通信与状态机封装** | 实现第 4 节的 Pub/Sub 接口与服务客户端。完成 INIT -> PRE_ARM -> OFFBOARD 的逻辑切换，并实现基于 Timer 的指令看门狗兜底逻辑。 |
| **Task 3** | **仿真联调与参数标定** | 在 Gazebo + PX4 SITL 闭环仿真中集成节点。重点验证不同飞行机动下的坐标变换正确性，标定 `hover_thrust` 参数，并人为阻断上层指令测试看门狗是否生效。 |

---

## 6. 工作空间落地 (Workspace Layout)

当前 `deploy_diffphy` 目录已按 catkin 工作空间方式初始化，核心包为 `diffphy_uav_controller`：

```text
deploy_diffphy/
    README.md
    src/
        CMakeLists.txt
        diffphy_uav_controller/
            CMakeLists.txt
            package.xml
            include/diffphy_uav_controller/attitude_mapper.h
            src/attitude_mapper.cpp
            src/offboard_controller_node.cpp
            launch/offboard_controller.launch
            config/controller.yaml
```

### 6.1 已实现的最小可开发骨架

1. `attitude_mapper` 库：完成速度/加速度到姿态四元数与推力映射。
2. `offboard_controller_node` 节点：完成订阅、发布、服务调用与基础状态机。
3. 指令看门狗：超时时自动回退到悬停指令。
4. 参数化配置：`hover_thrust`、`ctrl_rate`、`cmd_timeout` 可通过 yaml 与 rosparam 调整。

---

## 7. 构建与运行 (Build and Run)

### 7.1 构建

在 `deploy_diffphy` 根目录执行：

```bash
catkin_make
source devel/setup.bash
```

### 7.2 启动控制节点

```bash
roslaunch diffphy_uav_controller offboard_controller.launch
```

### 7.3 联调前置条件

1. PX4 + MAVROS 已正常启动。
2. `/mavros/state` 可持续输出连接状态。
3. 上层控制器可发布：
     - `/controller/des_vel` (`geometry_msgs/Vector3Stamped`)
     - `/controller/des_acc` (`geometry_msgs/Vector3Stamped`)

---

## 8. 验收标准 (Acceptance Criteria)

1. 节点启动后可稳定发布 `/mavros/setpoint_raw/attitude`，频率不低于配置的 `ctrl_rate`。
2. 在无指令输入超过 `cmd_timeout` 时，输出自动回退到悬停姿态与悬停推力。
3. 飞控可完成 OFFBOARD 切换与解锁流程（在 SITL 环境验证）。
4. 不同水平速度与垂向加速度输入下，姿态映射输出连续、无明显跳变。

---

## 9. 下一阶段开发建议 (Next Iteration)

1. 增加 `attitude_mapper` 的 gtest 单元测试，覆盖零速度、极端加速度与奇异方向。
2. 将推力模型升级为非线性标定模型（含拖曳补偿接口）。
3. 增加动态参数服务，支持飞行中在线调参。
4. 增加 rosbag 回放测试与 CI 编译检查。

## running cmd
cd /home/sean/work_ws/diff_deploy/deploy_diffphy &&  source devel/setup.bash && roslaunch diffphy_uav_controller offboard_controller.launch