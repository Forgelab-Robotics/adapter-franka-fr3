# Franka FR3v2 + 原生 Franka Hand 接入计划

> 状态：待实施  
> 接入目录：`franka_fr3/`  
> 计划依据：`Robot接入手册 .md`、`Robots接入验收.md`、`franka参数.md`、
> `agilex_piper/`、`franka_fr3_2f85/`、`franka-data-collect/franka-gello/`
> 与 `franka-data-collect/Gello遥操Franka讲解.md`。

## 目标、范围与硬约束

- 接入对象为 **Franka FR3v2 七轴机械臂 + 原生 Franka Hand**，不是 Robotiq 2F85。
- 基础真机驱动使用 [`franky`](https://github.com/TimSchneider42/franky) SDK，
  实现 Forge `BaseRobotDriver`、Dora 节点和真机 workflow。
- 同构遥操复用现有 Gello 工程的：
  `franka_gello_state_publisher`、`franka_fr3_arm_controllers`、
  `franka_gripper_manager`；不接入 RealSense、图像话题或数据采集代码。
- 仿真、Forge 真机驱动和 Gello 遥操统一逻辑动作顺序：
  `fr3v2_joint1`～`fr3v2_joint7`、`gripper`。
- 七轴位置统一使用 `rad`，速度使用 `rad/s`，力矩使用 `N·m`；
  `gripper` 表示两指**总开口宽度**，单位 `m`，范围 `0.0～0.08`。
- `fr3v2_joint*` 是当前官方资产名；Gello 旧链路中的 `fr3_joint*`
  只能通过显式映射兼容，不修改官方资产命名。
- `[0, 0, 0, 0, 0, 0, 0]` 不是 FR3 合法整机姿态。
  “归零测试”按单轴回到运动学零位、其余轴保持安全 home 的方式执行。
- Franky/Forge 真机 workflow 与 ROS 2/Gello 真机 workflow 是两套互斥控制后端，
  **不得同时连接并控制同一台 FR3**。启动脚本必须检测或明确提示该约束。
- 软件 `stop` 不是功能安全急停；所有真机阶段必须保留 Desk/外部急停、
  隔离工作区和现场监护。

## 计划交付目录

```text
franka_fr3/
├── README.md
├── SAFETY.md
├── THIRD_PARTY_NOTICES.md
├── PLAN.md
├── assets/
│   ├── urdf/                  # 官方 FR3v2 + Franka Hand、SRDF
│   ├── meshes/                # 仓内可解析的 visual/collision mesh
│   ├── mjcf/                  # 主模型、scene.xml、mesh
│   └── glb/                   # 可视化交付资产
├── config/
│   ├── robot.example.yaml     # IP、home/reset、安全参数
│   ├── calibration/
│   └── gello/                 # 串口、offset、sign、阻抗和夹爪配置
├── examples/
│   ├── 01_sdk_tests/
│   ├── 02_dora_tests/
│   └── 03_workflows/
│       ├── sim_basic_motion/
│       ├── real_basic_motion/
│       └── homomorphic_control/
├── src/
│   ├── robots_franka_fr3/     # Franky + Forge/Dora 适配器
│   └── gello_ros2/            # 仅保留遥操所需 ROS 2 源码
├── scripts/
├── tests/
└── validation_output/         # 默认不提交大体积视频，只记录索引/校验结果
```

## 1. 资产、开发环境准备

### 1.1 资产归档与来源固定

- [x] 以当前官方 `franka_description` 的 FR3v2 + white Franka Hand 为主来源，
  记录仓库 URL、tag/commit（当前本地参考为 `2.9.0` / `7aeeddc`）、许可证和同步日期。
- [x] 将 URDF 引用的 FR3v2、hand、finger visual/collision mesh 放入
  `franka_fr3/assets/`，消除运行时对相邻 `franka_description/` 仓库的隐式依赖。
- [x] 保留原始 URDF/SRDF，并记录任何为仓内相对路径、仿真兼容性所做的变更。
- [x] 从同一资产源生成/整理 MJCF；准备后续 GLB 转换脚本，所有派生产物可追溯。
- [x] 增加 `THIRD_PARTY_NOTICES.md` 和各资产目录 README，完成开源许可核对。

### 1.2 构型契约固化

- [x] 在 `description.md`/`README.md` 固化七轴名称、顺序、轴方向、零位、
  position/velocity/effort limit、home/reset 位姿和 flange→TCP。
- [x] 关节限位以官方 `fr3v2/joint_limits.yaml` 为单一来源生成配置，
  包含位置相关速度限位；代码中不维护相互漂移的手抄副本。
- [x] 原生夹爪契约固化为总开口 `0～0.08 m`；URDF 中单指 `0～0.04 m`
  通过 mimic/equality 映射，真机启动后通过 homing/state 再核对实际 `max_width`。
- [x] 记录当前待现场复核项：Robot IP `172.16.0.2`、System Image `5.8.1`、
  底座 Type Label 是否确认为 FR3v2，以及实际 Franka Hand 状态。

### 1.3 两套开发环境

- [x] 建立 Python/`uv` 工程，依赖 Forge 公共包、Dora、PyYAML 和 Franky；
  Franky 必须锁定已验证版本/commit，不能仅使用浮动版本。
- [ ] 在无动作条件下验证 Franky 与本机 Python、libfranka、Robot System
  版本兼容性；记录 Franky 实际暴露的 Robot/Gripper 状态、move、stop、
  recovery 和异步接口，禁止按未验证 API 猜测实现。
- [ ] 建立 ROS 2 Humble + `franka_ros2` + Gello 的独立 colcon 环境；
  锁定 `franka_ros2`/libfranka 与 System Image 的兼容矩阵。
- [ ] 只从 Gello 工程复制或引用 `src/` 源码、配置和许可证；排除
  `build/`、`install/`、`log/`、RealSense 包和相机启动项。
- [ ] 将机器相关绝对路径、串口、IP 全部参数化；配置仓库只提交 `.example.yaml`，
  现场配置与日志不得包含凭据或个人目录。

### 1.4 本阶段出口条件

- [x] URDF 在脱离相邻仓库后仍可解析，全部 mesh 路径存在。
- [x] 资产来源、许可证、构型参数和未确认项都有文档记录。
- [ ] Python 无硬件环境可安装并运行单元测试；ROS 2 三个遥操包可完成 colcon build。
- [ ] Franky/libfranka/Robot System 兼容性已通过“连接但不运动”的现场确认；
  若不兼容，先调整锁定版本，不进入真机动作阶段。

## 2. 仿真完成

### 2.1 FR3v2 + Franka Hand MJCF

- [x] 生成/修正 `assets/mjcf/scene.xml` 和主体 MJCF，确保 base 原点、朝向、
  缩放、body 层级、惯量、collision 和 mesh 路径正确。
- [x] 建立 8 个逻辑控制量：7 个 arm position actuator + 1 个 `gripper`；
  Franka Hand 两指由总开口米值驱动，状态回读也返回总开口米值。
- [x] 对齐 URDF、MJCF、Forge 配置中的 joint 顺序、方向、限位和 home；
  对 J4/J6 的非零合法区间单独加回归测试。
- [ ] 生成 GLB 并执行几何/材质/坐标系目视检查；GLB 只作交付可视化，
  仿真 SSOT 保持为 MJCF。
  当前环境未提供 `robot_asset_pipeline`，生成脚本已验证会明确返回退出码 2，
  不以替代工具或伪造产物标记通过。

### 2.2 仿真节点与 workflow

- [x] 参考 `franka_fr3_2f85` 建立 `sim_basic_motion`：
  MuJoCo → `proprio_state` → TaskRobot，测试动作源 → TaskRobot → MuJoCo。
- [x] `simulator.yaml`、`task_robot.yaml` 和测试动作源使用同一个
  `actuator_order`；不配置图像输出或相机节点。
- [x] 增加资产静态检查和 MuJoCo 编译检查：名称唯一、actuator/joint 可解析、
  限位和单位一致、无缺失 mesh。
- [x] 增加自动 joint sweep：逐轴从安全 home 出发，移动到带 margin 的近最小值、
  近最大值，再回到该轴零位或安全基准；夹爪完成开、半开、闭测试。
- [x] 增加稳定性 smoke test，检查 NaN、模型爆炸、持续抖动、穿模、
  action/proprio 顺序错位和夹爪单双指宽度混淆。

### 2.3 本阶段出口条件

- [x] `scene.xml` 可被 MuJoCo 和 Forge/Dora workflow 独立加载。
  MuJoCo 主体/scene 与仓内 `forge_runtime/` 三节点均已验证。
- [x] 七轴与夹爪自动 sweep 全部通过，仿真姿态与官方 URDF/参考图一致。
- [x] `sim_basic_motion` 可复现启动、正常停止，Dora topic 的名称、顺序和单位正确。
  已用仓内 `forge_runtime/` 验证三节点 ready、action/proprio_state 交换及 Ctrl-C 回收。
- [x] 形成 `JOINT_SWEEP_CHECKLIST.md`、模型对齐记录、终端验收日志和仿真视频。

## 3. 真机测试及完成

### 3.1 Franky SDK 最小测试（与 Dora 解耦）

- [ ] `read_state`：连接 FR3，读取 7 轴位置/速度/力矩、错误状态和 Franka Hand 状态，
  核对 SDK 数组顺序、SI 单位、采样新鲜度与真机 Desk 显示。
- [ ] `move_single_joint`：从现场批准的 home 位开始，使用低 dynamics factor、
  小步进逐轴验证正方向、限位裁剪、超时和停止行为。
- [ ] `joint_min_max_home`：只到带安全 margin 的验收范围，不撞机械限位；
  每次只测一轴并返回 home，记录耗时、实际位置和异常。
- [ ] `gripper_open_close`：执行 homing，读取 `max_width`，验证开/半开/闭、
  速度、力、夹持失败和物体脱落等返回状态。
- [ ] `safety_stop`/恢复：验证软件停止、用户停止、断网、FCI 错误、异常恢复；
  同时证明外部急停可用，但不把软件停止描述为安全急停。

### 3.2 Forge Robot Driver 与 Dora 节点

- [ ] 实现 `FrankaFR3Driver(BaseRobotDriver)`：connect/disconnect、fresh state、
  send action、stop、error recovery、gripper、超时和结构化日志。
- [ ] 根据锁定 Franky 版本选择并验证连续目标的实现方式；若 SDK move 调用是
  阻塞/轨迹级 API，则增加命令合并、速率限制和互斥，禁止每个 Dora tick
  无界创建运动任务。
- [ ] 支持稀疏 action：未出现的关节保持最近目标；首次命令从 fresh state 初始化缓存。
- [ ] 所有 action 先检查 finite、名称和 mode，再执行位置/速度/加速度/jerk margin、
  每周期最大步长和夹爪范围检查；拒绝未知关节和过期命令。
- [ ] 将 Franky 异常转换成清晰日志与安全停止；断连后不得继续发布伪新鲜状态，
  reconnect 必须显式执行。
- [ ] 提供 CLI、`robot.example.yaml`、Dora state/action/end-effector 单节点测试，
  以及 `real_basic_motion` workflow。
- [ ] 用 fake/mock backend 覆盖 CI：顺序、单位、裁剪、稀疏命令、过期反馈、
  超时、断连、夹爪宽度转换和生命周期；CI 绝不连接或移动真机。

### 3.3 分级真机验收

- [ ] L0：断电/无控制测试配置、日志、CLI 和错误路径。
- [ ] L1：Desk 解锁后只读状态，不下发动作。
- [ ] L2：夹爪与单关节低速小步动作，现场一人操作、一人监护。
- [ ] L3：安全范围内七轴 near-min/near-max/home 与夹爪全行程。
- [ ] L4：Dora 单节点和 `real_basic_motion` 端到端连续运行、正常停止和断线测试。
- [ ] 每一级均以前一级通过为前提；任何方向、单位、限位或通信异常立即退回整改。

### 3.4 本阶段出口条件

- [ ] SDK 最小测试、Forge driver、Dora 节点和真机 workflow 全部通过。
- [ ] 真机状态与动作的顺序/单位与仿真完全一致，夹爪统一使用总开口米值。
- [ ] 完成七轴安全范围 sweep、夹爪测试、断线/停止测试，保留终端日志与真机视频。
- [ ] README 包含安装、网络、最小测试、Dora、workflow、安全、急停和已知限制。

## 4. 同构遥操

### 4.1 精简和参数化 Gello 链路

- [ ] 复用并归档三个包及其许可证：状态发布器、FR3 关节阻抗控制器、
  原生夹爪管理器；删除相机节点、相机依赖、图像话题和采集脚本。
- [ ] 启动入口仅拉起 T0 状态发布、T1 机械臂控制、T2 夹爪管理，
  使用相对/可配置工作区路径，并提供可靠的信号转发、进程回收和分节点日志。
- [ ] 把 IP、Gello 串口、`best_offsets`、`joint_signs`、gripper raw range、
  阻抗增益、控制频率和超时策略移入 `config/gello/`。
- [ ] 显式实现 `fr3_joint1..7`（旧 Gello/ROS 链路）到
  `fr3v2_joint1..7`（资产/Forge 契约）的按名称映射；控制器不能仅按数组位置盲拷贝。

### 4.2 安全加固与标定

- [ ] 标定七轴 `best_offsets` 和 `joint_signs`，记录标定姿态、原始读数、
  计算方法、日期和操作者；不得直接把现有现场值当成新设备通用值。
- [ ] 标定 Gello 第八轴原始范围到 `[0,1]`，再统一映射到 Franka Hand
  `0～max_width m`；明确选择连续宽度控制还是经验证的二值开闭模式。
- [ ] 首帧到达前保持安全状态；初始对齐使用限速轨迹，并要求主从初姿态接近。
- [ ] 每次控制循环检查最后消息年龄、时间戳、数组长度、finite 和关节范围；
  静默断流超过阈值必须进入已验证的安全停止/hold 行为，而非保持旧目标无限运行。
- [ ] 复核并保守调节 K/D、控制器更新率、速度滤波和碰撞阈值；
  对目标跳变增加每周期限幅，不直接照搬旧机器增益。
- [ ] 记录 MDH、TCP/`t_ee`、rotation、scale：关节同构主链路的
  rotation/scale 记为“不参与控制”，同时记录其原因；TCP 使用官方 Franka Hand 值并实测复核。

### 4.3 仿真与真机遥操验收

- [ ] 仿真链路：Gello publisher → JointState/夹爪百分比桥接 → Dora action → MuJoCo；
  验证七轴方向、幅值、顺序、夹爪和断流行为。
- [ ] 真机链路：Gello publisher → `franka_fr3_arm_controllers` → FR3，
  夹爪百分比 → `franka_gripper_manager` → Franka Hand；此时 Franky/Forge
  真机 driver 必须停止。
- [ ] 按单轴小幅 → 多轴低速 → 工作空间任务逐级验证；对比记录 Gello 目标、
  仿真 state 和真机 state，而不是只看末端是否运动。
- [ ] 测试启动失败、串口断开、25 Hz 输入静默、ROS 时间异常、控制器退出、
  夹爪 action 超时、Ctrl+C 和二次启动，确认不会遗留控制进程。
- [ ] 输出无相机依赖的 `homomorphic_control` 仿真/真机启动说明、
  标定记录、终端验收记录，以及仿真和真机遥操视频。

### 4.4 最终出口条件

- [ ] 同一套 Gello 标定配置在仿真和真机中的七轴姿态、方向和夹爪动作一致。
- [ ] 断流、越界、异常时间戳和进程退出均触发明确、可复现的安全行为。
- [ ] `Robots接入验收.md` 中除相机/VR（本构型明确不在范围）外的适用项全部打勾；
  不适用项注明范围依据，不伪报通过。
- [ ] 所有资产、代码、锁定版本、配置模板、命令、参数来源、日志索引和视频索引完成交付。

## 关键风险与处理原则

| 风险 | 处理原则 |
|---|---|
| System Image 5.8.1 与 Franky/libfranka 不兼容 | 第一阶段先做只连接兼容性门禁，锁定经现场验证的组合后再开发动作链路 |
| FR3/FR3v2 命名混用 | 官方资产使用 `fr3v2_joint*`，Gello 旧名只在适配层出现，并用测试锁定映射 |
| Franky 轨迹 API 不适合高频 Dora action | 先测阻塞、取消和延迟语义，再实现合并/限速；不把每个 tick 直接变成独立轨迹 |
| 原生夹爪单双指单位混淆 | 对外只暴露总开口米值，URDF/MJCF/SDK 边界分别测试 |
| Gello 静默断流仍沿用旧目标 | 在控制循环而非只在订阅回调中检查消息年龄，并验证 stop/hold 行为 |
| ROS 2 遥操与 Franky 同时占用真机 | 两个 workflow 明确互斥，启动前检测/提示，文档和验收脚本均覆盖 |
| 极限 sweep 导致碰撞或奇异姿态 | 仿真可近限位；真机使用现场批准的 margin、单轴执行、逐次回 home，不碰机械硬限位 |
