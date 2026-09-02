# Franka FR3v2 + Franka Hand 构型契约

## 型号和逻辑接口

| 项目 | 契约 |
|---|---|
| 机械臂 | Franka FR3v2，7 DoF |
| 末端 | white Franka Hand，两指平行夹爪 |
| canonical arm names | `fr3v2_joint1`～`fr3v2_joint7` |
| 逻辑末端名称 | `gripper` |
| 动作/状态顺序 | J1, J2, J3, J4, J5, J6, J7, gripper |
| arm 单位 | position `rad`，velocity `rad/s`，effort `N·m` |
| gripper 单位 | 两指总开口 `m`，`0.0～0.08` |

Gello 旧链路的 `fr3_joint1`～`fr3_joint7` 不是资产 canonical name，后续只允许
在遥操适配层显式映射，不得据此重命名官方资产。

## 七轴方向和零位

官方生成 URDF 中 J1～J7 的 `<axis>` 均为各自 joint frame 的 `[0, 0, 1]`；
正方向遵循该局部 `+Z` 轴右手定则。joint frame 的变换来自归档的
`robots/fr3v2/kinematics.yaml`，运行时 URDF 不改变这些变换。

| 关节 | 轴 | 上游限位键 | 零位说明 |
|---|---|---|---|
| `fr3v2_joint1` | local `+Z` | `joint1` | `q1=0` 为运动学零位，合法 |
| `fr3v2_joint2` | local `+Z` | `joint2` | `q2=0` 为运动学零位，合法 |
| `fr3v2_joint3` | local `+Z` | `joint3` | `q3=0` 为运动学零位，合法 |
| `fr3v2_joint4` | local `+Z` | `joint4` | `q4=0` 在系统位置上限之外，不可下发 |
| `fr3v2_joint5` | local `+Z` | `joint5` | `q5=0` 为运动学零位，合法 |
| `fr3v2_joint6` | local `+Z` | `joint6` | `q6=0` 在系统位置下限之外，不可下发 |
| `fr3v2_joint7` | local `+Z` | `joint7` | `q7=0` 为运动学零位，合法 |

因此 `[0, 0, 0, 0, 0, 0, 0]` 不是合法整机姿态。“归零”只能指在安全姿态
下对合法单轴回到 `0`，J4/J6 必须回到项目安全基准值而非数值零。

## position / velocity / effort limit

唯一可编辑来源是：

```text
assets/source/franka_description/robots/fr3v2/joint_limits.yaml
```

其中同时包含 position-based velocity limit 的 `velocity_offset` 与
`deceleration_limit`。人类可读表由脚本生成，见：

**[config/generated/joint_limits.md](config/generated/joint_limits.md)**

URDF、生成表和 MJCF 的一致性由 `scripts/validate_assets.py`/单元测试检查。
真机仍由 Franky/libfranka 应用系统动态限位，不得只按固定表值限速。

## Home / Reset

首套配置使用同一安全基准作为 home 和 reset：

```yaml
fr3v2_joint1:  0.0
fr3v2_joint2: -0.7853981633974483
fr3v2_joint3:  0.0
fr3v2_joint4: -2.356194490192345
fr3v2_joint5:  0.0
fr3v2_joint6:  1.5707963267948966
fr3v2_joint7:  0.7853981633974483
gripper:       0.08
```

这是一组满足官方位置限位的项目默认姿态，不宣称是厂商定义的“零位”或已验证
安全位。首次真机使用前必须在 Desk/人工拖动姿态对照下逐轴小速度验证；未确认前
配置中的 `verification_status` 保持 `pending_on_hardware`。

## Franka Hand 契约

- SDK/Forge/MuJoCo 对外均使用**两指总开口宽度** `0～0.08 m`。
- URDF `fr3v2_finger_joint1`、`fr3v2_finger_joint2` 各为 `0～0.04 m`；
  第二指 mimic 第一指。
- MJCF 使用 equality 保持两指相同单指位移，`gripper` actuator 通过
  `gear=2` 使控制范围直接表达总开口 `0～0.08 m`。
- `0.08 m` 只是资产/名义最大值。真机接入时先 homing，再读取 Hand state 的
  `max_width`；实际值不得大于名义值，差异必须留档。
- finger 在官方 URDF 中以 DAE 作 visual，以四个 box 作 collision；上游不存在
  单独的 finger collision mesh 文件，本仓保持该设计。

## Frame 与 flange → TCP

官方链路中：

1. `fr3v2_link7 → fr3v2_link8`：平移 `[0, 0, 0.107] m`，无旋转；
2. `fr3v2_link8（flange） → fr3v2_hand`：平移 `[0, 0, 0] m`，
   RPY `[0, 0, -π/4]`；
3. `fr3v2_hand → fr3v2_hand_tcp`：平移 `[0, 0, 0.1034] m`，无旋转。

因此 flange → TCP 为：

```text
translation: [0.0, 0.0, 0.1034] m
rpy:         [0.0, 0.0, -0.7853981633974483] rad
```

矩阵表示：

```text
[ 0.70710678  0.70710678  0  0      ]
[-0.70710678  0.70710678  0  0      ]
[ 0           0           1  0.1034 ]
[ 0           0           0  1      ]
```

## 待现场复核

| 项目 | 当前记录 | 状态/获取方式 |
|---|---|---|
| Robot IP | `172.16.0.2` | 待在 Desk → Network 核对 |
| System Image | `5.8.1` | 待 Desk 核对并截图留档 |
| Robot Server | 暂按 `9` 配置 | 首次 Franky 只连接握手确认 |
| 硬件型号 | FR3v2 | System Image < 5.9.1 时 Desk 显示可能不准，需看底座 Type Label |
| 末端 | white Franka Hand | 核对实物、homing、state 与 `max_width` |
| home/reset | 见上文 | 低速逐轴确认后才能改为 verified |
