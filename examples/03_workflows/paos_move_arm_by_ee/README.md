# PAOS → FR3v2 末端运动链路

该目录把 `move-arm-by-ee` Quick Start 的稳定 Tool 协议适配到本仓库的
**Franka FR3v2 + Franka Hand**：

```text
PAOS Agent / ForgeToolClient
  → gateway
  → relative_pose_policy（fresh joint state 上做 FK/相对位姿解析）
  → motion_action_policy
  → motion_server（IK、规划、终态残差）
  → joint_trajectory_controller
  → robots_franka_fr3 Dora node
  → Franky/libfranka
  → FR3 真机
```

夹爪链路为：

```text
gripper.set_opening → gripper_action_policy → gripper_action_controller
  → robots_franka_fr3 → Franka Hand
```

上层只使用三个稳定 Tool ID：

- `motion.resolve_relative_pose`（Query，不运动）；
- `motion.move_pose`（Action，绝对 TCP pose）；
- `gripper.set_opening`（Action，两指总开口米值）。

## 当前验证结论

- `franka_fake` 已跑通 Tool context readiness、`resolve_relative_pose`、1 cm
  `move_pose`、Action status/result 终态及 `gripper.set_opening(0.04)`。
- fake 末端动作结果：`SUCCESS`，末端位置残差约 `0.00043 m`；夹爪结果
  `SUCCESS/reached_goal=true`。
- 自然语言 PAOS Agent 也已在 fake profile 上完成 live context 发现、fresh
  `resolve_relative_pose` 和 base +Z 5 mm `move_pose`；同一 invocation 的终态为
  `succeeded/SUCCESS`，位置残差约 `0.000198 m`。
- 仓库已有 Franky 真机 SDK 的读状态、J1 正负小步、单轴范围、Hand 和软件 stop
  日志；但 **PAOS→FR3 真机 Action 尚未执行**。
- 本次现场预检时主机为 `PREEMPT_DYNAMIC`、`rtprio=0`，且 `172.16.0.2:1337/1338`
  不可达，因此按 fail-closed 原则停在真机启动前。不能据 fake 结果宣称真机链路已通过。

## 与原 Quick Start 的关系

`skill/` 复用用户提供的 `move-arm-by-ee 0.2.0` 的 `SKILL.md`、Tool 编排和通用
Motion/Gripper provider 拓扑，但使用当前 PAOS Runtime 可校验的锁定 Node 格式，并新增
`franka_fake` / `franka_real` profile。它安装为独立名称
`move-arm-by-ee-franka-fr3`，不会覆盖已有的 Piper Skill。

FR3 Dora 节点来自当前源码仓库，而不是不可审计的临时二进制。因此启动前需要设置：

```bash
export FRANKA_FR3_PROJECT="$(pwd)"  # 当前目录必须是 franka_fr3/
export PHYAGENTOS_ROOT=../PhyAgentOS
```

## 1. 构建和安装

```bash
cd franka_fr3
bash examples/03_workflows/paos_move_arm_by_ee/build_skill_bundle.sh

PAOS_BIN=/path/to/paos
"$PAOS_BIN" skill install \
  dist/skills/move-arm-by-ee-franka-fr3-0.1.2.tar.gz --local --yes
"$PAOS_BIN" skill inspect move-arm-by-ee-franka-fr3
```

Skill 锁定七个通用 Forge Node。若目标机尚未安装这些 Node，PAOS 会按 manifest 从
Registry 获取；离线机器应先安装匹配的 Node archives，不能绕过 SHA-256 锁。

## 2. 先跑 fake

```bash
export FRANKA_FR3_PROJECT="$(realpath .)"
"$PAOS_BIN" skill start move-arm-by-ee-franka-fr3 --profile franka_fake
"$PAOS_BIN" skill status move-arm-by-ee-franka-fr3

# 该脚本会在 fake 上做 base +Z 1 cm，再把总开口设为 4 cm。
/path/to/PhyAgentOS/.venv/bin/python \
  examples/03_workflows/paos_move_arm_by_ee/smoke_fake_tool_chain.py --execute

"$PAOS_BIN" skill stop move-arm-by-ee-franka-fr3
```

也可在 fake profile 下验证自然语言 Agent：

```bash
"$PAOS_BIN" agent -m \
  "使用 FR3v2，在 base 坐标系沿正 Z 移动 1 cm，保持末端姿态，速度和加速度 scale 都为 0.05，位置容差 1 cm、姿态容差 0.05 rad"
```

Agent 必须先读 live context，再执行 fresh resolve → move；Action admission 不代表完成，
必须按 invocation ID 查到 `succeeded` terminal result。

已验证的 Agent 动作路径是完整的；当时只有任务归档返回值遇到 PAOS 的
`AgentTaskRecord is not JSON serializable` 序列化问题。该问题不影响已取得的
Tool terminal result，但会使本次 Agent task 的 evidence bundle 不完整。

## 3. 真机只读门禁

不要直接启动 `franka_real`。先完成：

```bash
uname -a                         # 必须是经验证的 PREEMPT_RT
ulimit -r                        # 必须大于 0
ip -brief address
ip route

uv run robots-franka-fr3 read-state \
  --backend franky --ip 172.16.0.2 --gripper-server-version 3 \
  --samples 20 --period 0.1 --max-age 0.5 --max-read-ms 500 \
  --log validation_output/paos_franka/read_state.jsonl
```

同时人工确认：

1. Desk/FCI、Robot Server 版本和实际 IP；
2. 工作区隔离，整个扫掠体积无碰撞；
3. 外部急停可达且有第二人监护；
4. `franka_ros2`、Gello 及其它控制客户端均已停止；
5. 当前 TCP/负载与 `fr3v2_hand_tcp` 契约一致。

## 4. 真机 profile

满足上述条件后，显式设置双门禁并启动。启动本身只连接/读状态，不做 Hand homing：

```bash
export FRANKA_FR3_PROJECT="$(realpath .)"
export PAOS_FRANKA_REAL_ACK=I_UNDERSTAND_THIS_MOVES_FR3
"$PAOS_BIN" skill start move-arm-by-ee-franka-fr3 --profile franka_real
"$PAOS_BIN" skill status move-arm-by-ee-franka-fr3
```

真机首次 Action 不能使用含糊的“测试一下”。由现场人员明确批准方向、坐标系和距离，
建议从 base frame 单轴 **1～2 mm**、`velocity_scale=0.03`、
`acceleration_scale=0.03` 开始。完成后立即核对 terminal result、实际姿态和视频，再决定
是否继续。当前 schema 还强制：单次各平移分量不超过 10 cm、各旋转向量分量不超过约
10°、速度/加速度 scale 不超过 0.1、Hand 总开口不超过 0.08 m。

停止：

```bash
"$PAOS_BIN" skill stop move-arm-by-ee-franka-fr3
```

Action cancel 已从 `trajectory_cancel` / `gripper_cancel` 映射到 FR3 node 的双通道软件
`stop()`；但软件 stop 不是功能安全急停，取消被接受也不等于机械臂已停止，仍需持续查询
同一 invocation 至 terminal result，并现场观察。

## 已知边界

- 通用 trajectory controller 以 waypoint 驱动 Franky 的轨迹级 `JointMotion`；fake
  成功不证明真机跟踪容差一定通过，必须以 1～2 mm 现场 commissioning 验证。
- Motion server 做 FK/IK、轨迹和终态残差，不提供环境碰撞场景；schema 限幅也不等于
  碰撞安全。
- Franka Hand 当前只做位置控制，不验证抓住物体；`STALLED` 是失败。
- `franka_real/robot.yaml` 中 IP 是已知候选值 `172.16.0.2`，使用前必须在 Desk 复核。
