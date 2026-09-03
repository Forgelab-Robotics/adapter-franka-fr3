# FR3 真机基础 workflow

当前目录提供真机 workflow 的配置约定。运行前必须确认：

1. Desk 已解锁、FCI 已启用、Robot Server 与 Franky wheel 匹配；
2. 工作站使用 PREEMPT_RT，用户具备实时权限；
3. 工作区隔离、外部急停可触达，并由第二人监护；
4. 没有运行 `franka_ros2`/Gello 控制链路。

```bash
export FORGE_RUNTIME_ROOT=/absolute/path/to/forge_runtime
cd franka_fr3/examples/03_workflows/real_basic_motion
dora build dataflow.yaml --uv
dora run dataflow.yaml --uv
```

默认配置为 fake backend，不会连接硬件。切换真实 FR3 前，复制
`robot.yaml` 为现场配置并显式设置 `robot.backend: franky`、
`control.allow_real_motion: true`。节点发布 `state`，测试动作源通过本地输入别名
`proprio_state` 订阅 `franka_fr3/state`。
首次只运行只读状态，之后按 L0→L4 分级验收。

本 workflow 使用 `position_command_semantics: relative`。驱动在 connect/recover 时
把 fresh state 固定为本次会话的参考零点，action 中的 `position` 是相对偏移；重复
同一个 action 只刷新 watchdog，不会重复累加。本例只发送稀疏 J1 偏移 `+0.02 rad`，
即从启动位置移动到 `q1_start + 0.02 rad`，其余关节和 Hand 保持启动目标。

相对命令仍必须满足机械限位内侧 margin；若启动位置已经靠近对应方向的限位，命令会
被拒绝并安全停止。当前示例设置 `require_homing: false`，因此连接阶段不会移动 Hand。
