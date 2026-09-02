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
`robot.yaml` 为现场配置并显式设置 `backend: franky`、`allow_real_motion: true`。
首次只运行只读状态，之后按 L0→L4 分级验收。
