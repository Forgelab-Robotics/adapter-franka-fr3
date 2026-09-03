# 配置说明

- `robot.example.yaml`：首套 Franky/Forge 配置模板；复制为本地配置后使用。
- `generated/joint_limits.md`：由官方 SSOT 自动生成的只读文档。
- `calibration/`：后续保存脱敏的现场校验记录，不提交机器私有配置。

数值关节限位只允许从
`../assets/source/franka_description/robots/fr3v2/joint_limits.yaml` 读取。
修改上游版本后必须重新运行：

```bash
uv run python scripts/generate_contract_docs.py
uv run python scripts/generate_mjcf.py
uv run python scripts/validate_assets.py
```

`verification_status: pending_on_hardware` 只能在完成对应真机验收并留下记录后改为
`verified`。

驱动配置按 `robot`、`safety`、`control` 分区。旧版顶层 timeout 和标量
`dynamics_factor` 仍可读取；新配置应使用独立的 velocity/acceleration/jerk
`relative_dynamics_factors`。`position_command_semantics` 默认为 `relative`：
`JointCommand.position` 是相对于本次 connect/recover fresh state 的偏移，重复相同
action 不累加。`absolute` 仅为显式兼容模式。`allow_real_motion` 默认必须为 false。
