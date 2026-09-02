# 仿真验收产物

- `simulation_validation.log`：原生 MuJoCo sweep 与稳定性验证终端结果，提交仓库。
- `simulation_sweep.mp4`：同一脚本生成的四倍速目视视频；大体积媒体不提交，
  由交付系统或人工复制保存。

复现：

```bash
uv run --frozen python scripts/run_mujoco_sweep.py
MUJOCO_GL=egl uv run --frozen python scripts/run_mujoco_sweep.py \
  --video validation_output/simulation_sweep.mp4
```

GLB 和 Dora 端到端状态以
`examples/03_workflows/simulation/JOINT_SWEEP_CHECKLIST.md` 为准。
