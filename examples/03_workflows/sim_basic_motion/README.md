# FR3v2 + Franka Hand 仿真基础动作

本 workflow 不包含相机或图像节点。逻辑 action/proprio 顺序固定为
`fr3v2_joint1`～`fr3v2_joint7`、`gripper`；机械臂单位为 rad，`gripper`
是两指总开口，单位 m，范围 `0～0.08`。

Dora 测试动作源不依赖配置中的绝对 home：它把首个 fresh `proprio_state` 固定为
启动参考姿态，再依次请求 J1 的 `0 → +0.05 → 0 → -0.05 rad` 相对偏移。由于
TaskRobot/MuJoCo 的底层 position actuator 接受绝对目标，转换只在动作源内部完成；
上层测试轨迹始终以启动姿态为零点。

## 1. 原生 MuJoCo 验收（推荐先运行）

```bash
cd franka_fr3
uv run --frozen python scripts/validate_assets.py
uv run --frozen python scripts/run_mujoco_sweep.py
```

sweep 对每个机械臂关节执行 `near-min → home → near-max → home`。near limit
margin 固定为 `max(0.05 rad, 关节范围的 10%)`。J4/J6 的零点不合法，因此
“归零”是回到安全 home 中对应的 J4/J6 基准，不发送全零整机姿态。

生成四倍速验收视频（需要 `ffmpeg` 和可用的 MuJoCo 无头渲染后端）：

```bash
MUJOCO_GL=egl uv run --frozen python scripts/run_mujoco_sweep.py \
  --video validation_output/simulation_sweep.mp4
```

## 2. Forge/Dora workflow

本仓库上级目录已提供 Forge Runtime 源码（`robots_adapter/forge_runtime/`），
也可替换为其它兼容版本。`FORGE_RUNTIME_ROOT` 必须指向包含
`packages/nodes/simulators/mujoco/main.py` 和 `packages/nodes/task_robot/main.py`
的 Forge Runtime 源码目录：

```bash
cd /path/to/robots_adapter
export FORGE_RUNTIME_ROOT="$(realpath forge_runtime)"
cd franka_fr3/examples/03_workflows/sim_basic_motion
dora build dataflow.yaml --uv
dora run dataflow.yaml --uv
```

数据流：

```text
test_action_source/action → task_robot/action → mujoco
                                      mujoco/proprio_state → task_robot
```

本仓库的 `forge_runtime/` 已包含源码型 MuJoCo、TaskRobot 和测试动作节点；
若改用其它 Runtime，必须确认其版本同时提供这三个节点。仅有打包后的
`mujoco_sim` 二进制不能替代本 workflow 的 TaskRobot 端到端验收。

## 3. GLB

`robot_asset_pipeline` 不包含在本仓库或 `forge_runtime/` 中；它是独立的外部
资产转换工具。准备好该工具目录后，从仓库根目录执行：

```bash
ROBOT_ASSET_PIPELINE_ROOT=/absolute/path/to/robot_asset_pipeline \
  bash scripts/build_glb_robot_asset_pipeline.sh
```

GLB 仅供可视化，URDF/MJCF 和构型契约仍是运动学、限位和单位的 SSOT。
