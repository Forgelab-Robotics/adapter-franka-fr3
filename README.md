# Franka FR3v2 + 原生 Franka Hand

本目录接入 **Franka FR3v2 七轴机械臂 + white Franka Hand**。已完成资产归档、
构型契约、原生 MuJoCo 仿真验收，并提供 Franky/Forge 真机驱动、SDK 最小测试、
fake backend 和真机 workflow 骨架。当前仍未连接真机，所有真机验收项保持待现场确认。

> [!WARNING]
> 本项目未来可向真机发送动作，但不是功能安全系统。当前配置中的 Robot IP、
> System Image、Robot Server 版本和 home/reset 位姿均须现场复核。真机动作前
> 必须使用 PREEMPT_RT、保留 Desk/外部急停，并从低速小步开始。

## 当前资产

| 内容 | 路径 | 状态 |
|---|---|---|
| 官方原始生成 URDF/SRDF、参数、LICENSE/NOTICE | `assets/source/franka_description/` | 原样归档 |
| 仓内自包含 URDF/SRDF | `assets/urdf/` | 仅改 mesh URI |
| FR3v2 + white hand mesh | `assets/meshes/` | 官方文件原样归档 |
| 派生 MJCF | `assets/mjcf/scene.xml` | 可独立加载，7 arm + 总开口 gripper |
| GLB | `assets/glb/` | 转换脚本已准备；等待外部 `robot_asset_pipeline` |
| 来源清单与哈希 | `assets/SOURCE_MANIFEST.json` | 可机器校验 |

权威来源为：

- 仓库：<https://github.com/frankarobotics/franka_description.git>
- tag：`2.9.0`
- commit：`7aeeddc449edf8d62b594f9e36a81da53e7796f9`
- 同步日期：见 `assets/SOURCE_MANIFEST.json`
- 上游 `package.xml` 声明 Apache-2.0；完整上游 `LICENSE` 还保留 BSD-3-Clause
  文本。本仓不删改任何上游 LICENSE/NOTICE，详见
  [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 构型摘要

- 逻辑顺序：`fr3v2_joint1`～`fr3v2_joint7`、`gripper`。
- arm position/velocity：`rad` / `rad/s`；effort：`N·m`。
- `gripper`：两指总开口，单位 `m`，范围 `0～0.08`。
- URDF 两个 finger joint 各为 `0～0.04 m`，第二指 mimic 第一指。
- 数值关节限位的唯一来源：
  `assets/source/franka_description/robots/fr3v2/joint_limits.yaml`。
- wheel 内的 `data/fr3v2_joint_limits.yaml` 是由该 SSOT 逐字节生成的运行时镜像，
  由测试防漂移，不允许手工修改。
- 自动生成的可读限位表：[`config/generated/joint_limits.md`](config/generated/joint_limits.md)。
- 完整轴方向、零位、home/reset 和 TCP 契约见 [description.md](description.md)。

## 第一套 uv 开发环境

环境使用 Python 3.12，并将 Franky 固定为
`franky-control==2.0.0+libfranka.0.17.0`。该 wheel 来自 Franky 官方
Robot Server 9 索引：

```text
https://timschneider42.github.io/franky/whl/by-robot-server-version/9/
```

选择 Robot Server 9/libfranka 0.17.0 是依据当前已知 System Image 5.8.1
和 `franka参数.md` 中的兼容范围；**尚未替代真机握手验证**。若现场 Robot
Server 不是 9，必须修改 `pyproject.toml` 的 Franky index、重新生成
`uv.lock`，不得忽略版本不匹配异常。

安装和检查：

```bash
cd franka_fr3
uv sync --all-groups --frozen
uv run python -c "from importlib.metadata import version; print(version('franky-control'))"
uv run python scripts/validate_assets.py
uv run python -m unittest discover -s tests -v
```

此环境可做无硬件资产和 API 开发。执行 Franky 真机控制前还必须满足：

1. `uname -a` 显示 PREEMPT_RT；
2. 运行用户具有 `rtprio`/`memlock` 权限；
3. Desk 已核对 Robot Server 版本并启用 FCI；
4. 第一阶段仅连接读状态，不发送动作。

## 资产复现

从已锁定的相邻官方仓库重新同步：

```bash
uv run python scripts/sync_franka_description_assets.py \
  --source ../franka_description
uv run python scripts/generate_contract_docs.py
uv run python scripts/generate_mjcf.py
uv run python scripts/validate_assets.py
```

同步脚本会拒绝错误 commit、非官方 origin 或有本地修改的源仓库。运行后
`SOURCE_MANIFEST.json` 会记录同步时间及所有归档文件的 SHA-256。

后续有 `robot_asset_pipeline` 时生成 GLB：

```bash
ROBOT_ASSET_PIPELINE_ROOT=/absolute/path/to/robot_asset_pipeline \
  bash scripts/build_glb_robot_asset_pipeline.sh
```

GLB 是 MJCF 的派生产物，不作为运动学或限位的 SSOT。

## 仿真验收

原生 MuJoCo sweep 不依赖 Forge Runtime：

```bash
cd franka_fr3
uv run --frozen python scripts/validate_assets.py
uv run --frozen python scripts/run_mujoco_sweep.py
uv run --frozen python -m unittest discover -s tests -v
```

它逐轴执行 `near-min → home → near-max → home`，再执行 Franka Hand
`0.08 → 0.04 → 0 → 0.08 m`，同时覆盖 NaN/Inf、超限、错误顺序、未知关节、
重复 action、快速交替、长稳态和 reset。仿真 workflow 与外部依赖说明见
[`examples/03_workflows/sim_basic_motion/README.md`](examples/03_workflows/sim_basic_motion/README.md)。
检查结果见
[`JOINT_SWEEP_CHECKLIST.md`](examples/03_workflows/simulation/JOINT_SWEEP_CHECKLIST.md)。

## 当前边界

- 尚未执行真机连接或动作测试；`--backend fake` 是默认且唯一的 CI 后端。
- Franky 驱动和 CLI：见 `examples/01_sdk_tests/README.md`；Dora 节点入口为
  `src/robots_franka_fr3/node.py`，真机 workflow 见
  `examples/03_workflows/real_basic_motion/`。
- `172.16.0.2`、System Image `5.8.1`、FR3v2 Type Label 和实际 Hand
  状态均标为待现场确认。
- MJCF 当前用官方 collision STL 和 finger primitive 兼作可视化，保留的
  DAE 高精视觉资产供后续 GLB/可视化管线使用。
- `robot_asset_pipeline` 不在本仓库或 `forge_runtime/` 中；GLB 目视验收仍需通过
  `ROBOT_ASSET_PIPELINE_ROOT` 显式指定外部工具目录。源码型 Forge Runtime 已位于
  本仓库根目录 `forge_runtime/`，Dora 端到端 workflow 已完成启动、消息交换和停止验证。
- 不包含 Robotiq 2F85、相机或 RealSense 资产。
