# FR3v2 + Franka Hand 仿真检查清单

验证日期：2026-09-02  
仿真入口：`assets/mjcf/scene.xml`  
自动命令：`uv run --frozen python scripts/run_mujoco_sweep.py`

## 自动检查

- [x] 主体 MJCF 与 scene 均可编译，`nq=9, nu=8, njnt=9`。
- [x] joint/actuator/equality/sensor/keyframe 名称唯一且引用有效。
- [x] actuator 顺序为 `fr3v2_joint1..7, gripper`。
- [x] arm ctrlrange 与官方 FR3v2 joint limits 一致。
- [x] J4 上限小于 0，J6 下限大于 0，未错误假设其可归零。
- [x] 七轴完成 `near-min → home → near-max → home`。
- [x] near-limit margin 为 `max(0.05 rad, 关节范围的 10%)`。
- [x] 夹爪完成 `0.08 → 0.04 → 0 → 0.08 m`。
- [x] 两指 mimic 和“状态=两指位置之和”通过。
- [x] NaN、Inf、越界、未知 joint 和错误顺序 action 被拒绝。
- [x] 重复 action、快速交替、长稳态、reset/restart 无 NaN、模型爆炸或 warning。
- [x] workflow 配置无 camera/image 依赖，simulator/task_robot 契约一致。
- [x] `dora build dataflow.yaml --uv` 配置解析通过；并使用仓内
  `forge_runtime/` 完成 `dora run` 端到端启动、消息交换和 Ctrl-C 正常停止。
  运行时曾发现 MuJoCo 节点多余的 `cv2` 导入，已移除该无用导入后复测通过。

详细数值见 `validation_output/simulation_validation.log`。

## 目视与外部集成

- [x] 已生成本地四倍速 sweep 视频：
  `validation_output/simulation_sweep.mp4`（媒体按 `.gitignore` 不提交）。
- [x] 已抽查视频 2 s、8 s、14 s 帧：base 位于世界原点，机械臂和 Franka Hand
  层级完整，米制尺度一致，未见飞模、爆炸或明显几何断裂；当前显示为简化的
  collision proxy，不把白色简化材质当作 GLB 材质验收。
- [ ] GLB 材质、坐标、层级和手指动画目视检查：当前环境无
  `robot_asset_pipeline`，待提供工具目录后执行。
- [x] Forge/Dora 端到端启动/停止：使用
  `FORGE_RUNTIME_ROOT=./forge_runtime dora run ... --uv` 验证三节点 ready、
  action/proprio_state 持续交换及 Ctrl-C 后全部节点成功退出。

未完成的两项不得记录为通过，也不影响原生 MuJoCo 自动验收结论。
