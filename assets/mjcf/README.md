# MJCF

- 仿真入口：`scene.xml`
- 机器人主体：`fr3v2_franka_hand.xml`
- 生成记录：`PROVENANCE.json`

复现：

```bash
uv run python scripts/generate_mjcf.py
uv run python scripts/validate_assets.py
```

运动学、惯量、collision、关节 axis/limit 均从归档官方生成 URDF 提取。模型有
7 个 arm actuator 和 1 个 `gripper` actuator；`gripper` 表示两指总开口
`0～0.08 m`，两指通过 equality 同步。

- 所有机器人 body 启用 `gravcomp=1`，position actuator 配置速度反馈阻尼，
  用于模拟 FR3 已开启重力补偿时的关节位置控制，避免无阻尼振荡。
- contact exclude 从同版本官方 SRDF 的 `disable_collisions` 逐项生成。
- 场景地面只作世界原点和尺度参照，不参与碰撞；否则合法的 J2 near-max
  单轴姿态会进入安装平面下方并被误判为模型限位失败。
- keyframe 包含 home、七轴 near-min/near-max 和夹爪 open/half/closed。

MuJoCo 不直接采用保留的 DAE visual。为了让模型不依赖额外格式转换器，当前
MJCF 用同源 collision STL 和官方 finger box primitive 兼作显示。这不改变控制
契约；DAE 仍完整保存在 `../meshes/`，供后续 GLB/高精可视化管线使用。

禁止手改生成 XML；修改生成器或输入后必须重新生成，`PROVENANCE.json` 会记录
生成器、输入与输出 SHA-256。
