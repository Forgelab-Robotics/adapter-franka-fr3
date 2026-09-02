# GLB 交付目录

当前仅准备转换脚本，尚未把 GLB 作为本阶段交付结果。生成命令：

```bash
ROBOT_ASSET_PIPELINE_ROOT=/absolute/path/to/robot_asset_pipeline \
  bash scripts/build_glb_robot_asset_pipeline.sh
```

脚本不含个人绝对路径，默认输入为 `../mjcf/fr3v2_franka_hand.xml`，输出
`franka_fr3.glb` 和 `franka_fr3.provenance.json`。来源记录包含输入/输出
SHA-256、转换工具 origin/commit 以及 dirty 状态。生成后需补做坐标系、比例、
材质、关节层级和 Franka Hand 开合检查。GLB 仅用于可视化，不作为控制/限位
SSOT。
