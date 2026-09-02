# 资产总览

| 路径 | 角色 |
|---|---|
| `source/franka_description/` | 锁定上游输入、LICENSE/NOTICE，不做运行时修改 |
| `urdf/` | 自包含运行时 URDF/SRDF |
| `meshes/` | 官方 FR3v2 + white hand mesh 原样副本 |
| `mjcf/` | 由归档 URDF 整理的 MuJoCo 模型和来源记录 |
| `glb/` | 后续 Web/可视化派生产物 |
| `SOURCE_MANIFEST.json` | 来源、同步时间、修改说明、文件大小与 SHA-256 |

运动学/限位 SSOT 优先级：官方归档参数与原始生成 URDF → 自包含 URDF → 派生
MJCF/GLB。GLB 永远不反向作为控制参数来源。
