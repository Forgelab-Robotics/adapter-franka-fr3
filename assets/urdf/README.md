# URDF / SRDF

- `fr3v2_franka_hand.urdf`：运行时自包含版本。
- `fr3v2_franka_hand.srdf`：上游原文件逐字节副本。
- 未修改原文件位于 `../source/franka_description/urdfs/`。

URDF 唯一修改：

```text
package://franka_description/meshes/
→ ../meshes/
```

除此之外结构、joint/link 名、origin、axis、inertial、limit、mimic 和几何数值
均不改变。`scripts/validate_assets.py` 会反向替换路径并与原文件逐字节比较。
