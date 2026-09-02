# Mesh 归档

此目录原样复制自官方 `franka_description` 2.9.0：

- `robots/fr3v2/visual/link0.dae`～`link7.dae`
- `robots/fr3v2/collision/link0.stl`～`link7.stl`
- `robot_ee/franka_hand_white/visual/hand.dae`
- `robot_ee/franka_hand_white/visual/finger.dae`
- `robot_ee/franka_hand_white/collision/hand.stl`

官方 Franka Hand finger collision 使用 URDF 中的四个 box primitive，**没有单独
finger collision mesh 文件**；本仓不会伪造或从 2F85 资产补齐。所有文件哈希
见 `../SOURCE_MANIFEST.json`，许可见 `../source/franka_description/LICENSE`。
