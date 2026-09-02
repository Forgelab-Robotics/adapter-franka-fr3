# 锁定的上游输入

此目录只保存生成当前交付资产所需的最小官方输入子集：

- 原始生成 `fr3v2_franka_hand.urdf/.srdf`；
- FR3v2 joint limits、kinematics、dynamics、inertials；
- Franka Hand inertials；
- 上游 `package.xml`、完整 `LICENSE` 和 `NOTICE`。

来源固定为官方 `franka_description` tag `2.9.0`、commit
`7aeeddc449edf8d62b594f9e36a81da53e7796f9`。mesh 为体积较大的同版本输入，
按原目录结构放在 `../../meshes/`，并同样纳入根 `SOURCE_MANIFEST.json`。

不要直接编辑本目录。升级只能通过 `scripts/sync_franka_description_assets.py`
显式修改锁定版本、重新同步并重新审计许可证。
