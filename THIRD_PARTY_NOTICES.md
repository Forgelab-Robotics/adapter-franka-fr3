# Third-Party Notices

## Franka Robotics `franka_description`

- 项目：`franka_description`
- 官方仓库：<https://github.com/frankarobotics/franka_description.git>
- 归档版本：tag `2.9.0`
- commit：`7aeeddc449edf8d62b594f9e36a81da53e7796f9`
- Copyright 2023 Franka Robotics GmbH
- 上游 `package.xml` 声明：Apache License 2.0

本仓归档/派生使用范围：FR3v2 + white Franka Hand 的生成 URDF/SRDF、mesh、
joint limits、kinematics、dynamics、inertials，以及由这些输入整理的 MJCF/GLB。

上游根 `LICENSE` 在 Apache License 2.0 全文后还保留了一段 BSD License
文本；为避免丢失任何适用声明，本仓逐字节保存完整文件：

- `assets/source/franka_description/LICENSE`
- `assets/source/franka_description/NOTICE`

运行时 URDF 仅把 `package://franka_description/meshes/` 改为仓内相对路径；
MJCF/未来 GLB 属于可追溯派生产物，生成器和输入/输出 SHA-256 均有记录。
Apache-2.0 不授予 Franka 商标使用权。

## Franky

- 项目：`franky-control`
- 仓库：<https://github.com/TimSchneider42/franky>
- 固定发行版：`2.0.0`
- 许可证：MIT（从 2.0.0 起）

Franky 仅作为 uv 环境中的外部二进制依赖，不把其源码复制进本仓。首套环境从
Franky 官方 Robot Server 9 wheel 索引安装带 libfranka 0.17.0 的构建。
安装包自身携带的许可证仍适用。

## MuJoCo

- 项目：MuJoCo Python package
- 用途：开发/测试阶段编译和校验 MJCF
- 许可证：Apache License 2.0

MuJoCo 是开发依赖，不把其源码或二进制复制进资产目录。
