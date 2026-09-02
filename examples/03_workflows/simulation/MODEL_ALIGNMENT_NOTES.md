# FR3v2 + Franka Hand 模型对齐记录

## 来源与坐标约定

- 运动学、惯量、joint axis/limit、collision 和 SRDF 碰撞矩阵来自
  `franka_description` 2.9.0，commit
  `7aeeddc449edf8d62b594f9e36a81da53e7796f9`。
- `fr3v2_link0` 直接位于 MuJoCo world 原点，无额外平移、旋转或缩放；模型单位
  为 m、rad、kg、s。
- 七轴均沿各自 URDF joint frame 的 `+Z` 轴正向旋转。MJCF body pose 逐项复制
  URDF parent→child origin，未改 joint 正方向。
- 运动链为 `link0 → joint1/link1 → … → joint7/link7 → link8 → hand`。
- flange 位于 `fr3v2_link8`；hand 固定绕 Z 轴 `-π/4`，TCP 位于 hand frame
  `[0, 0, 0.1034] m`。

## 原生夹爪

- `fr3v2_finger_joint1/2` 各为 `0～0.04 m`。
- 对外 `gripper` 是两指总开口 `0～0.08 m`；position actuator `gear=2`，
  equality 令第二指复制第一指，proprio 使用两指位置之和。
- `gripper=0.04 m` 的含义是每根手指 `0.02 m`，不是每根 `0.04 m`。

## 仿真控制和碰撞

- 机器人 body 使用重力补偿，arm actuator 为 `kp=450, kv=40`，夹爪为
  `kp=100, kv=10`，以获得可收敛的位置控制；force range 保持官方 effort
  范围/夹爪 URDF effort。
- SRDF 的 `disable_collisions` 转成 MJCF contact exclude，避免官方已确认的
  adjacent/never collision pair 形成假接触。
- `scene.xml` 的地面位于 Z=0，但只作视觉参照、不参与碰撞。单独把 J2 推近
  上限时腕部可能进入安装平面下方；真实任务场景应另建工作台碰撞体和任务级
  安全范围，不能把本资产 sweep 场景当作无碰撞规划场景。

## 检查结论

- MuJoCo 3.12.0 可分别编译主体和 `scene.xml`：`nq=9, nu=8, njnt=9`。
- 自动 sweep 已验证七轴方向、范围、home 回归与 Franka Hand 总开口语义。
- 高精 visual DAE 已归档，但当前 MJCF 为降低转换依赖，使用同源 collision STL
  和官方 finger primitive 兼作显示。
- GLB 仍需外部 `robot_asset_pipeline` 生成并目视复核材质、层级和手指动画；
  当前环境缺少该工具，未伪造通过记录。
