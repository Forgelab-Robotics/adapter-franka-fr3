# Dora 单节点测试

驱动节点入口为 `robots-franka-fr3 run`，底层接口使用 Forge 的
`JointState` / `JointCommand`。默认 fake backend 只用于 topic、顺序和错误路径
测试；真机必须显式配置 `backend: franky`，并停止所有 ROS 2/Gello 控制器。

建议覆盖：

- state publisher：确认 8 个 canonical 名称和 SI 单位；
- action subscriber：发送 sparse position action，确认未指定关节保持目标；
- end-effector：发送 `gripper` 总开口米值；
- malformed/expired command：确认拒绝并触发 stop；
- stop/reconnect：确认退出后不发布伪 fresh state。

快速运行 fake smoke test：

```bash
PYTHONPATH=../../src python smoke_fake_driver.py
```
