# Dora 单节点测试

驱动节点入口为 `robots-franka-fr3 run`，底层接口使用 Forge 的
`JointState` / `JointCommand`。默认 fake backend 只用于 topic、顺序和错误路径
测试；真机必须显式配置 `robot.backend: franky`，并停止所有 ROS 2/Gello 控制器。

自动化测试已覆盖：

- state publisher：确认 8 个 canonical 名称和 SI 单位；
- action subscriber：发送以 connect fresh state 为零点的 sparse position 偏移，
  确认未指定关节保持目标、重复偏移不累加；
- end-effector：发送 `gripper` 总开口偏移米值；
- malformed/expired command：确认拒绝、双通道 stop 和故障锁存；
- stop/reconnect：确认退出后不发布伪 fresh state。

标准 Dora 输出名是 `state`；`action/<source>` 与 `action` 具有相同的启动参考姿态
relative sparse JointCommand 语义，测试用 `action/end_effector` 验证 Hand 总开口
偏移。

快速运行 fake smoke test：

```bash
PYTHONPATH=../../src python smoke_fake_driver.py
```

执行真正的 Dora 事件循环单节点测试（Fake Node，不启动 daemon、不访问网络）：

```bash
cd ../..
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q tests/test_dora_node.py
```
