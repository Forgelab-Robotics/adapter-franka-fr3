# Franky SDK 最小测试

这些命令与 Dora 解耦。无硬件时使用 `--backend fake`（默认），不会导入或连接
Franky；现场确认 PREEMPT_RT、FCI、Robot Server 版本和外部急停后，才允许使用
`--backend franky`。

```bash
cd franka_fr3
uv run robots-franka-fr3 read-state --backend fake
uv run robots-franka-fr3 move-single-joint fr3v2_joint1 --offset 0.02 --backend fake
uv run robots-franka-fr3 joint-min-max-home --backend fake --dry-run
uv run robots-franka-fr3 gripper-open-close --backend fake
uv run robots-franka-fr3 safety-stop --backend fake
```

真机示例（必须由现场批准后执行，且单独运行）：

```bash
uv run robots-franka-fr3 read-state --backend franky --ip 172.16.0.2
uv run robots-franka-fr3 move-single-joint fr3v2_joint1 --offset 0.01 --backend franky
```

软件 stop 不等价于安全急停；每次真机测试都必须保留 Desk/外部急停和监护人。
