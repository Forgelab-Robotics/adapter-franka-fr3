# Franky SDK 真机最小测试

本测试集与 Dora/ROS 2 解耦，覆盖 `read_state`、单关节小步、单轴验收范围、
Franka Hand、软件 stop、人工触发的停止/断网/FCI 错误观察和恢复。

输出为 JSONL；使用 `--log` 时同时追加到指定文件。`fake` 是默认后端，不连接真机。
改变真机状态的命令必须同时指定 `--backend franky` 和 `--execute`，否则程序拒绝执行。

先运行全部无硬件检查：

```bash
cd ~/workspace/paos_fr3/franka_fr3
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -q

uv run robots-franka-fr3 read-state --backend fake --samples 3 --period 0.05
uv run robots-franka-fr3 move-single-joint fr3v2_joint1 --offset 0.01 --backend fake
uv run robots-franka-fr3 joint-min-max-home fr3v2_joint1 \
  --acceptance-min -0.05 --acceptance-max 0.05 --backend fake --dry-run
uv run robots-franka-fr3 gripper-open-close --backend fake --hold-seconds 0
uv run robots-franka-fr3 safety-stop --backend fake --stop-after 0 --settle-seconds 0
uv run robots-franka-fr3 recover --backend fake
```

建议每个真机场次使用独立日志目录：

```bash
mkdir -p validation_output/sdk_20260903
```

## 1. L1：只读状态

以下命令只连接并采样，不 homing、不启动运动 worker、不发送 arm/gripper 指令：

```bash
uv run robots-franka-fr3 read-state \
  --backend franky --ip 172.16.0.2 --gripper-server-version 3 \
  --samples 20 --period 0.1 --max-age 0.5 --max-read-ms 500 \
  --log validation_output/sdk_20260903/read_state.jsonl
```

| 字段 | 顺序/单位 | 现场核对 |
|---|---|---|
| `position[0:7]` | joint1→joint7，rad | 与 Desk 七轴角度逐轴对照；Desk 若显示度需换算 |
| `velocity[0:7]` | joint1→joint7，rad/s | 静止时应接近 0 |
| `effort[0:7]` | joint1→joint7，N·m | 与负载、姿态趋势一致，不要求静止时为 0 |
| `position[7]` | 两指总开口，m | 与 Hand 实际开口、`gripper.width_m` 对照 |
| `sample_age_ms` | ms | 小于 `--max-age` |
| `read_duration_ms` | ms | 小于 `--max-read-ms` |
| `robot_source_delta_ms` | ms | 除首帧外应大于 0，证明 `RobotState.time` 持续前进 |
| `gripper_source_time_s` | s | Hand 控制器单调时间戳，供连续样本对照 |
| `diagnostics` | mode/errors/control/Hand | `current_errors=[]`，Hand server version 3、温度、`is_grasped` |

Desk 对照属于人工验收，需记录操作者、时间、Desk 截图或视频索引。

## 2. L2：逐轴正负方向小步

`move-single-joint` 只操作指定 arm 关节，不操作夹爪。它以启动时的当前七轴姿态为
测试基准，对指定轴做相对偏移，到位后返回该轴的测试开始位置。默认单程超时 10 s、
容差 0.002 rad；超时会 stop。运行前仍须由现场确认当前姿态及其扫掠空间安全。

先测 `+0.005 rad`，确认方向和返回起始位置，再单独测 `-0.005 rad`：

```bash
uv run robots-franka-fr3 move-single-joint fr3v2_joint1 \
  --offset 0.005 --max-offset 0.02 --dynamics-factor 0.03 \
  --max-step-rad 0.01 --timeout 10 --tolerance 0.002 \
  --backend franky --ip 172.16.0.2 --execute \
  --log validation_output/sdk_20260903/joint1_positive.jsonl

uv run robots-franka-fr3 move-single-joint fr3v2_joint1 \
  --offset -0.005 --max-offset 0.02 --dynamics-factor 0.03 \
  --max-step-rad 0.01 --timeout 10 --tolerance 0.002 \
  --backend franky --ip 172.16.0.2 --execute \
  --log validation_output/sdk_20260903/joint1_negative.jsonl
```

按相同模板依次替换为 `fr3v2_joint2` … `fr3v2_joint7`，日志名改为
`joint2_positive.jsonl` 等。每轴确认正方向、实际位移、返回起始位置和 `errors=[]` 后，
才能进入下一轴。每次只运行一条命令，不使用 shell 循环。

越界行为采用“执行前拒绝”，不把非法目标静默裁剪成另一个真机目标：

```bash
# 预期失败：不会连接真机
uv run robots-franka-fr3 move-single-joint fr3v2_joint1 \
  --offset 0.06 --max-offset 0.05 --backend fake
```

## 3. L3：单轴验收 min → home → max → home

这里的 min/max 是**现场批准的验收范围**，不是机械极限。程序要求显式提供
`--acceptance-min`/`--acceptance-max`，强制一次只测一轴，并检查目标与官方机械限位
保持 `--margin`。机械限位 margin 不能证明无自碰撞或环境碰撞。

以下是围绕当前 home 的保守 commissioning 候选值，必须现场逐轴批准：

| joint | acceptance min | home | acceptance max |
|---|---:|---:|---:|
| fr3v2_joint1 | -0.05 | 0.000000 | 0.05 |
| fr3v2_joint2 | -0.835398 | -0.785398 | -0.735398 |
| fr3v2_joint3 | -0.05 | 0.000000 | 0.05 |
| fr3v2_joint4 | -2.406194 | -2.356194 | -2.306194 |
| fr3v2_joint5 | -0.05 | 0.000000 | 0.05 |
| fr3v2_joint6 | 1.520796 | 1.570796 | 1.620796 |
| fr3v2_joint7 | 0.735398 | 0.785398 | 0.835398 |

每轴先 dry-run，再由现场复核后单独执行：

```bash
uv run robots-franka-fr3 joint-min-max-home fr3v2_joint1 \
  --acceptance-min -0.05 --acceptance-max 0.05 --margin 0.1 \
  --backend franky --ip 172.16.0.2 --dry-run

uv run robots-franka-fr3 joint-min-max-home fr3v2_joint1 \
  --acceptance-min -0.05 --acceptance-max 0.05 --margin 0.1 \
  --dynamics-factor 0.03 --max-step-rad 0.01 \
  --timeout 30 --tolerance 0.003 --retries 2 --retry-delay 1 \
  --recover-on-retry --recovery-delay 1 \
  --backend franky --ip 172.16.0.2 --execute \
  --log validation_output/sdk_20260903/joint1_range.jsonl
```

其余六轴使用表中对应值逐条执行。每个 waypoint 记录 target、actual、耗时和 errors；
每个 `min/home/max/home` phase 失败后会 stop、断开旧 driver、等待、重建连接，然后从
重新读取的当前位置继续同一 phase；已完成的 phase 不重复。默认额外重试 2 次，检测到
当前 Franka 错误时默认执行 automatic recovery。可用 `--retries 0` 禁止重试，或用
`--no-recover-on-retry` 禁止自动恢复。达到重试上限后才停止，不会跳过失败目标进入下一项。
`Ctrl+C` 属于操作员主动取消，不会触发重试；程序会停止当前运动、断开连接并立即退出。

## 4. L2/L3：Franka Hand

命令直接使用 Gripper SDK，不启动 arm worker。流程为 homing → open → half → closed →
reopen，并记录 `max_width`、目标/实际总开口、速度、SDK bool 返回值和耗时：

```bash
uv run robots-franka-fr3 gripper-open-close \
  --gripper-speed 0.02 --width-tolerance 0.003 \
  --backend franky --ip 172.16.0.2 --execute \
  --log validation_output/sdk_20260903/gripper_cycle.jsonl
```

无物体时验证预期夹持失败：

```bash
uv run robots-franka-fr3 gripper-open-close \
  --grasp-width 0.03 --expect-grasp failure \
  --gripper-speed 0.02 --gripper-force 20 --hold-seconds 1 \
  --backend franky --ip 172.16.0.2 --execute \
  --log validation_output/sdk_20260903/gripper_expected_failure.jsonl
```

放置经批准的测试物体后验证成功夹持和保持：

```bash
uv run robots-franka-fr3 gripper-open-close \
  --grasp-width 0.03 --expect-grasp success \
  --gripper-speed 0.02 --gripper-force 20 --hold-seconds 5 \
  --drop-threshold 0.002 \
  --backend franky --ip 172.16.0.2 --execute \
  --log validation_output/sdk_20260903/gripper_grasp.jsonl
```

`possible_drop` 只按保持期间指宽漂移给出提示，不能证明物体没有滑落；物体脱落必须
结合现场观察/视频。`grasp-width` 应按物体尺寸批准，不能盲用示例值。

## 5. stop、人工安全事件和恢复

### 5.1 软件 stop 中断运动

该命令发起 home 附近的小运动，在 `--stop-after` 后调用 `Robot.stop()` 并验证控制退出。
它不会测试或替代外部急停：

```bash
uv run robots-franka-fr3 safety-stop \
  --joint fr3v2_joint1 --offset 0.1 --max-offset 0.3 \
  --stop-after 0.2 --settle-seconds 0.5 --dynamics-factor 0.03 \
  --backend franky --ip 172.16.0.2 --execute \
  --log validation_output/sdk_20260903/software_stop.jsonl
```

若运动在 stop 前已经结束，命令会判本次无效，不会宣称 stop 已验证。

### 5.2 Desk 用户停止、外部急停、断网、FCI 错误

以下观察命令不发送运动。启动后，由现场人员在 30 s 内执行批准的人工动作：

```bash
# 启动后按 Desk 用户停止
uv run robots-franka-fr3 observe-safety --event user-stop \
  --duration 30 --period 0.1 --backend franky --ip 172.16.0.2 \
  --log validation_output/sdk_20260903/user_stop.jsonl

# 启动后按外部急停，同时保存按钮/Desk/视频证据
uv run robots-franka-fr3 observe-safety --event external-estop \
  --duration 30 --period 0.1 --backend franky --ip 172.16.0.2 \
  --log validation_output/sdk_20260903/external_estop.jsonl

# 启动后只断开隔离测试网线，不切换到另一控制客户端
uv run robots-franka-fr3 observe-safety --event disconnect \
  --duration 30 --period 0.1 --backend franky --ip 172.16.0.2 \
  --log validation_output/sdk_20260903/disconnect.jsonl

# 仅按现场批准的可恢复 FCI 故障注入步骤操作
uv run robots-franka-fr3 observe-safety --event fci-error \
  --duration 30 --period 0.1 --backend franky --ip 172.16.0.2 \
  --log validation_output/sdk_20260903/fci_error.jsonl
```

程序只能观察模式、SDK 错误和连接异常；外部急停必须由物理动作、Desk 状态及视频共同
证明。不要为了让日志出现错误而制造碰撞、越限或不受控运动。

### 5.3 异常恢复

先释放急停/用户停止、恢复网络并移除故障源，再单独调用恢复：

```bash
uv run robots-franka-fr3 recover \
  --backend franky --ip 172.16.0.2 --execute \
  --log validation_output/sdk_20260903/recover.jsonl

uv run robots-franka-fr3 read-state \
  --backend franky --ip 172.16.0.2 --samples 10 --period 0.1 \
  --log validation_output/sdk_20260903/post_recovery_state.jsonl
```

通过条件：SDK 返回成功、`has_errors=false`、`current_errors=[]`，后续只读状态正常。
恢复失败时停止测试，不自动重试或继续运动。

## 6. 验收记录

每个 JSONL 旁至少记录：日期、操作者/监护人、机器人资产编号、System Image、
Franky/libfranka 版本、Desk 截图、视频索引、批准的 home/验收范围及 PASS/FAIL。
程序和 fake 测试通过不等于真机验收通过；`PLAN.md` 的复选框应在现场证据审核后再勾选。
