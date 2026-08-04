# TSDA 速度模式第一阶段测试说明

## 本阶段目标

只验证 Chassis 驱动器能够通过 CAN1 进入速度模式，在目标速度为0时使能并锁轴。自动寻限、目标高度和速度规划尚未实现。

初始化命令顺序：

1. 读取 `E3`，确认驱动器在线。
2. 写 `0x4A=0` 清故障。
3. 写 `0x02=0x00C4` 和 `0x0A=0x0101`。
4. 写 `0x10=0RPM`。
5. 写 `0x00=1` 使能。
6. 等待1500ms后进入零速测试。

除首次读状态外，相邻初始化命令至少间隔10ms。零速测试完整周期为3ms：写0RPM、读E8/E9、读E4各占1ms。

## FreeMASTER 变量

- `tsda_app_state`：最终应稳定在 `TSDA_APP_ZERO_SPEED_TEST`。
- `tsda_status.online`：应为1。
- `tsda_status.servo_enabled`：应为1。
- `tsda_status.ready`：本阶段必须保持0。
- `tsda_status.commanded_speed_rpm`：必须保持0。
- `tsda_status.output_speed_rpm`：静止时应接近0。
- `tsda_status.current_position_raw`：手动施加允许范围内的外力时应能观察到变化。
- `tsda_status.tx_count`、`rx_count`：应持续增加。
- `tsda_status.rx_drop_count`：应保持0。
- `tsda_status.error_code`：应为 `TSDA_APP_ERROR_NONE`。
- `tsda_status.run_send_phase`：应按 `0、1、2` 循环。

## 实机判定

通过条件：电机不发生持续转动，Chassis 驱动器自动释放抱闸，伺服使能后能够锁轴；E8/E9 和 E4 均持续更新；状态机不进入寻限，`ready` 不变为1。

如需主动退出测试，在 FreeMASTER 把 `tsda_command.servo_enable` 写为0。App 会先再次确认 `0RPM`，收到 ACK 后失能，状态进入 `TSDA_APP_DISABLED`。

## 实测结果

2026-08-04 实机验证通过：状态机进入 `TSDA_APP_ZERO_SPEED_TEST`，驱动器在线，目标速度和 E4 反馈均为 `0RPM`；Chassis 抱闸自动释放，电机锁轴，没有缓慢下滑或持续转动。
