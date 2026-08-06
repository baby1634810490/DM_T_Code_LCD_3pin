# TSDA 3Pin 第一阶段历史实测记录

本文件记录3Pin抱闸、零速锁轴和板级限位输入的早期分阶段验收结果。当前统一状态机已经删除阶段性零速停留状态；完整构建会在硬件稳定后继续自动寻零、返回-100mm并进入DONE。

## 编译配置

确认 `App_User/tsda_config.h` 中 `TSDA_HARDWARE_VARIANT` 的默认值为 `TSDA_VARIANT_3PIN`，或由编译器定义 `TSDA_HARDWARE_VARIANT=3`。

## FreeMASTER 观察量

- `tsda_app_state`
- `tsda_status.online`
- `tsda_status.servo_enabled`
- `tsda_status.commanded_speed_rpm`
- `tsda_status.output_speed_rpm`
- `tsda_status.brake_release_command`
- `tsda_status.upper_limit_active`
- `tsda_status.lower_limit_active`
- `tsda_status.current_position_raw`
- `tsda_status.ready`
- `tsda_status.homing_done`
- `tsda_status.error_code`

## 预期流程

1. 上电后 PE15 保持低电平，机械抱闸闭合。
2. 完成 E3、清故障、速度模式、0RPM 和使能命令。
3. 进入 `TSDA_APP_ENABLE_STABLE_WAIT`，持续1500ms；PE15仍为低。
4. 1500ms结束后 PE15 变高，`brake_release_command=1`。
5. 进入 `TSDA_APP_BRAKE_RELEASE_SETTLE_WAIT`，完整持续2000ms。
6. 当前版本等待结束后继续进入自动寻零，不再停留在专用FreeMASTER测试状态。
7. 手动触发PD7、PB10时，对应限位状态按5ms采样周期更新。

## 正常失能检查

把 `tsda_command.servo_enable` 改为0。预期先收到0RPM ACK，然后 PE15 变低；保持200ms后发送驱动器失能命令并进入 `TSDA_APP_DISABLED`。

## 异常路径检查

通信发送失败或ACK重试耗尽时，App尽力补发一次0RPM但不等待回包，PE15立即变低并进入 `TSDA_APP_ERROR`。
