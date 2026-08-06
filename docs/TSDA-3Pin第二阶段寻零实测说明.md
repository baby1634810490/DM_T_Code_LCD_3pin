# TSDA 3Pin 寻零与完整流程实测说明

当前版本验证板级上限自动寻零后继续返回 `-100mm`，随后与Chassis一样进入DONE并开放外部目标控制。

## 预期状态流程

```text
TSDA_APP_ENABLE_STABLE_WAIT
→ TSDA_APP_BRAKE_RELEASE_SETTLE_WAIT
→ TSDA_APP_SEND_HOME_START_POSITION
→ TSDA_APP_WAIT_HOME_START_POSITION
→ TSDA_APP_HOME_FIND_UPPER
→ TSDA_APP_WAIT_HOME_STOP_ACK
→ TSDA_APP_WAIT_HOME_STABLE
→ TSDA_APP_WAIT_HOME_ZERO_POSITION
→ TSDA_APP_HOME_MOVE_RETURN
→ TSDA_APP_DONE
```

## FreeMASTER 观察量

- `tsda_app_state`
- `tsda_status.commanded_speed_rpm`
- `tsda_status.output_speed_rpm`
- `tsda_status.upper_limit_active`
- `tsda_status.lower_limit_active`
- `tsda_status.current_position_raw`
- `tsda_status.homing_start_position_raw`
- `tsda_status.position_origin_raw`
- `tsda_status.current_height_mm`
- `tsda_status.homing_elapsed_ms`
- `tsda_status.homing_travel_raw`
- `tsda_status.homing_done`
- `tsda_status.ready`
- `tsda_status.brake_release_command`
- `tsda_status.error_code`

## 实测步骤与通过条件

1. 上电前让滑台离开上限位置，保证PD7未触发。
2. 抱闸释放并稳定后，`commanded_speed_rpm` 应为 `-50`，滑台缓慢向上。
3. 寻限期间总线节拍应为：写 `0x10=-50RPM`、读 `E8/E9`、读 `E4`，不得出现 `0x58` 读取。
4. PD7触发后，当前任务周期下发 `0RPM`，不再下发 `-50RPM`。
5. 收到0RPM ACK后等待50ms，再读取一次E8/E9作为最终软件零点。
6. 建立零点后进入 `TSDA_APP_HOME_MOVE_RETURN`，固定 `+200RPM` 向下返回 `-100mm`。
7. 同时满足位置误差不超过 `±2mm` 且E4不超过 `2RPM` 后：
   - `current_height_mm` 接近 `-100mm`；
   - `homing_done=1`；
   - `ready=1`；
   - 状态进入 `TSDA_APP_DONE`。
8. PB10只更新 `lower_limit_active`，不得改变寻限、回位或DONE状态。

## 上限已触发时上电

如果进入寻限前PD7已经有效，App仍先读取E8/E9寻限起点，随后直接执行0RPM停止流程，不得发送负寻限速度。

## 保护检查

寻限累计原始位置变化达到300mm，或持续90s仍未触发PD7时，应尽力发送0RPM、立即闭合PE15抱闸并进入ERROR。
