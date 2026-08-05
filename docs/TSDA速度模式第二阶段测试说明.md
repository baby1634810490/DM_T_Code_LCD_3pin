# TSDA速度模式第二阶段测试说明

## 当前目的

验证Chassis使用负RPM向上寻找物理上限，正确解析驱动器 `0x58` 上限输入；触发后立即写 `0RPM`，等待停止ACK和50ms机械稳定，再读取E8/E9建立软件零点。

## 状态流程

```text
TSDA_APP_SEND_HOME_LIMIT_CHECK
→ TSDA_APP_SEND_HOME_START_POSITION
→ TSDA_APP_HOME_FIND_UPPER
→ TSDA_APP_SEND_HOME_STOP
→ TSDA_APP_WAIT_HOME_STOP_ACK
→ TSDA_APP_WAIT_HOME_STABLE
→ TSDA_APP_SEND_HOME_ZERO_POSITION
→ TSDA_APP_WAIT_HOME_ZERO_POSITION
→ TSDA_APP_HOME_MOVE_RETURN
```

寻限运行态按3ms周期执行“写 `-250RPM`、读E8/E9、读 `0x58`”。累计移动达到 `600000count`（300mm）或时间达到90秒时，先执行零速停止，再进入对应ERROR。

## 观察量

```text
tsda_app_state
tsda_status.commanded_speed_rpm
tsda_status.upper_limit_active
tsda_status.lower_limit_active
tsda_status.current_position_raw
tsda_status.homing_start_position_raw
tsda_status.homing_travel_raw
tsda_status.homing_elapsed_ms
tsda_status.position_origin_raw
tsda_status.ready
tsda_status.error_code
```

上限触发后的预期结果：目标RPM立即变为0，状态等待50ms后读取最终位置，`position_origin_raw`保存该位置，然后进入固定速度回 `-100mm` 阶段。此时 `ready` 仍为0。
