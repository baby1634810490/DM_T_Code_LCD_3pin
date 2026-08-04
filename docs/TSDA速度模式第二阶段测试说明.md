# TSDA 速度模式第二阶段测试说明

## 本阶段目标

验证 Chassis 在速度模式下以 `+50RPM` 向上寻找物理上限，正确识别驱动器 `0x58` 上限输入，并在触发后下发 `0RPM`、记录停止位置。暂不回到 `-100mm`，也不接受目标高度，`ready` 必须保持0。

## 状态流程

1. 第一阶段初始化和零速使能。
2. `TSDA_APP_SEND_HOME_LIMIT_CHECK`：使能稳定后先读一次 `0x58`。
3. `TSDA_APP_SEND_HOME_START_POSITION`：读取 E8/E9 作为寻限起点。
4. `TSDA_APP_HOME_FIND_UPPER`：按3ms周期执行“写 `+50RPM`、读E8/E9、读 `0x58`”。
5. `TSDA_APP_SEND_HOME_STOP`：上限触发后立即写 `0RPM` 并等待 ACK。
6. `TSDA_APP_SEND_HOME_ZERO_POSITION`：读取停止后的 E8/E9。
7. `TSDA_APP_HOME_UPPER_ZERO_HOLD`：保存软件零点并按“写0RPM、读E8/E9、读E4”保持。

如果累计移动达到 `600000 count`（300mm）或时间达到 `90000ms`，状态机同样先执行零速停止和最终位置读取，然后分别进入 `TSDA_APP_ERROR_HOME_TRAVEL` 或 `TSDA_APP_ERROR_HOME_TIMEOUT`。

## 上机前条件

- 确保滑台不在机械硬限位挤压状态，人员可以随时断电。
- 首次测试尽量让滑台位于上限下方较近位置，缩短低速寻限距离。
- 确认 FreeMASTER 已连接后再观察运动，不写入目标高度相关变量。

## FreeMASTER 变量

- `tsda_app_state`：运动时为 `TSDA_APP_HOME_FIND_UPPER`，成功后为 `TSDA_APP_HOME_UPPER_ZERO_HOLD`。
- `tsda_status.commanded_speed_rpm`：寻限时为 `50`，触发后为 `0`。
- `tsda_status.upper_limit_active`：未触发为0，触发后为1。
- `tsda_status.current_position_raw`：寻限运动期间持续变化。
- `tsda_status.homing_start_position_raw`：本轮寻限起点。
- `tsda_status.homing_travel_raw`：起点到当前位置的原始计数差绝对值。
- `tsda_status.homing_elapsed_ms`：本轮寻限经过时间。
- `tsda_status.position_origin_raw`：成功停止后记录的上限原始位置。
- `tsda_status.homing_done`：成功后为1。
- `tsda_status.output_speed_rpm`：零速保持阶段应回到接近0。
- `tsda_status.ready`：本阶段必须保持0。
- `tsda_status.error_code`：正常为 `TSDA_APP_ERROR_NONE`。

## 通过条件

电机以正方向低速向上运动；上限触发时 `upper_limit_active` 变为1，速度命令立即回到0；电机停止并锁轴，状态进入 `TSDA_APP_HOME_UPPER_ZERO_HOLD`；`homing_done=1`，`position_origin_raw` 保存停止位置，没有进入 ERROR。

