# TSDA速度规划阶段设计与测试说明

## 目标

本阶段只在 `TSDA_APP_DONE` 中接入 `position_velocity_planner.c`。自动向上寻限继续使用固定负转速，建立零点后继续使用固定 `100RPM` 回到 `-100mm`；位置和E4速度都停稳后才置 `ready=1`，开放协议控制和反馈。

通用规划器文件保持原样。`App_User/tsda_slide_motion.c` 负责保存浮点规划速度、零速保持回差、软件行程边界以及用户速度到驱动器RPM的换算。

## 协议接口

通信层只写以下全局结构体：

```c
typedef struct
{
    float target_height_mm;
    float max_speed_mm_s;
    float acceleration_mm_s2;
} TSDA_SlideControl;
```

通信层只读以下全局结构体：

```c
typedef struct
{
    float current_height_mm;
    float current_speed_mm_s;
} TSDA_SlideFeedback;
```

全局对象为 `tsda_slide_control` 和 `tsda_slide_feedback`。原有 `tsda_command` 只保留 `servo_enable`，目标高度不再有第二个写入来源。

默认控制参数为 `-100mm`、`20mm/s`、`80mm/s^2`。目标高度由App钳位到 `[-300,0]mm`；最大速度和加速度的合法性由通信层保证，执行层不重复清洗。

## 坐标和换算

- 上限软件零点为 `0mm`，向下为负，软件下限为 `-300mm`。
- 用户速度向上为正、向下为负。
- 实机驱动器负RPM向上、正RPM向下。
- E8/E9按 `2000count/mm` 换算为浮点真实高度。
- E4反馈按 `current_speed_mm_s = -E4 * 5 / 60` 换算。
- 规划速度按 `target_rpm = round(-planned_velocity_mm_s * 12)` 换算。

只有最后的RPM发送发生整数取整，`planned_velocity_mm_s` 始终保留浮点累积，因此低加速度不会因每拍不足1RPM而永久停在零速。

## 状态流程

```text
初始化速度模式
→ 使能并等待Chassis自动释放抱闸
→ 固定-250RPM寻找上限
→ 上限触发后写0RPM
→ 等待停止ACK和50ms机械稳定
→ 读取E8/E9建立0mm软件零点
→ 固定100RPM回到-100mm
→ 位置误差≤2mm且|E4|≤2RPM
→ ready=1
→ DONE实时速度规划
```

自动回位超过20秒时，App尽力发送一次 `0RPM`，不等待ACK，立即锁存 `TSDA_APP_ERROR_HOME_TIMEOUT`。

## DONE通信节拍

`MotorTask` 每1ms调用一次App，DONE状态按三个相位循环：

1. 快照实时目标高度、最大速度和加速度，推进一次PVP并写0x10目标RPM。
2. 读取E8/E9，更新真实浮点高度。
3. 读取E4，更新实际速度反馈。

同一个写速度相位只使用一份参数快照。目标或约束发生变化时不重置规划速度，由PVP连续减速、调整限速或平滑反向。

## 零速保持和软件边界

- 位置误差进入 `2mm` 且规划速度接近零、`|E4|<=2RPM` 时进入保持。
- 保持期间误差不超过 `3mm`，持续发送 `0RPM`。
- 误差超过 `3mm` 时退出保持并从零规划速度重新启动。
- 实际高度到达 `0mm` 或 `-300mm` 后，禁止继续向边界外运动，但允许反向离开。
- 真实反馈高度不钳位，短暂过冲可以在通信层和FreeMASTER中观察。
- 物理下限只读取，不参与正常运动控制。

## FreeMASTER测试

先空载、低速测试，确认机械行程内无人和障碍物。观察：

```text
tsda_app_state
tsda_status.ready
tsda_status.error_code
tsda_status.commanded_speed_rpm
tsda_status.output_speed_rpm
tsda_status.planned_velocity_mm_s
tsda_status.motion_hold_active
tsda_slide_control.target_height_mm
tsda_slide_control.max_speed_mm_s
tsda_slide_control.acceleration_mm_s2
tsda_slide_feedback.current_height_mm
tsda_slide_feedback.current_speed_mm_s
```

测试顺序：

1. 上电后确认反馈保持0，自动回到约 `-100mm` 且E4停稳后 `ready=1`。
2. 保持默认 `20mm/s`、`80mm/s^2`，把目标从 `-100mm` 改为 `-150mm`。命令RPM应为正，反馈速度应为负。
3. 运动中把目标改为 `-50mm`。规划速度应连续减速、过零后变正，命令RPM变负。
4. 把加速度改为 `5mm/s^2` 后执行长距离运动。规划速度应逐拍累积，整数RPM最终离开0。
5. 目标误差在2mm内且E4停稳后，`motion_hold_active=1`；目标变化不超过3mm时继续0RPM，超过3mm后重新规划。
6. 分别把目标写成大于0和小于-300的值，确认写回值被钳位到0和-300。
7. 接近软件边界时确认不会继续向外发送速度，但反向目标能够离开边界。

实机通过标准：CAN收发计数持续增长、`rx_drop_count=0`、无ERROR；速度方向、加速度斜率、保持回差和反馈符号均符合本说明。
