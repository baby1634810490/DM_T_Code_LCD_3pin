# TSDA速度模式代码设计说明

## 文档入口

当前完整状态机、速度规划接口和FreeMASTER测试步骤见 [TSDA速度规划阶段设计与测试说明.md](./TSDA速度规划阶段设计与测试说明.md)。本文只保留稳定的模块边界。

## 模块边界

| 职责 | 文件 | 约束 |
|---|---|---|
| TSDA厂家CAN协议 | `Driver_User/driver_tsda_servo.c/.h` | 不包含状态机、HAL、CAN通道或硬件变体 |
| 初始化、寻限和运动状态机 | `App_User/app_tsda_servo.c/.h` | 只通过Driver访问寄存器 |
| TSDA速度规划适配 | `App_User/tsda_slide_motion.c/.h` | 保存规划速度、回差和软件边界，不访问CAN |
| 通用PVP数学算法 | `MathFunction_User/position_velocity_planner.c/.h` | 保持通用无状态接口，不读取TSDA数据 |
| CAN1发送和1ms调度 | `Task_User/task_motor.c` | 只注入发送函数并调用AppUpdate |
| CAN1接收入口 | `Bsp_User/bsp_fdcan.c` | 中断只调用既有 `TSDA_AppOnCanRx()` |

## 不变量

- Driver始终保留完整协议能力，不读取配置文件。
- 当前Chassis只绑定CAN1，接收函数数量不增加。
- 负RPM向上、正RPM向下；用户高度和速度向上为正。
- 初始化ACK后的命令间隔为10ms。
- 寻限和自动回位不读取协议速度规划参数。
- `ready=0` 时协议高度和速度反馈均为0。
- `ready=1` 后目标高度、最大速度和加速度实时生效。
- E4是实际速度的唯一反馈来源，不使用位置差分测速和滤波。
