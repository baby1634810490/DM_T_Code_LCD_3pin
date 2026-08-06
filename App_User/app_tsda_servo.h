/**
  ******************************************************************************
  * @file    app_tsda_servo.h
  * @brief   TSDA统一速度模式App - 分硬件能力执行抱闸、寻限和位置闭环。
  *
  * 本阶段在寻限完成基础上建立 [0,-300]mm 用户坐标系。自动回位保持固定
  * 速度，ready=1后的done模式使用MCU浮点速度规划和零速保持回差。
  ******************************************************************************
  */

#ifndef APP_TSDA_SERVO_H
#define APP_TSDA_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "driver_tsda_servo.h"

#define TSDA_APP_POWER_WAIT_MS            (500U)  /*!< App启动后等待驱动器上电稳定。 */
#define TSDA_APP_COMMAND_INTERVAL_MS      (10U)   /*!< 初始化ACK后下一条配置命令的最小间隔。 */
#define TSDA_APP_ACK_TIMEOUT_MS           (300U)  /*!< 初始化读写命令等待匹配回包的超时。 */
#define TSDA_APP_RETRY_MAX                (3U)    /*!< 一条初始化命令超时后的最大重发次数。 */
#define TSDA_APP_ENABLE_STABLE_MS         (1500U) /*!< Chassis使能和自动抱闸释放后的静默稳定期。 */
#define TSDA_APP_HOME_STABLE_MS           (50U)   /*!< 上限触发后，等待机械稳定的时间。 */

/* 位置闭环控制参数 */
#define TSDA_APP_SOFT_LOWER_LIMIT_MM      (300)   /*!< 软件下限距离(mm)，上限为0，下限为-300。 */
#define TSDA_APP_HOME_RETURN_SPEED_RPM    (200)   /*!< 建零后固定速度回到-100mm。 */
#define TSDA_APP_POSITION_TOLERANCE_MM    (2)     /*!< 自动回位到位容差(mm)。 */
#define TSDA_APP_STOP_SPEED_RPM           (2)     /*!< 自动回位停稳E4阈值。 */
#define TSDA_APP_INITIAL_TARGET_MM        (-100)  /*!< 寻限完成后自动下移100mm的目标位置。 */
#define TSDA_APP_DEFAULT_MAX_SPEED_MM_S   (20.0f) /*!< 对外控制默认最大速度。 */
#define TSDA_APP_DEFAULT_ACCEL_MM_S2      (80.0f) /*!< 对外控制默认加速度。 */

/**
  * @brief TSDA App 非阻塞状态机。
  *
  * SEND 状态只负责发送一帧；WAIT 状态等待严格匹配的回包并处理超时重试；
  * HOME_FIND_UPPER 和 DONE 是1ms任务驱动的三相周期状态。
  */
typedef enum
{
	TSDA_APP_POWER_WAIT = 0U,               /*!< 上电等待500ms。 */
	TSDA_APP_SEND_READ_STATUS,              /*!< 发送读E3，建立在线基线。 */
	TSDA_APP_WAIT_READ_STATUS,              /*!< 等待E3的0x2B回包。 */
	TSDA_APP_SEND_CLEAR_FAULT,              /*!< 写0x4A=0清故障。 */
	TSDA_APP_WAIT_CLEAR_FAULT_ACK,          /*!< 等待清故障写ACK。 */
	TSDA_APP_SEND_SPEED_MODE_CONFIG,        /*!< 写速度模式0x00C4及加减速0x0101。 */
	TSDA_APP_WAIT_SPEED_MODE_CONFIG_ACK,    /*!< 等待速度模式配置ACK。 */
	TSDA_APP_SEND_ZERO_SPEED,               /*!< 使能前先写0x10=0RPM。 */
	TSDA_APP_WAIT_ZERO_SPEED_ACK,           /*!< 等待零速命令ACK。 */
	TSDA_APP_SEND_ENABLE,                   /*!< 写0x00=1使能伺服。 */
	TSDA_APP_ENABLE_STABLE_WAIT,            /*!< 等待驱动器使能后的基础稳定时间。 */
	TSDA_APP_BRAKE_RELEASE_SETTLE_WAIT,     /*!< 3Pin释放抱闸后的2000ms机械稳定窗口。 */
	TSDA_APP_SEND_HOME_LIMIT_CHECK,         /*!< 运动前先读取一次0x58。 */
	TSDA_APP_WAIT_HOME_LIMIT_CHECK,         /*!< 等待初始限位状态。 */
	TSDA_APP_SEND_HOME_START_POSITION,      /*!< 读取寻限起点E8/E9。 */
	TSDA_APP_WAIT_HOME_START_POSITION,      /*!< 等待并保存寻限起点。 */
	TSDA_APP_HOME_FIND_UPPER,               /*!< 负转速向上寻限三相运行状态。 */
	TSDA_APP_SEND_HOME_STOP,                /*!< 限位或保护触发后写0RPM。 */
	TSDA_APP_WAIT_HOME_STOP_ACK,            /*!< 等待零速停止命令ACK。 */
	TSDA_APP_WAIT_HOME_STABLE,              /*!< 停止后等待50ms机械稳定。 */
	TSDA_APP_SEND_HOME_ZERO_POSITION,       /*!< 停止后重新读取E8/E9。 */
	TSDA_APP_WAIT_HOME_ZERO_POSITION,       /*!< 保存软件零点或在保护停止后进入ERROR。 */
	TSDA_APP_HOME_MOVE_RETURN,              /*!< 固定200RPM向-100移动，停稳后置ready。 */
	TSDA_APP_DONE,                          /*!< done模式：3ms三相周期执行PVP规划。 */
	TSDA_APP_SEND_ZERO_BEFORE_DISABLE,      /*!< 外部请求失能时先写0RPM。 */
	TSDA_APP_WAIT_ZERO_BEFORE_DISABLE_ACK,  /*!< 确认零速命令后再失能。 */
	TSDA_APP_WAIT_BRAKE_ENGAGE,             /*!< 3Pin闭合抱闸后等待200ms再失能。 */
	TSDA_APP_SEND_DISABLE,                  /*!< 写0x00=0失能。 */
	TSDA_APP_DISABLED,                      /*!< 已失能，等待新的使能请求。 */
	TSDA_APP_ERROR                          /*!< 锁存错误，等待重新初始化。 */
} TSDA_AppState;

/** @brief FreeMASTER 可见的 App 错误原因。 */
typedef enum
{
	TSDA_APP_ERROR_NONE = 0U,      /*!< 无错误。 */
	TSDA_APP_ERROR_SEND,           /*!< 板级 CAN 发送适配器返回失败。 */
	TSDA_APP_ERROR_BOARD_IO,       /*!< 当前能力配置要求的板级 IO 回调未注入。 */
	TSDA_APP_ERROR_ACK_TIMEOUT,    /*!< 初始化命令达到最大重试次数仍无匹配回包。 */
	TSDA_APP_ERROR_HOME_TIMEOUT,   /*!< 寻限达到90s仍未触发上限。 */
	TSDA_APP_ERROR_HOME_TRAVEL,    /*!< 寻限累计达到300mm仍未触发上限。 */
} TSDA_AppError;

typedef struct
{
	uint8_t servo_enable; /*!< 1=请求使能并重新寻限，0=先零速再失能。 */
} TSDA_Command;

/** @brief 3Pin 板级 IO 适配接口；Chassis 配置不会调用这些函数。 */
typedef uint8_t (*TSDA_ReadDigitalInput)(void* user);
typedef void (*TSDA_WriteBrakeRelease)(uint8_t release, void* user);

typedef struct
{
	TSDA_ReadDigitalInput read_upper_limit; /*!< 返回1表示板级上限位有效。 */
	TSDA_ReadDigitalInput read_lower_limit; /*!< 返回1表示板级下限位有效。 */
	TSDA_WriteBrakeRelease write_brake_release; /*!< 1=释放抱闸，0=闭合抱闸。 */
	void* user;                             /*!< 原样传给三个板级回调。 */
} TSDA_BoardIo;

/**
  * @brief FreeMASTER 可直接观察的状态与诊断快照。
  * @note 这些字段由 App 写入，通信层和调试工具只读。
  */
typedef struct
{
	uint32_t rx_count;                   /*!< 已从中断缓存取出并处理的CAN帧数。 */
	uint32_t tx_count;                   /*!< 已被板级发送接口接受的TSDA帧数。 */
	uint32_t rx_drop_count;              /*!< 接收环形队列满导致的丢帧数，正常应为0。 */
	int32_t current_position_raw;        /*!< E8/E9组合得到的驱动器原始位置。 */
	int32_t homing_start_position_raw;   /*!< 本轮向上寻限开始位置。 */
	int32_t position_origin_raw;         /*!< 上限停止后记录的软件零点原始位置。 */
	float current_height_mm;             /*!< 真实浮点高度(mm)，相对上限零点，向下为负。 */
	float target_height_mm;              /*!< done状态当前生效的协议目标高度(mm)。 */
	uint32_t homing_elapsed_ms;          /*!< 本轮向上寻限已运行时间。 */
	uint32_t homing_travel_raw;          /*!< 当前位置与寻限起点的原始计数差绝对值。 */
	int16_t output_speed_rpm;            /*!< E4返回的实际有符号转速。 */
	int16_t commanded_speed_rpm;         /*!< App最后成功交给底层的0x10目标转速。 */
	float planned_velocity_mm_s;         /*!< PVP内部连续规划速度，用户坐标符号。 */
	uint8_t online;                      /*!< 启动阶段收到匹配E3回包后锁存为1。 */
	uint8_t servo_enabled;               /*!< App完成使能稳定期后置1，失能后清0。 */
	uint8_t ready;                       /*!< 正常目标控制开放标志，寻限建立零点后置1。 */
	uint8_t upper_limit_active;          /*!< 统一上限状态：来自板级GPIO或0x58，1=触发。 */
	uint8_t lower_limit_active;          /*!< 统一下限状态：来自板级GPIO或0x58，1=触发。 */
	uint8_t brake_release_command;       /*!< MCU抱闸命令快照：1=释放，0=闭合/驱动器自控。 */
	uint8_t homing_done;                 /*!< 最终零点位置读取并保存成功后置1。 */
	uint8_t run_send_phase;              /*!< 三相节拍索引，按0、1、2循环。 */
	uint8_t motion_hold_active;           /*!< 1=处于2mm进入、3mm退出的零速保持。 */
	TSDA_AppError error_code;            /*!< 锁存的状态机错误原因。 */
} TSDA_Status;

/** @brief 通信层写入的滑台控制字段；ready=1后按3ms周期实时生效。 */
typedef struct
{
	float target_height_mm;      /*!< 绝对目标高度(mm)，App钳位到[-300,0]。 */
	float max_speed_mm_s;        /*!< 最大速度(mm/s)，合法性由通信层保证。 */
	float acceleration_mm_s2;    /*!< 加速度(mm/s^2)，加减速共用。 */
} TSDA_SlideControl;

/** @brief 通信层读取的滑台反馈；ready=0时两个字段都保持0。 */
typedef struct
{
	float current_height_mm;     /*!< 真实高度，不隐藏短暂的软件边界过冲。 */
	float current_speed_mm_s;    /*!< 用户坐标速度：向上为正、向下为负。 */
} TSDA_SlideFeedback;

extern volatile TSDA_Command tsda_command;
extern volatile TSDA_Status tsda_status;
extern volatile TSDA_AppState tsda_app_state;
extern volatile TSDA_SlideControl tsda_slide_control;
extern volatile TSDA_SlideFeedback tsda_slide_feedback;

/**
  * @brief 初始化 App、清空诊断状态并从上电等待状态开始。
  * @param send 注入的 CAN1 发送适配器，App 和 Driver 不直接访问 FDCAN 句柄。
  * @param send_user 原样传给发送适配器的上下文。
  * @param now_ms 当前单调递增毫秒时刻。
  */
void TSDA_AppInit(TSDA_SendFunc send,
                  void* send_user,
                  const TSDA_BoardIo* board_io,
                  uint32_t now_ms);

/** @brief 由1ms MotorTask调用一次，推进非阻塞状态机和三相节拍。 */
void TSDA_AppUpdate(uint32_t now_ms);

/**
  * @brief 写入目标高度(mm)，超出[-300,0]范围自动钳位到边界。
  * @note 仅当 ready=1（寻限完成）后生效；done模式每拍按此目标计算速度方向。
  */
void TSDA_AppSetTargetHeightMm(float target_mm);

/**
  * @brief CAN1接收中断的唯一TSDA入口，只复制完整8字节帧到环形队列。
  * @note 不在中断中解析协议或改变运动状态；解析统一由 TSDA_AppUpdate 完成。
  */
void TSDA_AppOnCanRx(uint32_t can_id, const uint8_t* data, uint8_t len);

/** @brief 返回状态机是否已经进入锁存ERROR。 */
uint8_t TSDA_AppIsError(void);

/** @brief 返回 App 记录的伺服使能状态，供LED任务显示。 */
uint8_t TSDA_AppIsServoEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TSDA_SERVO_H */
