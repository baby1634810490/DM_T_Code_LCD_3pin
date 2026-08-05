/**
  ******************************************************************************
  * @file    app_tsda_servo.c
  * @brief   TSDA Chassis 速度模式重写 - 位置闭环控制实现。
  *
  * 状态机流程：
  *   1. 读取 E3，确认 CAN1 上驱动器在线；
  *   2. 清故障；
  *   3. 写 0x02=0x00C4 和 0x0A=0x0101；
  *   4. 在使能前写 0x10=0RPM；
  *   5. 使能并等待驱动器稳定；
  *   6. 读取 0x58 和寻限起点 E8/E9；
  *   7. 未触发上限时进入 3ms 三相节拍：写负转速、读E8/E9、读0x58；
  *   8. 上限触发后写0RPM，读取停止位置建立0点；
  *   9. 建立 [0,-300]mm 坐标系，ready=1，自动下移100mm到 -100mm。
  *
  * 坐标约定（已由实机确认）：
  *   - 负转速 = 向上（朝上限），正转速 = 向下（远离上限）
  *   - 原始位置随向下移动而增加
  *   - position_mm = 0 在上限零点，负值表示零点下方距离
  ******************************************************************************
  */

#include "app_tsda_servo.h"

#include <string.h>

#define TSDA_SPEED_MODE_ACC_TIME         (1U)     /*!< 驱动器速度模式最短加速时间。 */
#define TSDA_SPEED_MODE_DEC_TIME         (1U)     /*!< 驱动器速度模式最短减速时间。 */
#define TSDA_RX_QUEUE_SIZE               (8U)     /*!< ISR单生产者、任务单消费者队列深度。 */
#define TSDA_HOME_SPEED_RPM              (-250)   /*!< 实机确认负方向向上的首次寻限速度。 */
#define TSDA_HOME_TIMEOUT_MS             (90000U) /*!< 50RPM走满300mm约72s，预留至90s。 */
#define TSDA_HOME_RETURN_TIMEOUT_MS      (20000U) /*!< 回位到-100mm保护：100RPM≈8.3mm/s走100mm≈12s，预留20s。 */
#define TSDA_POSITION_COUNT_PER_MM       (2000U)  /*!< 10000count/rev、5mm/rev。 */
#define TSDA_HOME_MAX_TRAVEL_MM          (300U)   /*!< 软件允许的最大寻限位移。 */
#define TSDA_HOME_MAX_TRAVEL_RAW         (TSDA_POSITION_COUNT_PER_MM * \
                                          TSDA_HOME_MAX_TRAVEL_MM)

/** @brief 限位触发后保持当前位置并等待机械稳定的时间，单位 ms。 */
#define TSDA_APP_HOME_HOLD_SETTLE_MS     (50U)

/* 位置闭环内部参数 */
#define TSDA_APP_COUNTS_PER_MM           (TSDA_POSITION_COUNT_PER_MM)
#define TSDA_APP_POSITION_TOLERANCE_RAW  ((int32_t)TSDA_APP_POSITION_TOLERANCE_MM * \
                                          (int32_t)TSDA_APP_COUNTS_PER_MM)
#define TSDA_APP_SW_LOWER_LIMIT_RAW \
	((int32_t)TSDA_APP_SOFT_LOWER_LIMIT_MM * TSDA_APP_COUNTS_PER_MM)

typedef enum
{
	TSDA_WAIT_NONE = 0U, /*!< 当前状态不等待协议回包。 */
	TSDA_WAIT_READ,      /*!< 期望0x2B读回包。 */
	TSDA_WAIT_WRITE      /*!< 期望0x1B写ACK。 */
} TSDA_WaitType;

/** @brief ISR与任务之间传递的一份完整CAN帧快照。 */
typedef struct
{
	uint32_t can_id;                  /*!< 接收帧标准ID。 */
	uint8_t len;                      /*!< 当前只缓存完整8字节帧。 */
	uint8_t data[TSDA_CAN_DATA_LEN];  /*!< 在发布写索引前复制完成。 */
} TSDA_RxFrame;

/**
  * @brief CAN接收环形队列。
  * @note ISR只写write_index，1ms任务只写read_index，索引发布顺序保证帧完整性。
  */
typedef struct
{
	TSDA_RxFrame frame[TSDA_RX_QUEUE_SIZE]; /*!< 固定容量帧存储。 */
	volatile uint8_t write_index;           /*!< ISR下一写入位置。 */
	volatile uint8_t read_index;            /*!< 任务下一读取位置。 */
} TSDA_RxQueue;

/** @brief App私有上下文，不对通信层或FreeMASTER开放写权限。 */
typedef struct
{
	TSDA_Servo servo;                  /*!< 纯协议Driver对象。 */
	TSDA_RxQueue rx_queue;             /*!< 中断到任务的帧队列。 */
	TSDA_WaitType wait_type;           /*!< 当前等待读回包或写ACK。 */
	TSDA_AckExpect expect;             /*!< 当前回包必须回显的地址。 */
	TSDA_AppState retry_send_state;    /*!< 超时后返回的SEND状态。 */
	uint32_t state_tick_ms;            /*!< 当前状态进入时刻。 */
	uint32_t homing_start_tick_ms;     /*!< 90s寻限保护计时起点。 */
	TSDA_AppError pending_error;       /*!< 先零速停止、后锁存的寻限错误。 */
	uint8_t retry;                     /*!< 当前初始化命令已重发次数。 */
} TSDA_AppContext;

volatile TSDA_Command tsda_command = {1U, 0};
volatile TSDA_Status tsda_status;
volatile TSDA_AppState tsda_app_state = TSDA_APP_POWER_WAIT;

static TSDA_AppContext tsda_app;

/* ---- 前向声明 ---- */
static void TSDA_AppSetState(TSDA_AppState state, uint32_t now_ms);
static void TSDA_AppSetWait(TSDA_AppState wait_state,
                            TSDA_AppState retry_send_state,
                            TSDA_WaitType wait_type,
                            TSDA_AckExpect expect,
                            uint32_t now_ms);
static uint8_t TSDA_AppCommandIntervalElapsed(uint32_t now_ms);
static void TSDA_AppHandleWaitTimeout(uint32_t now_ms);
static void TSDA_AppProcessRxQueue(uint32_t now_ms);
static void TSDA_AppProcessRxFrame(const TSDA_RxFrame* frame, uint32_t now_ms);
static void TSDA_AppRunUpperHoming(uint32_t now_ms);
static void TSDA_AppCheckHomingProtection(uint32_t now_ms);
static void TSDA_AppRunDone(uint32_t now_ms);
static int32_t TSDA_AppRawToPositionMm(int32_t current_raw);
static int32_t TSDA_AppPositionMmToRaw(int32_t position_mm);
static void TSDA_AppClampTargetPosition(void);
static uint8_t TSDA_AppSendSucceeded(TSDA_Result result, uint32_t now_ms);
static void TSDA_AppEnterError(TSDA_AppError error, uint32_t now_ms);

/* ---- 坐标转换 ---- */

/**
  * @brief 将驱动器原始编码器位置转换为用户 mm 坐标。
  * @note  约定：原始位置随向下（正转速）移动而增加。
  *         position_mm = -(current_raw - origin_raw) / COUNTS_PER_MM。
  *         上限零点 → 0mm，向下 100mm → -100mm，向下 300mm → -300mm。
  */
static int32_t TSDA_AppRawToPositionMm(int32_t current_raw)
{
	int32_t delta = current_raw - tsda_status.position_origin_raw;
	return -(delta / (int32_t)TSDA_APP_COUNTS_PER_MM);
}

/** @brief 将用户 mm 坐标转换为原始编码器位置。 */
static int32_t TSDA_AppPositionMmToRaw(int32_t position_mm)
{
	int32_t delta = -(position_mm * (int32_t)TSDA_APP_COUNTS_PER_MM);
	return tsda_status.position_origin_raw + delta;
}

/** @brief 将用户目标位置钳位到 [0, -300]mm 安全范围内。 */
static void TSDA_AppClampTargetPosition(void)
{
	if (tsda_command.target_position_mm > 0)
		tsda_command.target_position_mm = 0;
	if (tsda_command.target_position_mm < -(int32_t)TSDA_APP_SOFT_LOWER_LIMIT_MM)
		tsda_command.target_position_mm = -(int32_t)TSDA_APP_SOFT_LOWER_LIMIT_MM;
}

/* ---- 初始化 ---- */

/**
  * @brief 初始化 App。
  *
  * 所有运行态和接收队列都从零开始；servo_enable 默认置1，使烧录后能自动完成
  * 零速使能和上限寻限，并在寻限完成后自动向下移动 100mm。
  */
void TSDA_AppInit(TSDA_SendFunc send, void* send_user, uint32_t now_ms)
{
	memset(&tsda_app, 0, sizeof(tsda_app));
	memset((void*)&tsda_status, 0, sizeof(tsda_status));

	TSDA_Init(&tsda_app.servo,
	          TSDA_DEFAULT_CAN_ID,
	          TSDA_DEFAULT_GROUP,
	          send,
	          send_user);

	/* 上电后自动执行零速使能和寻限，完成后自动下移至 -100mm。 */
	tsda_command.servo_enable = 1U;
	tsda_command.target_position_mm = TSDA_APP_INITIAL_TARGET_MM;
	tsda_status.ready = 0U;
	tsda_status.target_position_mm = TSDA_APP_INITIAL_TARGET_MM;
	TSDA_AppSetState(TSDA_APP_POWER_WAIT, now_ms);
}

/* ---- 主状态机 ---- */

/**
  * @brief 在1ms任务中推进一次状态机。
  *
  * 函数先处理缓存回包，再执行由回包决定的新状态。因此0x58在本周期被解析为
  * 上限触发后，同一个AppUpdate即可发送0RPM，不必额外等待下一轮3ms节拍。
  */
void TSDA_AppUpdate(uint32_t now_ms)
{
	TSDA_Result result;

	/* 中断只缓存帧；所有协议判断和状态跳转都在1ms任务上下文完成。 */
	TSDA_AppProcessRxQueue(now_ms);

	switch (tsda_app_state)
	{
	case TSDA_APP_POWER_WAIT:
		if ((now_ms - tsda_app.state_tick_ms) >= TSDA_APP_POWER_WAIT_MS)
			TSDA_AppSetState(TSDA_APP_SEND_READ_STATUS, now_ms);
		break;

	case TSDA_APP_SEND_READ_STATUS:
		result = TSDA_ReadReg(&tsda_app.servo, TSDA_REG_ALARM_STATUS);
		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
		{
			TSDA_AppSetWait(TSDA_APP_WAIT_READ_STATUS,
			                TSDA_APP_SEND_READ_STATUS,
			                TSDA_WAIT_READ,
			                TSDA_MakeAckExpect(TSDA_REG_ALARM_STATUS, TSDA_REG_UNUSED),
			                now_ms);
		}
		break;

	case TSDA_APP_WAIT_READ_STATUS:
	case TSDA_APP_WAIT_CLEAR_FAULT_ACK:
	case TSDA_APP_WAIT_SPEED_MODE_CONFIG_ACK:
	case TSDA_APP_WAIT_ZERO_SPEED_ACK:
	case TSDA_APP_WAIT_HOME_LIMIT_CHECK:
	case TSDA_APP_WAIT_HOME_START_POSITION:
	case TSDA_APP_WAIT_HOME_STOP_ACK:
	case TSDA_APP_WAIT_HOME_ZERO_POSITION:
	case TSDA_APP_WAIT_ZERO_BEFORE_DISABLE_ACK:
		TSDA_AppHandleWaitTimeout(now_ms);
		break;

	case TSDA_APP_SEND_CLEAR_FAULT:
		if (TSDA_AppCommandIntervalElapsed(now_ms) == 0U)
			break;
		result = TSDA_ClearFault(&tsda_app.servo);
		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
		{
			TSDA_AppSetWait(TSDA_APP_WAIT_CLEAR_FAULT_ACK,
			                TSDA_APP_SEND_CLEAR_FAULT,
			                TSDA_WAIT_WRITE,
			                TSDA_MakeAckExpect(TSDA_REG_CLEAR_FAULT, TSDA_REG_UNUSED),
			                now_ms);
		}
		break;

	case TSDA_APP_SEND_SPEED_MODE_CONFIG:
		if (TSDA_AppCommandIntervalElapsed(now_ms) == 0U)
			break;
		result = TSDA_SetSpeedModeAndAccDec(&tsda_app.servo,
		                                        TSDA_SPEED_MODE_ACC_TIME,
		                                        TSDA_SPEED_MODE_DEC_TIME);
		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
		{
			TSDA_AppSetWait(TSDA_APP_WAIT_SPEED_MODE_CONFIG_ACK,
			                TSDA_APP_SEND_SPEED_MODE_CONFIG,
			                TSDA_WAIT_WRITE,
			                TSDA_MakeAckExpect(TSDA_REG_MODE, TSDA_REG_SPEED_ACC_DEC),
			                now_ms);
		}
		break;

	case TSDA_APP_SEND_ZERO_SPEED:
		if (TSDA_AppCommandIntervalElapsed(now_ms) == 0U)
			break;
		result = TSDA_SetTargetSpeedRpm(&tsda_app.servo, 0);
		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
		{
			tsda_status.commanded_speed_rpm = 0;
			TSDA_AppSetWait(TSDA_APP_WAIT_ZERO_SPEED_ACK,
			                TSDA_APP_SEND_ZERO_SPEED,
			                TSDA_WAIT_WRITE,
			                TSDA_MakeAckExpect(TSDA_REG_TARGET_SPEED, TSDA_REG_UNUSED),
			                now_ms);
		}
		break;

	case TSDA_APP_SEND_ENABLE:
		if (TSDA_AppCommandIntervalElapsed(now_ms) == 0U)
			break;
		if (tsda_command.servo_enable == 0U)
		{
			TSDA_AppSetState(TSDA_APP_DISABLED, now_ms);
			break;
		}
		result = TSDA_Enable(&tsda_app.servo);
		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
			TSDA_AppSetState(TSDA_APP_ENABLE_STABLE_WAIT, now_ms);
		break;

	case TSDA_APP_ENABLE_STABLE_WAIT:
		/*
		 * 沿用已经上机验证过的使能后静默窗口：驱动器使能时自动释放 Chassis 抱闸，
		 * MCU 不操作抱闸 GPIO，也不要求在此窗口内收到使能 ACK。
		 */
		if (tsda_command.servo_enable == 0U)
		{
			TSDA_AppSetState(TSDA_APP_SEND_DISABLE, now_ms);
			break;
		}
		if ((now_ms - tsda_app.state_tick_ms) >= TSDA_APP_ENABLE_STABLE_MS)
		{
			tsda_status.servo_enabled = 1U;
			tsda_status.homing_done = 0U;
			tsda_status.upper_limit_active = 0U;
			tsda_status.lower_limit_active = 0U;
			tsda_status.position_origin_raw = 0;
			tsda_status.current_position_mm = 0;
			tsda_status.homing_elapsed_ms = 0U;
			tsda_status.homing_travel_raw = 0U;
			tsda_status.run_send_phase = 0U;
			tsda_app.pending_error = TSDA_APP_ERROR_NONE;
			TSDA_AppSetState(TSDA_APP_SEND_HOME_LIMIT_CHECK, now_ms);
		}
		break;

	case TSDA_APP_SEND_HOME_LIMIT_CHECK:
		result = TSDA_ReadReg(&tsda_app.servo, TSDA_REG_PORT_LIMIT_STATUS);
		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
		{
			TSDA_AppSetWait(TSDA_APP_WAIT_HOME_LIMIT_CHECK,
			                TSDA_APP_SEND_HOME_LIMIT_CHECK,
			                TSDA_WAIT_READ,
			                TSDA_MakeAckExpect(TSDA_REG_PORT_LIMIT_STATUS,
			                                   TSDA_REG_UNUSED),
			                now_ms);
		}
		break;

	case TSDA_APP_SEND_HOME_START_POSITION:
		if (TSDA_AppCommandIntervalElapsed(now_ms) == 0U)
			break;
		result = TSDA_ReadReg2(&tsda_app.servo,
		                           TSDA_REG_FEEDBACK_POS_HIGH,
		                           TSDA_REG_FEEDBACK_POS_LOW);
		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
		{
			TSDA_AppSetWait(TSDA_APP_WAIT_HOME_START_POSITION,
			                TSDA_APP_SEND_HOME_START_POSITION,
			                TSDA_WAIT_READ,
			                TSDA_MakeAckExpect(TSDA_REG_FEEDBACK_POS_HIGH,
			                                   TSDA_REG_FEEDBACK_POS_LOW),
			                now_ms);
		}
		break;

	case TSDA_APP_HOME_FIND_UPPER:
		if (tsda_command.servo_enable == 0U)
		{
			TSDA_AppSetState(TSDA_APP_SEND_ZERO_BEFORE_DISABLE, now_ms);
			break;
		}
		TSDA_AppCheckHomingProtection(now_ms);
		if (tsda_app_state == TSDA_APP_HOME_FIND_UPPER)
			TSDA_AppRunUpperHoming(now_ms);
		break;

	case TSDA_APP_SEND_HOME_STOP:
		result = TSDA_SetTargetSpeedRpm(&tsda_app.servo, 0);

		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
		{
			tsda_status.commanded_speed_rpm = 0;
			TSDA_AppSetWait(TSDA_APP_WAIT_HOME_STOP_ACK,
			                TSDA_APP_SEND_HOME_STOP,
			                TSDA_WAIT_WRITE,
			                TSDA_MakeAckExpect(TSDA_REG_TARGET_SPEED,
			                                   TSDA_REG_UNUSED),
			                now_ms);
		}
		break;

	case TSDA_APP_SEND_HOME_ZERO_POSITION:
		if (TSDA_AppCommandIntervalElapsed(now_ms) == 0U)
			break;
		result = TSDA_ReadReg2(&tsda_app.servo,
		                           TSDA_REG_FEEDBACK_POS_HIGH,
		                           TSDA_REG_FEEDBACK_POS_LOW);
		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
		{
			TSDA_AppSetWait(TSDA_APP_WAIT_HOME_ZERO_POSITION,
			                TSDA_APP_SEND_HOME_ZERO_POSITION,
			                TSDA_WAIT_READ,
			                TSDA_MakeAckExpect(TSDA_REG_FEEDBACK_POS_HIGH,
			                                   TSDA_REG_FEEDBACK_POS_LOW),
			                now_ms);
		}
		break;

	case TSDA_APP_HOME_MOVE_RETURN:
		if (tsda_command.servo_enable == 0U)
		{
			TSDA_AppSetState(TSDA_APP_SEND_ZERO_BEFORE_DISABLE, now_ms);
			break;
		}
		/* 回位保护：20s 内未到 -100mm 视为故障。 */
		if ((now_ms - tsda_app.state_tick_ms) >= TSDA_HOME_RETURN_TIMEOUT_MS)
		{
			tsda_app.pending_error = TSDA_APP_ERROR_HOME_TIMEOUT;
			TSDA_AppSetState(TSDA_APP_ERROR, now_ms);
			break;
		}
		/* bang-bang 恒速向 -100mm 移动（与 DONE 相同三拍逻辑）。 */
		TSDA_AppRunDone(now_ms);
		/* 到位锁轴判定：位置在目标 ±容差 内 → 置 ready 进入 DONE。 */
		if ((tsda_status.current_position_mm - tsda_command.target_position_mm) >=
		        -(int32_t)TSDA_APP_POSITION_TOLERANCE_MM &&
		    (tsda_status.current_position_mm - tsda_command.target_position_mm) <=
		         (int32_t)TSDA_APP_POSITION_TOLERANCE_MM)
		{
			tsda_status.homing_done = 1U;
			tsda_status.ready = 1U;
			tsda_status.run_send_phase = 0U;
			TSDA_AppSetState(TSDA_APP_DONE, now_ms);
		}
		break;

	case TSDA_APP_DONE:
		if (tsda_command.servo_enable == 0U)
		{
			TSDA_AppSetState(TSDA_APP_SEND_ZERO_BEFORE_DISABLE, now_ms);
			break;
		}
		TSDA_AppClampTargetPosition();
		TSDA_AppRunDone(now_ms);
		break;

	case TSDA_APP_SEND_ZERO_BEFORE_DISABLE:
		result = TSDA_SetTargetSpeedRpm(&tsda_app.servo, 0);
		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
		{
			tsda_status.commanded_speed_rpm = 0;
			TSDA_AppSetWait(TSDA_APP_WAIT_ZERO_BEFORE_DISABLE_ACK,
			                TSDA_APP_SEND_ZERO_BEFORE_DISABLE,
			                TSDA_WAIT_WRITE,
			                TSDA_MakeAckExpect(TSDA_REG_TARGET_SPEED, TSDA_REG_UNUSED),
			                now_ms);
		}
		break;

	case TSDA_APP_SEND_DISABLE:
		if (TSDA_AppCommandIntervalElapsed(now_ms) == 0U)
			break;
		result = TSDA_Disable(&tsda_app.servo);
		if (TSDA_AppSendSucceeded(result, now_ms) != 0U)
		{
			tsda_status.servo_enabled = 0U;
			tsda_status.output_speed_rpm = 0;
			TSDA_AppSetState(TSDA_APP_DISABLED, now_ms);
		}
		break;

	case TSDA_APP_DISABLED:
		if (tsda_command.servo_enable != 0U)
			TSDA_AppSetState(TSDA_APP_SEND_ZERO_SPEED, now_ms);
		break;

	case TSDA_APP_ERROR:
	default:
		break;
	}
}

/* ---- CAN 接收 ---- */

/**
  * @brief CAN1接收中断的轻量入口。
  *
  * 中断中不做寄存器匹配、不写状态机、不执行浮点运算；仅在队列有空间时复制
  * 完整8字节帧，并在复制完成后发布write_index。队列满时只增加丢帧计数。
  */
void TSDA_AppOnCanRx(uint32_t can_id, const uint8_t* data, uint8_t len)
{
	uint8_t write_index;
	uint8_t next_index;
	uint8_t i;

	if ((data == NULL) || (len < TSDA_CAN_DATA_LEN))
		return;

	write_index = tsda_app.rx_queue.write_index;
	next_index = (uint8_t)((write_index + 1U) % TSDA_RX_QUEUE_SIZE);
	if (next_index == tsda_app.rx_queue.read_index)
	{
		tsda_status.rx_drop_count++;
		return;
	}

	tsda_app.rx_queue.frame[write_index].can_id = can_id;
	tsda_app.rx_queue.frame[write_index].len = TSDA_CAN_DATA_LEN;
	for (i = 0U; i < TSDA_CAN_DATA_LEN; i++)
		tsda_app.rx_queue.frame[write_index].data[i] = data[i];

	/* 最后发布写索引，保证任务看到索引时整帧数据已经复制完成。 */
	tsda_app.rx_queue.write_index = next_index;
}

/* ---- 外部查询接口 ---- */

/** @brief 给LED等外围任务提供稳定的布尔错误查询，不暴露私有上下文。 */
uint8_t TSDA_AppIsError(void)
{
	return (tsda_app_state == TSDA_APP_ERROR) ? 1U : 0U;
}

/** @brief 查询App认定的使能状态；该值在1500ms稳定期结束后才置1。 */
uint8_t TSDA_AppIsServoEnabled(void)
{
	return tsda_status.servo_enabled;
}

/**
  * @brief 写入目标高度(mm)并自动钳位到 [0,-300]mm 范围内。
  * @note 目标被钳位后，done模式不会试图越过软件上下限。
  */
void TSDA_AppSetTargetHeightMm(int32_t target_mm)
{
	if (target_mm > 0)
		target_mm = 0;
	if (target_mm < -(int32_t)TSDA_APP_SOFT_LOWER_LIMIT_MM)
		target_mm = -(int32_t)TSDA_APP_SOFT_LOWER_LIMIT_MM;

	tsda_command.target_position_mm = target_mm;
}

/* ---- 状态切换工具 ---- */

/**
  * @brief 统一完成状态切换和进入时刻记录。
  * @note 只有WAIT状态保留wait_type，其余状态自动清除等待类型，防止迟到ACK被误用。
  */
static void TSDA_AppSetState(TSDA_AppState state, uint32_t now_ms)
{
	tsda_app_state = state;
	tsda_app.state_tick_ms = now_ms;
	if ((state != TSDA_APP_WAIT_READ_STATUS) &&
	    (state != TSDA_APP_WAIT_CLEAR_FAULT_ACK) &&
	    (state != TSDA_APP_WAIT_SPEED_MODE_CONFIG_ACK) &&
	    (state != TSDA_APP_WAIT_ZERO_SPEED_ACK) &&
	    (state != TSDA_APP_WAIT_HOME_LIMIT_CHECK) &&
	    (state != TSDA_APP_WAIT_HOME_START_POSITION) &&
	    (state != TSDA_APP_WAIT_HOME_STOP_ACK) &&
	    (state != TSDA_APP_WAIT_HOME_ZERO_POSITION) &&
	    (state != TSDA_APP_WAIT_ZERO_BEFORE_DISABLE_ACK))
	{
		tsda_app.wait_type = TSDA_WAIT_NONE;
	}
}

/** @brief 保存超时重发目标和回包地址期望，然后进入指定WAIT状态。 */
static void TSDA_AppSetWait(TSDA_AppState wait_state,
                            TSDA_AppState retry_send_state,
                            TSDA_WaitType wait_type,
                            TSDA_AckExpect expect,
                            uint32_t now_ms)
{
	tsda_app.retry_send_state = retry_send_state;
	tsda_app.wait_type = wait_type;
	tsda_app.expect = expect;
	TSDA_AppSetState(wait_state, now_ms);
}

/** @brief 判断上一条初始化ACK后是否已经满足统一的10ms命令间隔。 */
static uint8_t TSDA_AppCommandIntervalElapsed(uint32_t now_ms)
{
	return ((now_ms - tsda_app.state_tick_ms) >= TSDA_APP_COMMAND_INTERVAL_MS) ? 1U : 0U;
}

/**
  * @brief 处理初始化命令的300ms超时和最多3次重发。
  * @note 寻限运行态和位置控制态不等待周期ACK，因此不会被此超时阻塞三相节拍。
  */
static void TSDA_AppHandleWaitTimeout(uint32_t now_ms)
{
	if ((now_ms - tsda_app.state_tick_ms) < TSDA_APP_ACK_TIMEOUT_MS)
		return;

	if (tsda_app.retry >= TSDA_APP_RETRY_MAX)
	{
		TSDA_AppEnterError(TSDA_APP_ERROR_ACK_TIMEOUT, now_ms);
		return;
	}

	tsda_app.retry++;
	TSDA_AppSetState(tsda_app.retry_send_state, now_ms);
}

/* ---- 回包解析 ---- */

/** @brief 在任务上下文取出当前全部缓存帧，并按接收顺序解析。 */
static void TSDA_AppProcessRxQueue(uint32_t now_ms)
{
	while (tsda_app.rx_queue.read_index != tsda_app.rx_queue.write_index)
	{
		uint8_t read_index = tsda_app.rx_queue.read_index;
		TSDA_RxFrame frame = tsda_app.rx_queue.frame[read_index];
		tsda_app.rx_queue.read_index =
			(uint8_t)((read_index + 1U) % TSDA_RX_QUEUE_SIZE);
		TSDA_AppProcessRxFrame(&frame, now_ms);
	}
}

/**
  * @brief 解析一帧TSDA回包并执行由该回包授权的状态跳转。
  *
  * 解析优先级为0x58、E8/E9、E4、初始化E3、通用写ACK。运行态回包先更新
  * FreeMASTER观测量，再根据当前WAIT/HOME/DONE状态决定是否切换。
  */
static void TSDA_AppProcessRxFrame(const TSDA_RxFrame* frame, uint32_t now_ms)
{
	const uint8_t* data;

	if (frame == NULL)
		return;

	data = frame->data;
	tsda_status.rx_count++;

	/*
	 * 0x58 返回驱动器端口限位状态。沿用 Chassis 已验证的极性：
	 * data[3] 为上限输入、data[4] 为下限输入，0表示触发。
	 */
	if (TSDA_IsExpectedReadResponse(&tsda_app.servo,
	                                frame->can_id,
	                                data,
	                                frame->len,
	                                TSDA_MakeAckExpect(TSDA_REG_PORT_LIMIT_STATUS,
	                                                   TSDA_REG_UNUSED)) != 0U)
	{
		tsda_status.upper_limit_active = (data[3] == 0U) ? 1U : 0U;
		tsda_status.lower_limit_active = (data[4] == 0U) ? 1U : 0U;

		if (tsda_app_state == TSDA_APP_WAIT_HOME_LIMIT_CHECK)
		{
			tsda_app.retry = 0U;
			TSDA_AppSetState(TSDA_APP_SEND_HOME_START_POSITION, now_ms);
		}
		else if ((tsda_app_state == TSDA_APP_HOME_FIND_UPPER) &&
		         (tsda_status.upper_limit_active != 0U))
		{
			tsda_app.pending_error = TSDA_APP_ERROR_NONE;
			TSDA_AppSetState(TSDA_APP_SEND_HOME_STOP, now_ms);
		}
		return;
	}

	/* E8/E9 始终更新原始位置和用户坐标，并在等待状态中承担状态跳转职责。 */
	if (TSDA_IsExpectedReadResponse(&tsda_app.servo,
	                                frame->can_id,
	                                data,
	                                frame->len,
	                                TSDA_MakeAckExpect(TSDA_REG_FEEDBACK_POS_HIGH,
	                                                   TSDA_REG_FEEDBACK_POS_LOW)) != 0U)
	{
		uint16_t high = (uint16_t)TSDA_GetResponseValue1(data);
		uint16_t low = (uint16_t)TSDA_GetResponseValue2(data);
		tsda_status.current_position_raw =
			(int32_t)(((uint32_t)high << 16U) | (uint32_t)low);
		tsda_status.current_position_mm =
			(int32_t)TSDA_AppRawToPositionMm(tsda_status.current_position_raw);

		if (tsda_app_state == TSDA_APP_WAIT_HOME_START_POSITION)
		{
			tsda_app.retry = 0U;
			tsda_status.homing_start_position_raw = tsda_status.current_position_raw;
			tsda_status.homing_elapsed_ms = 0U;
			tsda_status.homing_travel_raw = 0U;
			tsda_status.run_send_phase = 0U;
			tsda_app.homing_start_tick_ms = now_ms;
			tsda_app.pending_error = TSDA_APP_ERROR_NONE;

			if (tsda_status.upper_limit_active != 0U)
				TSDA_AppSetState(TSDA_APP_SEND_HOME_STOP, now_ms);
			else
				TSDA_AppSetState(TSDA_APP_HOME_FIND_UPPER, now_ms);
		}
		else if (tsda_app_state == TSDA_APP_WAIT_HOME_ZERO_POSITION)
		{
			tsda_app.retry = 0U;
			if (tsda_app.pending_error != TSDA_APP_ERROR_NONE)
			{
				TSDA_AppEnterError(tsda_app.pending_error, now_ms);
			}
			else
			{
				/*
				 * 上限停止位置记录为软件零点，建立 [0,-300]mm 坐标系。
				 * 下发初始目标 -100mm，进入 bang-bang 回位阶段；
				 * ready 在回位到位锁轴后才置1。
				 */
				tsda_status.position_origin_raw = tsda_status.current_position_raw;
				tsda_status.current_position_mm = 0;
				tsda_status.run_send_phase = 0U;
				tsda_command.target_position_mm = TSDA_APP_INITIAL_TARGET_MM;
				tsda_status.target_position_mm = TSDA_APP_INITIAL_TARGET_MM;
				TSDA_AppSetState(TSDA_APP_HOME_MOVE_RETURN, now_ms);
			}
		}
		return;
	}

	if (TSDA_IsExpectedReadResponse(&tsda_app.servo,
	                                frame->can_id,
	                                data,
	                                frame->len,
	                                TSDA_MakeAckExpect(TSDA_REG_OUTPUT_SPEED,
	                                                   TSDA_REG_UNUSED)) != 0U)
	{
		tsda_status.output_speed_rpm = TSDA_GetResponseValue1(data);
		return;
	}

	if ((tsda_app_state == TSDA_APP_WAIT_READ_STATUS) &&
	    (tsda_app.wait_type == TSDA_WAIT_READ) &&
	    (TSDA_IsExpectedReadResponse(&tsda_app.servo,
	                                 frame->can_id,
	                                 data,
	                                 frame->len,
	                                 tsda_app.expect) != 0U))
	{
		tsda_status.online = 1U;
		tsda_app.retry = 0U;
		TSDA_AppSetState(TSDA_APP_SEND_CLEAR_FAULT, now_ms);
		return;
	}

	if ((tsda_app.wait_type == TSDA_WAIT_WRITE) &&
	    (TSDA_IsExpectedWriteAck(&tsda_app.servo,
	                             frame->can_id,
	                             data,
	                             frame->len,
	                             tsda_app.expect) != 0U))
	{
		tsda_app.retry = 0U;
		switch (tsda_app_state)
		{
		case TSDA_APP_WAIT_CLEAR_FAULT_ACK:
			TSDA_AppSetState(TSDA_APP_SEND_SPEED_MODE_CONFIG, now_ms);
			break;
		case TSDA_APP_WAIT_SPEED_MODE_CONFIG_ACK:
			TSDA_AppSetState(TSDA_APP_SEND_ZERO_SPEED, now_ms);
			break;
		case TSDA_APP_WAIT_ZERO_SPEED_ACK:
			TSDA_AppSetState(TSDA_APP_SEND_ENABLE, now_ms);
			break;
		case TSDA_APP_WAIT_HOME_STOP_ACK:
			TSDA_AppSetState(TSDA_APP_SEND_HOME_ZERO_POSITION, now_ms);
			break;
		case TSDA_APP_WAIT_ZERO_BEFORE_DISABLE_ACK:
			TSDA_AppSetState(TSDA_APP_SEND_DISABLE, now_ms);
			break;
		default:
			break;
		}
	}
}

/* ---- 寻限 ---- */

/**
  * @brief 执行 Chassis 上限寻限的3相通信节拍。
  *
  * 负转速已经由实机方向确认表示向上。每3ms依次写负转速、读取 E8/E9、
  * 读取0x58。限位回包在下一次 AppUpdate 开始时处理，一旦触发就立即转入
  * TSDA_APP_SEND_HOME_STOP，不再发送下一拍速度。
  */
static void TSDA_AppRunUpperHoming(uint32_t now_ms)
{
	TSDA_Result result;

	switch (tsda_status.run_send_phase)
	{
	case 0U:
		result = TSDA_SetTargetSpeedRpm(&tsda_app.servo, TSDA_HOME_SPEED_RPM);
		tsda_status.commanded_speed_rpm = TSDA_HOME_SPEED_RPM;
		break;
	case 1U:
		result = TSDA_ReadReg2(&tsda_app.servo,
		                           TSDA_REG_FEEDBACK_POS_HIGH,
		                           TSDA_REG_FEEDBACK_POS_LOW);
		break;
	default:
		result = TSDA_ReadReg(&tsda_app.servo, TSDA_REG_PORT_LIMIT_STATUS);
		break;
	}

	if (TSDA_AppSendSucceeded(result, now_ms) == 0U)
		return;

	tsda_status.run_send_phase = (uint8_t)((tsda_status.run_send_phase + 1U) % 3U);
}

/**
  * @brief 检查寻限最大行程和最长时间保护。
  *
  * 软件零点尚未建立，因此不能使用 [-300,0]mm 用户坐标。这里直接比较寻限起点
  * 与最新 E8/E9 的原始计数差，方向无关；600000 count 对应 300mm。
  * 保护触发时先转入统一零速停止流程，最终位置读回后才进入 ERROR。
  */
static void TSDA_AppCheckHomingProtection(uint32_t now_ms)
{
	int64_t travel = (int64_t)tsda_status.current_position_raw -
	                 (int64_t)tsda_status.homing_start_position_raw;

	if (travel < 0)
		travel = -travel;

	tsda_status.homing_travel_raw = (uint32_t)travel;
	tsda_status.homing_elapsed_ms = now_ms - tsda_app.homing_start_tick_ms;

	if (tsda_status.homing_travel_raw >= TSDA_HOME_MAX_TRAVEL_RAW)
	{
		tsda_app.pending_error = TSDA_APP_ERROR_HOME_TRAVEL;
		TSDA_AppSetState(TSDA_APP_SEND_HOME_STOP, now_ms);
		return;
	}

	if (tsda_status.homing_elapsed_ms >= TSDA_HOME_TIMEOUT_MS)
	{
		tsda_app.pending_error = TSDA_APP_ERROR_HOME_TIMEOUT;
		TSDA_AppSetState(TSDA_APP_SEND_HOME_STOP, now_ms);
	}
}

/* ---- 位置闭环控制 ---- */

/**
  * @brief done模式三相节拍（3ms周期，每相1ms）。
  *
  * MotorTask 每1ms调用一次 AppUpdate，三个相位依次占用三个调度周期：
  *   phase0：写入目标速度（恒定速度，由位置误差决定方向，到位时为0RPM锁轴）；
  *   phase1：读 E8/E9，更新当前位置坐标；
  *   phase2：读 E4，更新实际转速。
  */
static void TSDA_AppRunDone(uint32_t now_ms)
{
	TSDA_Result result;
	int32_t target_raw;
	int32_t error;
	int32_t abs_error;
	int16_t move_speed;

	/* 根据最新位置误差计算本拍目标速度，误差超出容差才运动 */
	target_raw = TSDA_AppPositionMmToRaw(tsda_command.target_position_mm);
	error = target_raw - tsda_status.current_position_raw;
	abs_error = (error < 0) ? -error : error;

	if (abs_error > TSDA_APP_POSITION_TOLERANCE_RAW)
		move_speed = (error > 0)
			? (int16_t)TSDA_APP_POSITION_MOVE_SPEED_RPM
			: (int16_t)(-(int16_t)TSDA_APP_POSITION_MOVE_SPEED_RPM);
	else
		move_speed = 0;

	/* 同步用户可见的目标位置 */
	tsda_status.target_position_mm = tsda_command.target_position_mm;

	/* 三拍：写目标速度 → 读位置 → 读实际转速 */
	switch (tsda_status.run_send_phase)
	{
	case 0U:
		result = TSDA_SetTargetSpeedRpm(&tsda_app.servo, move_speed);
		tsda_status.commanded_speed_rpm = move_speed;
		break;
	case 1U:
		result = TSDA_ReadReg2(&tsda_app.servo,
		                           TSDA_REG_FEEDBACK_POS_HIGH,
		                           TSDA_REG_FEEDBACK_POS_LOW);
		break;
	default:
		result = TSDA_ReadReg(&tsda_app.servo, TSDA_REG_OUTPUT_SPEED);
		break;
	}

	if (TSDA_AppSendSucceeded(result, now_ms) == 0U)
		return;

	tsda_status.run_send_phase = (uint8_t)((tsda_status.run_send_phase + 1U) % 3U);
}

/* ---- 通用工具 ---- */

/**
  * @brief 统一处理Driver发送结果并维护tx_count。
  * @retval 1表示底层接受发送，可以继续当前时序；0表示已转入ERROR。
  * @note tx_count不是驱动器执行计数，执行成功仍需由匹配回包或后续反馈确认。
  */
static uint8_t TSDA_AppSendSucceeded(TSDA_Result result, uint32_t now_ms)
{
	if (result != TSDA_OK)
	{
		TSDA_AppEnterError(TSDA_APP_ERROR_SEND, now_ms);
		return 0U;
	}

	tsda_status.tx_count++;
	return 1U;
}

/**
  * @brief 锁存错误并关闭后续目标控制。
  * @note 如果最后一条速度命令非零，会先尽力补发0RPM；该安全尝试不覆盖原错误码。
  */
static void TSDA_AppEnterError(TSDA_AppError error, uint32_t now_ms)
{
	/*
	 * 若速度运行期间发生本地发送错误，驱动器可能仍保持上一条非零速度。
	 * ERROR 入口因此尽力补发一次0RPM；即使底层发送已经失效，这次尝试也不会
	 * 掩盖原错误。正常限位和行程/时间保护在到达这里前已经完成零速停止。
	 */
	if ((tsda_status.servo_enabled != 0U) &&
	    (tsda_status.commanded_speed_rpm != 0))
	{
		if (TSDA_SetTargetSpeedRpm(&tsda_app.servo, 0) == TSDA_OK)
			tsda_status.tx_count++;
		tsda_status.commanded_speed_rpm = 0;
	}

	tsda_status.error_code = error;
	tsda_status.ready = 0U;
	TSDA_AppSetState(TSDA_APP_ERROR, now_ms);
}
