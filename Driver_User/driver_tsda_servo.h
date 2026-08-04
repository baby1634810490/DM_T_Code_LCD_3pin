/**
  ******************************************************************************
  * @file    driver_tsda_servo.h
  * @brief   TSDA-C12D 伺服驱动器 CAN 协议层。
  *
  * Driver 只负责厂家协议的组帧、发送和回包识别，不包含状态机、硬件变体、
  * CAN 外设句柄或业务控制策略。App 通过 TSDA_SendFunc 注入实际 CAN 通道。
  ******************************************************************************
  */

#ifndef DRIVER_TSDA_SERVO_H
#define DRIVER_TSDA_SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define TSDA_DEFAULT_CAN_ID              (0x01U)  /*!< 当前驱动器从站号，也是命令帧 CAN ID。 */
#define TSDA_DEFAULT_GROUP               (0x00U)  /*!< 当前项目使用的驱动器组号。 */
#define TSDA_CAN_DATA_LEN                (8U)     /*!< 厂家自定义 CAN 协议固定数据长度。 */
#define TSDA_ACK_ID_OFFSET               (0x100U) /*!< 回包 ID = 从站号 + 0x100。 */

/**
  * @brief Driver 注入式 CAN 发送函数。
  * @param can_id 标准帧 ID，Driver 传入 TSDA 从站号。
  * @param data 固定8字节厂家协议数据。
  * @param len 固定为 TSDA_CAN_DATA_LEN。
  * @param user 初始化时保存的调用方上下文，Driver 不解释其内容。
  * @retval 0表示底层已接受发送，非0表示发送失败。
  */
typedef uint8_t (*TSDA_SendFunc)(uint32_t can_id,
                                 const uint8_t* data,
                                 uint8_t len,
                                 void* user);

typedef enum
{
	TSDA_OK = 0U,       /*!< 协议帧成功交给底层发送接口。 */
	TSDA_ERROR_PARAM,   /*!< 对象、数据或发送回调为空。 */
	TSDA_ERROR_SEND     /*!< 底层 CAN 发送接口返回失败。 */
} TSDA_Result;

/**
  * @brief 一个 TSDA 从站的纯协议对象。
  * @note 对象不拥有 CAN 外设，也不保存 App 状态；因此同一 Driver 可复用于不同板卡。
  */
typedef struct
{
	uint8_t id;          /*!< 从站号及命令帧 CAN ID。 */
	uint8_t group;       /*!< 协议 Byte0 组号。 */
	TSDA_SendFunc send;  /*!< 调用方注入的 CAN 发送适配器。 */
	void* user;          /*!< 原样传回 send 的板级上下文。 */
} TSDA_Servo;

/** @brief App 等待回包时保存的两个寄存器地址回显。 */
typedef struct
{
	uint8_t reg1; /*!< Byte2 应回显的第一个寄存器地址。 */
	uint8_t reg2; /*!< Byte5 应回显的第二个寄存器地址。 */
} TSDA_AckExpect;

/**
  * @brief 本项目使用的 TSDA 寄存器地址。
  * @note  地址和数据能力属于协议事实，因此由 Driver 完整表达，不读取配置文件。
  */
typedef enum
{
	TSDA_REG_ENABLE = 0x00U,              /*!< 0=停止，1=启动。 */
	TSDA_REG_MODE = 0x02U,                /*!< 控制模式。 */
	TSDA_REG_SPEED_ACC_DEC = 0x0AU,       /*!< 速度模式加减速时间，高8位加速、低8位减速。 */
	TSDA_REG_TARGET_SPEED = 0x10U,        /*!< 速度模式目标转速，有符号 RPM。 */
	TSDA_REG_CLEAR_FAULT = 0x4AU,         /*!< 写0清除当前故障。 */
	TSDA_REG_PORT_LIMIT_STATUS = 0x58U,   /*!< 驱动器端口限位状态，后续 Chassis 寻限使用。 */
	TSDA_REG_ALARM_STATUS = 0xE3U,        /*!< 驱动器状态/报警字。 */
	TSDA_REG_OUTPUT_SPEED = 0xE4U,        /*!< 实际输出转速，有符号 RPM。 */
	TSDA_REG_FEEDBACK_POS_HIGH = 0xE8U,   /*!< 实际位置高16位。 */
	TSDA_REG_FEEDBACK_POS_LOW = 0xE9U,    /*!< 实际位置低16位。 */
	TSDA_REG_UNUSED = 0xFFU               /*!< 单寄存器命令的空地址。 */
} TSDA_Register;

typedef enum
{
	TSDA_MODE_SPEED_PC = 0x00C4           /*!< 速度模式，PC/CAN 数字输入。 */
} TSDA_ModeValue;

/**
  * @brief 初始化纯协议对象。
  * @note 只保存参数，不发送 CAN 帧；允许在任务启动后重复初始化。
  */
void TSDA_Init(TSDA_Servo* servo,
               uint8_t id,
               uint8_t group,
               TSDA_SendFunc send,
               void* user);

/** @brief 使用0x1A功能码写一个16位寄存器，第二地址自动填0xFF。 */
TSDA_Result TSDA_WriteReg(TSDA_Servo* servo, uint8_t reg, int16_t value);

/** @brief 使用0x1A功能码在一帧中写两个16位寄存器。 */
TSDA_Result TSDA_WriteReg2(TSDA_Servo* servo,
                           uint8_t reg1,
                           int16_t value1,
                           uint8_t reg2,
                           int16_t value2);
/** @brief 使用0x2A功能码读一个16位寄存器。 */
TSDA_Result TSDA_ReadReg(TSDA_Servo* servo, uint8_t reg);

/** @brief 使用0x2A功能码在一帧中请求两个16位寄存器。 */
TSDA_Result TSDA_ReadReg2(TSDA_Servo* servo, uint8_t reg1, uint8_t reg2);

/** @brief 写0到0x4A清除驱动器当前故障。 */
TSDA_Result TSDA_ClearFault(TSDA_Servo* servo);

/** @brief 一帧写入速度PC模式0x00C4及0x0A加减速时间。 */
TSDA_Result TSDA_SetSpeedModeAndAccDec(TSDA_Servo* servo, uint8_t acc, uint8_t dec);

/** @brief 向0x10写入有符号目标转速，单位RPM。 */
TSDA_Result TSDA_SetTargetSpeedRpm(TSDA_Servo* servo, int16_t speed_rpm);

/** @brief 写0x00=1使能；Chassis 抱闸由驱动器内部联动释放。 */
TSDA_Result TSDA_Enable(TSDA_Servo* servo);

/** @brief 写0x00=0失能；Chassis 抱闸由驱动器内部联动闭合。 */
TSDA_Result TSDA_Disable(TSDA_Servo* servo);

/** @brief 计算当前从站的回包 CAN ID。 */
uint32_t TSDA_GetAckCanId(const TSDA_Servo* servo);

/** @brief 构造供 App 保存的寄存器回显期望值。 */
TSDA_AckExpect TSDA_MakeAckExpect(uint8_t reg1, uint8_t reg2);

/** @brief 严格检查 CAN ID、组号、0x1B 功能码和两个写地址回显。 */
uint8_t TSDA_IsExpectedWriteAck(const TSDA_Servo* servo,
                                uint32_t can_id,
                                const uint8_t* data,
                                uint8_t len,
                                TSDA_AckExpect expect);
/** @brief 严格检查 CAN ID、组号、0x2B 功能码和两个读地址回显。 */
uint8_t TSDA_IsExpectedReadResponse(const TSDA_Servo* servo,
                                    uint32_t can_id,
                                    const uint8_t* data,
                                    uint8_t len,
                                    TSDA_AckExpect expect);
/** @brief 将回包 Byte3:Byte4 按有符号16位大端数据取出。 */
int16_t TSDA_GetResponseValue1(const uint8_t* data);

/** @brief 将回包 Byte6:Byte7 按有符号16位大端数据取出。 */
int16_t TSDA_GetResponseValue2(const uint8_t* data);

#ifdef __cplusplus
}
#endif

#endif /* DRIVER_TSDA_SERVO_H */
