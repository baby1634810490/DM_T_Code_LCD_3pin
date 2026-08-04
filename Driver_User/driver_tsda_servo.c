/**
  ******************************************************************************
  * @file    driver_tsda_servo.c
  * @brief   TSDA-C12D 伺服驱动器 CAN 协议实现。
  *
  * 厂家协议固定使用8字节标准帧：
  *   Byte0  组号；
  *   Byte1  功能码，写/写ACK/读/读回包分别为0x1A/0x1B/0x2A/0x2B；
  *   Byte2  寄存器1地址；Byte3:4 为寄存器1大端16位数据；
  *   Byte5  寄存器2地址；Byte6:7 为寄存器2大端16位数据。
  *
  * 本文件不主动等待ACK。Driver 负责构造和识别单帧，App 负责命令时序、
  * 10ms间隔、300ms超时、重试以及运动状态跳转。
  ******************************************************************************
  */

#include "driver_tsda_servo.h"

#define TSDA_FUNC_WRITE                  (0x1AU)
#define TSDA_FUNC_WRITE_ACK              (0x1BU)
#define TSDA_FUNC_READ                   (0x2AU)
#define TSDA_FUNC_READ_ACK               (0x2BU)
#define TSDA_ENABLE_VALUE                (0x0001)
#define TSDA_DISABLE_VALUE               (0x0000)

static TSDA_Result TSDA_SendFrame(TSDA_Servo* servo,
                                  const uint8_t data[TSDA_CAN_DATA_LEN]);
static void TSDA_PackInt16(uint8_t* high, uint8_t* low, int16_t value);

/** @brief 保存从站协议参数和注入式发送接口，不产生任何总线流量。 */
void TSDA_Init(TSDA_Servo* servo,
               uint8_t id,
               uint8_t group,
               TSDA_SendFunc send,
               void* user)
{
	if (servo == NULL)
		return;

	servo->id = id;
	servo->group = group;
	servo->send = send;
	servo->user = user;
}

TSDA_Result TSDA_WriteReg(TSDA_Servo* servo, uint8_t reg, int16_t value)
{
	return TSDA_WriteReg2(servo, reg, value, TSDA_REG_UNUSED, 0);
}

/**
  * @brief 写一个或两个16位寄存器。
  *
  * CAN 数据固定为：组号、0x1A、地址1、值1高低字节、地址2、值2高低字节。
  * 只写一个寄存器时，地址2使用 0xFF；该格式继续兼容已验证的 TSDA CAN 接口。
  */
TSDA_Result TSDA_WriteReg2(TSDA_Servo* servo,
                           uint8_t reg1,
                           int16_t value1,
                           uint8_t reg2,
                           int16_t value2)
{
	uint8_t data[TSDA_CAN_DATA_LEN];

	if (servo == NULL)
		return TSDA_ERROR_PARAM;

	data[0] = servo->group;
	data[1] = TSDA_FUNC_WRITE;
	data[2] = reg1;
	TSDA_PackInt16(&data[3], &data[4], value1);
	data[5] = reg2;
	TSDA_PackInt16(&data[6], &data[7], value2);

	return TSDA_SendFrame(servo, data);
}

/**
  * @brief 构造读单寄存器命令。
  * @note 第二寄存器地址使用0xFF，回包匹配时也必须期望0xFF。
  */
TSDA_Result TSDA_ReadReg(TSDA_Servo* servo, uint8_t reg)
{
	return TSDA_ReadReg2(servo, reg, TSDA_REG_UNUSED);
}

/**
  * @brief 构造读两个寄存器命令。
  * @note 读命令的数据字节固定填0；返回数据由0x2B回包携带。
  */
TSDA_Result TSDA_ReadReg2(TSDA_Servo* servo, uint8_t reg1, uint8_t reg2)
{
	uint8_t data[TSDA_CAN_DATA_LEN];

	if (servo == NULL)
		return TSDA_ERROR_PARAM;

	data[0] = servo->group;
	data[1] = TSDA_FUNC_READ;
	data[2] = reg1;
	data[3] = 0U;
	data[4] = 0U;
	data[5] = reg2;
	data[6] = 0U;
	data[7] = 0U;

	return TSDA_SendFrame(servo, data);
}

/** @brief 清故障只是协议写命令，故障是否真正消失由后续状态读取确认。 */
TSDA_Result TSDA_ClearFault(TSDA_Servo* servo)
{
	return TSDA_WriteReg(servo, TSDA_REG_CLEAR_FAULT, 0);
}

/**
  * @brief 一帧同时设置速度 PC 模式和速度模式内部加减速时间。
  * @note  acc/dec 的值1表示驱动器允许的最短时间；MCU 后续负责外层速度规划。
  */
TSDA_Result TSDA_SetSpeedModeAndAccDec(TSDA_Servo* servo, uint8_t acc, uint8_t dec)
{
	int16_t acc_dec = (int16_t)(((uint16_t)acc << 8U) | (uint16_t)dec);

	return TSDA_WriteReg2(servo,
	                      TSDA_REG_MODE,
	                      (int16_t)TSDA_MODE_SPEED_PC,
	                      TSDA_REG_SPEED_ACC_DEC,
	                      acc_dec);
}

TSDA_Result TSDA_SetTargetSpeedRpm(TSDA_Servo* servo, int16_t speed_rpm)
{
	return TSDA_WriteReg(servo, TSDA_REG_TARGET_SPEED, speed_rpm);
}

/** @brief 使能命令不包含抱闸控制；Chassis由驱动器内部自动联动抱闸。 */
TSDA_Result TSDA_Enable(TSDA_Servo* servo)
{
	return TSDA_WriteReg(servo, TSDA_REG_ENABLE, TSDA_ENABLE_VALUE);
}

/** @brief 失能命令不等待机械动作完成，相关时序由App或驱动器承担。 */
TSDA_Result TSDA_Disable(TSDA_Servo* servo)
{
	return TSDA_WriteReg(servo, TSDA_REG_ENABLE, TSDA_DISABLE_VALUE);
}

/** @brief TSDA回包使用独立ID，当前从站0x01对应回包ID 0x101。 */
uint32_t TSDA_GetAckCanId(const TSDA_Servo* servo)
{
	return (servo == NULL) ? TSDA_ACK_ID_OFFSET :
	       ((uint32_t)servo->id + TSDA_ACK_ID_OFFSET);
}

/** @brief 用显式结构保存两个地址，避免App用松散字节判断回包。 */
TSDA_AckExpect TSDA_MakeAckExpect(uint8_t reg1, uint8_t reg2)
{
	TSDA_AckExpect expect;
	expect.reg1 = reg1;
	expect.reg2 = reg2;
	return expect;
}

/**
  * @brief 严格识别写ACK。
  * @note ACK不回显写入值，只回显寄存器地址，因此App不能用ACK区分同地址的
  *       不同速度值；运动停止仍以先写0RPM、再读取最终位置的时序保证。
  */
uint8_t TSDA_IsExpectedWriteAck(const TSDA_Servo* servo,
                                uint32_t can_id,
                                const uint8_t* data,
                                uint8_t len,
                                TSDA_AckExpect expect)
{
	if ((servo == NULL) || (data == NULL) || (len < TSDA_CAN_DATA_LEN))
		return 0U;

	return ((can_id == TSDA_GetAckCanId(servo)) &&
	        (data[0] == servo->group) &&
	        (data[1] == TSDA_FUNC_WRITE_ACK) &&
	        (data[2] == expect.reg1) &&
	        (data[5] == expect.reg2)) ? 1U : 0U;
}

/** @brief 严格识别读回包，避免把自动上传帧或其他寄存器回包误作当前响应。 */
uint8_t TSDA_IsExpectedReadResponse(const TSDA_Servo* servo,
                                    uint32_t can_id,
                                    const uint8_t* data,
                                    uint8_t len,
                                    TSDA_AckExpect expect)
{
	if ((servo == NULL) || (data == NULL) || (len < TSDA_CAN_DATA_LEN))
		return 0U;

	return ((can_id == TSDA_GetAckCanId(servo)) &&
	        (data[0] == servo->group) &&
	        (data[1] == TSDA_FUNC_READ_ACK) &&
	        (data[2] == expect.reg1) &&
	        (data[5] == expect.reg2)) ? 1U : 0U;
}

/** @brief 将协议大端字节组合后按int16_t解释，保留负转速补码。 */
int16_t TSDA_GetResponseValue1(const uint8_t* data)
{
	if (data == NULL)
		return 0;
	return (int16_t)(((uint16_t)data[3] << 8U) | (uint16_t)data[4]);
}

/** @brief 取第二个寄存器值，E8/E9位置读取会再按uint16_t重新组合为int32_t。 */
int16_t TSDA_GetResponseValue2(const uint8_t* data)
{
	if (data == NULL)
		return 0;
	return (int16_t)(((uint16_t)data[6] << 8U) | (uint16_t)data[7]);
}

/**
  * @brief Driver与板级CAN之间的唯一出口。
  * @retval TSDA_OK仅表示底层接受发送，不表示驱动器已经执行或返回ACK。
  */
static TSDA_Result TSDA_SendFrame(TSDA_Servo* servo,
                                  const uint8_t data[TSDA_CAN_DATA_LEN])
{
	if ((servo == NULL) || (data == NULL) || (servo->send == NULL))
		return TSDA_ERROR_PARAM;

	return (servo->send((uint32_t)servo->id,
	                    data,
	                    TSDA_CAN_DATA_LEN,
	                    servo->user) == 0U) ? TSDA_OK : TSDA_ERROR_SEND;
}

/** @brief 将有符号16位值按协议要求拆为高字节在前的两个字节。 */
static void TSDA_PackInt16(uint8_t* high, uint8_t* low, int16_t value)
{
	uint16_t raw = (uint16_t)value;
	*high = (uint8_t)(raw >> 8U);
	*low = (uint8_t)(raw & 0x00FFU);
}
