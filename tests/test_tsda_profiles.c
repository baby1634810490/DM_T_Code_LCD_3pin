#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_tsda_servo.h"
#include "tsda_config.h"

typedef struct
{
	uint8_t frame[TSDA_CAN_DATA_LEN];
	uint32_t send_count;
	uint8_t send_fail;
	uint8_t upper_limit;
	uint8_t lower_limit;
	uint8_t brake_release;
	uint8_t saw_port58_read;
	uint32_t brake_write_count;
} TestContext;

static uint8_t TestSend(uint32_t can_id,
                        const uint8_t* data,
                        uint8_t len,
                        void* user)
{
	TestContext* context = (TestContext*)user;
	assert(can_id == TSDA_DEFAULT_CAN_ID);
	assert(len == TSDA_CAN_DATA_LEN);
	memcpy(context->frame, data, TSDA_CAN_DATA_LEN);
	if ((data[1] == 0x2AU) && (data[2] == TSDA_REG_PORT_LIMIT_STATUS))
		context->saw_port58_read = 1U;
	context->send_count++;
	return context->send_fail;
}

static uint8_t TestReadUpper(void* user)
{
	return ((TestContext*)user)->upper_limit;
}

static uint8_t TestReadLower(void* user)
{
	return ((TestContext*)user)->lower_limit;
}

static void TestWriteBrake(uint8_t release, void* user)
{
	TestContext* context = (TestContext*)user;
	context->brake_release = release;
	context->brake_write_count++;
}

static void FeedResponse(uint8_t function, uint8_t reg1, uint8_t reg2)
{
	uint8_t data[TSDA_CAN_DATA_LEN] = {0U};
	data[0] = TSDA_DEFAULT_GROUP;
	data[1] = function;
	data[2] = reg1;
	data[5] = reg2;
	TSDA_AppOnCanRx(TSDA_DEFAULT_CAN_ID + TSDA_ACK_ID_OFFSET,
	                data,
	                TSDA_CAN_DATA_LEN);
}

static void FeedPosition(int32_t position_raw)
{
	uint8_t data[TSDA_CAN_DATA_LEN] = {0U};
	uint32_t raw = (uint32_t)position_raw;
	data[0] = TSDA_DEFAULT_GROUP;
	data[1] = 0x2BU;
	data[2] = TSDA_REG_FEEDBACK_POS_HIGH;
	data[3] = (uint8_t)(raw >> 24U);
	data[4] = (uint8_t)(raw >> 16U);
	data[5] = TSDA_REG_FEEDBACK_POS_LOW;
	data[6] = (uint8_t)(raw >> 8U);
	data[7] = (uint8_t)raw;
	TSDA_AppOnCanRx(TSDA_DEFAULT_CAN_ID + TSDA_ACK_ID_OFFSET,
	                data,
	                TSDA_CAN_DATA_LEN);
}

static void FeedOutputSpeed(int16_t speed_rpm)
{
	uint8_t data[TSDA_CAN_DATA_LEN] = {0U};
	uint16_t raw = (uint16_t)speed_rpm;
	data[0] = TSDA_DEFAULT_GROUP;
	data[1] = 0x2BU;
	data[2] = TSDA_REG_OUTPUT_SPEED;
	data[3] = (uint8_t)(raw >> 8U);
	data[4] = (uint8_t)raw;
	data[5] = TSDA_REG_UNUSED;
	TSDA_AppOnCanRx(TSDA_DEFAULT_CAN_ID + TSDA_ACK_ID_OFFSET,
	                data,
	                TSDA_CAN_DATA_LEN);
}

#if (TSDA_HARDWARE_VARIANT == TSDA_VARIANT_CHASSIS)
static void FeedPortLimit(uint8_t upper_active, uint8_t lower_active)
{
	uint8_t data[TSDA_CAN_DATA_LEN] = {0U};
	data[0] = TSDA_DEFAULT_GROUP;
	data[1] = 0x2BU;
	data[2] = TSDA_REG_PORT_LIMIT_STATUS;
	data[3] = (upper_active != 0U) ? 0U : 1U;
	data[4] = (lower_active != 0U) ? 0U : 1U;
	data[5] = TSDA_REG_UNUSED;
	TSDA_AppOnCanRx(TSDA_DEFAULT_CAN_ID + TSDA_ACK_ID_OFFSET,
	                data,
	                TSDA_CAN_DATA_LEN);
}
#endif

/* 推进共用初始化链：E3 -> 清故障 -> 速度模式 -> 0RPM -> 使能。 */
static void ReachEnableStable(TestContext* context, const TSDA_BoardIo* board_io)
{
	TSDA_AppInit(TestSend, context, board_io, 0U);
	assert(tsda_app_state == TSDA_APP_POWER_WAIT);

	TSDA_AppUpdate(500U);
	TSDA_AppUpdate(501U);
	assert(context->frame[1] == 0x2AU);
	assert(context->frame[2] == TSDA_REG_ALARM_STATUS);

	FeedResponse(0x2BU, TSDA_REG_ALARM_STATUS, TSDA_REG_UNUSED);
	TSDA_AppUpdate(502U);
	TSDA_AppUpdate(512U);
	assert(context->frame[2] == TSDA_REG_CLEAR_FAULT);

	FeedResponse(0x1BU, TSDA_REG_CLEAR_FAULT, TSDA_REG_UNUSED);
	TSDA_AppUpdate(513U);
	TSDA_AppUpdate(523U);
	assert(context->frame[2] == TSDA_REG_MODE);
	assert(context->frame[5] == TSDA_REG_SPEED_ACC_DEC);

	FeedResponse(0x1BU, TSDA_REG_MODE, TSDA_REG_SPEED_ACC_DEC);
	TSDA_AppUpdate(524U);
	TSDA_AppUpdate(534U);
	assert(context->frame[2] == TSDA_REG_TARGET_SPEED);
	assert(context->frame[3] == 0U && context->frame[4] == 0U);

	FeedResponse(0x1BU, TSDA_REG_TARGET_SPEED, TSDA_REG_UNUSED);
	TSDA_AppUpdate(535U);
	TSDA_AppUpdate(545U);
	assert(context->frame[2] == TSDA_REG_ENABLE);
	assert(context->frame[4] == 1U);
	assert(tsda_app_state == TSDA_APP_ENABLE_STABLE_WAIT);
}

int main(void)
{
	TestContext context = {0};
	TSDA_BoardIo board_io = {
		TestReadUpper,
		TestReadLower,
		TestWriteBrake,
		&context
	};

	ReachEnableStable(&context, &board_io);

#if (TSDA_HARDWARE_VARIANT == TSDA_VARIANT_3PIN)
	/* 初始化立即给出闭合命令；1500ms窗口结束前不得释放抱闸。 */
	assert(context.brake_write_count == 1U);
	assert(context.brake_release == 0U);
	TSDA_AppUpdate(2044U);
	assert(context.brake_release == 0U);
	TSDA_AppUpdate(2045U);
	assert(tsda_app_state == TSDA_APP_BRAKE_RELEASE_SETTLE_WAIT);
	assert(context.brake_release == 1U);
	assert(tsda_status.brake_release_command == 1U);

	/* 释放后必须完整等待2000ms，且板级上下限按高电平有效被观察。 */
	context.upper_limit = 0U;
	context.lower_limit = 1U;
	TSDA_AppUpdate(4044U);
	assert(tsda_app_state == TSDA_APP_BRAKE_RELEASE_SETTLE_WAIT);
	assert(tsda_status.upper_limit_active == 0U);
	assert(tsda_status.lower_limit_active == 1U);
	TSDA_AppUpdate(4045U);
	assert(tsda_app_state == TSDA_APP_SEND_HOME_START_POSITION);
	assert(tsda_status.ready == 0U);
	assert(tsda_status.homing_done == 0U);

	/* 3Pin不读取0x58：先读寻限起点，再按-50RPM、E8/E9、E4循环。 */
	TSDA_AppUpdate(4055U);
	assert(context.frame[1] == 0x2AU);
	assert(context.frame[2] == TSDA_REG_FEEDBACK_POS_HIGH);
	assert(context.frame[5] == TSDA_REG_FEEDBACK_POS_LOW);
	FeedPosition(100000);
	TSDA_AppUpdate(4056U);
	assert(tsda_app_state == TSDA_APP_HOME_FIND_UPPER);
	assert(context.frame[1] == 0x1AU);
	assert(context.frame[2] == TSDA_REG_TARGET_SPEED);
	assert(context.frame[3] == 0xFFU && context.frame[4] == 0xCEU);
	TSDA_AppUpdate(4057U);
	assert(context.frame[2] == TSDA_REG_FEEDBACK_POS_HIGH);
	TSDA_AppUpdate(4058U);
	assert(context.frame[1] == 0x2AU);
	assert(context.frame[2] == TSDA_REG_OUTPUT_SPEED);
	assert(context.saw_port58_read == 0U);

	/* PD7触发周期必须立即发送0RPM，随后停稳并读取最终位置建立零点。 */
	context.upper_limit = 1U;
	TSDA_AppUpdate(4059U);
	TSDA_AppUpdate(4060U);
	TSDA_AppUpdate(4061U);
	assert(tsda_app_state == TSDA_APP_WAIT_HOME_STOP_ACK);
	assert(context.frame[1] == 0x1AU);
	assert(context.frame[2] == TSDA_REG_TARGET_SPEED);
	assert(context.frame[3] == 0U && context.frame[4] == 0U);
	FeedResponse(0x1BU, TSDA_REG_TARGET_SPEED, TSDA_REG_UNUSED);
	TSDA_AppUpdate(4062U);
	assert(tsda_app_state == TSDA_APP_WAIT_HOME_STABLE);
	TSDA_AppUpdate(4111U);
	assert(tsda_app_state == TSDA_APP_WAIT_HOME_STABLE);
	TSDA_AppUpdate(4112U);
	TSDA_AppUpdate(4122U);
	assert(context.frame[2] == TSDA_REG_FEEDBACK_POS_HIGH);
	FeedPosition(123456);
	TSDA_AppUpdate(4123U);
	assert(tsda_app_state == TSDA_APP_HOME_MOVE_RETURN);
	assert(tsda_status.position_origin_raw == 123456);
	assert(tsda_status.current_height_mm == 0.0f);
	assert(tsda_status.homing_done == 0U);
	assert(tsda_status.ready == 0U);
	assert(context.frame[2] == TSDA_REG_TARGET_SPEED);
	assert(context.frame[3] == 0U && context.frame[4] == 200U);

	/* 两种Profile共用固定+200RPM回位。到-100mm且E4停稳后直接进入DONE。 */
	TSDA_AppUpdate(4124U);
	assert(context.frame[2] == TSDA_REG_FEEDBACK_POS_HIGH);
	FeedPosition(223456);
	TSDA_AppUpdate(4125U);
	assert(context.frame[2] == TSDA_REG_OUTPUT_SPEED);
	FeedOutputSpeed(200);
	TSDA_AppUpdate(4126U);
	assert(tsda_app_state == TSDA_APP_HOME_MOVE_RETURN);
	TSDA_AppUpdate(4127U);
	FeedPosition(323456);
	TSDA_AppUpdate(4128U);
	FeedOutputSpeed(0);
	TSDA_AppUpdate(4129U);
	assert(tsda_app_state == TSDA_APP_DONE);
	assert(tsda_status.current_height_mm == -100.0f);
	assert(tsda_status.homing_done == 1U);
	assert(tsda_status.ready == 1U);
	assert(context.frame[2] == TSDA_REG_TARGET_SPEED);
	assert(context.frame[3] == 0U && context.frame[4] == 0U);

	/* 正常停机：0RPM ACK后立即闭合抱闸，满200ms时发送失能。 */
	tsda_command.servo_enable = 0U;
	TSDA_AppUpdate(4130U);
	TSDA_AppUpdate(4131U);
	FeedResponse(0x1BU, TSDA_REG_TARGET_SPEED, TSDA_REG_UNUSED);
	TSDA_AppUpdate(4132U);
	assert(tsda_app_state == TSDA_APP_WAIT_BRAKE_ENGAGE);
	assert(context.brake_release == 0U);
	TSDA_AppUpdate(4331U);
	assert(tsda_app_state == TSDA_APP_WAIT_BRAKE_ENGAGE);
	TSDA_AppUpdate(4332U);
	assert(tsda_app_state == TSDA_APP_DISABLED);
	assert(context.frame[2] == TSDA_REG_ENABLE);
	assert(context.frame[3] == 0U && context.frame[4] == 0U);

	/* 异常发送失败不等待ACK和200ms：补救后立即闭合抱闸并进入ERROR。 */
	memset(&context, 0, sizeof(context));
	ReachEnableStable(&context, &board_io);
	TSDA_AppUpdate(2045U);
	TSDA_AppUpdate(4045U);
	assert(context.brake_release == 1U);
	context.send_fail = 1U;
	TSDA_AppUpdate(4055U);
	assert(tsda_app_state == TSDA_APP_ERROR);
	assert(tsda_status.error_code == TSDA_APP_ERROR_SEND);
	assert(context.brake_release == 0U);

	/* 3Pin寻限90s保护：只尝试一次0RPM，不等ACK，立即抱闸并进入ERROR。 */
	memset(&context, 0, sizeof(context));
	ReachEnableStable(&context, &board_io);
	TSDA_AppUpdate(2045U);
	TSDA_AppUpdate(4045U);
	TSDA_AppUpdate(4055U);
	FeedPosition(100000);
	TSDA_AppUpdate(4056U);
	assert(tsda_app_state == TSDA_APP_HOME_FIND_UPPER);
	TSDA_AppUpdate(94056U);
	assert(tsda_app_state == TSDA_APP_ERROR);
	assert(tsda_status.error_code == TSDA_APP_ERROR_HOME_TIMEOUT);
	assert(tsda_status.commanded_speed_rpm == 0);
	assert(context.brake_release == 0U);
#else
	/* Chassis不调用MCU抱闸，使用0x58寻限，但后续回位和DONE与3Pin相同。 */
	assert(context.brake_write_count == 0U);
	TSDA_AppUpdate(2045U);
	assert(tsda_app_state == TSDA_APP_SEND_HOME_LIMIT_CHECK);
	assert(tsda_status.brake_release_command == 0U);
	TSDA_AppUpdate(2046U);
	assert(context.frame[1] == 0x2AU);
	assert(context.frame[2] == TSDA_REG_PORT_LIMIT_STATUS);
	FeedPortLimit(0U, 0U);
	TSDA_AppUpdate(2047U);
	assert(tsda_app_state == TSDA_APP_SEND_HOME_START_POSITION);
	TSDA_AppUpdate(2057U);
	FeedPosition(100000);
	TSDA_AppUpdate(2058U);
	assert(tsda_app_state == TSDA_APP_HOME_FIND_UPPER);
	assert(context.frame[2] == TSDA_REG_TARGET_SPEED);
	assert(context.frame[3] == 0xFFU && context.frame[4] == 0x06U);
	TSDA_AppUpdate(2059U);
	TSDA_AppUpdate(2060U);
	assert(context.frame[2] == TSDA_REG_PORT_LIMIT_STATUS);
	FeedPortLimit(1U, 0U);
	TSDA_AppUpdate(2061U);
	assert(tsda_app_state == TSDA_APP_WAIT_HOME_STOP_ACK);
	assert(context.frame[2] == TSDA_REG_TARGET_SPEED);
	assert(context.frame[3] == 0U && context.frame[4] == 0U);
	FeedResponse(0x1BU, TSDA_REG_TARGET_SPEED, TSDA_REG_UNUSED);
	TSDA_AppUpdate(2062U);
	TSDA_AppUpdate(2112U);
	TSDA_AppUpdate(2122U);
	FeedPosition(123456);
	TSDA_AppUpdate(2123U);
	assert(tsda_app_state == TSDA_APP_HOME_MOVE_RETURN);
	assert(context.frame[2] == TSDA_REG_TARGET_SPEED);
	assert(context.frame[3] == 0U && context.frame[4] == 200U);
	TSDA_AppUpdate(2124U);
	FeedPosition(223456);
	TSDA_AppUpdate(2125U);
	FeedOutputSpeed(200);
	TSDA_AppUpdate(2126U);
	TSDA_AppUpdate(2127U);
	FeedPosition(323456);
	TSDA_AppUpdate(2128U);
	FeedOutputSpeed(0);
	TSDA_AppUpdate(2129U);
	assert(tsda_app_state == TSDA_APP_DONE);
	assert(tsda_status.current_height_mm == -100.0f);
	assert(tsda_status.homing_done == 1U);
	assert(tsda_status.ready == 1U);
#endif

	puts("TSDA profile regression passed");
	return 0;
}
