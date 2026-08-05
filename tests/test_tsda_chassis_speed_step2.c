#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "app_tsda_servo.h"

typedef struct
{
	uint32_t can_id;
	uint8_t data[TSDA_CAN_DATA_LEN];
	uint32_t count;
} MockCan;

static MockCan mock_can;

static uint8_t MockSend(uint32_t can_id,
                        const uint8_t* data,
                        uint8_t len,
                        void* user)
{
	MockCan* mock = (MockCan*)user;
	assert(len == TSDA_CAN_DATA_LEN);
	mock->can_id = can_id;
	memcpy(mock->data, data, TSDA_CAN_DATA_LEN);
	mock->count++;
	return 0U;
}

static void FeedResponse(uint8_t function, uint8_t reg1, int16_t value1,
                         uint8_t reg2, int16_t value2)
{
	uint8_t data[TSDA_CAN_DATA_LEN] = {0U};
	uint16_t raw1 = (uint16_t)value1;
	uint16_t raw2 = (uint16_t)value2;

	data[0] = TSDA_DEFAULT_GROUP;
	data[1] = function;
	data[2] = reg1;
	data[3] = (uint8_t)(raw1 >> 8U);
	data[4] = (uint8_t)raw1;
	data[5] = reg2;
	data[6] = (uint8_t)(raw2 >> 8U);
	data[7] = (uint8_t)raw2;
	TSDA_AppOnCanRx(TSDA_DEFAULT_CAN_ID + TSDA_ACK_ID_OFFSET,
	                data,
	                TSDA_CAN_DATA_LEN);
}

static void FeedPosition(int32_t position)
{
	FeedResponse(0x2BU,
	             TSDA_REG_FEEDBACK_POS_HIGH,
	             (int16_t)((uint32_t)position >> 16U),
	             TSDA_REG_FEEDBACK_POS_LOW,
	             (int16_t)((uint32_t)position & 0xFFFFU));
}

int main(void)
{
	memset(&mock_can, 0, sizeof(mock_can));
	TSDA_AppInit(MockSend, &mock_can, 0U);

	/* 初始化：在线、清故障、速度模式1/1、0RPM、使能。 */
	TSDA_AppUpdate(500U);
	TSDA_AppUpdate(501U);
	FeedResponse(0x2BU, TSDA_REG_ALARM_STATUS, 0, TSDA_REG_UNUSED, 0);
	TSDA_AppUpdate(502U);
	TSDA_AppUpdate(512U);
	FeedResponse(0x1BU, TSDA_REG_CLEAR_FAULT, 0, TSDA_REG_UNUSED, 0);
	TSDA_AppUpdate(513U);
	TSDA_AppUpdate(523U);
	FeedResponse(0x1BU, TSDA_REG_MODE, TSDA_MODE_SPEED_PC,
	             TSDA_REG_SPEED_ACC_DEC, 0x0101);
	TSDA_AppUpdate(524U);
	TSDA_AppUpdate(534U);
	FeedResponse(0x1BU, TSDA_REG_TARGET_SPEED, 0, TSDA_REG_UNUSED, 0);
	TSDA_AppUpdate(535U);
	TSDA_AppUpdate(545U);

	/* 使能稳定后先读0x58；0x0101表示上下限均未触发。 */
	TSDA_AppUpdate(2045U);
	TSDA_AppUpdate(2046U);
	assert(mock_can.data[1] == 0x2AU);
	assert(mock_can.data[2] == TSDA_REG_PORT_LIMIT_STATUS);
	FeedResponse(0x2BU, TSDA_REG_PORT_LIMIT_STATUS, 0x0101,
	             TSDA_REG_UNUSED, 0);
	TSDA_AppUpdate(2047U);

	/* 读取寻限起点后，第一拍必须下发实机确认的向上 -250RPM。 */
	TSDA_AppUpdate(2057U);
	assert(mock_can.data[2] == TSDA_REG_FEEDBACK_POS_HIGH);
	FeedPosition(100000);
	TSDA_AppUpdate(2058U);
	assert(tsda_app_state == TSDA_APP_HOME_FIND_UPPER);
	assert(mock_can.data[2] == TSDA_REG_TARGET_SPEED);
	assert(mock_can.data[3] == 0xFFU);
	assert(mock_can.data[4] == 0x06U);

	/* 第二、三拍依次读取位置和0x58。 */
	TSDA_AppUpdate(2059U);
	assert(mock_can.data[2] == TSDA_REG_FEEDBACK_POS_HIGH);
	FeedPosition(100100);
	TSDA_AppUpdate(2060U);
	assert(mock_can.data[2] == TSDA_REG_PORT_LIMIT_STATUS);

	/* data[3]=0 表示上限触发；处理回包的同一周期应立即写0RPM。 */
	FeedResponse(0x2BU, TSDA_REG_PORT_LIMIT_STATUS, 0x0001,
	             TSDA_REG_UNUSED, 0);
	TSDA_AppUpdate(2061U);
	assert(mock_can.data[2] == TSDA_REG_TARGET_SPEED);
	assert(mock_can.data[3] == 0U);
	assert(mock_can.data[4] == 0U);

	FeedResponse(0x1BU, TSDA_REG_TARGET_SPEED, 0, TSDA_REG_UNUSED, 0);
	TSDA_AppUpdate(2062U);
	assert(tsda_app_state == TSDA_APP_WAIT_HOME_STABLE);

	/* 零速ACK后必须等待50ms，再满足10ms命令间隔后读取停止位置。 */
	TSDA_AppUpdate(2112U);
	assert(tsda_app_state == TSDA_APP_SEND_HOME_ZERO_POSITION);
	TSDA_AppUpdate(2122U);
	assert(mock_can.data[2] == TSDA_REG_FEEDBACK_POS_HIGH);
	FeedPosition(100120);
	TSDA_AppUpdate(2123U);

	assert(tsda_app_state == TSDA_APP_HOME_MOVE_RETURN);
	assert(tsda_status.homing_done == 0U);
	assert(tsda_status.position_origin_raw == 100120);
	assert(tsda_status.commanded_speed_rpm == TSDA_APP_HOME_RETURN_SPEED_RPM);
	assert(tsda_status.ready == 0U);
	assert(tsda_status.error_code == TSDA_APP_ERROR_NONE);

	return 0;
}
