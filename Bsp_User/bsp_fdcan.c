//
// Created by Lenovo on 2025/5/26.
//

/**
  ******************************************************************************
  * @file    bsp_fdcan.c
  * @author  Shuai Yang
  * @brief   bsp fdcan
  *          这个文件提供函数对 FDCAN 通信的初始化、发送和回调
  *           + FDCAN Init
  *           + FDCAN Send Msg
  *           + FDCAN RxFifo Callback
  *
  @verbatim
  ==============================================================================

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * Don't forget the author
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_fdcan.h"
#include "app_tsda_servo.h"
#include "driver_dm_motor.h"
#include "driver_rs_motor.h"

/* Exported variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;

extern FDCAN_HandleTypeDef hfdcan2;

extern FDCAN_HandleTypeDef hfdcan3;

extern RS_MotorStruct FirJointMotor;

/* ----------------------- Function Implements ---------------------------- */
/**
  * @brief  Configures the CAN1 reception filter
  *         所配置的过滤器均允许通过
  * @retval None
  */
void FDCAN1_Filter_Config(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	/* 扩展帧过滤器 */
	fdcan_filter.IdType = FDCAN_EXTENDED_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //扩展帧为 FIFO0
	fdcan_filter.FilterID1 = 0x000; //掩码 ID
	fdcan_filter.FilterID2 = 0x000; //Mask 后两位设置为 0 几乎不过滤报文
	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
	/* 标准帧过滤器 */
	fdcan_filter.IdType = FDCAN_STANDARD_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //标准帧为 FIFO0
	fdcan_filter.FilterID1 = 0x000; //掩码 ID
	fdcan_filter.FilterID2 = 0x000; //Mask 后两位设置为 0 几乎不过滤报文
	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

/**
  * @brief  Configures the CAN2 reception filter
  *         所配置的过滤器均允许通过
  * @retval None
  */
void FDCAN2_Filter_Config(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	/* 扩展帧过滤器 */
	fdcan_filter.IdType = FDCAN_EXTENDED_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //扩展帧为 FIFO0
	fdcan_filter.FilterID1 = 0x000; //掩码 ID
	fdcan_filter.FilterID2 = 0x000; //Mask 后两位设置为 0 几乎不过滤报文
	HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
	/* 标准帧过滤器 */
	fdcan_filter.IdType = FDCAN_STANDARD_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //标准帧为 FIFO0
	fdcan_filter.FilterID1 = 0x000; //掩码 ID
	fdcan_filter.FilterID2 = 0x000; //Mask 后两位设置为 0 几乎不过滤报文
	HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

/**
  * @brief  Configures the CAN2 reception filter
  *         所配置的过滤器均允许通过
  * @retval None
  */
void FDCAN3_Filter_Config(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	/* 扩展帧过滤器 */
	fdcan_filter.IdType = FDCAN_EXTENDED_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; //扩展帧为 FIFO1
	fdcan_filter.FilterID1 = 0x000; //掩码 ID
	fdcan_filter.FilterID2 = 0x000; //Mask 后两位设置为 0 几乎不过滤报文
	HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter);
	/* 标准帧过滤器 */
	fdcan_filter.IdType = FDCAN_STANDARD_ID;
	fdcan_filter.FilterIndex = 0;
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; //标准帧为 FIFO1
	fdcan_filter.FilterID1 = 0x000; //掩码 ID
	fdcan_filter.FilterID2 = 0x000; //Mask 后两位设置为 0 几乎不过滤报文
	HAL_FDCAN_ConfigFilter(&hfdcan3, &fdcan_filter);
	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

/**
  * @brief  Enable FDCAN1
  * @retval None
  */
void FDCAN1Config(void)
{
	FDCAN1_Filter_Config();
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
  * @brief  Enable FDCAN2
  * @retval None
  */
void FDCAN2Config(void)
{
	FDCAN2_Filter_Config();
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

/**
  * @brief  Enable FDCAN3
  * @retval None
  */
void FDCAN3Config(void)
{
	FDCAN3_Filter_Config();
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

/**
  * @brief  CAN1 Send the massage
  * @param  msg array containing the payload of the Tx frame.
  * @param  len the length of the frame that will be transmitted
  * @param  id StdId
  * @param  ide FDCAN_id_type FDCAN ID Type
  * @retval None
  */
uint8_t FDCAN1_Send_Msg(const uint8_t* msg, uint8_t len, uint32_t id, uint32_t ide)
{
	FDCAN_TxHeaderTypeDef pTxHeader;
	uint8_t i;
	uint8_t data[8];

	pTxHeader.Identifier = id;
	pTxHeader.IdType = ide;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength = len;
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch = FDCAN_BRS_ON;
	pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker = 0;

	for (i = 0; i < len; i++)
		data[i] = msg[i];
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &pTxHeader, data) != HAL_OK)
		return 1; //发送
	return 0;
}

/**
  * @brief  CAN2 Send the massage
  * @param  msg array containing the payload of the Tx frame.
  * @param  len the length of the frame that will be transmitted
  * @param  id StdId or ExtId
  * @param  ide FDCAN_id_type FDCAN ID Type
  * @retval None
  */
uint8_t FDCAN2_Send_Msg(const uint8_t* msg, uint8_t len, uint32_t id, uint32_t ide)
{
	FDCAN_TxHeaderTypeDef pTxHeader;
	uint8_t i;
	uint8_t data[8];

	pTxHeader.Identifier = id;
	pTxHeader.IdType = ide;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength = len;
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch = FDCAN_BRS_ON;
	pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker = 0;

	for (i = 0; i < len; i++)
		data[i] = msg[i];
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &pTxHeader, data) != HAL_OK)
		return 1; //发送
	return 0;

}

/**
  * @brief  CAN3 Send the massage
  * @param  msg array containing the payload of the Tx frame.
  * @param  len the length of the frame that will be transmitted
  * @param  id StdId or ExtId
  * @param  ide FDCAN_id_type FDCAN ID Type
  * @retval None
  */
uint8_t FDCAN3_Send_Msg(const uint8_t* msg, uint8_t len, uint32_t id, uint32_t ide)
{
	FDCAN_TxHeaderTypeDef pTxHeader;
	uint8_t i;
	uint8_t data[8];

	pTxHeader.Identifier = id;
	pTxHeader.IdType = ide;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength = len;
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch = FDCAN_BRS_ON;
	pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker = 0;

	for (i = 0; i < len; i++)
		data[i] = msg[i];
	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &pTxHeader, data) != HAL_OK)
		return 1; //发送
	return 0;
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t CAN_Rx_Buffer[8];

	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, CAN_Rx_Buffer);
	if (hfdcan == &hfdcan1)
	{
		/*
		 * CAN1是当前Chassis TSDA的唯一接收通道。中断只把帧交给App缓存；
		 * 寄存器解析和运动状态跳转统一在1ms MotorTask中执行。
		 */
		//RS_MotorDataReceive(&FirJointMotor, CAN_Rx_Buffer, pRxHeader.Identifier);
		TSDA_AppOnCanRx(pRxHeader.Identifier, CAN_Rx_Buffer, 8);
	}
	if (hfdcan == &hfdcan2)
	{
		// TSDA_AppOnCanRx(pRxHeader.Identifier, CAN_Rx_Buffer, 8);

	}
}

/**
  * @brief  Rx FIFO 1 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo1ITs indicates which Rx FIFO 1 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo1_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t CAN_Rx_Buffer[8];

	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &pRxHeader, CAN_Rx_Buffer);
	if (hfdcan == &hfdcan3)
	{
	}
}

/***************************** (C) END OF FILE ******************************/
