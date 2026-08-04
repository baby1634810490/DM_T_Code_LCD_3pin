//
// Created by Lenovo on 2025/1/6.
//

/**
  ******************************************************************************
  * @file    driver_remote.c
  * @author  Shuai Yang
  * @brief   这个文件提供函数对遥控器接收器反馈值进行处理得到摇杆和拨杆的数据，
  *          对机器人控制量进行更新
  *
  @verbatim
  ==============================================================================
                        ##### How to use this driver #####
  ==============================================================================
    [..]
      (#) Update the remote data by implementing the crsfDataUpdate():
          (++) Update the remote receiver parameters, including:
		       (+++) CrsfData

      (#) Update the remote data by implementing the RemoteDataUpdate():
          (++) Update the robot remote parameters, including:
		       (+++) RobotRemote

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * 这个驱动文件完成了将遥控器接收器的反馈数据解算为各个通道的数据，并进一步得到机器人的控制数据。
  * 使得其他驱动文件可以调用，并完成对机器人控制目标的设定。
  *
  * Please add comments after adding or deleting functions to ensure code
  * specification.
  *
  ******************************************************************************
  */

/* Includes ---------------------------------------------------------------- */
#include "driver_remote.h"

/* ----------------------- Internal Data ----------------------------------- */
/** @brief The data received from the remote receiver, is the memory address of
  *        the serial port DMA transfer
  */
uint8_t remoteData[50] = {0};

/** @brief According to the Crsf protocol, the data of each channel of the
  *        remote control is solved
  */
CrsfData_t CrsfData;

/** @brief A structure that stores remote data instructions
  */
RemoteDataPortStruct RobotRemote = {0};

/** @brief A structure that stores robot control instructions
  */
RobotCtlPortStruct RobotControl = {0};

/* ----------------------- Function Implements ---------------------------- */
/**
  * @brief  遥控器摇杆通道数据死区处理
  * @param  rawData : original received data
  * @retval the processed data
  */
float RemoteDeadBand(uint16_t rawData, float midData)
{
    float data;
    if ((float)rawData <= (midData + 10.0f) && (float)rawData >= (midData - 10.0f))
        data = midData;
    else
        data = (float)rawData;

    return data;
}

/**
  * @brief  Solve the remote data
  * @param  data : original received data
  * @param  crsfData : processed structure data
  * @retval Is the data correct? True: ENABLE, incorrect: DISABLE
  */
uint8_t CrsfDataUpdate(const uint8_t* data, CrsfData_t* crsfData)
{
    if (data[0] == 0xC8 && data[1] == 0x18 && data[2] == 0x16)
    {
        crsfData->CH1 = ((int16_t)data[3] >> 0 | ((int16_t)data[4] << 8)) & 0x07FF;
        crsfData->CH2 = ((int16_t)data[4] >> 3 | ((int16_t)data[5] << 5)) & 0x07FF;
        crsfData->CH3 = ((int16_t)data[5] >> 6 | ((int16_t)data[6] << 2) | (int16_t)data[7] << 10) & 0x07FF;
        crsfData->CH4 = ((int16_t)data[7] >> 1 | ((int16_t)data[8] << 7)) & 0x07FF;
        crsfData->CH5 = ((int16_t)data[8] >> 4 | ((int16_t)data[9] << 4)) & 0x07FF;
        crsfData->CH6 = ((int16_t)data[9] >> 7 | ((int16_t)data[10] << 1) | (int16_t)data[11] << 9) & 0x07FF;
        crsfData->CH7 = ((int16_t)data[11] >> 2 | ((int16_t)data[12] << 6)) & 0x07FF;
        crsfData->CH8 = ((int16_t)data[12] >> 5 | ((int16_t)data[13] << 3)) & 0x07FF;
        crsfData->CH9 = ((int16_t)data[14] << 0 | ((int16_t)data[15] << 8)) & 0x07FF;
        crsfData->CH10 = ((int16_t)data[15] >> 3 | ((int16_t)data[16] << 5)) & 0x07FF;
        crsfData->CH11 = ((int16_t)data[16] >> 6 | ((int16_t)data[17] << 2) | (int16_t)data[18] << 10) & 0x07FF;
        crsfData->CH12 = ((int16_t)data[18] >> 1 | ((int16_t)data[19] << 7)) & 0x07FF;
        crsfData->CH13 = ((int16_t)data[19] >> 4 | ((int16_t)data[20] << 4)) & 0x07FF;
        crsfData->CH14 = ((int16_t)data[20] >> 7 | ((int16_t)data[21] << 1) | (int16_t)data[22] << 9) & 0x07FF;
        crsfData->CH15 = ((int16_t)data[22] >> 2 | ((int16_t)data[23] << 6)) & 0x07FF;
        crsfData->CH16 = ((int16_t)data[23] >> 5 | ((int16_t)data[24] << 3)) & 0x07FF;

        return ENABLE;
    }
    return DISABLE;
}

/**
  * @brief  将解算后的通道数据进行进一步解算，得到用以控制机器人的遥控器指令值
  * @param  crsfData : data before processing
  * @param  RemoteDataStruct : the processed data
  * @retval None
  */
void RemoteDataUpdate(CrsfData_t crsfData, RemoteDataPortStruct* RemoteDataStruct)
{
    /* RX向右增大，向左减小 */
    RemoteDataStruct->RX = (RemoteDeadBand(crsfData.CH1, CH_VALUE_OFFSET) - CH_VALUE_OFFSET) * 2.0f
        / (CH_VALUE_MAX - CH_VALUE_MIN);
    /* RY向上增大，向下减小 */
    RemoteDataStruct->RY = (RemoteDeadBand(crsfData.CH2, CH_VALUE_OFFSET) - CH_VALUE_OFFSET) * 2.0f
        / (CH_VALUE_MAX - CH_VALUE_MIN);
    /* LY向上增大，向下减小 */
    RemoteDataStruct->LY = (RemoteDeadBand(crsfData.CH3, CH_VALUE_MIN) - CH_VALUE_MIN) * 2.0f
        / (CH_VALUE_MAX - CH_VALUE_MIN);
    /* LX向右增大，向左减小 */
    RemoteDataStruct->LX = (RemoteDeadBand(crsfData.CH4, CH_VALUE_OFFSET) - CH_VALUE_OFFSET) * 2.0f
        / (CH_VALUE_MAX - CH_VALUE_MIN);
    switch (crsfData.CH5)
    {
    case SW_UP:
        RemoteDataStruct->L_SW2 = SW_Up2;
        break;
    case SW_DOWN:
        RemoteDataStruct->L_SW2 = SW_Down2;
        break;
    }
    switch (crsfData.CH6)
    {
    case SW_UP:
        RemoteDataStruct->L_SW3 = SW_Up3;
        break;
    case SW_MID:
        RemoteDataStruct->L_SW3 = SW_Mid3;
        break;
    case SW_DOWN:
        RemoteDataStruct->L_SW3 = SW_Down3;
        break;
    }
    switch (crsfData.CH7)
    {
    case SW_UP:
        RemoteDataStruct->R_SW3 = SW_Up3;
        break;
    case SW_MID:
        RemoteDataStruct->R_SW3 = SW_Mid3;
        break;
    case SW_DOWN:
        RemoteDataStruct->R_SW3 = SW_Down3;
        break;
    }
    switch (crsfData.CH8)
    {
    case SW_UP:
        RemoteDataStruct->R_SW2 = SW_Up2;
        break;
    case SW_DOWN:
        RemoteDataStruct->R_SW2 = SW_Down2;
        break;
    }
}

/**
  * @brief  根据遥控器指令值，得到机器人控制量的设定值
  * @param  RemoteDataPortStruct : data before processing
  * @param  RobotCtlPortStruct : the processed data
  * @retval None
  */
void RobotCtlDataUpdate(RemoteDataPortStruct RemoteDataStruct, RobotCtlPortStruct* RobotCtl)
{
    RobotCtl->X_Set = RemoteDataStruct.LY;
    RobotCtl->Y_Set = -1.0f * RemoteDataStruct.LX;
    RobotCtl->Z_Set = RemoteDataStruct.RY;
    RobotCtl->Spin_Set = RemoteDataStruct.RX * -0.5f;
}

/************************ (C) COPYRIGHT XJTU ROBOMASTER ********END OF FILE****/
