#ifndef WTR_CAN_H
#define WTR_CAN_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "DJI.h"
#include "can.h"
#include "stm32f4xx_hal.h"

HAL_StatusTypeDef CANFilterInit(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef CAN2FilterInit(CAN_HandleTypeDef* hcan);
void DecodeCANData(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, DJI_t* hDJI );

#ifdef __cplusplus
}
#endif



#endif
