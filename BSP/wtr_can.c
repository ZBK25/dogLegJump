/*过滤器配置，can的fifo0接收*/

#include "wtr_can.h"
#include "can.h"
#include "encoder.h"
//#include "FreeRTOS.h"
//#include "DJI.h"

uint8_t CanReceiveData[8];

HAL_StatusTypeDef CANFilterInit(CAN_HandleTypeDef* hcan){
  CAN_FilterTypeDef  sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
	
  if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    return HAL_ERROR;
  }
	
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    return HAL_ERROR;
  }
	return HAL_OK;
}

HAL_StatusTypeDef CAN2FilterInit(CAN_HandleTypeDef* hcan){
    CAN_FilterTypeDef  sFilterConfig;

    sFilterConfig.FilterBank = 14;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 28;


    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    return HAL_OK;
}

void CanDataDecode(CAN_RxHeaderTypeDef RxHeader){
  /* Can message Decode */
  if( RxHeader.IDE == CAN_ID_STD ){
    DJI_CanMsgDecode(RxHeader.StdId, CanReceiveData);
  }
  if( RxHeader.IDE == CAN_ID_EXT ){
    // vesc反馈关掉这里就不会有消息
    ;;
  }
  
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    CAN_RxHeaderTypeDef   RxHeader;
    uint8_t RxData[8];
    /*
    if( hcan->Instance == hcan1.Instance ){
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CanReceiveData) != HAL_OK)
        {
            Error_Handler();
        }
        CanDataDecode(RxHeader);
    }
    */
    if(hcan->Instance == hcan1.Instance){
          // 检查接收FIFO是否有消息
      if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
          // 接收消息
          if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
              //printf("Received message ID: %X, Data: ", RxHeader.StdId);
              //printf("Received message ID: %d, Data: ", (int)RxHeader.StdId);
              for (int i = 0; i < RxHeader.DLC; i++) {
                //printf("%02X ", RxData[i]);
              }
              //printf("\n");
              // 解析数据
              //ParseCANData(&RxHeader, RxData, &encoderData,&encoderCalculateData);
              DecodeCANData(&RxHeader, RxData, hDJI );
          } else {
              printf("Failed to get message from FIFO\n");
          }
      } else {
          printf("No message in FIFO\n");
      }
    }
}

void DecodeCANData(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, DJI_t* hDJI ) {
  switch(RxHeader->StdId){
		case 0x201:
		case 0x202:
		case 0x203:
		case 0x204:
			{
				static uint8_t i;
				i = RxHeader->StdId - 0x201;

        float last_angle = hDJI[0].FdbData.RotorAngle_0_360;
	      hDJI[i].FdbData.RotorAngle_0_360 = (uint16_t)(RxData[0]<<8 | RxData[1]) * 0.01;
	      hDJI[i].FdbData.rpm  = (int16_t)(RxData[2]<<8 | RxData[3]) * 0.1;
	      hDJI[i].FdbData.current = (RxData[4]<<8 | RxData[5])*5.f/16384.f;
        
	      if(hDJI[i].FdbData.RotorAngle_0_360 - last_angle > 180){
          hDJI[i].Calculate.RotorRound --;
        }
	      else if (hDJI[i].FdbData.RotorAngle_0_360 - last_angle < -180){
          hDJI[i].Calculate.RotorRound ++;
        }
        //以下的角度计算仍有误
	      hDJI[i].Calculate.RotorAngle_all = hDJI[i].Calculate.RotorRound * 360 + hDJI[i].FdbData.RotorAngle_0_360 - hDJI[i].Calculate.RotorAngle_0_360_OffSet;
        hDJI[i].AxisData.AxisAngle_inDegree = hDJI[i].Calculate.RotorAngle_all;
			}
			break;
	}
}