/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#include "can.h"
#include "bsp_can.h"
#include "encoder.h"


moto_measure_t moto_chassis[4] = {0};//4 chassis moto



void get_total_angle(moto_measure_t *p);
void get_moto_offset(moto_measure_t *ptr, uint8_t* RxData);

/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef		CAN_FilterConfigStructure;
	// static CAN_TxHeaderTypeDef		Tx1Message;
	// static CAN_RxHeaderTypeDef 		Rx1Message;


	CAN_FilterConfigStructure.FilterBank = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	//CAN_FilterConfigStructure.BankNumber = 14;//can1(0-13)和can2(14-27)分别得到一半的filter
	CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
		//err_deadloop(); //show error!
	}


	// if(_hcan == &hcan1){
	// 	_hcan->pTxMsg = &Tx1Message;
	// 	_hcan->pRxMsg = &Rx1Message;
	// }


}

//uint32_t FlashTimer;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
{
	//接收信息闪烁灯
	// if(HAL_GetTick() - FlashTimer>500){
			
	// 	HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	// 	FlashTimer = HAL_GetTick();
		
	// }

	static CAN_RxHeaderTypeDef RxHeader;
	static uint8_t RxData[8];
	printf("into rx\n");

	// 接收消息（推荐方式）
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_FILTER_FIFO0, &RxHeader, RxData) == HAL_OK)
	{
			//ignore can1 or can2.
	switch(RxHeader.StdId){
		case CAN_2006Moto1_ID:
		case CAN_2006Moto2_ID:
		case CAN_2006Moto3_ID:
		case CAN_2006Moto4_ID:
			{
				static u8 i;
				i = RxHeader.StdId - CAN_2006Moto1_ID;
				
				get_moto_measure(&moto_chassis[i], _hcan, RxData);
				//ParseCANData(&RxHeader, RxData, &encoderData, &encoderCalculateData);
			}
			break;
		
		
	}
    	printf("StdID: %x, DLC: %d\n", RxHeader.StdId, RxHeader.DLC);
    	for (int i = 0; i < RxHeader.DLC; i++) {
    	    printf("Data[%d] = %x\n", i, RxData[i]);
    	}
	}

	// //ignore can1 or can2.
	// switch(_hcan->pRxMsg->StdId){
	// 	case CAN_2006Moto1_ID:
	// 	case CAN_2006Moto2_ID:
	// 	case CAN_2006Moto3_ID:
	// 	case CAN_2006Moto4_ID:
	// 		{
	// 			static u8 i;
	// 			i = _hcan->pRxMsg->StdId - CAN_2006Moto1_ID;
				
	// 			get_moto_measure(&moto_chassis[i], _hcan);
	// 		}
	// 		break;
		
		
	// }
		
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    接收3508电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan, uint8_t* RxData)
{

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RxData[0]<<8 | RxData[1]) ;
	ptr->speed_rpm  = (int16_t)(RxData[2]<<8 | RxData[3]);
	ptr->real_current = (RxData[4]<<8 | RxData[5])*5.f/16384.f;

	ptr->hall = RxData[6];
	
	
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

// void ParseCANData(CAN_RxHeaderTypeDef *RxHeader, uint8_t *RxData, EncoderData *data,EncoderCalculateData *caldata) {
//     // 解析数据
//     if (RxData[0] == 0x55 && RxData[1] == 0x55) {
//         // 角度计算
//         uint16_t angleReg = (RxData[3] << 8) | RxData[2]; // 角度寄存器数值
//         data->angle = angleReg * 360.0f / 32768.0f; // 角度计算
//         int32_t angleScaled = (int32_t)(data->angle * 100); // 乘以100并存储为int32_t
//         caldata->angle = angleScaled;
//         // printf("Debug: Angle Register Value: %u, Calculated Angle: %ld degrees\n", angleReg, (long)angleScaled);
//         // printf("Debug:caldata->angle: %ld degrees\n", (long)caldata->angle);
//         // 角速度计算
//         uint16_t angularSpeedReg = (RxData[5] << 8) | RxData[4]; // 角速度寄存器数值
//         data->angularSpeed = angularSpeedReg * 360.0f / 32768.0f / 0.1f; // 角速度计算
//         int32_t angularSpeedScaled = (int32_t)(data->angularSpeed * 100); // 乘以100并存储为int32_t
//         caldata->angularSpeed = angularSpeedScaled;
//         // printf("Debug: Angular Speed Register Value: %u, Calculated Angular Speed: %ld degrees/s\n", angularSpeedReg, (long)angularSpeedScaled);

//         // 转数计算
//         data->revolutions = (int16_t)((RxData[7] << 8) | RxData[6]); // 转数计算，使用int16_t
//         // printf("Debug: Revolutions Register Value: %d\n", data->revolutions);
//         // printf("Parsed Data (Type 0x55 0x55): Angle: %ld (x100), Angular Speed: %ld (x100), Revolutions: %d\n",
//             // (long)angleScaled, (long)angularSpeedScaled, data->revolutions);
//     } else if (RxData[0] == 0x55 && RxData[1] == 0x56) {
//         // 温度计算
//         uint16_t tempReg = (RxData[3] << 8) | RxData[2]; // 注意这里组合的是bb在高位，aa在低位
//         data->temperature = tempReg / 100.0f; // 确保先组合后再除
//         // printf("Debug: Temperature Register Value: %u, Calculated Temperature: %d °C\n", tempReg, (int)data->temperature);
//         // printf("Parsed Data (Type 0x55 0x56): Temperature: %d°C\n", (int)data->temperature);
//     } else {
//         // printf("Unknown data type received.\n");
//     }
// }

/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, uint8_t* RxData)
{
	ptr->angle = (uint16_t)(RxData[0]<<8 | RxData[1]) ;
	ptr->offset_angle = ptr->angle;
}

#define ABS(x)	( (x>0) ? (x) : (-x) )
/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的
	if(ABS(res1)<ABS(res2))
		delta = res1;
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}

void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4){

	// hcan->pTxMsg->StdId = 0x200;
	// hcan->pTxMsg->IDE = CAN_ID_STD;
	// hcan->pTxMsg->RTR = CAN_RTR_DATA;
	// hcan->pTxMsg->DLC = 0x08;
	// hcan->pTxMsg->Data[0] = (iq1 >> 8);
	// hcan->pTxMsg->Data[1] = iq1;
	// hcan->pTxMsg->Data[2] = (iq2 >> 8);
	// hcan->pTxMsg->Data[3] = iq2;
	// hcan->pTxMsg->Data[4] = iq3 >> 8;
	// hcan->pTxMsg->Data[5] = iq3;
	// hcan->pTxMsg->Data[6] = iq4 >> 8;
	// hcan->pTxMsg->Data[7] = iq4;
	
	// HAL_CAN_Transmit(hcan, 100);

	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxData[8] = {0};
	uint32_t TxMailbox;

	// 配置消息头
	TxHeader.StdId = 0x200;                    // 替代 hcan->pTxMsg->StdId
	TxHeader.ExtId = 0x0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x08;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxData[0] = (iq1 >> 8);
	TxData[1] = iq1;
	TxData[2] = (iq2 >> 8);
	TxData[3] = iq2;
	TxData[4] = iq3 >> 8;
	TxData[5] = iq3;
	TxData[6] = iq4 >> 8;
	TxData[7] = iq4;

	// 发送消息（新 API）
	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
    	Error_Handler();
	}

}	
