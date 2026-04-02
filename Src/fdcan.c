/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */

// 29비트 CAN ID�??????????? ?��?��?��?�� 구조�???????????
// SST ID
struct CanId29Bit {
    unsigned int DestnationAddr : 8;
    unsigned int SourceAddr : 8;
    unsigned int MSG_ONJ_ID : 5;
    unsigned int Command : 5;
    unsigned int errCode : 3;
};

void parseCanId29Bit(struct CanId29Bit *canId, unsigned int rawId) {
    canId->DestnationAddr = (rawId) & 0xFF;  // ?��?�� 8비트 추출
    canId->SourceAddr = (rawId >> 8) & 0xFF;  // ?��?�� 8비트 추출
    canId->MSG_ONJ_ID = (rawId >> 16) & 0x1F;  // ?��?�� 5비트 추출
    canId->Command = (rawId >> 21) & 0x1F;  // ?��?�� 5비트 추출
    canId->errCode = (rawId >> 26) & 0x7;  // ?��?�� 3비트 추출
}

#define RING_BUFFER_SIZE 128
#define DATA_SIZE 9

#define TIME_QUERY_MASSAGE	250

typedef struct {
	uint8_t data[DATA_SIZE];
} Message;

typedef struct {
    Message buffer[RING_BUFFER_SIZE];
    int head;
    int tail;
} RingBuffer;

RingBuffer ringBuffer_for_pcs;			// CAN RING BUFFER
RingBuffer ringBuffer_for_sst;			// CAN RING BUFFER

Message transmittedMessage_pcs;			// CAN TRANS MASSAGE
Message transmittedMessage_sst;			// CAN TRANS MASSAGE

uint8_t can_rx_data_pcs[8];				// CAN RECEIVE MASSAGE
uint8_t can_rx_data_sst[8];				// CAN RECEIVE MASSAGE

FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_TxHeaderTypeDef TxHeader_SST;

FDCAN_TxHeaderTypeDef TxHeader_SST_m1_R_obj1;
FDCAN_TxHeaderTypeDef TxHeader_SST_m1_R_obj2;
FDCAN_TxHeaderTypeDef TxHeader_SST_m1_R_obj3;
FDCAN_TxHeaderTypeDef TxHeader_SST_m1_R_obj4;
FDCAN_TxHeaderTypeDef TxHeader_SST_m1_R_obj5;
FDCAN_TxHeaderTypeDef TxHeader_SST_m1_R_obj6;

FDCAN_TxHeaderTypeDef TxHeader_PCS1;
FDCAN_TxHeaderTypeDef TxHeader_PCS2;
FDCAN_TxHeaderTypeDef TxHeader_PCS3;
FDCAN_TxHeaderTypeDef TxHeader_PCS4;
FDCAN_TxHeaderTypeDef TxHeader_PCS5;
FDCAN_TxHeaderTypeDef TxHeader_SENSOR_BD;

FDCAN_RxHeaderTypeDef RxHeader_SST;
FDCAN_RxHeaderTypeDef RxHeader_PCS;

TYPE_CAN_DATA mCanData;

TYPE_CAN_SST_DATA mCanDataSST;

uint8_t msgerror = 0;

uint16_t unCountSendPCS = 0;
uint16_t unCountSendSST = 0;

// FOR PCS CMD
// BEG1K075G
uint8_t TxDataPCS_ON[] = {0x11, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x01};
uint8_t TxDataPCS_OFF[] = {0x11, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1, 0x01};

// A0 RECTIFIER
// A1 DISCHARGE
// A2 GIRID-OFF INVERTER

// RECTIFIER MODE - CHARGE ALL
uint8_t TxDataPCS_TO_DC[] =  {0x21, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x01};

// GRID ON IVERTER MODE - DISCHARGE ALL
uint8_t TxDataPCS_TO_AC[] =  {0x21, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1, 0x01};

// PCS LOAD SET
uint8_t TxDataPCS_LOAD0[]  = {0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
uint8_t TxDataPCS_LOAD2[]  = {0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x03, 0xEB, 0x04};
uint8_t TxDataPCS_LOAD5[]  = {0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x09, 0xC4, 0x04};
uint8_t TxDataPCS_LOAD10[] = {0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x13, 0x88, 0x04};
uint8_t TxDataPCS_LOAD15[] = {0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x1D, 0x4C, 0x04};
uint8_t TxDataPCS_LOAD20[] = {0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x27, 0x10, 0x04};
uint8_t TxDataPCS_LOAD25[] = {0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x30, 0xD4, 0x04};
uint8_t TxDataPCS_LOAD32[] = {0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x3E, 0x80, 0x04};

// PCS SET VDC CHARGE - SYSTEM VOLTAGE - 770V
uint8_t TxDataPCS_VDC_CHARGE_SET[] = {0x10, 0x01, 0x00, 0x00, 0x00, 0x0B, 0xBF, 0xD0, 0x01};

// PCS SET VDC - SYSTEM OVER VOLTAGE SET - 790V
uint8_t TxDataPCS_VDC_OVER_VOLTAGE_SET[] = {0x11, 0x31, 0x00, 0x00, 0x00, 0x0C, 0x0D, 0xF0, 0x01};

// PCS SET VDC 650 DISCHARGE - CUT OFF
uint8_t TxDataPCS_VDC_DISCHARGE_SET[] = {0x11, 0x32, 0x00, 0x00, 0x00, 0x09, 0xEB, 0x10, 0x01};

// PCS SET GROUP
uint8_t TxDataPCS_SET_GROUP_NUM1[] = {0x11, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01};

// PCS Read Operating Status of Module
uint8_t TxDataPCS_ReadSystemDcVoltage[] = {0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
uint8_t TxDataPCS_ReadSystemDcCurrent[] = {0x10, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};
uint8_t TxDataPCS_ReadSystemNumofDodules[] = {0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02};

// PCS Module Read
uint8_t TxDataPCS_ReadSystemTemperature[] = {0x11, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};

// PCS Module Read Voltage Curr
uint8_t TxDataPCS_ReadPowerDcVoltage[] = {0x11, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
uint8_t TxDataPCS_ReadPowerDcCurr[] = {0x11, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};

// PCS MAX MIN Data Query
uint8_t TxDataPCS_ReadMaxDcVoltage[]  = {0x11, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
uint8_t TxDataPCS_ReadMinDcVoltage[]  = {0x11, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
uint8_t TxDataPCS_ReadMaxDcCurr[]     = {0x11, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};

// PCS QUERY STATUS
uint8_t TxDataPCS_ReadModuleStatus[]  = {0x11, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};

// FOR SST CMD
uint8_t TxDataSST_ON[] = {0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x06};
uint8_t TxDataSST_OFF[] = {0x4C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA1, 0x06};

uint8_t TxDataSST_R_Obj[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07};

// FOR SENSOR BOARD QUERY CMD
uint8_t TxDataSensorBoard[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08};


void Make_Message_pcs(uint8_t *data);
void Make_Message_sst(uint8_t *data);
void initRingBuffer(RingBuffer *ringBuffer);
int addToRingBuffer(RingBuffer *ringBuffer, const Message *message);
void ReturnCheckCanPCS();
void ReturnCheckCanSST();
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 10;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 2;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 1;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

	FDCAN_FilterTypeDef sFilterConfig;

	// FOR SST 0x02600128
	TxHeader_SST.Identifier = 0x02600028;
	TxHeader_SST.IdType = FDCAN_EXTENDED_ID;
	TxHeader_SST.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_SST.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_SST.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_SST.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_SST.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_SST.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_SST.MessageMarker = 0;

	TxHeader_SST_m1_R_obj1.Identifier = 0x02410038;
	TxHeader_SST_m1_R_obj1.IdType = FDCAN_EXTENDED_ID;
	TxHeader_SST_m1_R_obj1.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_SST_m1_R_obj1.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_SST_m1_R_obj1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_SST_m1_R_obj1.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_SST_m1_R_obj1.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_SST_m1_R_obj1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_SST_m1_R_obj1.MessageMarker = 0;

	TxHeader_SST_m1_R_obj2.Identifier = 0x02420038;
	TxHeader_SST_m1_R_obj2.IdType = FDCAN_EXTENDED_ID;
	TxHeader_SST_m1_R_obj2.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_SST_m1_R_obj2.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_SST_m1_R_obj2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_SST_m1_R_obj2.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_SST_m1_R_obj2.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_SST_m1_R_obj2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_SST_m1_R_obj2.MessageMarker = 0;

	TxHeader_SST_m1_R_obj3.Identifier = 0x02430038;
	TxHeader_SST_m1_R_obj3.IdType = FDCAN_EXTENDED_ID;
	TxHeader_SST_m1_R_obj3.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_SST_m1_R_obj3.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_SST_m1_R_obj3.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_SST_m1_R_obj3.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_SST_m1_R_obj3.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_SST_m1_R_obj3.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_SST_m1_R_obj3.MessageMarker = 0;

	TxHeader_SST_m1_R_obj4.Identifier = 0x02440038;
	TxHeader_SST_m1_R_obj4.IdType = FDCAN_EXTENDED_ID;
	TxHeader_SST_m1_R_obj4.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_SST_m1_R_obj4.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_SST_m1_R_obj4.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_SST_m1_R_obj4.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_SST_m1_R_obj4.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_SST_m1_R_obj4.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_SST_m1_R_obj4.MessageMarker = 0;

	TxHeader_SST_m1_R_obj5.Identifier = 0x02450038;
	TxHeader_SST_m1_R_obj5.IdType = FDCAN_EXTENDED_ID;
	TxHeader_SST_m1_R_obj5.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_SST_m1_R_obj5.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_SST_m1_R_obj5.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_SST_m1_R_obj5.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_SST_m1_R_obj5.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_SST_m1_R_obj5.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_SST_m1_R_obj5.MessageMarker = 0;

	TxHeader_SST_m1_R_obj6.Identifier = 0x02460038;
	TxHeader_SST_m1_R_obj6.IdType = FDCAN_EXTENDED_ID;
	TxHeader_SST_m1_R_obj6.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_SST_m1_R_obj6.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_SST_m1_R_obj6.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_SST_m1_R_obj6.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_SST_m1_R_obj6.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_SST_m1_R_obj6.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_SST_m1_R_obj6.MessageMarker = 0;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
	/* Filter configuration Error */
	Error_Handler();
	}

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x11111111;
	sFilterConfig.FilterID2 = 0x22222222;
	sFilterConfig.RxBufferIndex = 0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
	/* Filter configuration Error */
	Error_Handler();
	}

	if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
	Error_Handler();
	}

  // ring buffer init
  initRingBuffer(&ringBuffer_for_sst);

  /* USER CODE END FDCAN1_Init 2 */

}
/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{
	/* USER CODE BEGIN FDCAN2_Init 0 */
	/* USER CODE END FDCAN2_Init 0 */

	/* USER CODE BEGIN FDCAN2_Init 1 */
	/* USER CODE END FDCAN2_Init 1 */
	hfdcan2.Instance = FDCAN2;
	hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan2.Init.AutoRetransmission = DISABLE;
	hfdcan2.Init.TransmitPause = DISABLE;
	hfdcan2.Init.ProtocolException = DISABLE;
	hfdcan2.Init.NominalPrescaler = 40;
	hfdcan2.Init.NominalSyncJumpWidth = 1;
	hfdcan2.Init.NominalTimeSeg1 = 2;
	hfdcan2.Init.NominalTimeSeg2 = 2;
	hfdcan2.Init.DataPrescaler = 1;
	hfdcan2.Init.DataSyncJumpWidth = 1;
	hfdcan2.Init.DataTimeSeg1 = 1;
	hfdcan2.Init.DataTimeSeg2 = 1;
	hfdcan2.Init.MessageRAMOffset = 0;
	hfdcan2.Init.StdFiltersNbr = 2;
	hfdcan2.Init.ExtFiltersNbr = 8;
	hfdcan2.Init.RxFifo0ElmtsNbr = 1;
	hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.RxFifo1ElmtsNbr = 2;
	hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.RxBuffersNbr = 1;
	hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.TxEventsNbr = 0;
	hfdcan2.Init.TxBuffersNbr = 0;
	hfdcan2.Init.TxFifoQueueElmtsNbr = 32;
	hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
	if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
	{
	Error_Handler();
	}
	/* USER CODE BEGIN FDCAN2_Init 2 */

	FDCAN_FilterTypeDef sFilterConfig;

	// FOR PCS 0x0A243FF0 BRODCAST
	// ERRCODE 3BIT : 0
	// DEVICE NUM 4BIT : 0A
	// CMD 6BIT : 24
	// 8BIT DA : 3F
	// 8BIT SA : F0
	TxHeader_PCS1.Identifier = 0x02A43FF0;
	TxHeader_PCS1.IdType = FDCAN_EXTENDED_ID;
	TxHeader_PCS1.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_PCS1.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_PCS1.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_PCS1.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_PCS1.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_PCS1.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_PCS1.MessageMarker = 0;

	// Q 1 PCS DODULE
	TxHeader_PCS2.Identifier = 0x02A33FF0;
	TxHeader_PCS2.IdType = FDCAN_EXTENDED_ID;
	TxHeader_PCS2.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_PCS2.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_PCS2.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_PCS2.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_PCS2.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_PCS2.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_PCS2.MessageMarker = 0;

	// Q 2 PCS DODULE NUMBER 1
	TxHeader_PCS3.Identifier = 0x02A301F0;
	TxHeader_PCS3.IdType = FDCAN_EXTENDED_ID;
	TxHeader_PCS3.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_PCS3.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_PCS3.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_PCS3.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_PCS3.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_PCS3.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_PCS3.MessageMarker = 0;

	// Q 2 PCS DODULE GROUP COMMAND
	TxHeader_PCS4.Identifier = 0x02A401F0;
	TxHeader_PCS4.IdType = FDCAN_EXTENDED_ID;
	TxHeader_PCS4.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_PCS4.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_PCS4.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_PCS4.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_PCS4.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_PCS4.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_PCS4.MessageMarker = 0;

	TxHeader_PCS5.Identifier = 0x02A400F0;
	TxHeader_PCS5.IdType = FDCAN_EXTENDED_ID;
	TxHeader_PCS5.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_PCS5.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_PCS5.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_PCS5.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_PCS5.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_PCS5.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_PCS5.MessageMarker = 0;

	// TO SENSOR BOARD
	TxHeader_SENSOR_BD.Identifier = 0x02A5F001;
	TxHeader_SENSOR_BD.IdType = FDCAN_EXTENDED_ID;
	TxHeader_SENSOR_BD.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader_SENSOR_BD.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader_SENSOR_BD.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader_SENSOR_BD.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader_SENSOR_BD.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader_SENSOR_BD.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader_SENSOR_BD.MessageMarker = 0;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x11111111;
	sFilterConfig.FilterID2 = 0x22222222;
	sFilterConfig.RxBufferIndex = 0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
	/* Filter configuration Error */
	Error_Handler();
	}

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x11111111;
	sFilterConfig.FilterID2 = 0x22222222;
	sFilterConfig.RxBufferIndex = 0;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
	/* Filter configuration Error */
	Error_Handler();
	}

	/* Configure global filter to reject all non-matching frames */
	//HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);


	if(HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
	Error_Handler();
	}

	// ring buffer init
	initRingBuffer(&ringBuffer_for_pcs);

  /* USER CODE END FDCAN2_Init 2 */

}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN2 GPIO Configuration
    PB12     ------> FDCAN2_RX
    PB13     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


// COIL DATA TO MAKE MASSAGE CAN
void enable_device(uint16_t relay, unsigned char state)
{
    switch(relay)
    {
    	// FOT PCS
        case 16:
            if(state == 1) Make_Message_pcs(TxDataPCS_ON);
            break;
        case 15:
			if(state == 1) Make_Message_pcs(TxDataPCS_OFF);
			break;
        case 14:
        	;
            break;
        case 13:
            if(state == 1){
            	// ChargeModeSet
            	Make_Message_pcs(TxDataPCS_TO_DC);
            	Make_Message_pcs(TxDataPCS_VDC_CHARGE_SET);
            	Make_Message_pcs(TxDataPCS_VDC_OVER_VOLTAGE_SET);
            	Make_Message_pcs(TxDataPCS_LOAD0);
			}
            break;
        case 12:
            if(state == 1){
            	// DischargeModeSet
            	Make_Message_pcs(TxDataPCS_TO_AC);
            	Make_Message_pcs(TxDataPCS_VDC_DISCHARGE_SET);
            	Make_Message_pcs(TxDataPCS_VDC_OVER_VOLTAGE_SET);
            	Make_Message_pcs(TxDataPCS_LOAD0);
            }
            break;
        case 11:
            if(state == 1) Make_Message_pcs(TxDataPCS_LOAD0);
            break;
        case 10:
            if(state == 1) Make_Message_pcs(TxDataPCS_LOAD2);
            break;
        case 9:
            if(state == 1) Make_Message_pcs(TxDataPCS_LOAD5);
            break;
        case 8:
            if(state == 1) Make_Message_pcs(TxDataPCS_LOAD10);
            break;
        case 7:
            if(state == 1) Make_Message_pcs(TxDataPCS_LOAD15);
            break;
        case 6:
            if(state == 1) Make_Message_pcs(TxDataPCS_LOAD20);
            break;
        case 5:
            if(state == 1) Make_Message_pcs(TxDataPCS_LOAD25);
            break;
        case 4:
            if(state == 1) Make_Message_pcs(TxDataPCS_LOAD32);
            break;

        // FOR SST
        case 3:
            if(state == 1) Make_Message_sst(TxDataSST_ON);
            break;
        case 2:
            if(state == 1) Make_Message_sst(TxDataSST_OFF);
            break;
        case 1:
            if(state == 1) Make_Message_sst(TxDataSST_R_Obj);
            break;

    }
}

void check_device(uint16_t fan_map)
{
	uint16_t i;
	uint16_t bit;
	unsigned char state;

    for(i=0; i < 16; i++) {
        bit = (fan_map&(0x01<<i));
        state = ((bit != 0)? 1 : 0);
        enable_device( (16-i), state);
    }
}

// RING BUFFER RESET
void initRingBuffer(RingBuffer *ringBuffer) {
    ringBuffer->head = 0;
    ringBuffer->tail = 0;
}

int addToRingBuffer(RingBuffer *ringBuffer, const Message *message) {
    int nextTail = (ringBuffer->tail + 1) % RING_BUFFER_SIZE;
    if (nextTail == ringBuffer->head) {
        return 0;
    }

    ringBuffer->buffer[ringBuffer->tail] = *message;
    ringBuffer->tail = nextTail;

    return 1;
}

int removeFromRingBuffer(RingBuffer *ringBuffer, Message *message) {
    if (ringBuffer->head == ringBuffer->tail) {
        return 0;
    }

    *message = ringBuffer->buffer[ringBuffer->head];
    ringBuffer->head = (ringBuffer->head + 1) % RING_BUFFER_SIZE;

    return 1;
}

void Make_Message_pcs(uint8_t *data)
{
    Message testMessage;
    for (int i = 0; i < DATA_SIZE; ++i) {
        testMessage.data[i] = data[i];
    }

    addToRingBuffer(&ringBuffer_for_pcs, &testMessage);
}

void Make_Message_sst(uint8_t *data)
{
    Message testMessage;
    for (int i = 0; i < DATA_SIZE; ++i) {
        testMessage.data[i] = data[i];
    }

    addToRingBuffer(&ringBuffer_for_sst, &testMessage);
}

uint8_t BufferCmp8b(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while(BufferLength--)
  {
    if(*pBuffer1 != *pBuffer2)
    {
      return 1;
    }

    pBuffer1++;
    pBuffer2++;
  }
  return 0;
}


TYPE_CAN_SST_DATA* CAN_GetSSTData(void)
{
    return (&mCanDataSST);
}

void CAN_SetSSTData(TYPE_CAN_SST_DATA* pData)
{
	mCanDataSST = *pData;
}


TYPE_CAN_DATA* CAN_GetData(void)
{
    return (&mCanData);
}

void CAN_SetData(TYPE_CAN_DATA* pData)
{
	mCanData = *pData;
}

// CAN RX MASSAGE PROCESS SST
void CanRxData2()
{
	ReturnCheckCanSST();
}

// CAN TX MASSAGE PROCESS SST
void CanTxData2()
{
	// Wait TxFifo Empty FOR SST
	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) > 0)
	{
		if (removeFromRingBuffer(&ringBuffer_for_sst, &transmittedMessage_sst))
		{
			// FOR SST COMMAND
			if(transmittedMessage_sst.data[8] == 0x06)
			{
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_SST, transmittedMessage_sst.data);
				osDelay(30);
			}

			// FOR SST QUERY
			if(transmittedMessage_sst.data[8] == 0x07)
			{
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_SST_m1_R_obj1, transmittedMessage_sst.data);
				osDelay(30);
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_SST_m1_R_obj2, transmittedMessage_sst.data);
				osDelay(30);
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_SST_m1_R_obj3, transmittedMessage_sst.data);
				osDelay(30);
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_SST_m1_R_obj4, transmittedMessage_sst.data);
				osDelay(30);
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_SST_m1_R_obj5, transmittedMessage_sst.data);
				osDelay(30);
				HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader_SST_m1_R_obj6, transmittedMessage_sst.data);
				osDelay(30);
			}

		}
	}

	// EVERY 250MS QUERY SST
	unCountSendSST++;
	if(unCountSendSST > TIME_QUERY_MASSAGE)
	{
		Make_Message_sst(TxDataSST_R_Obj);

		unCountSendSST = 0;
	}

}

void ReturnCheckCanSST()
{
	// WAITING SST CAN DATA
	if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
	{
		HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader_SST, can_rx_data_sst);

		// PCS System Dc Side Voltage
		if(RxHeader_SST.Identifier == 0x02413800)
		{
			mCanDataSST.MSG_OBJ1.status_etc_byte7 = can_rx_data_sst[7];
			mCanDataSST.MSG_OBJ1.fault_flag_MV_byte6 = can_rx_data_sst[6];
			mCanDataSST.MSG_OBJ1.fault_flag_byte5 = can_rx_data_sst[5];
			mCanDataSST.MSG_OBJ1.MV_status_byte4 = can_rx_data_sst[4];
			mCanDataSST.MSG_OBJ1.LV_status_byte3 = can_rx_data_sst[3];
			mCanDataSST.MSG_OBJ1.cc_MV_byte2 = can_rx_data_sst[2];
			mCanDataSST.MSG_OBJ1.cc_byte1 = can_rx_data_sst[1];
			mCanDataSST.MSG_OBJ1.precharge_NUM_byte0 = can_rx_data_sst[0];
		}
		else if(RxHeader_SST.Identifier == 0x02423800)
		{
			;
		}
		else if(RxHeader_SST.Identifier == 0x02433800)
		{
			mCanDataSST.MSG_OBJ3.TEMP_AFE_a_B2_byte7 = can_rx_data_sst[7];
			mCanDataSST.MSG_OBJ3.TEMP_AFE_a_B1_byte6 = can_rx_data_sst[6];
			mCanDataSST.MSG_OBJ3.TEMP_AFE_a_T2_byte5 = can_rx_data_sst[5];
			mCanDataSST.MSG_OBJ3.TEMP_AFE_a_T1_byte4 = can_rx_data_sst[4];
			mCanDataSST.MSG_OBJ3.TEMP_DAB_B2_MV_byte3 = can_rx_data_sst[3];
			mCanDataSST.MSG_OBJ3.TEMP_DAB_B1_MV_byte2 = can_rx_data_sst[2];
			mCanDataSST.MSG_OBJ3.TEMP_DAB_T2_MV_byte1 = can_rx_data_sst[1];
			mCanDataSST.MSG_OBJ3.TEMP_DAB_TI_MV_byte0 = can_rx_data_sst[0];
		}
		else if(RxHeader_SST.Identifier == 0x02443800)
		{
			;
		}
		else if(RxHeader_SST.Identifier == 0x02453800)
		{
			;
		}
		else if(RxHeader_SST.Identifier == 0x02463800)
		{
			;
		}
	}

}


/*
  PCS CODE
  Send   0x0073200F    00 55 00 20 00 00 00 00
  Return 0x00220F20    00 55 00 20 00 00 00 00
*/
void CanTxData()
{
	// Wait TxFifo Empty FOR PCS
	if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0)
	{
		if (removeFromRingBuffer(&ringBuffer_for_pcs, &transmittedMessage_pcs))
		{
    		// FOR PCS 0X01
    		if(transmittedMessage_pcs.data[8] == 0x01)
    		{
    			// BRODCAST 0X3F
    			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_PCS1, transmittedMessage_pcs.data);
    		}

    		if(transmittedMessage_pcs.data[8] == 0x02)
    		{
    			// BRODCAST 0x23
    			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_PCS2, transmittedMessage_pcs.data);
    		}

    		if(transmittedMessage_pcs.data[8] == 0x03)
    		{
    			// BRODCAST 0x23
    			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_PCS3, transmittedMessage_pcs.data);
    		}

    		// PCS MODULE CMD
    		if(transmittedMessage_pcs.data[8] == 0x04)
    		{
    			// MODULE SET
    			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_PCS4, transmittedMessage_pcs.data);
    			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_PCS5, transmittedMessage_pcs.data);
    		}

    		// SENSOR BOARD QUERY CMD
    		if(transmittedMessage_pcs.data[8] == 0x08)
    		{
    			// MODULE SET
    			HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader_SENSOR_BD, transmittedMessage_pcs.data);
    		}

			// CAN DELAY 50MS
			osDelay(50);
    	}

    }

	// EVERY 250MS QUERY PCS
	unCountSendPCS++;
	if(unCountSendPCS > TIME_QUERY_MASSAGE)
	{
		// FOR PCS
		Make_Message_pcs(TxDataPCS_ReadSystemDcVoltage);
		Make_Message_pcs(TxDataPCS_ReadSystemDcCurrent);
	    Make_Message_pcs(TxDataPCS_ReadSystemTemperature);
		Make_Message_pcs(TxDataPCS_ReadPowerDcCurr);
		Make_Message_pcs(TxDataPCS_ReadModuleStatus);
		Make_Message_pcs(TxDataPCS_ReadMaxDcCurr);

		// FOR SENSOR BOARD
		Make_Message_pcs(TxDataSensorBoard);

		unCountSendPCS = 0;
	}

}

// CAN RX DATA PROCESS
void CanRxData()
{
	// CHEACK RECEIVE MASSAGE CAN FROM PCS
	ReturnCheckCanPCS();
}

void ReturnCheckCanPCS()
{
	// WAITING PCS CAN DATA
	if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0)
	{
		HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader_PCS, can_rx_data_pcs);

		// RECEIVE SENSOR BOARD DATA
		if(RxHeader_PCS.Identifier == 0x02A501F0)
		{
			mCanData.Bitval.SensorBoardVoltDC = (can_rx_data_pcs[0]<<8) + can_rx_data_pcs[1];
			mCanData.Bitval.SensorBoardCurrDC = (can_rx_data_pcs[2]<<8) + can_rx_data_pcs[3];
			mCanData.Bitval.SensorBoardVoltAC = (can_rx_data_pcs[4]<<8) + can_rx_data_pcs[5];
			mCanData.Bitval.SensorBoardCurrAC = (can_rx_data_pcs[6]<<8) + can_rx_data_pcs[7];
		}
		else
		{
			// RECEIVE PCS DATA
			// PCS System Dc Side Voltage
			if((can_rx_data_pcs[0] == 0x10) && (can_rx_data_pcs[1] == 0x01))
			{
				mCanData.Bitval.SystemDcSideTotalVoltage = (can_rx_data_pcs[4]<<24) + (can_rx_data_pcs[5]<<16) + (can_rx_data_pcs[6]<<8) + can_rx_data_pcs[7];
			}

			// PCS System Dc Side Curr
			else if((can_rx_data_pcs[0] == 0x10) && (can_rx_data_pcs[1] == 0x02))
			{
				if(RxHeader_PCS.Identifier == 0x2a3f03f){
					mCanData.Bitval.SystemDcSideTotalCurrent = (can_rx_data_pcs[4]<<24) + (can_rx_data_pcs[5]<<16) + (can_rx_data_pcs[6]<<8) + can_rx_data_pcs[7];
				}
			}

			// PCS Module Read Voltage
			else if((can_rx_data_pcs[0] == 0x11) && (can_rx_data_pcs[1] == 0x01))
			{
				mCanData.Bitval.SystemDcSideModuleVoltage = (can_rx_data_pcs[4]<<24) + (can_rx_data_pcs[5]<<16) + (can_rx_data_pcs[6]<<8) + can_rx_data_pcs[7];
			}

			// PCS Module Read Curr
			else if((can_rx_data_pcs[0] == 0x11) && (can_rx_data_pcs[1] == 0x02))
			{
				if((RxHeader_PCS.Identifier == 0x2a4f000) || (RxHeader_PCS.Identifier == 0x2a4f001))
				{
					mCanData.Bitval.SystemDcSideSetCurrent = (can_rx_data_pcs[4]<<24) + (can_rx_data_pcs[5]<<16) + (can_rx_data_pcs[6]<<8) + can_rx_data_pcs[7];
				}
				else
				{
					mCanData.Bitval.SystemDcSideModuleCurrent = (can_rx_data_pcs[4]<<24) + (can_rx_data_pcs[5]<<16) + (can_rx_data_pcs[6]<<8) + can_rx_data_pcs[7];
				}
			}

			// PCS Module Read Status
			else if((can_rx_data_pcs[0] == 0x11) && (can_rx_data_pcs[1] == 0x10))
			{
				mCanData.Bitval.SystemeModuleStatus1 = can_rx_data_pcs[7];
				mCanData.Bitval.SystemeModuleStatus2 = can_rx_data_pcs[6];
				mCanData.Bitval.SystemeModuleStatus3 = can_rx_data_pcs[5];
			}

			// PCS Module Temputure
			else if((can_rx_data_pcs[0] == 0x11) && (can_rx_data_pcs[1] == 0x06))
			{
				mCanData.Bitval.ModuleAmbientTemperature = (can_rx_data_pcs[4]<<24) + (can_rx_data_pcs[5]<<16) + (can_rx_data_pcs[6]<<8) + can_rx_data_pcs[7];
			}
		}
	}

}

/* USER CODE END 1 */
