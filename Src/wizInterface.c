/*
 * wizInterface.c
 *
 *  Created on: 2020. 5. 31.
 *      Author: eziya76@gmail.com
 */
#include "main.h"
#include "wizchip_conf.h"
#include "wizInterface.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
/* ----------------------- System includes ----------------------------------*/
#include <stdio.h>
#include <string.h>
#include "socket.h"
#include "dhcp.h"
#include "dns.h"
#include "sntp.h"
#include "w5500.h"
#include "mb.h"
#include "mbproto.h"
#include "mbutils.h"

#include "Sequence.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "port.h"
#include "gpio.h"

#include "global.h"

#include "fdcan.h"
#include "CommSpiSensor.h"

/* ----------------------- MBAP Header --------------------------------------*/
#define MB_TCP_UID          6
#define MB_TCP_LEN          4
#define MB_TCP_FUNC         7
#define HTTP_SOCKET         0

#define MB_TCP_DEFAULT_PORT  502
#define MB_TCP_BUF_SIZE     (256 + 7)

#define SET_BIT1(var, bit_num) ((var) |= (1 << (bit_num)))
#define CLR_BIT1(var, bit_num) ((var) &= ~(1 << (bit_num)))

/* ----------------------- Prototypes ---------------------------------------*/

#define true 	1
#define false 	0

#define SEPARATOR            "=============================================\r\n"
#define WELCOME_MSG  		 "Welcome to STM32Nucleo Ethernet configuration\r\n"
#define NETWORK_MSG  		 "Network configuration:\r\n"
#define IP_MSG 		 		 "  IP ADDRESS:  %d.%d.%d.%d\r\n"
#define NETMASK_MSG	         "  NETMASK:     %d.%d.%d.%d\r\n"
#define GW_MSG 		 		 "  GATEWAY:     %d.%d.%d.%d\r\n"
#define MAC_MSG		 		 "  MAC ADDRESS: %x:%x:%x:%x:%x:%x\r\n"
#define GREETING_MSG 		 "Well done guys! Welcome to the IoT world. Bye!\r\n"
#define CONN_ESTABLISHED_MSG "Connection established with remote IP: %d.%d.%d.%d:%d\r\n"
#define SENT_MESSAGE_MSG	 "Sent a message. Let's close the socket!\r\n"
#define WRONG_RETVAL_MSG	 "Something went wrong; return value: %d\r\n"
#define WRONG_STATUS_MSG	 "Something went wrong; STATUS: %d\r\n"
#define LISTEN_ERR_MSG		 "LISTEN Error!\r\n"


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t dhcp_buffer[1024];
uint8_t dns_buffer[200];
uint8_t ntp_buf[48];
volatile BOOL ip_assigned = false;

uint8_t ntp_domain_serv[] = "ntp5.stratum1.ru";
uint8_t ntp_ip_serv[4] = {0};
datetime ntp_time;

uint16_t unCoilOldData = 0;
volatile BOOL time_assigned = false;

uint16_t usRegInputBuf[REG_INPUT_NREGS] = {0x1000,0x1001,0x1002,0x1003,0x1004,0x1005,0x1006,0x1007};
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0x2000,0x2001,0x2002,0x2003,0x2004,0x2005,0x2006,0x2007};
uint8_t ucRegCoilsBuf[REG_COILS_SIZE/8] = {0xaa, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE/8] = {0x98, 0x00};

#define ETH_MAX_BUF_SIZE		2048

#define PRINT_STR(msg) do  {										\
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);		\
} while(0)

#define PRINT_HEADER() do  {													\
  HAL_UART_Transmit(&huart1, (uint8_t*)SEPARATOR, strlen(SEPARATOR), 100);		\
  HAL_UART_Transmit(&huart1, (uint8_t*)WELCOME_MSG, strlen(WELCOME_MSG), 100);	\
  HAL_UART_Transmit(&huart1, (uint8_t*)SEPARATOR, strlen(SEPARATOR), 100);		\
} while(0)

#define PRINT_NETINFO(netInfo){ 																						\
  HAL_UART_Transmit(&huart1, (uint8_t*)NETWORK_MSG, strlen(NETWORK_MSG), 100);											\
  sprintf(msg, MAC_MSG, netInfo.mac[0], netInfo.mac[1], netInfo.mac[2], netInfo.mac[3], netInfo.mac[4], netInfo.mac[5]);\
  printf(msg);																											\
  sprintf(msg, IP_MSG, netInfo.ip[0], netInfo.ip[1], netInfo.ip[2], netInfo.ip[3]);										\
  printf(msg);																											\
  sprintf(msg, NETMASK_MSG, netInfo.sn[0], netInfo.sn[1], netInfo.sn[2], netInfo.sn[3]);								\
  printf(msg);																											\
  sprintf(msg, GW_MSG, netInfo.gw[0], netInfo.gw[1], netInfo.gw[2], netInfo.gw[3]);										\
  printf(msg);																											\
}


extern SPI_HandleTypeDef hspi1;
uint8_t ethBuf0[ETH_MAX_BUF_SIZE];
char msg[60];

extern SPI_HandleTypeDef hspi1;

wiz_NetTimeout timeout = {
		.retry_cnt = 3, 		//RCR = 3
		.time_100us = 5000};    //500ms


typedef enum
{
	PROC_WIZNET_INIT,
	PROC_WIZNET_INIT2,
	PROC_WIZNET_RUN,
}REGE_GATE_PROC_NAME;

static unsigned char g_ubTransportProcState = PROC_WIZNET_INIT;

void WizNetInit(void);
void WizNetInit2(void);
void WizNetRun(void);
void ShiftTransportEntPapBuf(void);


void (*TransportProcFunc[3])(void)={
	WizNetInit,
	WizNetInit2,
    WizNetRun,
};


enum GPIOState {
    GPIO_OFF,
    GPIO_ON
};

// INPUT GPIO 구조�??? ?��?��
struct GPIOConfig {
    GPIO_TypeDef *port;
    uint16_t pin;
    enum GPIOState state;
};

// INPUT GPIO 구조�??? 배열 ?��?�� �??? 초기?��
struct GPIOConfig gpioConfigArray[] = {
    {GPIOA, GPIO_PIN_8, GPIO_OFF},
    {GPIOC, GPIO_PIN_9, GPIO_OFF},
    {GPIOC, GPIO_PIN_8, GPIO_OFF},
    {GPIOC, GPIO_PIN_7, GPIO_OFF},
    {GPIOC, GPIO_PIN_6, GPIO_OFF},
    {GPIOD, GPIO_PIN_15, GPIO_OFF},
    {GPIOD, GPIO_PIN_14, GPIO_OFF},
    {GPIOD, GPIO_PIN_13, GPIO_OFF},
    // Add more GPIO configurations as needed
};

#define INPUT_NUM_PIN			sizeof(gpioConfigArray) / sizeof(gpioConfigArray[0])

enum GPIOState gpioStates[INPUT_NUM_PIN];

uint8_t readMultipleGPIO(struct GPIOConfig *gpioConfigArray, size_t numPins, enum GPIOState *resultArray) {

	uint8_t InputData;

    for (size_t i = 0; i < numPins; ++i) {
        resultArray[i] = HAL_GPIO_ReadPin(gpioConfigArray[i].port, gpioConfigArray[i].pin) == GPIO_PIN_SET ? GPIO_ON : GPIO_OFF;

        if(gpioStates[i] == GPIO_ON)CLR_BIT1(InputData, i);
        else SET_BIT1(InputData, i);
    }

    return InputData;
}


void cs_sel() {
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); //CS LOW
}

void cs_desel() {
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET); //CS HIGH
}

uint8_t spi_rb(void) {
	uint8_t rbuf;
	HAL_SPI_Receive(&hspi1, &rbuf, 1, 0xFFFFFFFF);
	return rbuf;
}

void spi_wb(uint8_t b) {
	HAL_SPI_Transmit(&hspi1, &b, 1, 0xFFFFFFFF);
}

void WIZ_SPI_TxBuffer(uint8_t *buffer, uint16_t len)
{
	HAL_SPI_Transmit(&hspi1, buffer, len, 0xFFFFFFFF);
}

void WIZ_SPI_RxBuffer(uint8_t *buffer, uint16_t len)
{
	HAL_SPI_Receive(&hspi1, buffer, len, 0xFFFFFFFF);
}

void Callback_IPAssigned(void) {
    printf("Callback: IP assigned! Leased time: %d sec\r\n", getDHCPLeasetime());
    ip_assigned = true;
}
void Callback_IPConflict(void) {
    printf("Callback: IP conflict!\r\n");
}
void Callback_TimeAssigned(void) {
    printf("Callback: Time assigned! \r\n");
    time_assigned = true;
    printf("NTP Time: %d.%d.%d %d:%d:%d \r\n",ntp_time.dd, ntp_time.mm, ntp_time.yy, ntp_time.hh, ntp_time.mm,ntp_time.ss);
}

void init_w5500()
{

  printf("\r\ninit() called!\r\n");

  //power reset arduino ethernet shield
  HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, GPIO_PIN_RESET);
  osDelay(700);
  HAL_GPIO_WritePin(SPI1_RST_GPIO_Port, SPI1_RST_Pin, GPIO_PIN_SET);
  osDelay(700);

  printf("Registering W5500 Call Backs...\r\n");
  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
  reg_wizchip_spiburst_cbfunc(WIZ_SPI_RxBuffer, WIZ_SPI_TxBuffer);

  printf("Calling wizchip_init()...\r\n");
  uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
  wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

  wiz_NetInfo netInfo = { .mac 	= {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},	// Mac address
                          .ip 	= {192, 168, 0, 192},					// IP address
                          .sn 	= {255, 255, 255, 0},					// Subnet mask
                          .gw 	= {192, 168, 0, 1}};					// Gateway address
  wizchip_setnetinfo(&netInfo);
  wizchip_getnetinfo(&netInfo);
  PRINT_NETINFO(netInfo);

}

void TcpModbusCtrl(void)
{
    TransportProcFunc[g_ubTransportProcState]();
}

void WizNetInit()
{
	init_w5500();
	g_ubTransportProcState = PROC_WIZNET_INIT2;
	osDelay(1000);
}

void WizNetInit2()
{
	eMBTCPInit(MBTCP_PORT);
	osDelay(1000);
	eMBEnable();

	printf("\r\n PROGRAM Ready!\r\n");
	g_ubTransportProcState = PROC_WIZNET_RUN;
}


void WizNetRun()
{
	static int DataIndex = 0;

	TYPE_CAN_DATA mTCanData;
	TYPE_CAN_SST_DATA mCanDataSST;

	uint16_t unTempData;

    modbus_tcps(HTTP_SOCKET, MBTCP_PORT);

    // CAN SST 데이터를 받아온다.
    mCanDataSST = *CAN_GetSSTData();

    // CAN 데이터를 받아온다.
    mTCanData = *CAN_GetData();

    // MODBUS HOLDING REG에 전달한다.
    usRegHoldingBuf[0] = ((uint32_t)mTCanData.Bitval.SensorBoardVoltAC);
	usRegHoldingBuf[1] = ((uint32_t)mTCanData.Bitval.SensorBoardCurrAC);
    usRegHoldingBuf[2] = ((uint32_t)mTCanData.Bitval.SensorBoardVoltDC);
	usRegHoldingBuf[3] = ((uint32_t)mTCanData.Bitval.SensorBoardCurrDC);

	// MODBUS HOLDING REG에 PCS CAN 데이터를 전달한다.
	usRegHoldingBuf[4] = ((uint32_t)mTCanData.Bitval.SystemDcSideTotalVoltage/100);
	usRegHoldingBuf[5] = ((uint32_t)mTCanData.Bitval.SystemDcSideTotalCurrent/100);
	usRegHoldingBuf[6] = ((uint32_t)mTCanData.Bitval.SystemDcSideModuleCurrent/100);
	usRegHoldingBuf[7] = ((uint32_t)mTCanData.Bitval.SystemDcSideSetCurrent/100);
	usRegHoldingBuf[8] = ((uint32_t)mTCanData.Bitval.SystemeModuleStatus2);
	usRegHoldingBuf[9] = ((uint32_t)mTCanData.Bitval.SystemeModuleStatus3);
	usRegHoldingBuf[10] = ((uint32_t)mTCanData.Bitval.ModuleAmbientTemperature/100);

	usRegHoldingBuf[36] = ((uint32_t)mCanDataSST.MSG_OBJ3.TEMP_DAB_TI_MV_byte0);
	usRegHoldingBuf[37] = ((uint32_t)mCanDataSST.MSG_OBJ3.TEMP_DAB_T2_MV_byte1);
	usRegHoldingBuf[38] = ((uint32_t)mCanDataSST.MSG_OBJ3.TEMP_DAB_B1_MV_byte2);
	usRegHoldingBuf[39] = ((uint32_t)mCanDataSST.MSG_OBJ3.TEMP_DAB_B2_MV_byte3);
	usRegHoldingBuf[40] = ((uint32_t)mCanDataSST.MSG_OBJ3.TEMP_AFE_a_T1_byte4);
	usRegHoldingBuf[41] = ((uint32_t)mCanDataSST.MSG_OBJ3.TEMP_AFE_a_T2_byte5);
	usRegHoldingBuf[42] = ((uint32_t)mCanDataSST.MSG_OBJ3.TEMP_AFE_a_B1_byte6);
	usRegHoldingBuf[43] = ((uint32_t)mCanDataSST.MSG_OBJ3.TEMP_AFE_a_B2_byte7);

	if(DataIndex > 200){DataIndex = 0;}

    // MODBUS COIL 2개번지 데이터를 릴레이 데이터로 활용한다.
	unTempData = (ucRegCoilsBuf[1]&0xff) + (ucRegCoilsBuf[2]<<8);

	// COIL 데이터가 변할때만 반응한다.
	check_Seq(unTempData);

    // MODBUS SEQ 데이터 송신 완료 후 0으로 초기화 한다.
	// 111111 COIL 1번째 6BIT는 처리 후 0 초기화 한다.
    ucRegCoilsBuf[1] &= ~0x3F;

    ucRegCoilsBuf[0] =  readMultipleGPIO(gpioConfigArray, INPUT_NUM_PIN, gpioStates);

    // MODBUS COIL 3개번지 데이터를 디바이스 데이터로 활용한다.
	unTempData = (ucRegCoilsBuf[3]&0xff) + (ucRegCoilsBuf[4]<<8);
    check_device(unTempData);

    // Device 제어 CAN 데이터 송신 완료 후 0으로 초기화 한다.
    ucRegCoilsBuf[3] = 0;
    ucRegCoilsBuf[4] = 0;
    unTempData = 0x00;


}

void SetModBusCoil(uint8_t addr, uint8_t state)
{
	// Turn ON
	if(state == 1)
	{
		switch(addr)
		{
		case 1: _SET_BIT(ucRegCoilsBuf[1], 0); 	  break;
		case 2: _SET_BIT(ucRegCoilsBuf[1], 1); 	  break;
		case 3: _SET_BIT(ucRegCoilsBuf[1], 2); 	  break;
		case 4: _SET_BIT(ucRegCoilsBuf[1], 3); 	  break;
		case 5: _SET_BIT(ucRegCoilsBuf[1], 4); 	  break;
		case 6: _SET_BIT(ucRegCoilsBuf[1], 5); 	  break;
		case 7: _SET_BIT(ucRegCoilsBuf[1], 6); 	  break;
		case 8: _SET_BIT(ucRegCoilsBuf[1], 7); 	  break;
		case 9: _SET_BIT(ucRegCoilsBuf[2], 0); 	  break;
		case 10: _SET_BIT(ucRegCoilsBuf[2], 1); 	  break;
		case 11: _SET_BIT(ucRegCoilsBuf[2], 2); 	  break;
		case 12: _SET_BIT(ucRegCoilsBuf[2], 3); 	  break;
		case 13: _SET_BIT(ucRegCoilsBuf[2], 4); 	  break;
		case 14: _SET_BIT(ucRegCoilsBuf[2], 5); 	  break;
		case 15: _SET_BIT(ucRegCoilsBuf[2], 6); 	  break;
		case 16: _SET_BIT(ucRegCoilsBuf[2], 7); 	  break;
		case 17: _SET_BIT(ucRegCoilsBuf[3], 0); 	  break;
		case 18: _SET_BIT(ucRegCoilsBuf[3], 1); 	  break;
		case 19: _SET_BIT(ucRegCoilsBuf[3], 2); 	  break;
		case 20: _SET_BIT(ucRegCoilsBuf[3], 3); 	  break;
		case 21: _SET_BIT(ucRegCoilsBuf[3], 4); 	  break;
		case 22: _SET_BIT(ucRegCoilsBuf[3], 5); 	  break;
		case 23: _SET_BIT(ucRegCoilsBuf[3], 6); 	  break;
		case 24: _SET_BIT(ucRegCoilsBuf[3], 7); 	  break;
		case 25: _SET_BIT(ucRegCoilsBuf[4], 0); 	  break;
		case 26: _SET_BIT(ucRegCoilsBuf[4], 1); 	  break;
		case 27: _SET_BIT(ucRegCoilsBuf[4], 2); 	  break;
		case 28: _SET_BIT(ucRegCoilsBuf[4], 3); 	  break;
		case 29: _SET_BIT(ucRegCoilsBuf[4], 4); 	  break;
		case 30: _SET_BIT(ucRegCoilsBuf[4], 5); 	  break;
		case 31: _SET_BIT(ucRegCoilsBuf[4], 6); 	  break;
		case 32: _SET_BIT(ucRegCoilsBuf[4], 7); 	  break;
		}
	}
	// Turn OFF
	else
	{
		switch(addr)
		{
		case 1: _CLR_BIT(ucRegCoilsBuf[1], 0); 	  break;
		case 2: _CLR_BIT(ucRegCoilsBuf[1], 1); 	  break;
		case 3: _CLR_BIT(ucRegCoilsBuf[1], 2); 	  break;
		case 4: _CLR_BIT(ucRegCoilsBuf[1], 3); 	  break;
		case 5: _CLR_BIT(ucRegCoilsBuf[1], 4); 	  break;
		case 6: _CLR_BIT(ucRegCoilsBuf[1], 5); 	  break;
		case 7: _CLR_BIT(ucRegCoilsBuf[1], 6); 	  break;
		case 8: _CLR_BIT(ucRegCoilsBuf[1], 7); 	  break;
		case 9: _CLR_BIT(ucRegCoilsBuf[2], 0); 	  break;
		case 10: _CLR_BIT(ucRegCoilsBuf[2], 1); 	  break;
		case 11: _CLR_BIT(ucRegCoilsBuf[2], 2); 	  break;
		case 12: _CLR_BIT(ucRegCoilsBuf[2], 3); 	  break;
		case 13: _CLR_BIT(ucRegCoilsBuf[2], 4); 	  break;
		case 14: _CLR_BIT(ucRegCoilsBuf[2], 5); 	  break;
		case 15: _CLR_BIT(ucRegCoilsBuf[2], 6); 	  break;
		case 16: _CLR_BIT(ucRegCoilsBuf[2], 7); 	  break;
		case 17: _CLR_BIT(ucRegCoilsBuf[3], 0); 	  break;
		case 18: _CLR_BIT(ucRegCoilsBuf[3], 1); 	  break;
		case 19: _CLR_BIT(ucRegCoilsBuf[3], 2); 	  break;
		case 20: _CLR_BIT(ucRegCoilsBuf[3], 3); 	  break;
		case 21: _CLR_BIT(ucRegCoilsBuf[3], 4); 	  break;
		case 22: _CLR_BIT(ucRegCoilsBuf[3], 5); 	  break;
		case 23: _CLR_BIT(ucRegCoilsBuf[3], 6); 	  break;
		case 24: _CLR_BIT(ucRegCoilsBuf[3], 7); 	  break;
		case 25: _CLR_BIT(ucRegCoilsBuf[4], 0); 	  break;
		case 26: _CLR_BIT(ucRegCoilsBuf[4], 1); 	  break;
		case 27: _CLR_BIT(ucRegCoilsBuf[4], 2); 	  break;
		case 28: _CLR_BIT(ucRegCoilsBuf[4], 3); 	  break;
		case 29: _CLR_BIT(ucRegCoilsBuf[4], 4); 	  break;
		case 30: _CLR_BIT(ucRegCoilsBuf[4], 5); 	  break;
		case 31: _CLR_BIT(ucRegCoilsBuf[4], 6); 	  break;
		case 32: _CLR_BIT(ucRegCoilsBuf[4], 7); 	  break;
		}
	}
}

void setMovementCompleted(uint8_t status)
{
  //if (status == 1) ucRegDiscBuf[0] |= 1UL << 0;
  //if (status == 0) ucRegDiscBuf[0] &= ~(1UL << 0);
}

void setZeroReached(uint8_t zero)
{
  //if (zero == 1) ucRegDiscBuf[0] |= 1UL << 1;
  //if (zero == 0) ucRegDiscBuf[0] &= ~(1UL << 1);
}

int32_t getTargetPos_Deg()
{
 // uint32_t word  =  usRegHoldingBuf[0];
 //          word |= (usRegHoldingBuf[1] << 16);
 // return (int32_t)(word);
	return 0;
}

float getMovementDuration_s()
{
 // return ((float)(usRegHoldingBuf[2])/1000.0f);
	return 0;
}

BOOL getNewCommand()
{
    return  ((ucRegCoilsBuf[0] & 0x01) != 0x00);
}

void clearCommand()
{
	ucRegCoilsBuf[0] &= ~(0x01);
}

eMBErrorCode
eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                eMBRegisterMode eMode)
{
  eMBErrorCode eStatus = MB_ENOERR;
  int iRegIndex;

  if ((usAddress >= REG_HOLDING_START) &&
      (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
  {
    iRegIndex = (int)(usAddress - REG_HOLDING_START);
    switch (eMode)
    {
    /* Pass current register values to the protocol stack. */
    case MB_REG_READ:
      while (usNRegs > 0)
      {
        *pucRegBuffer++ =
            (unsigned char)(usRegHoldingBuf[iRegIndex] >> 8);
        *pucRegBuffer++ =
            (unsigned char)(usRegHoldingBuf[iRegIndex] & 0xFF);
        iRegIndex++;
        usNRegs--;
      }
      break;
    /* Update current register values with new values from the protocol stack. */
    case MB_REG_WRITE:
      while (usNRegs > 0)
      {
        usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;        usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
        iRegIndex++;
        usNRegs--;
      }
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}

eMBErrorCode
eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
  eMBErrorCode eStatus = MB_ENOERR;
  int iRegIndex;

  if ((usAddress >= REG_INPUT_START) && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
  {
    iRegIndex = (int)(usAddress - REG_INPUT_START);
    while (usNRegs > 0)
    {
      *pucRegBuffer++ =
          (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
      *pucRegBuffer++ =
          (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
      iRegIndex++;
      usNRegs--;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }

  return eStatus;
}

eMBErrorCode
eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils,
              eMBRegisterMode eMode)
{
  eMBErrorCode eStatus = MB_ENOERR;
  int iNCoils = (int)usNCoils;
  unsigned short usBitOffset;

  /* Check if we have registers mapped at this block. */
  if ((usAddress >= REG_COILS_START) &&
      (usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE))
  {
    usBitOffset = (unsigned short)(usAddress - REG_COILS_START);
    switch (eMode)
    {
    /* Read current values and pass to protocol stack. */
    case MB_REG_READ:
      while (iNCoils > 0)
      {
        *pucRegBuffer++ =
            xMBUtilGetBits(ucRegCoilsBuf, usBitOffset,
                           (unsigned char)(iNCoils >
                                                   8
                                               ? 8
                                               : iNCoils));
        iNCoils -= 8;
        usBitOffset += 8;
      }
      break;
      /* Update current register values. */
    case MB_REG_WRITE:
      while (iNCoils > 0)
      {
        xMBUtilSetBits(ucRegCoilsBuf, usBitOffset,
                       (unsigned char)(iNCoils > 8 ? 8 : iNCoils),
                       *pucRegBuffer++);
        iNCoils -= 8;
        usBitOffset += 8;
      }
      break;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}

eMBErrorCode
eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
  eMBErrorCode eStatus = MB_ENOERR;
  short iNDiscrete = (short)usNDiscrete;
  unsigned short usBitOffset;

  /* Check if we have registers mapped at this block. */
  if ((usAddress >= REG_DISCRETE_START) &&
      (usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE))
  {
    usBitOffset = (unsigned short)(usAddress - REG_DISCRETE_START);
    while (iNDiscrete > 0)
    {
      *pucRegBuffer++ =
          xMBUtilGetBits( ucRegDiscreteBuf, usBitOffset,
                                    ( unsigned char )( iNDiscrete >
                                                       8 ? 8 : iNDiscrete ) );
      iNDiscrete -= 8;
      usBitOffset += 8;
    }
  }
  else
  {
    eStatus = MB_ENOREG;
  }
  return eStatus;
}

