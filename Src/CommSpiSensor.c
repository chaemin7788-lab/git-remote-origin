/*
 * Max22530_SPI.c
 *
 *  Created on: 2023. 10. 17.
 *      Author: C최동희
 */


#include "CommSpiSensor.h"

#include "main.h"
#include "wizchip_conf.h"
#include "wizInterface.h"
#include "usart.h"
#include "spi.h"
#include "port.h"
#include "string.h"

/******************************************************************************/
/* MAX22530      Programming Guide Functions                     */
/******************************************************************************/


/*MAX22530 Registers*/
#define PROD_ID           0x00
#define ADC1              0x01
#define ADC2              0x02
#define ADC3              0x03
#define ADC4              0x04
#define FADC1             0x05
#define FADC2             0x06
#define FADC3             0x07
#define FADC4             0x08
#define COUTHI1           0x09
#define COUTHI2           0x0a
#define COUTHI3           0x0b
#define COUTHI4           0x0c
#define COUTLO1           0x0d
#define COUTLO2           0x0e
#define COUTLO3           0x0f
#define COUTLO4           0x10
#define COUT_STATUS       0x11
#define INTERRUPT_STATUS  0x12
#define INTERRUPT_ENABLE  0x13
#define CONTROL           0x14

#define SPI2_CS_Pin GPIO_PIN_3
#define SPI2_CS_GPIO_Port GPIOC

#define MAX22530_ID       0x80
#define VREF              1.80
#define VREF_FIX		  0.000439453

/*uC SPI + GPI/O settings*/
//#define CS_PIN               " "   // Pin of the uC to which the ADC Chip Select pin is connected.
//#define CS_PIN_OUT           " "   // defining the pin Mode on uC side
#define CS_LOW1                HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_RESET); //CS LOW   // digital Write of uC CS_PIN LOW
#define CS_HIGH1               HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_SET); //CS LOW   // digital Write of uC CS_PIN HIGH

#define CS_LOW2                HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_RESET); //CS LOW   // digital Write of uC CS_PIN LOW
#define CS_HIGH2               HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_SET); //CS LOW   // digital Write of uC CS_PIN HIGH
//#define GPIO1_PIN            " "   // Pin of the uC to which the ADC Hardware EOC pin INTB Ready pin is connected for determining the end of conversion using a polling sequence.
//#define GPIO1_PIN_IN         " "   // Defining the pin mode of GPIO1_PIN
//#define GPIO1_STATE          " "   // Configuring the GPIO1_PIN as read state
//#define SPI_SETTINGS         " "   // Configure the SPI settings on uC side for. ex: // 8 MHz clock, MSB   first, SPI CPOL 0, CPHA 0

/* MAX22530 HARDWARE INTERRUPT PIN-GPO */
#define MAX22530_INTB_RDY_STATE       //GPIO_STATE used by uC


/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
/* Reads the value of the selected register. */
uint16_t MAX22530_read_register(uint16_t regAddress);

uint16_t MAX22530_read_register2(uint16_t regAddress, uint8_t SpiChannel);

/* Writes a value to the register. */
uint16_t MAX22530_write_register(uint8_t regAddress, uint8_t regValue);

/* Burst Reads the value of the selected register. */
void MAX22530_Burst_read_register(uint8_t regAddress);

/* Initialize MAX22530 and check if the device is present*/
uint8_t MAX22530_Init(void);

/* Resets the device*/
void MAX22530_Reset(void);

/* Resets the device*/
void MAX22530_softReset(void);

/* Enables the Hardware End of Conversion Interrupt Ready Bit to go LOW for ADC */
void MAX2253x_EN_EOC();

/* Disables the Hardware Interrupt Ready Bit to go LOW for ADC */
void MAX2253x_DIS_EOC();

/* Enables CRC*/
void MAX2253x_EN_CRC();

/* Disables CRC*/
void MAX2253x_DIS_CRC();

/* Toggles a bit based on bit position to the given register. */
void MAX22530_Register_bit_toggle(uint8_t regAddress, uint8_t bit_position);


/* Returns converted binary value to Voltage of ADCx, FADCx, COUTx HI and LO Threshold registers*/
float Convert_to_Voltage(uint8_t regAddress);

/******************************************************************************/
/* Supporting function for CRC */
uint8_t crc_compute_2(uint32_t frame);
uint8_t crc_compute_burst(uint32_t frame1, uint32_t frame2, uint32_t frame3);
uint8_t getCRC(uint8_t[] , unsigned int );

/******************************************************************************/
/***    Global Variables, Declarations                     ***/
/******************************************************************************/
/* CRC Table for CRC8 polynomial c(x) = (x8 + x2 + x1 + 1) */
unsigned char CRCTable[256] = {
  0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
  0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
  0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
  0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
  0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
  0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
  0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
  0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
  0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
  0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
  0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
  0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
  0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
  0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
  0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
  0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

BOOL crc_enable = FALSE;

/* Global variables declarations for Burst read function */
uint8_t Burst_reg1;    // ADC1 data in case of Burst read(ADC1), FADC1 data when Burst read(FADC1)
uint8_t Burst_reg2;    // ADC2 data in case of Burst read(ADC1), FADC2 data when Burst read(FADC1)
uint8_t Burst_reg3;    // ADC3 data in case of Burst read(ADC1), FADC3 data when Burst read(FADC1)
uint8_t Burst_reg4;    // ADC4 data in case of Burst read(ADC1), FADC4 data when Burst read(FADC1)
uint8_t Burst_INT_status;

uint8_t result_DATA;

uint16_t mcontrol, mstatus;

#define PAEK_ARRAY_LENTH	100
#define PAEK_MEDIAN_LENTH	10
#define ADC_MEMBER_COUNT	4

uint16_t unAcPeackArray[ADC_MEMBER_COUNT][PAEK_ARRAY_LENTH];
uint16_t unAcPeackMedian[ADC_MEMBER_COUNT][PAEK_MEDIAN_LENTH];

uint16_t unAcPeackArrayCount[ADC_MEMBER_COUNT]  = {0,0,0,0};
uint16_t unAcPeackMedianCount[ADC_MEMBER_COUNT] = {0,0,0,0};
uint16_t unPeackAcData[ADC_MEMBER_COUNT] = {0,0,0,0};
uint16_t unTempData = 0;

float fResult_data;

TYPE_ADC_DATA mAdcData;

void SPI_Init()
{
    MAX22530_Init();

}

// ADDR은 6BIT / W/R 1BIT / BURST 1BIT
// 2 bit Shift << 8bit Shift
uint16_t MAX22530_read_register(uint16_t regAddress)
{
  uint16_t result = 0;
  uint8_t frame = (regAddress << 2) ;
  uint8_t rbuf[2] = {0};

  CS_LOW1

  HAL_SPI_Transmit(&hspi2, &frame, 1, 0xFFFFFFFF);
  HAL_SPI_Receive(&hspi2, rbuf, 2, 0xFFFFFFFF);

  result = rbuf[0];
  result <<= 8;
  result |= rbuf[1];

  CS_HIGH1

  return result;

}

uint16_t MAX22530_read_register2(uint16_t regAddress, uint8_t SpiChannel)
{
  uint16_t result = 0;
  uint8_t frame = (regAddress << 2) ;
  uint8_t rbuf[2] = {0};

  if(SpiChannel == 1){CS_LOW1;}
  else if(SpiChannel == 2) {CS_LOW2;}

  HAL_SPI_Transmit(&hspi2, &frame, 1, 0xFFFFFFFF);
  HAL_SPI_Receive(&hspi2, rbuf, 2, 0xFFFFFFFF);

  result = rbuf[0];
  result <<= 8;
  result |= rbuf[1];

  if(SpiChannel == 1){CS_HIGH1;}
  else if(SpiChannel == 2){CS_HIGH2;}

  return result;

}


/***************************************************************************//**
   @brief Writes to any register.
   @param regAddress - The address of the register to read.
   @regvalue - The register value to write to the register.

*******************************************************************************/

uint16_t MAX22530_write_register(uint8_t regAddress , uint8_t regValue)
// Register_address address = COUTH1, data = 8B32h or 35634d
{
  uint8_t data_frame1 = 0x0000;
  uint8_t crc_code = 0;

  if ((!crc_enable))      /// Read/ Write bit set to 1, burst bit set to 0.
  {
    data_frame1 = (uint32_t)((regAddress << 2) + (1 << 1));
  }
  else        /// crc enabled, construct frame + compute crc.
  {
    data_frame1 = (uint32_t) ((regAddress << 2) + (1 << 1));
    crc_code = crc_compute_2((data_frame1 << 16) + (regValue << 0));
  }

  CS_LOW1;
  //SPI.transfer8(data_frame1);
  //SPI.transfer16(regValue);
  HAL_SPI_Transmit(&hspi2, &data_frame1, 1, 0xFFFFFFFF);
  HAL_SPI_Transmit(&hspi2, &regValue, 1, 0xFFFFFFFF);

  //SPI_writeDataBlockingNonFIFO(SPIB_BASE, data_frame1);
  //SPI_writeDataBlockingNonFIFO(SPIB_BASE, regValue);

  if (crc_enable) {
      //SPI.transfer8(crc_code);
	  HAL_SPI_Transmit(&hspi2, &crc_code, 1, 0xFFFFFFFF);
      //SPI_writeDataBlockingNonFIFO(SPIB_BASE, crc_code);
  }
  CS_HIGH1;

  return 0;
}



/***************************************************************************//**
   @brief Function to perform burst read on ADC1 or FADC1 registers only.
   @param regAddress - The address of the register to burst read. (ADC1 or FADC1)

   @return status: Global Burst read variables Burst_reg0, Burst_reg1, Burst_reg2,
                   Burst_reg3, Burst_INT_status are updated
                   0 - Burst read is not performed and no action is taken
           0x10000 – CRC mismatch occured

*******************************************************************************/
void MAX22530_Burst_read_register(uint8_t regAddress)
// Only Two Options for Burst read ADC1 & FADC1
// Only Two Options for Burst read ADC1 & FADC1. Register_address address = FADC1
{
  if (regAddress == ADC1 || regAddress == FADC1)
  {
    uint8_t crc, crc_read = 0;
    uint8_t frame = (uint32_t)((regAddress << 2) + (1 << 0)) ;
    static uint8_t result[5];
    if (!crc_enable)      // Read/ Write bit set to 0 and burst bit set to 1. 18th bit=0 and 17th bit= 1
    {
      CS_LOW1;
	  HAL_SPI_Transmit(&hspi2, &frame, 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &Burst_reg1, 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &Burst_reg2, 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &Burst_reg3, 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &Burst_reg4, 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &Burst_INT_status, 1, 0xFFFFFFFF);
      CS_HIGH1;
    }
    else if (crc_enable)
    {
      CS_LOW1;
	  HAL_SPI_Transmit(&hspi2, &frame, 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &result[0], 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &result[1], 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &result[2], 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &result[3], 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &result[4], 1, 0xFFFFFFFF);
      HAL_SPI_Receive(&hspi2, &crc_read, 1, 0xFFFFFFFF);

      CS_HIGH1;
      crc = crc_compute_burst(((frame << 16) + (result[0] << 0)), ((result[1] << 16) + (result[2] << 0)),     ((result[3] << 16) + (result[4] << 0)));
      if (crc != crc_read)
        {
          // printf("CRC Read from MAX22530 is incorrect\n");
           return(0x10000);
        }
      else
        {
        //  printf("CRC Matched \n");
        }
      Burst_reg1 = result[0];
      Burst_reg2 = result[1];
      Burst_reg3 = result[2];
      Burst_reg4 = result[3];
      Burst_INT_status = result[4];
    }
  }
  else {
     return;
       }
}

/***************************************************************************//**
   @brief Initializes the MAX22530 and checks if the device is present for first time.
           If the device is present, write CONTROL Register b'[15] = 0 to disable CRC.

   @return status - Result of the initialization procedure.
                    Example: 1 - if initialization was successful (ID is 0x0B).
                             0 - if initialization was unsuccessful.
*******************************************************************************/
//
// Vin = Vout / (3.9 / (3.9+100))

uint8_t MAX22530_Init(void)
{
  uint8_t status = 0x1;

  if ((MAX22530_read_register(PROD_ID) != MAX22530_ID))
  {
    status = 0x0;
  }

  return (status);
}

uint16_t Median_Filter(uint16_t *a,uint16_t last_item)
{
        int loop1,loop2;
        int temp;
        for (loop1=0;loop1<last_item;loop1++)
        {
                for (loop2=0;loop2<(last_item-loop1);loop2++)
                {
                        if (a[loop2]>a[loop2+1])
                        {
                                temp=a[loop2];
                                a[loop2]=a[loop2+1];
                                a[loop2+1]=temp;
                        }
                }
        }
        return a[PAEK_MEDIAN_LENTH/2];
}

// 중간값 필터
uint16_t InsultData(uint16_t index, uint16_t Data, uint16_t *OutData)
{
    uint16_t i = 0;

    unAcPeackArray[index][unAcPeackArrayCount[index]++] = Data;

    if(unAcPeackArrayCount[index] >= PAEK_ARRAY_LENTH)
    {
    	// 배열에서 최대값을 찾는다
    	for (i = 0; i < PAEK_ARRAY_LENTH; i++)
    	{
    		if (unAcPeackArray[index][i] > unPeackAcData[index]) unPeackAcData[index] = unAcPeackArray[index][i]; // 더 큰값을 최대값으로 한다.
    	}

    	// 최대값을 중간값 필터 배열에 저장
        unAcPeackMedian[index][unAcPeackMedianCount[index]++] = unPeackAcData[index];

        // 중간값 필터 배열에서 중간값을 추출한다.
        if(unAcPeackMedianCount[index] >= PAEK_MEDIAN_LENTH)
        {
        	*OutData = Median_Filter(unAcPeackMedian[index], PAEK_MEDIAN_LENTH);

			memset(unAcPeackMedian[index], 0, sizeof(unAcPeackMedian[index]));
			memset(unAcPeackArray[index], 0, sizeof(unAcPeackArray[index]));
			unAcPeackArrayCount[index] = 0;
			unAcPeackMedianCount[index] = 0;
			unPeackAcData[index] = 0;

			return 1;

        }

		memset(unAcPeackArray[index], 0, sizeof(unAcPeackArray[index]));
		unAcPeackArrayCount[index] = 0;
		unPeackAcData[index] = 0;

    }

    return 0;
}

// Every Time Update
void MAX22530_Update(int ChIndex)
{
	// FADC2번 전압 AC 0.015 = 1V
	if(InsultData(1, MAX22530_read_register2(FADC2,1), &unTempData))mAdcData.Bitval.AdcData1 = (((((float)VREF*unTempData/4096.0)-0.8)/0.000515)*0.707)*10.0;

	// FADC1번 전류 AC 0.00071111 = 1A
	//if(InsultData(2, MAX22530_read_register2(FADC1,1), &unTempData))mAdcData.Bitval.AdcData2 = (((((float)VREF*unTempData/4096.0)-0.8)/0.015)*0.707)*10.0;
	if(InsultData(2, MAX22530_read_register2(FADC1,1), &unTempData))mAdcData.Bitval.AdcData2 = (((((float)VREF*unTempData/4096.0))/0.015)*0.707)*10.0;

	// FADC1번 DC 전류 0.015 = 1A
	//if(InsultData(3, MAX22530_read_register2(FADC1,2), &unTempData))mAdcData.Bitval.AdcData3 = ((((float)VREF*unTempData/4096.0)-0.8)/0.015)*10.0;
	if(InsultData(3, MAX22530_read_register2(FADC4,1), &unTempData))mAdcData.Bitval.AdcData3 = ((((float)VREF*unTempData/4096.0))/0.015)*10.0;

	// FADC2번 DC 전압 0.002272V 당 1V로 계산
	// 750V 1.3V 입력이 맞음. 실측 계산한 결과 1V 당 0.000183
	mAdcData.Bitval.AdcData4 = (VREF_FIX*(float)MAX22530_read_register2(FADC3,1))/0.000183;
	//mAdcData.Bitval.AdcData4 = (float)MAX22530_read_register2(FADC2,2);
}


TYPE_ADC_DATA* MAX22530_GetData(void)
{
    return (&mAdcData);
}

void MAX22530_SetData(TYPE_ADC_DATA* pData)
{
    mAdcData = *pData;
}

/********************************Prototype Interface Functions***********************************/


/***************************************************************************//**
   @brief Write bit field 0 of Control register in order to reset the part.

   @return  None.
*******************************************************************************/
void MAX22530_Reset(void)
{
  MAX22530_write_register(CONTROL, 0);
  crc_enable = FALSE;
}

/***************************************************************************//**
   @brief Write bit field 1 of Control register in order to soft reset the part.

   @return  None.
*******************************************************************************/
void MAX22530_softReset(void)
{
  MAX22530_write_register(CONTROL, 2);
}


/***************************************************************************//**
   @brief Toggles bit in a given register. Used in conjunction with INT_EN and CONTROL to
          toggle a bit. n = 0 for First bit position and so on.

   @param regAddress - The address of the register to read.
   @bit_position - The position of the register to toggle.

   @return data - 0 : Use hardware Reset Function to reset the device
*******************************************************************************/
void MAX22530_Register_bit_toggle (uint8_t regAddress, uint8_t bit_position)
// Register_address address = INTERRUPT ENABLE, bit_position = 12
{
  if (regAddress == 0x14 && bit_position == 0)
  {
   // printf("Use hardware Reset function");
    return;
  }
  else
  {
    int n = bit_position;
    uint16_t current_data = MAX22530_read_register(regAddress);
    uint16_t new_data = current_data ^ (1UL << n);
    MAX22530_write_register(regAddress, new_data);
  }
}


/***************************************************************************//**
   @brief Enables CRC option. Usually Used during First Time initialization
   @param - None
*******************************************************************************/
void MAX2253x_EN_CRC()
{
  uint16_t register_value = MAX22530_read_register(CONTROL);
  /*set EN CRC bit in Control Register*/
  if (register_value < 32768)
  {
    register_value |= 32768; // if register value = 0x8000
    MAX22530_write_register(CONTROL, register_value);
    crc_enable = TRUE;
  }
}



/***************************************************************************//**
   @brief Disables CRC.
   @param - None
*******************************************************************************/
void MAX2253x_DIS_CRC()
{
  uint16_t register_value = MAX22530_read_register(CONTROL);
  /*disable CRC bit in Control Register*/
  register_value &= 32767;  // if register value = 0x7FFF
  MAX22530_write_register(CONTROL, register_value);
  crc_enable = FALSE;
}



/***************************************************************************//**
   @brief Enables End of conversion Hardware pin @ MAX2253x INT.
   @param - None
*******************************************************************************/
void MAX2253x_EN_EOC()
{
  uint16_t register_value = MAX22530_read_register(INTERRUPT_ENABLE);
  /*disable CRC bit in Control Register*/
  register_value |= 4096; // if register value = 0x1000
  MAX22530_write_register(INTERRUPT_ENABLE, register_value);
}



/***************************************************************************//**
   @brief Disbles End of conversion Hardware pin @ MAX2253x INT.
   @param - None
*******************************************************************************/
void MAX2253x_DIS_EOC()
{
  uint16_t register_value = MAX22530_read_register(INTERRUPT_ENABLE);
  /*disable CRC bit in Control Register*/
  register_value &= 4095; // if register value = 0x0FFF
  MAX22530_write_register(INTERRUPT_ENABLE, register_value);
}



/***************************************************************************//**
   @brief returns voltage value of content in register. Used in conjunction with
   ADC1, ADC2, ADC3, ADC4, FADC1, FADC2, FADC3, FADC4, COUTHI1, COUTHI2, COUTHI3
   COUTHI4, COUTLO1, COUTLO2, COUTLO3, COUTLO4 registers

   @param regAddress - The address of the register to read.

   @return voltage_result - voltage converted binary value of register data
                          - 100 if ADC1-4 data not updated since last read
*******************************************************************************/
float Convert_to_Voltage(uint8_t regAddress)
{
  uint16_t result = 0;
  float voltage_result = 0.0;
  result = MAX22530_read_register(regAddress);
   if(regAddress == ADC1 || regAddress == ADC2 || regAddress == ADC3 ||  regAddress == ADC4 )
   {
      if( result > 4096) // 16th bit of ADCx set to 1 meaning ADC not updated since last read operation
      {
        return 100.00;
      }
      voltage_result = (float)VREF*result/4096;

   }
   else if(regAddress == COUTHI1 || regAddress == COUTHI2 || regAddress == COUTHI3 ||  regAddress == COUTHI4)
   {
      result = result & 0x0FFF;
      voltage_result = (float)VREF*result/4096;
   }
   else
   {
      voltage_result = (float)VREF*result/4096;
   }
   return voltage_result;
}

/***************************************************************************//**
   @brief supporting functions in computing CRC and returning CRC
     and returning CRC value from CRCTable.

*******************************************************************************/

/* Used with Register Read/ Write functions */
uint8_t crc_compute_2(uint32_t frame)
{
  uint8_t frame_high = (uint8_t) ((frame & 0x00FF0000) >> 16);
  uint8_t frame_mid =  (uint8_t) ((frame & 0x0000FF00) >> 8);
  uint8_t frame_low =  (uint8_t) (frame & 0x000000FF);
  uint8_t message[] = {frame_high, frame_mid, frame_low, 0x00};

  message[3] = getCRC(message, 3);
  return (message[3]);
}


/* Used with Register Burst Read function */
uint8_t crc_compute_burst(uint32_t frame1, uint32_t frame2, uint32_t frame3)
{

  uint8_t frame1_high = (uint8_t) ((frame1 & 0x00FF0000) >> 16);
  uint8_t frame1_mid =  (uint8_t) ((frame1 & 0x0000FF00) >> 8);
  uint8_t frame1_low =  (uint8_t) (frame1 & 0x000000FF);
  uint8_t frame2_high = (uint8_t) ((frame2 & 0xFF000000) >> 24);
  uint8_t frame2_mid1 =  (uint8_t) ((frame2 & 0x00FF0000) >> 16);
  uint8_t frame2_mid2 =  (uint8_t) ((frame2 & 0x0000FF00) >> 8);
  uint8_t frame2_low =  (uint8_t) (frame2 & 0x000000FF);
  uint8_t frame3_high = (uint8_t) ((frame3 & 0xFF000000) >> 24);
  uint8_t frame3_mid1 =  (uint8_t) ((frame3 & 0x00FF0000) >> 16);
  uint8_t frame3_mid2 =  (uint8_t) ((frame3 & 0x0000FF00) >> 8);
  uint8_t frame3_low =  (uint8_t) (frame3 & 0x000000FF);

  uint8_t crc_calculated = 0x00;

  uint8_t message[] = {frame1_high, frame1_mid, frame1_low, frame2_high, frame2_mid1, frame2_mid2, frame2_low, frame3_high, frame3_mid1, frame3_mid2, frame3_low,  0x00};

  crc_calculated  = getCRC(message, 11);
  return crc_calculated;
}



uint8_t getCRC( uint8_t message[], unsigned int length)
{
  uint8_t i, crc = 0;

  for (i = 0; i < length; i++)
    crc = CRCTable[crc ^ message[i]];
  return crc;
}
