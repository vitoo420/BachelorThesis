/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "platform.h"
#include "Region.h" /* Needed for LORAWAN_DEFAULT_DATA_RATE */
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "lora_app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
#include "adc_if.h"
#include "sys_conf.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ds18b20.h"
#include "mh_z16.h"
#include "app_fatfs.h"
#include "fatfs_sd.h"
#include "DS1307.h"

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief Appdata Transmission external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */
struct LogData
{
	uint16_t avgPpmStart;
	uint16_t avgPpmEnd;
	float avgTempStart;
	float avgTempEnd;
	float flux;
};

struct LoRaWANData
{
	uint16_t avgPpmStart;
	uint16_t avgPpmEnd;
	int16_t avgTempStart;
	int16_t avgTempEnd;
	uint16_t flux;
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa End Node send request
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  context ptr of timer context
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  join event callback function
  * @param  joinParams status of join
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief  tx event callback function
  * @param  params status of last Tx
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRa application has received a frame
  * @param appData data received in the last Rx
  * @param params status of last Rx
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/*!
 * Will be called each time a Radio IRQ is handled by the MAC layer
 *
 */
static void OnMacProcessNotify(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =           GetBatteryLevel,
  .GetTemperature =            GetTemperatureLevel,
  .GetUniqueId =               GetUniqueId,
  .GetDevAddr =                GetDevAddr,
  .OnMacProcess =              OnMacProcessNotify,
  .OnJoinRequest =             OnJoinRequest,
  .OnTxData =                  OnTxData,
  .OnRxData =                  OnRxData
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .PingPeriodicity =          LORAWAN_DEFAULT_PING_SLOT_PERIODICITY
};

/**
  * @brief Type of Event to generate application Tx
  */
static TxEventType_t EventType = TX_ON_TIMER;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/* USER CODE BEGIN PV */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

FIL fil;
FRESULT fresult;	//to store result
char buffer[1024];	//to store data
UINT br, bw;
/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_1 */

  /* USER CODE END LoRaWAN_Init_1 */

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks);

  LmHandlerConfigure(&LmHandlerParams);

  /* USER CODE BEGIN LoRaWAN_Init_2 */
  /* USER CODE END LoRaWAN_Init_2 */

  LmHandlerJoin(ActivationType);

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_SetPeriod(&TxTimer,  APP_TX_DUTYCYCLE);
    UTIL_TIMER_Start(&TxTimer);
  }
  else
  {
    /* USER CODE BEGIN LoRaWAN_Init_3 */
    /* USER CODE END LoRaWAN_Init_3 */
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */

  /* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */
	if ((appData != NULL) || (params != NULL))
	  {
	    /*BSP_LED_On(LED_BLUE) ;

	    UTIL_TIMER_Start(&RxLedTimer);*/

	    static const char *slotStrings[] = { "1", "2", "C", "C Multicast", "B Ping-Slot", "B Multicast Ping-Slot" };

	    APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Indication ==========\r\n");
	    APP_LOG(TS_OFF, VLEVEL_H, "###### D/L FRAME:%04d | SLOT:%s | PORT:%d | DR:%d | RSSI:%d | SNR:%d\r\n",
	            params->DownlinkCounter, slotStrings[params->RxSlot], appData->Port, params->Datarate, params->Rssi, params->Snr);
	    switch (appData->Port)
	    {
	      case LORAWAN_SWITCH_CLASS_PORT:
	        /*this port switches the class*/
	        if (appData->BufferSize == 1)
	        {
	          switch (appData->Buffer[0])
	          {
	            case 0:
	            {
	              LmHandlerRequestClass(CLASS_A);
	              break;
	            }
	            case 1:
	            {
	              LmHandlerRequestClass(CLASS_B);
	              break;
	            }
	            case 2:
	            {
	              LmHandlerRequestClass(CLASS_C);
	              break;
	            }
	            default:
	              break;
	          }
	        }
	        break;
	      case LORAWAN_USER_APP_PORT:
	    	  printf("here");
	        /*if (appData->BufferSize == 1)
	        {
	          AppLedStateOn = appData->Buffer[0] & 0x01;
	          if (AppLedStateOn == RESET)
	          {
	            APP_LOG(TS_OFF, VLEVEL_H,   "LED OFF\r\n");
	            BSP_LED_Off(LED_RED) ;
	          }
	          else
	          {
	            APP_LOG(TS_OFF, VLEVEL_H, "LED ON\r\n");
	            BSP_LED_On(LED_RED) ;
	          }
	        }*/
	        break;

	      default:

	        break;
	    }
	  }
  /* USER CODE END OnRxData_1 */
}

static void SendTxData(void)
{
  /* USER CODE BEGIN SendTxData_1 */
	printf("################## MEASURING CYCLE STARTED ##################\r\n");

	uint32_t i = 0;
	UTIL_TIMER_Time_t nextTxIn = 0;

	struct LogData saveData;
	struct LoRaWANData sendData;
	/*VLEVEL_L 1       !< just essential traces */
	/*VLEVEL_M 2       !< functional traces */
	/*VLEVEL_H 3       !< all traces */
	//APP_LOG(TS_ON, VLEVEL_L, "SendTxData started\r\n");

	saveData.avgPpmStart = 0;
	saveData.avgPpmEnd = 0;
	saveData.avgTempStart = 0;
	saveData.avgTempEnd = 0;

	//APP_LOG(TS_OFF, VLEVEL_M, "\r\n################## MEASURING CYCLE STARTED ##################\r\n");
	for(int i = 0; i < NUM_OF_SAMPLES; i++)
	{
		saveData.avgPpmStart += MHZ16_Read();
		saveData.avgTempStart += DS18B20_ReadTemperature();
		HAL_Delay(1000);
	}

	saveData.avgPpmStart = saveData.avgPpmStart / NUM_OF_SAMPLES;
	saveData.avgTempStart = saveData.avgTempStart / NUM_OF_SAMPLES;
	printf("%d ppm %f°C avg \r\n", saveData.avgPpmStart, saveData.avgTempStart);

	HAL_Delay(DELAY_MIN * 60000);

	//sber vzorku z konce
	for(int i = 0; i < NUM_OF_SAMPLES; i++)
	{
		saveData.avgPpmEnd += MHZ16_Read();
		saveData.avgTempEnd += DS18B20_ReadTemperature();
		HAL_Delay(1000);
	}

	saveData.avgPpmEnd = saveData.avgPpmEnd / NUM_OF_SAMPLES;
	saveData.avgTempEnd = saveData.avgTempEnd / NUM_OF_SAMPLES;

	printf("%d ppm %f°C\r\n", saveData.avgPpmEnd, saveData.avgTempEnd);

	float avgTemp = (saveData.avgTempStart + saveData.avgTempEnd) / 2;
	float deltaCdt = (saveData.avgPpmEnd-saveData.avgPpmStart) / (DELAY_MIN * 60);
	saveData.flux = ((float)(10140 * CHAMBER_VOLUME) / (GAS_CONSTANT * (273.15 + (avgTemp)) * COVERED_AREA)) * deltaCdt;
	printf("%f umol m^-2 s-1\r\n", saveData.flux);

	printf("################## MEASURING CYCLE FINISHED ##################\r\n");

	printf("################## WRITING SD CARD STARTED ##################\r\n");
	fresult = f_open(&fil, "night_session.csv", FA_OPEN_APPEND | FA_WRITE | FA_READ);

	//konverze float na string
	char temperatureStartString[13];
	char temperatureEndString[13];
	sprintf(temperatureStartString, "%f", saveData.avgTempStart);
	sprintf(temperatureEndString, "%f", saveData.avgTempEnd);
	//printf("%s\r\n", ppmStartString);

	//nacteni aktualniho casu
	getTime();

	/*write to .csv file to SD card START*/
	f_printf(&fil, "20%d-%02d-%02d %02d:%02d:%02d", s_time.year, s_time.month, s_time.dayofmonth, s_time.hours, s_time.minutes, s_time.seconds);
	fresult = f_write(&fil, ";", 1, &bw);

	f_printf(&fil, "%d", saveData.avgPpmStart);
	fresult = f_write(&fil, ";", 1, &bw);

	strcpy(buffer, temperatureStartString);
	fresult = f_write(&fil, buffer, bufsize(buffer), &bw);
	bufclear(buffer, sizeof(buffer)/sizeof(buffer[0]));
	fresult = f_write(&fil, ";", 1, &bw);

	f_printf(&fil, "%d", saveData.avgPpmStart);
	fresult = f_write(&fil, ";", 1, &bw);

	strcpy(buffer, temperatureEndString);
	fresult = f_write(&fil, buffer, bufsize(buffer), &bw);
	bufclear(buffer, sizeof(buffer)/sizeof(buffer[0]));

	fresult = f_write(&fil, "\r\n", 1, &bw);
	fresult = f_close(&fil);
	printf("################## WRITING SD CARD FINISHED ##################\r\n");
	/*write csv END*/


	/*write to LoRaWAN buffer*/
	printf("################## SENDING OVER LORAWAN STARTED ##################\r\n");
	AppData.Port = LORAWAN_USER_APP_PORT;
	//teplota start
	sendData.avgTempStart = saveData.avgTempStart * 100;
	AppData.Buffer[i++] = (uint8_t)(sendData.avgTempStart >> 8 & 0xFF);
	AppData.Buffer[i++] = (uint8_t)(sendData.avgTempStart & 0xFF);
	//teplota end
	sendData.avgTempEnd = saveData.avgTempEnd * 100;
	AppData.Buffer[i++] = (uint8_t)(sendData.avgTempEnd >> 8 & 0xFF);
	AppData.Buffer[i++] = (uint8_t)(sendData.avgTempEnd & 0xFF);
	//ppmstart
	sendData.avgPpmStart = saveData.avgPpmStart;
	AppData.Buffer[i++] = (uint8_t)(sendData.avgPpmStart >> 8 & 0xFF);
	AppData.Buffer[i++] = (uint8_t)(sendData.avgPpmStart & 0xFF);
	//ppmend
	sendData.avgPpmEnd = saveData.avgPpmEnd;
	AppData.Buffer[i++] = (uint8_t)(sendData.avgPpmEnd >> 8 & 0xFF);
	AppData.Buffer[i++] = (uint8_t)(sendData.avgPpmEnd & 0xFF);
	//flux
	/*LoRaParser.f = saveData.flux;
	AppData.Buffer[i++] = LoRaParser.ui8[3];
	AppData.Buffer[i++] = LoRaParser.ui8[2];
	AppData.Buffer[i++] = LoRaParser.ui8[1];
	AppData.Buffer[i++] = LoRaParser.ui8[0];*/

	AppData.Buffer[i++] = GetBatteryLevel();        /* 1 (very low) to 254 (fully charged)*/
	/*float temp = 0;
	int16_t gas = 0;

	temp = DS18B20_ReadTemperature();
	gas = MHZ16_Read();

	LoRaParser.f = temp;
	printf("%f aaa", LoRaParser.f);

	AppData.Port = LORAWAN_USER_APP_PORT;
	AppData.Buffer[i++] = AppLedStateOn;
	AppData.Buffer[i++] = (uint8_t)((gas >> 8) & 0xFF);
	AppData.Buffer[i++] = (uint8_t)(gas & 0xFF);
	AppData.Buffer[i++] = LoRaParser.ui8[4];
	AppData.Buffer[i++] = LoRaParser.ui8[3]; //(((uint8_t)temp >> 16) & 0xFF);
	AppData.Buffer[i++] = LoRaParser.ui8[2]; //(((uint8_t)temp >> 8) & 0xFF);
	AppData.Buffer[i++] = LoRaParser.ui8[1]; //((uint8_t)temp & 0xFF);*/


	AppData.BufferSize = i;

	if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false))
	{
		APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
		printf("SEND REQUEST\r\n");
	}
	else if (nextTxIn > 0)
	{
		APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
		printf("Next Tx in  : ~%lu second(s)\r\n", (nextTxIn / 1000));
	}

	printf("################## SENDING OVER LORAWAN FINISHED ##################\r\n");
  /* USER CODE END SendTxData_1 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */

  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */

  /* USER CODE END OnTxTimerEvent_2 */
}

/* USER CODE BEGIN PrFD_LedEvents */

/* USER CODE END PrFD_LedEvents */

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */
	if ((params != NULL))
	{
		/* Process Tx event only if its a mcps response to prevent some internal events (mlme) */
		if (params->IsMcpsConfirm != 0)
		{

		  APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
		  APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
				  params->AppData.Port, params->Datarate, params->TxPower);

		  APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
		  if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
		  {
			APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
		  }
		  else
		  {
			APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
		  }
		}
	}
  /* USER CODE END OnTxData_1 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
	  if (joinParams != NULL)
	  {
	    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
	    {

	      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
	      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
	      {
	        APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
	      }
	      else
	      {
	        APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
	      }
	    }
	    else
	    {
	      APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
	    }
	  }
  /* USER CODE END OnJoinRequest_1 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}
