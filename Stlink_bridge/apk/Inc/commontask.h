/**
  ******************************************************************************
  * @file           : cm4gateway_commontask.h
  * @brief          : Contain all the error code for cm0 apk
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CM4_GATEWAYCOMMONTASK_H
#define CM4_GATEWAYCOMMONTASK_H

#include "stm32f7xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "event_groups.h"

#ifdef __cplusplus
 extern "C" {
#endif


 extern TaskHandle_t swMaster_TaskHandle;
 extern TaskHandle_t swSlave_TaskHandle;



 typedef enum {
	 REQUEST,
	 ACK,
	 DATA_FROM_MASTER,
	 DATA_FROM_ISR,
	 LINE_RESET_FULL
 }notifTypeTypedef;

 typedef struct ns{
	 notifTypeTypedef type;
	 uint32_t value;
 } notificationStruct;

 extern notificationStruct slaveNotif,masterNotif;



 typedef enum  {ICARE_NOTDEF_SW=00U,
 	            ICARE_DEBUG_SW=0xA5U,
 	           ICARE_RELEASE_SW =0x5AU}swCfgTypeDef;

 typedef struct{
 	           uint32_t Swd_Master; /* SWD master Tick systask */
 	           uint32_t Swd_Slave;  /*Swd Slave  TaskTick*/
 	           uint32_t NetworkTaskTick; /* Network tick */
 	 	       uint32_t ParserTask;  /*Parser TaskTick*/
               uint32_t NetworkTask; /*GateWay*/

 }TaskTickTypeDef;

typedef enum {NUCLEO_BOARD_, PROTOTYPE_A}HardwareTypeDef;
typedef enum{  SYS_IDLE_STATE,
	           SYS_DECODE_STATE,

}stSystemTaskTypedDef;

 typedef struct{
		TaskTickTypeDef  TaskTick;  /*used for safety control ----*/
		HardwareTypeDef  Board;
		uint32_t         FirmwareVer;
		swCfgTypeDef     sw_Config;
	  stSystemTaskTypedDef   sysTaskState;

 }globalHandlerTypeDef;


 extern globalHandlerTypeDef  h_global;
 extern EventGroupHandle_t    xFlagSlave;
 extern EventGroupHandle_t      xFlagMaster;
 extern QueueHandle_t xQueueSlave;


 void vCommontask_StartApk(void );
 uint8_t SetEventFlag(const EventGroupHandle_t xEventGroup, uint32_t Flag,uint8_t isr);
 uint8_t WaitEventFlag(const EventGroupHandle_t xEventGroup, uint32_t Flag, TickType_t xTicksToWait );
 uint8_t GetEventFlag(const EventGroupHandle_t xEventGroup, uint32_t Flag);

/*bit Definition FlagSystem*/
#define NEW_RX_FROM_M4_FLAG              (1U<<0)  /*SET by Callback IPX when New data is comming on the sharedbuffer*/






 /*Bit management */
#define SET_SYSTEM_NEW_RXFRAME_ISR_FLAG       if (xFlagm4System!=NULL){(void)SetEventFlag(xFlagm4System,NEW_RX_FROM_M4_FLAG,1);}

/*Clear Flags*/
#define CLEAR_SYSTEM_NEW_RXFRAME_FLAG         if (xFlagm4System!=NULL){(void)ClearEventFlag(xFlagm4System, NEW_RX_FROM_M4_FLAG);}




#ifdef __cplusplus
}
#endif
#endif /*CM4_GATEWAYCOMMONTASK_H*/
