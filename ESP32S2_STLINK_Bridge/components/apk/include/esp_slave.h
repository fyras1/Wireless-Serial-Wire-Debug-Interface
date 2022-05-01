/*
 * esp_slave.h
 *
 *  Created on: May 1, 2022
 *      Author: firas
 */

#ifndef COMPONENTS_APK_INCLUDE_ESP_SLAVE_H_
#define COMPONENTS_APK_INCLUDE_ESP_SLAVE_H_


#include "esp_common.h"

 typedef enum
  {  //SWD_SELF_ACK_WAIT,
	 SWD_REQUEST,
	 SWD_SLAVE_WAIT_FOR_START,
	 SWD_TURNAROUND_RQ_ACK,



	 SWD_LINE_RESET_1,
	 SWD_JTAG_SELECT,
	 SWD_LINE_RESET_2,
	 SWD_WAIT_FOR_REQUEST,

	 SWD_ACKNOWLEDGE,
	 SWD_TURNAROUND_ACK_RQ,
	 SWD_TURNAROUND_ACK_DAT,
	 SWD_DATA_TRANSFER,
	 SWD_TURNAROUND_DAT_RQ


 }SlaveStateTypeDef;





typedef struct request{
	uint8_t Start;
	uint8_t APnDP;
	uint8_t RnW;
	uint8_t A2;
	uint8_t A3;
	uint8_t Parity;
	uint8_t Stop;
	uint8_t Park;

} RequestTypeDef;


/*function prototype ---------------*/

void vSlaveswd_Task(void * argument);

SlaveStateTypeDef unexpected_error_handler(SlaveStateTypeDef State);
void requestParser(uint8_t rq, RequestTypeDef* request);
void changeEdgeTrigger(uint8_t newEdge);
SlaveStateTypeDef SwitchToRisingAndSkipEdge(uint8_t newEdge,SlaveStateTypeDef sourceState, SlaveStateTypeDef targetState);

void sendNotif( notifTypeTypedef notifType, uint32_t val1 ,uint32_t val2 ,TaskHandle_t *swSlave_TaskHandle );


void Swd_SlaveStateMachineShifter(void);

#endif /* COMPONENTS_APK_INCLUDE_ESP_SLAVE_H_ */
