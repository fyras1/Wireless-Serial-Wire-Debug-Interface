/**
  ******************************************************************************
  * @file           : swd_slave.h
  * @brief          : Contain supervisor prototype
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SWD_SLAVE_TASK_H
#define SWD_SLAVE_TASK_H

#ifdef __cplusplus
 extern "C" {
#endif



 typedef enum
  {
	 SWD_SLAVE_WAIT_FOR_START,
	 SWD_LINE_RESET_1,
	 SWD_JTAG_SELECT,
	 SWD_LINE_RESET_2,
	 SWD_WAIT_FOR_REQUEST,
	 SWD_REQUEST,
	 SWD_TURNAROUND_RQ_ACK,
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
 void Swd_SlaveStateMachineShifter();
  SlaveStateTypeDef unexpected_error_handler(SlaveStateTypeDef State);
   void requestParser(uint8_t rq, RequestTypeDef* request);
  void changeEdgeTrigger(uint8_t newEdge);
   SlaveStateTypeDef SwitchToRisingAndSkipEdge(uint8_t newEdge,SlaveStateTypeDef sourceState, SlaveStateTypeDef targetState);




#ifdef __cplusplus
}
#endif
#endif /*SWD_SLAVE_TASK_H*/
