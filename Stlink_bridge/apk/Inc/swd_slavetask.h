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
	 JTAG_TO_SWD_SWITCH,
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


/*function prototype ---------------*/
 void vSlaveswd_Task(void * argument);
 void Swd_SlaveStateMachineShifter(void);
 SlaveStateTypeDef unexpected_error_handler(SlaveStateTypeDef State);



#ifdef __cplusplus
}
#endif
#endif /*SWD_SLAVE_TASK_H*/
