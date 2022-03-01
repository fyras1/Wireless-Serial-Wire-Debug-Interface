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
	 SWD_SLAVE_WAIT_FOR_END

 }SlaveStateTypeDef;


/*function prototype ---------------*/
 void vSlaveswd_Task(void * argument);
 void Swd_SlaveStateMachineShifter(void);


#ifdef __cplusplus
}
#endif
#endif /*SWD_SLAVE_TASK_H*/
