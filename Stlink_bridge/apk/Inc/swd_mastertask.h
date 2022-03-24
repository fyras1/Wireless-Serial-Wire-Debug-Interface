/**
  ******************************************************************************
  * @file           : swd_mastertast.h
  * @brief          : Contain supervisor prototype
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SWD_MASTER_TASK_H
#define SWD_MASTER_TASK_H

#ifdef __cplusplus
 extern "C" {
#endif




/*function prototype ---------------*/
 void vMasterswd_Task(void * argument);
 void printRequest(uint32_t rq);
 void printReset(void);


#ifdef __cplusplus
}
#endif
#endif /*SWD_MASTER_TASK_H*/
