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

 void swclk_reset(void);
 void swclk_set(void);
 void swclk_cycle(void);
 void swdio_output(uint8_t bit);
 void swdio_mode_input(void);
 void swdio_mode_output(void);



#ifdef __cplusplus
}
#endif
#endif /*SWD_MASTER_TASK_H*/
