/**
  ******************************************************************************
  * @file           : cm4_error_code.h
  * @brief          : Contain all the error code for cm4 apk
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CM4_ERROR_CODE_H
#define CM4_ERROR_CODE_H

#ifdef __cplusplus
 extern "C" {
#endif


/*iCare ERROR CODE */
#define TASK_CREATION_FAIL_                    1U
#define QUEUE_CREATION_FAIL_                   2U
#define SYSTEM_CLOCK_FAIL_                     3U
#define PERIPH_INIT_FAIL_                      4U
#define SYSTEM_FAULT_                          5U
#define OVERSTACK_FAIL_                        6U
#define HAL_API_FAIL_                          7U

 void GlobalError(uint16_t errorCode);


#ifdef __cplusplus
}
#endif
#endif /*__CM4_ERROR_CODE_H*/
