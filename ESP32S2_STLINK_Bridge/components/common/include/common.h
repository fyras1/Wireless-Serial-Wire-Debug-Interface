/*
 * esp_common.h
 *
 *  Created on: May 1, 2022
 *      Author: firas
 */

#ifndef COMPONENTS_APK_INCLUDE_ESP_COMMON_H_
#define COMPONENTS_APK_INCLUDE_ESP_COMMON_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



void vCommontask_StartApk(void );

 extern TaskHandle_t  swMaster_TaskHandle;
 extern TaskHandle_t swSlave_TaskHandle;



 typedef enum {
	 REQUEST,
	 ACK,
	 DATA_FROM_MASTER,
	 DATA_FROM_ISR,
	 LINE_RESET_FULL,
	 DATA_WRITE_FINISH,
	 LINE_RESET_FROM_MASTER
 }notifTypeTypedef;

 typedef struct ns{
	 notifTypeTypedef type;
	 uint32_t value1;
	 uint32_t value2;
 } notificationStruct;


 extern notificationStruct slaveNotif,masterNotif;




#endif /* COMPONENTS_APK_INCLUDE_ESP_COMMON_H_ */
