/*
 * esp_common.c
 *
 *  Created on: May 1, 2022
 *      Author: firas
 */

#include <stdio.h>
#include "common.h"
#include "main.h"




TaskHandle_t swMaster_TaskHandle;
TaskHandle_t swSlave_TaskHandle;

notificationStruct slaveNotif,masterNotif;


void vCommontask_StartApk(void )
{



    /*Init global handler*/


	/* Creat Supervisor Task */
   xTaskCreate(vSlaveswd_Task,"SLAVE",1000U ,NULL,(tskIDLE_PRIORITY+4U), &swSlave_TaskHandle );



   xTaskCreate(vMasterswd_Task,"MASTER",1000U ,NULL,(tskIDLE_PRIORITY+4U), &swMaster_TaskHandle );




}
