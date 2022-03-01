/**
  ******************************************************************************
  * @file           : serial wire debug interface
  * @brief          : contain the project error entry point
  ******************************************************************************
  */
/*---------------------------------------------------------------------------------
 INCLUDES
-----------------------------------------------------------------------------------*/
#include "main.h"

#include "swd_slavetask.h"

/** @defgroup SupervTask SERIAL WIRE DEBUG INTERFACE
  * @ingroup TaskModule
 * @brief Supervisor Task
 * @{
 */

/**
  * @brief  SuperVisor Task
  * @param  None
  * @retval None
  *
*/
void vSlaveswd_Task(void * argument)
{
	while(1)
	{
		/*Increment TaskTick*/
		h_global.TaskTick.Swd_Slave++;



		osDelay(200);


	}



}

/**
 * @}
 */
