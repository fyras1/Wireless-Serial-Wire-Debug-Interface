/**
  ******************************************************************************
  * @file           : serial wire debug master interface
  * @brief          : contain the project error entry point
  ******************************************************************************
  */
/*---------------------------------------------------------------------------------
 INCLUDES
-----------------------------------------------------------------------------------*/
#include "main.h"

#include "swd_mastertask.h"

/** @defgroup SupervTask SERIAL WIRE DEBUG MASTER INTERFACE
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
void vMasterswd_Task(void * argument)
{
	while(1)
	{
		/*Increment TaskTick*/
		h_global.TaskTick.Swd_Master++;



		osDelay(200);


	}



}

/**
 * @}
 */
