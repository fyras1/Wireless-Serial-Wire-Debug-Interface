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


	uint32_t master_notif;
	while(1)
	{




				xTaskNotifyWait(0, 0xffffffff, &master_notif, portMAX_DELAY);




				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, 1);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, 0);

		/*Increment TaskTick*/
		h_global.TaskTick.Swd_Master++;



		//osDelay(200);

	}




}

/**
 * @}
 */
