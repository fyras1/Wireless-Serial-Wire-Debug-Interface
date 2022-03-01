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
/*Create FLAG */
#define RX_DATA_READY          1<<1U;
#define RX_DATA_BSY            1<<2U;



/**
  * @brief  Swd_SlaveStateMachineShifter  FROM ISR()
  * @param  None
  * @retval None
  *
*/
void Swd_SlaveStateMachineShifter(void)
{

	SlaveStateTypeDef State;

	switch (State)
	{
	  case SWD_SLAVE_WAIT_FOR_START:
	  {
		break;
	  }
	  case SWD_SLAVE_WAIT_FOR_END:
	  {
		break;
	  }


	}



     /**/
	//SetEventFlag(xFlagSlave,RX_DATA_READY, 1U)


}




/**
 * @}
 */
