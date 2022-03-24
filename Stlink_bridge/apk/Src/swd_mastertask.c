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

#define SWD_Select_Seq 0x79E7

void vMasterswd_Task(void * argument)
{


	notificationStruct notif;
	while(1)
	{

				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, 1);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, 0);

				xTaskNotifyWait(0, 0xffffffff, NULL, portMAX_DELAY);
				notif=masterNotif;

				switch(notif.type){

							case LINE_RESET_FULL:
							{
								printReset();
								break;
							}

							case REQUEST:
							{
								printRequest(notif.value);
							 break;
							}

							case ACK:
							{
								break;
							}

							case DATA:
							{
								break;
							}
						}




		/*Increment TaskTick*/
		h_global.TaskTick.Swd_Master++;



		//osDelay(200);

	}




}





/* Print an 8 bit request (rq) on  falling edges*/
inline void printRequest(uint32_t rq)
{
	for (int i=7;i>=0;i--)
	{

		//MANUAL FALLING EDGE
		// note to self: chagne to GPIOD->ODR later
		HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 1);

		HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin, (rq>>i)&0x01 );

		HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 0);
	}
}


/*Prints the :
 * 53 bits line reset  1
 * SWD select sequnce MSB
 * 53 bits LINE RESET 2
 * 3 empty bits*/
inline void printReset(void)
{

	HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin,1 );

	for (int i=0;i<53;i++)
		{

			//MANUAL FALLING EDGE
			// note to self: chagne to GPIOD->ODR later
			HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 0);

			HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 1);

		}


	for (int i=15;i>=0;i--)
		{

			//MANUAL FALLING EDGE
			// note to self: chagne to GPIOD->ODR later
			HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 1);

			HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin, (SWD_Select_Seq>>i)&0x01 );

			HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 0);
		}



	HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin,1 );
	for (int i=0;i<53;i++)
			{

				//MANUAL FALLING EDGE
				// note to self: chagne to GPIOD->ODR later

				HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 1);
				HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 0);



			}



	HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin,0 );
	for (int i=0;i<3;i++)
			{

				//MANUAL FALLING EDGE
				// note to self: chagne to GPIOD->ODR later

				HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 1);
				HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 0);



			}




	HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 1);














}

/**
 * @}
 */
