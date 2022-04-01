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



uint32_t response=0;
uint8_t ackMaster=0;
uint8_t bit=0;
uint32_t dataMaster=0;


void vMasterswd_Task(void * argument)
{


	notificationStruct notif;
	while(1)
	{
				xTaskNotifyWait(0, 0xffffffff, NULL, portMAX_DELAY);
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, 1);
				notif=masterNotif;

				switch(notif.type){

							case LINE_RESET_FULL:
							{
								printReset();
								break;
							}

							case REQUEST:
							{
								if ((notif.value & (1<<5)) !=0) //READ REQUEST
									{
									printRequest(notif.value);
									readAck();
									if (ackMaster==0x04) //ACK OK
										{
											readData();
											slaveNotif.type=DATA_FROM_MASTER;
											slaveNotif.value=dataMaster;
											xTaskNotify(swSlave_TaskHandle,0,eNoAction);
										}



									}
								else
									{

									}
							 break;
							}

							case ACK:
							{
								break;
							}

							case DATA_FROM_ISR:
							{
								//printData(notif.value);
								break;
							}
						}




		/*Increment TaskTick*/
		h_global.TaskTick.Swd_Master++;



		//osDelay(200);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, 0);

	}




}







/* Print an 8 bit request (rq) on  falling edges*/
inline void printRequest(uint32_t rq)
{
	swclk_set();
	for (int i=7;i>=0;i--)
	{

		swdio_Write(  (rq>>i)&0x01  );
		swclk_cycle();


//		GPIOD->ODR = ((GPIOD->ODR & ~(SWD_MASTER_DATA_Pin)) | ( ((rq>>i)&0x01) << 13)); // equivalent to :
//		//HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin, (rq>>i)&0x01 );
//
//		GPIOD->ODR|=SWD_MASTER_CLk_Pin; // 1
//
//
//
//		GPIOD->ODR&=~SWD_MASTER_CLk_Pin; // 0



		//MANUAL FALLING EDGE
		// note to self: chagne to GPIOD->ODR later

		/*HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 1);
		HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin, (rq>>i)&0x01 );

		HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 0);*/
	}


	/*turnarund*/

	HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin, 0); //



}

inline void readAck()
{

	   GPIOD->MODER&= ~(1<<26); // DATA PIN INPUT MODE

	    /*Turanrund*/
		swclk_cycle();

		/*ACK read*/
		ackMaster=0;
		swdio_Read(&ackMaster);
		swdio_Read(&ackMaster);
		swdio_Read(&ackMaster);


		GPIOD->MODER|=(1<<26); // DATA PIN OUTPUT MODE



		//HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 1);

}

inline void readData()
{
	GPIOD->MODER&= ~(1<<26); // DATA PIN INPUT MODE

	for (uint8_t i=0;i<32;i++)
	{
		swdio_Read(&dataMaster);
	}

	uint8_t parity;
	swdio_Read(&parity);


	GPIOD->MODER|=(1<<26); // DATA PIN OUTPUT MODE



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
		swclk_cycle();

		}


	for (int i=15;i>=0;i--)
		{

			//MANUAL FALLING EDGE
			// note to self: chagne to GPIOD->ODR later
		HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin, (SWD_Select_Seq>>i)&0x01 );
		swclk_cycle();

		}



	HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin,1 );
	for (int i=0;i<53;i++)
			{

				//MANUAL FALLING EDGE
				// note to self: chagne to GPIOD->ODR later

		swclk_cycle();

			}


	HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin,0 );
	for (int i=0;i<3;i++)
			{

				//MANUAL FALLING EDGE
				// note to self: chagne to GPIOD->ODR later

		swclk_cycle();


			}

	//HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin,1 );


	//HAL_GPIO_WritePin(GPIOD, SWD_MASTER_CLk_Pin, 1);


}

/*Enable SWDIO pin (PD13) as INPUT*/
inline void swdio_mode_input(){
	GPIOD->MODER&= ~(1<<26); // DATA PIN INPUT MODE

}

/*Enable SWDIO pin (PD13) as OUTOUT*/
inline void swdio_mode_output(){
	GPIOD->MODER|= (1<<26); // DATA PIN OuTPUT MODE

}

/*Reset SWCLK pin (PD12) to 0*/
inline void swclk_reset(){
	GPIOD->ODR&=~SWD_MASTER_CLk_Pin; // 0
}

/*Set SWCLK pin (PD12) to 1*/

inline void swclk_set(){
	GPIOD->ODR|=SWD_MASTER_CLk_Pin; // 1
}

/*Reset and Set the SWCLK pin (PD12)*/
inline void swclk_cycle(){

	// the order set and reset of the cycle can be inversed later
	swclk_reset();
	swclk_set();


}

inline void swdio_Write(uint8_t bit)
{
	GPIOD->ODR = ((GPIOD->ODR & ~(SWD_MASTER_DATA_Pin)) | ( (bit) << 13)); // equivalent to :
	//HAL_GPIO_WritePin(GPIOD, SWD_MASTER_DATA_Pin, bit );

}

inline void swdio_Read(uint32_t* response)
{
	swclk_reset();

	uint8_t bit = (GPIOD->IDR &SWD_MASTER_DATA_Pin) >> 13;
	*response=((*response)<<1)|bit;

	swclk_set();
}


/**
 * @}
 */
