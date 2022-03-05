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

#define SWD_Select_Seq 			0x79E7




/**
  * @brief  Swd_SlaveStateMachineShifter  FROM ISR()
  * @param  None
  * @retval None
  *
*/


/*uint32_t clockCounter;
uint16_t buffer[50];
uint8_t bitPlacement = 15;
uint8_t bufferCtr = 0;*/

uint8_t JTAGtoSWDSwitchFlag=0;

SlaveStateTypeDef State=SWD_SLAVE_WAIT_FOR_START;

	uint16_t nbBitsRst1=0;
	uint16_t SWDseq=0;
	uint8_t seqCounter=0;

void Swd_SlaveStateMachineShifter(void)
{

	//SlaveStateTypeDef State;
	//clockCounter++;
	/*if(bufferCtr<50)
	{
		pinState=HAL_GPIO_ReadPin(SWD_SLAVE_DATA_GPIO_Port, SWD_SLAVE_DATA_Pin);
		if (pinState==0 && JTAGtoSWDSwitchFlag==0)
		{		JTAGtoSWDSwitchFlag=1;
				bufferCtr++;
				bitPlacement=15;

		}

		buffer[bufferCtr] |= (pinState<<bitPlacement);
		bitPlacement--;
		if(255U == bitPlacement)
		{
			bitPlacement = 15;
			bufferCtr++;
		}
	}*/


	GPIO_PinState pinState=HAL_GPIO_ReadPin(SWD_SLAVE_DATA_GPIO_Port, SWD_SLAVE_DATA_Pin);

	switch (State)
	{
	  case SWD_SLAVE_WAIT_FOR_START:
	  	  {
	  	    if(pinState == 1)
	  	    {
	  	    	State=SWD_LINE_RESET_1;
	  	    	nbBitsRst1=1;
	  	    }
	  		  break;
	  	  }

	  case SWD_LINE_RESET_1:
	  	  {
	  		  if(pinState ==1)
	  			  nbBitsRst1++;
	  		  else if (pinState==0)
	  		  {
	  			if(nbBitsRst1>=50)
	  				{State=JTAG_TO_SWD_SWITCH; seqCounter=1;}
	  			else
	  				State = unexpected_error_handler(SWD_LINE_RESET_1);
	  		  }

	  		break;
	  	  }
	  case SWD_LINE_RESET_2:
	  	  {
	  		break;
	  	  }
	  case JTAG_TO_SWD_SWITCH:
	  	  {
	  		  SWDseq=(SWDseq<<1)|pinState;
	  		  seqCounter++;
	  		  if(SWDseq == SWD_Select_Seq)
	  			  State=SWD_LINE_RESET_2;
	  		  else
	  			  if (seqCounter==16)
	  				  State = unexpected_error_handler(JTAG_TO_SWD_SWITCH);

	  		break;
	  	  }
	  case SWD_WAIT_FOR_REQUEST:
	  	  {
	  		break;
	  	  }
	  case SWD_REQUEST:
	  	  {
	  		break;
	  	  }
	  case SWD_ACKNOWLEDGE:
	  	  {
	  		break;
	  	  }
	  case SWD_DATA_TRANSFER:
	  	  {
	  		break;
	  	  }
	  case SWD_TURNAROUND_RQ_ACK:
	  	  	  {
	  	  		break;
	  	  	  }
	  case SWD_TURNAROUND_ACK_RQ:
	  	  	  {
	  	  		break;
	  	  	  }
	  case SWD_TURNAROUND_ACK_DAT:
	  	  	  {
	  	  		break;
	  	  	  }
	  case SWD_TURNAROUND_DAT_RQ:
	  	  	  {
	  	  		break;
	  	  	  }



/*

	  case SWD_SLAVE_WAIT_FOR_END:
	  {
		break;
	  }

*/




	}





     /**/
	//SetEventFlag(xFlagSlave,RX_DATA_READY, 1U)


}

SlaveStateTypeDef unexpected_error_handler(SlaveStateTypeDef State){

	/*
	 *
	 *
	 *
	 */
	return SWD_SLAVE_WAIT_FOR_START;

}





/**
 * @}
 */
