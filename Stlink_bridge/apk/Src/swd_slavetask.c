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

#define WRITE					0
#define READ					1
#define W 						WRITE
#define R						READ

#define DP						0
#define AP						1

#define FALLING					0
#define RISING					1





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
	uint16_t nbBitsRst2=0;
	uint16_t SWDseq=0;
	uint8_t seqCounter=0;
	uint32_t EXTIcounter=0;
	uint32_t diffCounter=0;
	uint8_t rq=0;
	uint8_t ack=0;
	uint8_t ackCounter=0;
	uint8_t turnAroundSkipCounter=0;
	uint32_t data=0;
	uint32_t dataCounter=0;
	uint8_t dataParity=0;

	RequestTypeDef request;


	//for debug
	SlaveStateTypeDef StateBuffer[1000];
	uint32_t dataBuffer[1000];
	uint8_t ackBuffer[1000];
	uint8_t requestBuffer[1000];

	uint32_t stateBufferCounter=0;
	uint32_t dataBufferCounter=0;
	uint32_t ackBufferCounter=0;
	uint32_t requestBufferCounter=0;
	uint8_t requestBitCounter=0;

	uint8_t pinState;


	SlaveStateTypeDef oldState=SWD_SLAVE_WAIT_FOR_START;


void Swd_SlaveStateMachineShifter(void)
{

	//if(pinState){


	pinState= (GPIOE->IDR & 0x0200)>>9;
//	EXTIcounter++;

	//SlaveStateTypeDef State;
	//clockCounter++;


	//pinState=HAL_GPIO_ReadPin(SWD_SLAVE_DATA_GPIO_Port, SWD_SLAVE_DATA_Pin);
    //pinState= (GPIOE->IDR & 0x0200)>>9;

	//}



	switch (State)
	{
	  case SWD_SLAVE_WAIT_FOR_START:
	  	  { //wait for first bit to start line reset
	  	    if(pinState == 1)
	  	    {
	  	    	State=SWD_LINE_RESET_1;
	  	    	nbBitsRst1=1;
	  	    }
	  		  break;
	  	  }

	  case SWD_LINE_RESET_1:
	  	  {	//increase reset 1 bits if 1
	  		  if(pinState ==1)
	  			  nbBitsRst1++;
	  		  // if pinState is 0 check if if consecutive reset bits >=50 or not
	  		  else if (pinState==0)
	  		  {
	  			if(nbBitsRst1>=50)
	  				{State=JTAG_TO_SWD_SWITCH; seqCounter=1;}
	  			else
	  				//generic state machine error handler call to return to a preivous state (SWD_SLAVE_WAIT_FOR_START);
	  				State = unexpected_error_handler(SWD_LINE_RESET_1);
	  		  }

	  		break;
	  	  }
	  case SWD_LINE_RESET_2:
	  	  {
	  		 if(pinState ==1)
				  nbBitsRst2++;
			 else if (pinState==0)
				  {
					if(nbBitsRst2>=50)
						{State=SWD_WAIT_FOR_REQUEST;}
					else
						 State = unexpected_error_handler(SWD_LINE_RESET_2);
	  			  		  }
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

	  		  if (pinState==1){
	  			  State=SWD_REQUEST;
	  			  rq=1;
	  			  requestBitCounter=1;
	  		  }
	  		  else {
	  			  //do nothing and wait for next edge
	  		  }

	  		break;
	  	  }
	  case SWD_REQUEST:
	  	  {
	  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1); //*************DEBUG START***************

	  		  //add new bit to rq
	  		  rq=(rq<<1)|pinState;
	  		  requestBitCounter++;
	  		  //when request is filled (8 bits)
	  		  if (requestBitCounter == 8){
	  			//requestBuffer[requestBufferCounter++] = rq;
	  			requestBufferCounter++;
	  			  requestParser(rq,&request);
	  			  rq=0;
	  			  requestBitCounter=0;
	  			  State= SWD_TURNAROUND_RQ_ACK;

	  		  }

		  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0); //*********DEBUG END ******************
	  		break;
	  	  }
	  case SWD_ACKNOWLEDGE:
	  	  {
	  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1); //*************DEBUG START***************

	  		ack=(ack<<1)|pinState;
	  		  ackCounter++;
	  		  if (ackCounter==3){
	  			 //ackBuffer[ackBufferCounter++]=ack;
	  			  if (ack==0x4) // ACK OK -- (0b001 LSB)
	  			  {
	  				  if (request.RnW == READ )
	  					  State= SWD_DATA_TRANSFER;
	  				  else
	  				  {
	  					  State= SWD_TURNAROUND_ACK_DAT;

	  				  }

	  			  }
	  			  else if (ack == 0x2) // ACK WAIT -- (0b010 LSB)
	  			  {
	  				  	State=SWD_TURNAROUND_ACK_RQ;
	  			  }
	  			  else if (ack==0x1) //ACK FAULT -- (0b100 LSB)
	  			  {
	  				  	 State= unexpected_error_handler(SWD_ACKNOWLEDGE);
	  			  }
	  			  else
	  			  {

	  			  }

	  			  ack=0;
	  			  ackCounter=0;

	  		  }


		  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0); //*********DEBUG END ******************

	  		break;
	  	  }

	  case SWD_DATA_TRANSFER:
	  	  {
	  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1); //*************DEBUG START***************

	  		  if (dataCounter==32)
	  		  { //dataBuffer[dataBufferCounter++]=data;
	  			  dataParity=pinState;
	  			  if(request.RnW==READ)
	  				  State=SWD_TURNAROUND_DAT_RQ;
	  			  else
	  				  State=SWD_REQUEST;

	  			  dataCounter=0;
	  			  data=0;
	  		  }
	  		  else
	  		  {
	  		  data=(data<<1)|pinState;
	  		  dataCounter++;


	  		  }

		  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0); //*********DEBUG END ******************

	  			  break;
	  	  }


	  case SWD_TURNAROUND_RQ_ACK: // Switch trigger on next falling edge to RISING
	  	  	  {
	  	  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1); //*************DEBUG START***************

	  	  		changeEdgeTrigger(FALLING);
	  	  		State=SWD_ACKNOWLEDGE;

		  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0); //*********DEBUG END ******************

	  	  		break;
	  	  	  }


	  case SWD_TURNAROUND_ACK_RQ: // if ACK WAIT : Switch trigger to FALLING, and skip next falling edge
	  	  	  {

	  	  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1); //*************DEBUG START***************

	  	  		  State= switchAndSkipEdge(RISING,SWD_TURNAROUND_ACK_RQ,SWD_REQUEST);

			    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0); //*********DEBUG END ******************

	  	  		break;
	  	  	  }


	  case SWD_TURNAROUND_ACK_DAT: // if  WRITE request : Switch trigger to FALLING, and skip next falling edge
	  	  	  {
	  	  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1); //*************DEBUG START***************

	  	  		State= switchAndSkipEdge(RISING,SWD_TURNAROUND_ACK_DAT,SWD_DATA_TRANSFER);

		  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0); //*********DEBUG END ******************


	  	  		break;
	  	  	  }


	  case SWD_TURNAROUND_DAT_RQ: // if  READ request : Switch trigger and skip next falling edge
	  	  	  {
	  	  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1); //*************DEBUG START***************

	  	  		State= switchAndSkipEdge(RISING,SWD_TURNAROUND_DAT_RQ,SWD_REQUEST);

		  		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0); //*********DEBUG END ******************


	  	  		break;
	  	  	  }


	/*  case SWD_SLAVE_WAIT_FOR_END:
	  {
		break;
	  }
*/


	}



/*

 if( State!=oldState){

	StateBuffer[stateBufferCounter++]=State;
	//diffCounter++;
	oldState=State;
}
*/


     /**/
	//SetEventFlag(xFlagSlave,RX_DATA_READY, 1U)

	//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1); //*************DEBUG START***************
	//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0); //*********DEBUG END *******************/



}

//in case of ACK FAULT or any other error, this funtion will return to a previous state
 SlaveStateTypeDef unexpected_error_handler(SlaveStateTypeDef State){

	/*
	 *
	 * this will be implemented later
	 *
	 */
	return SWD_SLAVE_WAIT_FOR_START;

}

  void requestParser(uint8_t req,RequestTypeDef* request )
{
	request->Start  = (req>>7) & 0x1;
	request->APnDP  = (req>>6) & 0x1;
	request->RnW    = (req>>5) & 0x1;
	request->A2     = (req>>4) & 0x1;
	request->A3     = (req>>3) & 0x1;
	request->Parity = (req>>2) & 0x1;
	request->Stop   = (req>>1) & 0x1;
	request->Park   = (req>>0) & 0x1;

}

 void changeEdgeTrigger(uint8_t newEdge){ //change EXTI edge trigger to newEdge value (FALLING 0 RISING 1)


	if (newEdge==RISING)
	{
		EXTI->FTSR &=  ~(1<<11); //disable Falling edge trigger for SWD_SLAVVE_CLK_Pin
		EXTI->RTSR |= (1<<11);   // enable Rising edge trigger
	}
	else if (newEdge==FALLING)
	{
		EXTI->RTSR &=  ~(1<<11);   //
		EXTI->FTSR |= (1<<11);    //
	}

}
		/*
		 * Switch EXTI edge to newEdge (FALLING,RISING)
		 * and skip the next coming edge
		 * and move to targetState
		 */
	   SlaveStateTypeDef switchAndSkipEdge(uint8_t newEdge,SlaveStateTypeDef sourceState, SlaveStateTypeDef targetState)
	  {


		  turnAroundSkipCounter++;
		  	  	if (turnAroundSkipCounter==1)
		  	  		{
		  			 changeEdgeTrigger(newEdge);

		  	  		 return sourceState; //do one more edge on the same state;
		  	  		}
		  		 else
		  			{
		  			 	turnAroundSkipCounter=0;

		  				return targetState; // after skipping the second edge, return the new edge

		  			}

	  }







/**
 * @}
 */
