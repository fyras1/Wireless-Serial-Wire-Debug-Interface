/**
 ******************************************************************************
 *@file           : serial wire debug interface
 *@brief          : contain the project error entry point
 ******************************************************************************
 */
/*---------------------------------------------------------------------------------
 INCLUDES
-----------------------------------------------------------------------------------*/
#include "main.h"
#include "swd_slavetask.h"

/**@defgroup SupervTask SERIAL WIRE DEBUG INTERFACE
 *@ingroup TaskModule
 *@brief Supervisor Task
 * @{
 */

/**
 *@brief  SuperVisor Task
 *@param  None
 *@retval None
 *
 */

uint8_t dataReceived=0;
uint8_t errFlagSlave=0;
void vSlaveswd_Task(void *argumen0t)
{
	notificationStruct notif;

	while (1)
	{

		xTaskNotifyWait(0x00, 0xffffffff, NULL, portMAX_DELAY);


		//HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);
		notif=slaveNotif;



		switch(notif.type){
			case REQUEST:
			{
			    masterNotif=notif;
				xTaskNotify(swMaster_TaskHandle,0,eNoAction);
			 break;
			}

			case ACK:
			{
				break;
			}

			case DATA_FROM_ISR:
			{
			    masterNotif=notif;
				xTaskNotify(swMaster_TaskHandle,0,eNoAction);
				break;
			}

			case DATA_FROM_MASTER:
			{
				dataReceived=1;
				break;
			}

			case LINE_RESET_FULL:
			{
			    masterNotif=notif;
				xTaskNotify(swMaster_TaskHandle,0,eNoAction);
				break;
			}
		}

		/*Increment TaskTick*/
		h_global.TaskTick.Swd_Slave++;
	//    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);
		//osDelay(200);


	}

}

/*Create FLAG */
#define RX_DATA_READY 1 << 1U;

#define RX_DATA_BSY   1 << 2U;

#define SWD_Select_Seq 0x79E7
#define JTAG_Select_Seq 0x3CE7

#define WRITE 			0
#define READ 			1

#define DP				0
#define AP 				1

#define FALLING 		0
#define RISING 			1

#define SWD 			0
#define JTAG 			1

/**
 *@brief  Swd_SlaveStateMachineShifter  FROM ISR()
 *@param  None
 *@retval None
 *
 */

/*uint32_t clockCounter;
uint16_t buffer[50];
uint8_t bitPlacement = 15;
uint8_t bufferCtr = 0; */

uint8_t JTAGtoSWDSwitchFlag = 0;

SlaveStateTypeDef State = SWD_SLAVE_WAIT_FOR_START;

uint32_t nbBitsRst1 = 0;
uint32_t nbBitsRst2 = 0;
uint16_t SWDseq = 0;
uint8_t seqCounter = 0;
uint32_t EXTIcounter = 0;
uint32_t diffCounter = 0;
uint8_t rq = 0;
uint8_t ack = 0;
uint8_t ackCounter = 0;
uint8_t turnAroundSkipCounter = 0;
uint32_t data = 0;
uint32_t dataCounter = 0;
uint8_t dataParity = 0;
uint8_t SWDorJTAG = 0;
uint8_t requestBitCounter = 0;
uint8_t pinState;

uint8_t data_parity=0;

uint8_t retAckWait=0;
uint8_t retAckOk=0;

uint8_t saveData=0;

uint8_t requestPending=0;



RequestTypeDef request;

//for debug

SlaveStateTypeDef StateBuffer[1000];
//uint32_t dataBuffer[1000];
//uint8_t ackBuffer[1000];
//uint8_t requestBuffer[1000];
//
uint32_t stateBufferCounter = 0;
//uint32_t dataBufferCounter = 0;
//uint32_t ackBufferCounter = 0;
//uint32_t requestBufferCounter = 0;

uint32_t nbexti=0;

SlaveStateTypeDef oldState = SWD_SLAVE_WAIT_FOR_START;



void Swd_SlaveStateMachineShifter(void)
{
	pinState = (GPIOE->IDR &0x0200) >> 9;



	nbexti++;

	switch (State)
	{



		case SWD_SLAVE_WAIT_FOR_START:
			{
				//wait for first bit to start line reset
				if (pinState == 1)
				{
					State = SWD_LINE_RESET_1;
					nbBitsRst1 = 1;
				}

				break;
			}

		case SWD_LINE_RESET_1:
			{
				//increase reset 1 bits if 1

				if (pinState == 1)
					{
					nbBitsRst1++;
					}
				// if pinState is 0 check if if consecutive reset bits >=50 or not
				else //if (pinState == 0)
				{
					if (nbBitsRst1 >= 50)
					{
						State = SWD_JTAG_SELECT;
						seqCounter = 1;
						nbBitsRst1 = 0;
					}
					else
						//generic state machine error handler call to return to a preivous state (SWD_SLAVE_WAIT_FOR_START);
						State = unexpected_error_handler(SWD_LINE_RESET_1);
				}

				break;
			}

		case SWD_JTAG_SELECT:
			{

				SWDseq = (SWDseq << 1) | pinState;
				seqCounter++;
				if (SWDseq == SWD_Select_Seq)
				{
					SWDorJTAG = SWD;
					State = SWD_LINE_RESET_2;
					SWDseq = 0;
				}
				else if (SWDseq == JTAG_Select_Seq)
				{
					SWDorJTAG = JTAG;
					State = SWD_LINE_RESET_2;
					SWDseq = 0;
				}
				else if (seqCounter > 16)	// should never come here
					State = unexpected_error_handler(SWD_JTAG_SELECT);
				else
				{

				}
				break;
			}

		case SWD_LINE_RESET_2:
			{
				if (pinState == 1)
					nbBitsRst2++;
				else //if (pinState == 0)
				{
					if ((SWDorJTAG == SWD && nbBitsRst2 >= 50) || (SWDorJTAG == JTAG && nbBitsRst2 >= 5))
					{
						State = SWD_WAIT_FOR_REQUEST;
						nbBitsRst2 = 0;
						sendNotif(LINE_RESET_FULL,0,0 , &swSlave_TaskHandle);
						requestPending=0;

					}

					else
						State = unexpected_error_handler(SWD_LINE_RESET_2);
				}

				break;
			}

		case SWD_WAIT_FOR_REQUEST:
			{
				if (pinState == 1)
				{
					State = SWD_REQUEST;
					rq = 1;
					requestBitCounter = 1;
				}
				else
				{
					//do nothing and wait for next edge
				}

				break;
			}

		case SWD_REQUEST:
			{
//				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);
//				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);

				//add new bit to rq

				if(!requestBitCounter)
					rq=0;

				rq = (rq << 1) | pinState;
				requestBitCounter++;

				//when request is filled (8 bits)
				if (requestBitCounter == 8)
				{
					if (rq == 0xff)
					{
						// an 8 bit sequence of 0b11111111 = start of a line reset 1
						State = SWD_LINE_RESET_1;
						nbBitsRst1 = 8;
						break;

					}
					else
					{
						//requestBuffer[requestBufferCounter++] = rq;	// DEBUG buffer
						// requestBufferCounter++;
						requestParser(rq, &request);	// parse the bits of rq

						//requestBuffer[requestBufferCounter++]=rq;

							State = SWD_TURNAROUND_RQ_ACK;
						requestBitCounter = 0;



					 if (request.RnW == READ){
						if (dataReceived==1){
							requestPending=0;
							retAckOk=1; // when data returned from master , write it on swdio to st-link
						}
						else{
							retAckWait=1; // if no data returned from master, send ack wait and repeat
							if (!requestPending)
								{
								sendNotif( REQUEST, rq,0,  &swSlave_TaskHandle ); // NOTE TO SELF: change later to run only once
								requestPending=1;
								}
						}
					 }

					 else
					 {
						 if (requestPending==1)
							 {
							 retAckWait=1; // if a WRITE request and
							 }
						 else
							 { retAckOk=1;
							   saveData=1;
							 }
					 }

					}
				}

				break;
			}

/********** Acknowledge state ************/
		case SWD_ACKNOWLEDGE:
			{

			//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, 1);

				if (retAckWait) //inside ack wait (return ack wait)
							{
							switch(retAckWait) //0 -> 1 -> 0
								{
								case 2:   {GPIOE -> ODR |= SWD_SLAVE_DATA_Pin; break;}
								default: { GPIOE -> ODR &= ~SWD_SLAVE_DATA_Pin; break;}
								}
							}
				else
							{
							switch(retAckOk) //1 -> 0 -> 0
								{
								case 1:   {GPIOE -> ODR |= SWD_SLAVE_DATA_Pin; break;}
								default: { GPIOE -> ODR &= ~SWD_SLAVE_DATA_Pin; break;}
								}
							}

			//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, 0);





							//changeEdgeTrigger(FALLING);


				// add new acknowledge bits to ack

				//GPIOE->MODER&= ~(1<<18); //added last night
				pinState = (GPIOE->IDR &0x0200) >> 9;

				ack = (ack << 1) | pinState;
				ackCounter++;

				if (ackCounter<3){


					//changeEdgeTrigger(RISING);

					(retAckWait!=0)?retAckWait++:retAckOk++; // only increase the variable that wasn't initially 0 ( has 1 from SWD_REQUEST)

//					retAckWait+=(1*retAckWait);
//					retAckOk+=(1*retAckOk);

				}


				else if (ackCounter == 3)
				{
					retAckWait=0;
					retAckOk=0;
					// ackBuffer[ackBufferCounter++]=ack;	// for DEBUG
					switch (ack)
					{
						case 0x4:	// ACK OK -- (0b001 LSB)
							{
								if (request.RnW == READ)
								{
									State = SWD_DATA_TRANSFER;
								}
								else
								{
									State = SWD_TURNAROUND_ACK_DAT;

								}

								break;

							}

						case 0x2:	// ACK WAIT -- (0b010 LSB)
							{
								State = SWD_TURNAROUND_ACK_RQ;
								break;
							}

						case 0x1:	//ACK FAULT -- (0b100 LSB)
							{
								State = unexpected_error_handler(SWD_ACKNOWLEDGE);
								break;
							}

						default: {}
					}

					ack = 0;
					ackCounter = 0;

				}

				else
				{
					//
				}


				break;
			}


			/********** Daata transfer state ************/

		case SWD_DATA_TRANSFER:
			{
				//data pin already in output mode from rq_ack_turnaournd
				/*condition is entered in case we have data from master (read request)*/
				if (dataReceived)
				{
					uint8_t bit = (slaveNotif.value1>>(31-dataCounter))&0x01;
					GPIOE->ODR = ((GPIOE->ODR & ~(SWD_SLAVE_DATA_Pin)) | ( (bit) << 9)); //write bit to swdio

					dataParity^=bit;

					//if (dataCounter==31) //we wrote the last bit of the data response
						//dataReceived=0;

				}

				/******/

				if (dataCounter == 32) //this condition is for reading the parity and for preparing the enxt state
				{

					if (dataReceived)
					{
						dataReceived=0;
						GPIOE->ODR = ((GPIOE->ODR & ~(SWD_SLAVE_DATA_Pin)) | ( (dataParity) << 9));

						dataParity=0;
					}
					//else
					//dataParity = pinState;

					/**********/
					/*Decides if the data will be sent*/
					if(saveData)
					{
						sendNotif(   DATA_FROM_ISR, rq , data, &swSlave_TaskHandle ); // NOTE TO SELF: change later to run only once
						saveData=0;
					}


					if (request.RnW == READ)
						State = SWD_TURNAROUND_DAT_RQ;
					else
						State = SWD_REQUEST;

					//dataBuffer[dataBufferCounter++]=data;

					dataCounter = 0;
					data = 0;
				}
				else
				{
					pinState = (GPIOE->IDR &0x0200) >> 9;
					data = (data << 1) | pinState;
					dataCounter++;

				}


				break;
			}

		case SWD_TURNAROUND_RQ_ACK:	// Switch trigger on next rising edge to FALLING
			{
				GPIOE->MODER|=(1<<18); //output mode
				changeEdgeTrigger(FALLING);
				State = SWD_ACKNOWLEDGE;


				break;
			}

		case SWD_TURNAROUND_DAT_RQ:	// if  READ request : Switch trigger and skip next RISING edge
			{
				GPIOE -> ODR |= SWD_SLAVE_DATA_Pin;
				GPIOE->MODER&= ~(1<<18); // DATA PIN INPUT MODE


				State = SwitchToRisingAndSkipEdge(RISING, SWD_TURNAROUND_DAT_RQ, SWD_REQUEST); // chang


				break;
			}

		case SWD_TURNAROUND_ACK_RQ:	// if ACK WAIT : Switch trigger to RISING, and skip next RISING edge
			{

//					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 1);
				//	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, 0);
				GPIOE -> ODR |= SWD_SLAVE_DATA_Pin;

				GPIOE->MODER&= ~(1<<18); // DATA PIN INPUT MODE

				State = SwitchToRisingAndSkipEdge(RISING, SWD_TURNAROUND_ACK_RQ, SWD_REQUEST);


				break;
			}

		case SWD_TURNAROUND_ACK_DAT:	// if  WRITE request : Switch trigger to v, and skip next RISING edge
			{
				GPIOE -> ODR |= SWD_SLAVE_DATA_Pin;
				GPIOE->MODER&= ~(1<<18); // DATA PIN INPUT MODE

				State = SwitchToRisingAndSkipEdge(RISING, SWD_TURNAROUND_ACK_DAT, SWD_DATA_TRANSFER);


				break;
			}
	}


    if (State != oldState || State==SWD_REQUEST) {
      StateBuffer[stateBufferCounter]=State;

      if (StateBuffer[stateBufferCounter] == 189 )
      {
    	  oldState = State;
      }
      stateBufferCounter++;
      oldState = State;
   }



	//SetEventFlag(xFlagSlave,RX_DATA_READY, 1U)

}

//in case of ACK FAULT or any other error, this funtion will return to a previous state
SlaveStateTypeDef unexpected_error_handler(SlaveStateTypeDef State)
{
	/*
	 *
	 *this will be implemented later
	 *
	 */
	return SWD_LINE_RESET_1;

}

void requestParser(uint8_t req, RequestTypeDef *request)
{
	request->Start = (req >> 7) &0x1;
	request->APnDP = (req >> 6) &0x1;
	request->RnW = (req >> 5) &0x1;
	request->A2 = (req >> 4) &0x1;
	request->A3 = (req >> 3) &0x1;
	request->Parity = (req >> 2) &0x1;
	request->Stop = (req >> 1) &0x1;
	request->Park = (req >> 0) &0x1;

}

inline void changeEdgeTrigger(uint8_t newEdge)
{
	//change EXTI edge trigger to newEdge value (FALLING 0 RISING 1)

	switch (newEdge)
	{
		case RISING:
			{
				EXTI->RTSR |= (1 << 11);	// enable Rising edge trigger
				EXTI->FTSR &= ~(1 << 11);	//disable Falling edge trigger for SWD_SLAVVE_CLK_Pin

				break;
			}

		case FALLING:
			{
				EXTI->FTSR |= (1 << 11);	//
				EXTI->RTSR &= ~(1 << 11);	//

				break;
			}
	}
}

/*
 *Switch EXTI edge to newEdge (FALLING,RISING)
 *and skip the next coming edge
 *and move to targetState
 */
inline SlaveStateTypeDef SwitchToRisingAndSkipEdge(uint8_t newEdge, SlaveStateTypeDef sourceState, SlaveStateTypeDef targetState)
{
	changeEdgeTrigger(RISING);
	if (turnAroundSkipCounter == 0)
	{
		turnAroundSkipCounter = 1;
		return sourceState;

	}
	else
	{
		turnAroundSkipCounter = 0;
		return targetState;
	}
}


inline void sendNotif( notifTypeTypedef notifType, uint32_t val1,uint32_t val2, TaskHandle_t *swSlave_TaskHandle ) {

			slaveNotif.value1=val1;
			slaveNotif.value2=val2;

			slaveNotif.type = notifType;

			BaseType_t xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken=pdFALSE;

			xTaskNotifyFromISR(*swSlave_TaskHandle,0,eNoAction,&xHigherPriorityTaskWoken);


			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
 *@}

 */
