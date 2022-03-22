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
void vSlaveswd_Task(void *argumen0t)
{
	while (1)
	{
		/*Increment TaskTick*/
		h_global.TaskTick.Swd_Slave++;

		osDelay(200);

	}
}

/*Create FLAG */
#define RX_DATA_READY 1 << 1 U;

#define RX_DATA_BSY   1 << 2 U;

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

uint8_t retAckWait=0;

RequestTypeDef request;

//for debug

SlaveStateTypeDef StateBuffer[1000];
uint32_t dataBuffer[1000];
uint8_t ackBuffer[1000];
uint8_t requestBuffer[1000];

uint32_t stateBufferCounter = 0;
uint32_t dataBufferCounter = 0;
uint32_t ackBufferCounter = 0;
uint32_t requestBufferCounter = 0;

SlaveStateTypeDef oldState = SWD_SLAVE_WAIT_FOR_START;



void Swd_SlaveStateMachineShifter(void)
{
	pinState = (GPIOE->IDR &0x0200) >> 9;

	switch (State)
	{

		case SWD_SELF_ACK_WAIT:
		{

			GPIOE->MODER|=(1<<18);

			switch(retAckWait)
			{
			case 2:   {GPIOE -> ODR |= SWD_SLAVE_DATA_Pin; break;}
			default: { GPIOE -> ODR &= ~SWD_SLAVE_DATA_Pin; break;}


			}
			GPIOE->MODER|=(1<<18);
			changeEdgeTrigger(FALLING);
			GPIOE->MODER&= ~(1<<18);

			State = SWD_ACKNOWLEDGE;


			break;
		}

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
					nbBitsRst1++;
				// if pinState is 0 check if if consecutive reset bits >=50 or not
				else if (pinState == 0)
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

				break;
			}

		case SWD_LINE_RESET_2:
			{
				if (pinState == 1)
					nbBitsRst2++;
				else if (pinState == 0)
				{
					if (SWDorJTAG == SWD && nbBitsRst2 >= 50)
					{
						State = SWD_WAIT_FOR_REQUEST;
						nbBitsRst2 = 0;
					}
					else if (SWDorJTAG == JTAG && nbBitsRst2 >= 5)
					{
						State = SWD_WAIT_FOR_REQUEST;
						nbBitsRst2 = 0;
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
				//add new bit to rq
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

					}
					else
					{
						//requestBuffer[requestBufferCounter++] = rq;	// DEBUG buffer
						// requestBufferCounter++;
						requestParser(rq, &request);	// parse the bits of rq
						rq = 0;
						requestBitCounter = 0;

						//State = SWD_TURNAROUND_RQ_ACK;
						retAckWait=1;
						State= SWD_SELF_ACK_WAIT;
					}
				}

				break;
			}

		case SWD_ACKNOWLEDGE:
			{

				// add new acknowledge bits to ack
				ack = (ack << 1) | pinState;
				ackCounter++;

				if (ackCounter<3){
					changeEdgeTrigger(RISING);
					retAckWait++;
					State=SWD_SELF_ACK_WAIT;
				}


				else if (ackCounter == 3)
				{ retAckWait=0;
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


				break;
			}

		case SWD_DATA_TRANSFER:
			{

				if (dataCounter == 32)
				{
					//dataBuffer[dataBufferCounter++]=data;
					dataParity = pinState;
					if (request.RnW == READ)
						State = SWD_TURNAROUND_DAT_RQ;
					else
						State = SWD_REQUEST;

					dataCounter = 0;
					data = 0;
				}
				else
				{
					data = (data << 1) | pinState;
					dataCounter++;

				}


				break;
			}

		case SWD_TURNAROUND_RQ_ACK:	// Switch trigger on next rising edge to FALLING
			{

				changeEdgeTrigger(FALLING);
				State = SWD_ACKNOWLEDGE;


				break;
			}

		case SWD_TURNAROUND_DAT_RQ:	// if  READ request : Switch trigger and skip next RISING edge
			{

				State = SwitchToRisingAndSkipEdge(RISING, SWD_TURNAROUND_DAT_RQ, SWD_REQUEST); // chang


				break;
			}

		case SWD_TURNAROUND_ACK_RQ:	// if ACK WAIT : Switch trigger to RISING, and skip next RISING edge
			{

				State = SwitchToRisingAndSkipEdge(RISING, SWD_TURNAROUND_ACK_RQ, SWD_REQUEST);


				break;
			}

		case SWD_TURNAROUND_ACK_DAT:	// if  WRITE request : Switch trigger to v, and skip next RISING edge
			{

				State = SwitchToRisingAndSkipEdge(RISING, SWD_TURNAROUND_ACK_DAT, SWD_DATA_TRANSFER);


				break;
			}
	}


    if (State != oldState) {
      StateBuffer[stateBufferCounter++]=State;
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
	return SWD_SLAVE_WAIT_FOR_START;

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

/**
 *@}

 */
