/**
  ******************************************************************************
  * @file           : cm4_gateway_globalerror.c
  * @brief          : contain the project error entry point
  ******************************************************************************
  */
/*---------------------------------------------------------------------------------
 INCLUDES
-----------------------------------------------------------------------------------*/
#include "main.h"


/**
  * @brief  Global error function which will manage all error code from iCare System
  * @param  errorCode
  * @retval None
  *
*/
void GlobalError(uint16_t errorCode)
{

  /*Set Network Led*/
   //LED_RED_ON;


  switch (errorCode)
  {
    case TASK_CREATION_FAIL_ :
    {
      /*iCarePlus task is not created */
    	for(;;);
      break;
    }

    case QUEUE_CREATION_FAIL_:
    {
      	for(;;);

        break;
    }


    case SYSTEM_CLOCK_FAIL_:
    {
        /*Continuation Bip-------*/

    	for(;;)
    	  {

    	  }

    	break;
    }

    case PERIPH_INIT_FAIL_:
    {
        /*Issue from systemInit peripheral */
    	
    	for(;;);
    	break;
    }


    case SYSTEM_FAULT_:
    {
#ifdef RELEASE
    	/*issue from hard fault and system fault */
     	(void)HAL_NVIC_SystemReset();
#else
    	for(;;);
#endif

    	break;
    }





    case OVERSTACK_FAIL_:
    {
#ifdef RELEASE
    	/*issue from hard fault and system fault */
     	(void)HAL_NVIC_SystemReset();
#else
    	for(;;);
#endif

    	break;

    }



    default:
    {
    	for(;;){};
    	break;
    }




  }
}
