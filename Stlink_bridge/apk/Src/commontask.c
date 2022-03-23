/**
  ******************************************************************************
  * @file           : Commontask.c
  * @brief          : contain the project error entry point
  ******************************************************************************
  */
/*---------------------------------------------------------------------------------
 INCLUDES
-----------------------------------------------------------------------------------*/
#include "main.h"

TaskHandle_t swMaster_TaskHandle;
TaskHandle_t swSlave_TaskHandle;


/*Event Flag*/
EventGroupHandle_t xFlagSlave;

/*Queue Definition*/
QueueHandle_t xQueueSlave;

globalHandlerTypeDef  h_global;

static void InitHandler(void);

/** @defgroup CommonTsk COMMON START TASK
  * @ingroup TaskModule
 * @brief Create & Start Task
 * @{
 */




/**
  * @brief  vCommontask_StartApk: Creat and Start All Task
  * @param  None
  * @retval None
  *
*/
void vCommontask_StartApk(void )
{

	BaseType_t xReturned;


    /*Init global handler*/
	InitHandler();

	/* Creat Supervisor Task */
   xReturned = xTaskCreate(vSlaveswd_Task,"SLAVE",200U ,NULL,(tskIDLE_PRIORITY+4U), &swSlave_TaskHandle );

	if (xReturned!=pdPASS )
	{
		GlobalError(TASK_CREATION_FAIL_);
	}

   xReturned = xTaskCreate(vMasterswd_Task,"MASTER",200U ,NULL,(tskIDLE_PRIORITY+4U), &swMaster_TaskHandle );

	if (xReturned!=pdPASS )
	{
		GlobalError(TASK_CREATION_FAIL_);
	}



	/*System Event Flag used for (managing RX_READY / TX...) */
	xFlagSlave= xEventGroupCreate();

	/* Was the event group created successfully? */
	if( xFlagSlave == NULL )
	{
		/* The event group was not created because there was insufficient
    FreeRTOS heap available. */
		GlobalError(QUEUE_CREATION_FAIL_);
	}

	/*Create Queue system------------------------------------------- */
	xQueueSlave     = xQueueCreate( 4U, sizeof( uint8_t ) );

	if (xQueueSlave==NULL)
	{
		GlobalError(QUEUE_CREATION_FAIL_);
	}


	  /* Start scheduler */
	  osKernelStart();
}
/**
 * @}
 */
/**
  * @brief  Init global Handler
  * @param  None
  * @retval None
  *
*/
static void InitHandler(void)
{

#ifdef NUCLEO_BOARD
	h_global.Board=NUCLEO_BOARD_;
#else
   h_global.Board=PROTOTYPE_A;
#endif


#ifdef DEBUG
   h_global.sw_Config=ICARE_DEBUG_SW;
#endif
#ifdef RELEASE
   h_global.sw_Config=ICARE_RELEASE_SW;
#endif


   h_global.FirmwareVer=FW_VER;



}

/*Flag Mangaement function ---------------------*/
/**
 * @brief  SetSystemFlag: System Flags for EventSystem Flag object
 * @param  Flag :ABORT_REQUEST_FLAG/ABORT_ISDONE_FLAG/DOOR_ISOPENED_SYS_FLAG/DOOR_ISOPENED_FRONTP_FLAG
 * @retval None
 *
 */
uint8_t SetEventFlag(const EventGroupHandle_t xEventGroup, uint32_t Flag,uint8_t isr)
{
	EventBits_t uxBits;
	BaseType_t xHigherPriorityTaskWoken;
	if (isr==1)
	{
		   xEventGroupSetBitsFromISR(  xEventGroup,   /* The event group being updated. */
				                       Flag, /* The bits being set. */
		                              &xHigherPriorityTaskWoken );
		return 0;
	}
	else
	{

	/* Set Flag in xFlagSystem. */
	uxBits = xEventGroupSetBits(xEventGroup,    /* The event group being updated. */
			Flag );/* The bits being set. */

	if ((uxBits & Flag)!=Flag)
	{
		/*the Flag are not correctly set*/
		return 1U;

	}
	else
	{
		/*the Flag are correctly set*/
		return 0U;
	}
	}
}
/**
 * @brief  GetEventFlag: System Flags for EventSystem Flag object
 * @param  Flag :ABORT_REQUEST_FLAG/ABORT_ISDONE_FLAG/DOOR_ISOPENED_SYS_FLAG/DOOR_ISOPENED_FRONTP_FLAG
 * @retval 1: if flag is set 0: flag not yet set
 *
 */
uint8_t GetEventFlag(const EventGroupHandle_t xEventGroup, uint32_t Flag)

{
	EventBits_t uxBits;

	/*Get Flag in xFlagSystem. */
	uxBits = xEventGroupGetBits(xEventGroup);


	if ((uxBits & Flag)==Flag)
	{
		/*the Flag are  set*/
		return 1U;

	}
	else
	{
		/*the Flag are not yet set */
		return 0U;
	}

}
/**
 * @brief  WaitSystemFlag: wait dedicated System Flags, then clear the flag if it is set
 * @param  Flag :ABORT_REQUEST_FLAG/ABORT_ISDONE_FLAG/DOOR_ISOPENED_SYS_FLAG/DOOR_ISOPENED_FRONTP_FLAG
 * @retval 1: Timeout / 0: flag are set
 *
 */
uint8_t WaitEventFlag(const EventGroupHandle_t xEventGroup, uint32_t Flag, TickType_t xTicksToWait )
{
	EventBits_t uxBits;

	/*Wait end clear bit */
	uxBits = xEventGroupWaitBits(
			xEventGroup,   /* The event group being tested. */
			Flag,          /* The bits within the event group to wait for. */
			pdTRUE,        /* Flag bit should be cleared before returning. */
			pdTRUE ,       /*  wait for all bit */
			xTicksToWait );/* timeout */

	if( ( uxBits &  Flag ) == Flag )
	{
		/* xEventGroupWaitBits() returned because both bits were set. */

		uxBits = xEventGroupGetBits(xEventGroup);
		return 1U;
	}
	else
	{
		return 0U;
	}
}


