#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "main.h"


static void IRAM_ATTR Slave_Clk_ISR_Handler(void* arg);

void app_main(void){


    gpio_init();

    gpio_install_isr_service(0); //create isr for gpio

    //add ISR hander for SWD slave CLK pin
    gpio_isr_handler_add(SWD_SLAVE_CLK_Pin, Slave_Clk_ISR_Handler, (void*)SWD_SLAVE_CLK_Pin);



    while (1)
    {

    }
}



static void IRAM_ATTR Slave_Clk_ISR_Handler(void* arg)
{
	Swd_SlaveStateMachineShifter();
}

void gpio_init(void)
{


	    gpio_config_t io_conf = {};

	        /*GPIO PIN: SWD_SLAVE_CLK_Pin*/
	        io_conf.intr_type = GPIO_INTR_NEGEDGE;
	        io_conf.mode = GPIO_MODE_INPUT;
	        io_conf.pin_bit_mask = (1<<SWD_SLAVE_CLK_Pin) ;
	        io_conf.pull_down_en = 0;
	        io_conf.pull_up_en = 0;
	        gpio_config(&io_conf);


		    /*GPIO PIN: SWD_SLAVE_DATA_Pin*/
	        io_conf.intr_type = GPIO_INTR_DISABLE;
	        io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
	        io_conf.pin_bit_mask = (1<<SWD_SLAVE_DATA_Pin) ;
	        io_conf.pull_down_en = 1;
	        io_conf.pull_up_en = 0;
	        gpio_config(&io_conf);


		    /*GPIO PIN: SWD_MASTER_CLK_Pin*/
	        io_conf.intr_type = GPIO_INTR_DISABLE;
	        io_conf.mode = GPIO_MODE_OUTPUT;
	        io_conf.pin_bit_mask = (1<<SWD_MASTER_CLK_Pin) ;
	        io_conf.pull_down_en = 0;
	        io_conf.pull_up_en = 0;
	        gpio_config(&io_conf);


	        /*GPIO PIN: SWD_MASTER_DATA_Pin*/
	        io_conf.intr_type = GPIO_INTR_DISABLE;
	        io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
	        io_conf.pin_bit_mask = (1<<SWD_MASTER_DATA_Pin) ;
	        io_conf.pull_down_en = 1;
	        io_conf.pull_up_en = 0;
	        gpio_config(&io_conf);

	        /*DEBUG_PIN_1*/
	        io_conf.intr_type = GPIO_INTR_DISABLE;
	        io_conf.mode = GPIO_MODE_OUTPUT;
	        io_conf.pin_bit_mask = (1<<DEBUG_PIN_1) ;
	        io_conf.pull_down_en = 0;
	        io_conf.pull_up_en = 0;
	        gpio_config(&io_conf);

	        /*DEBUG_PIN_2*/
	        io_conf.intr_type = GPIO_INTR_DISABLE;
	        io_conf.mode = GPIO_MODE_OUTPUT;
	        io_conf.pin_bit_mask = (1<<DEBUG_PIN_2) ;
	        io_conf.pull_down_en = 0;
	        io_conf.pull_up_en = 0;
	        gpio_config(&io_conf);

	        /*DEBUG_PIN_3*/
	        io_conf.intr_type = GPIO_INTR_DISABLE;
	        io_conf.mode = GPIO_MODE_OUTPUT;
	        io_conf.pin_bit_mask = (1<<DEBUG_PIN_3) ;
	        io_conf.pull_down_en = 0;
	        io_conf.pull_up_en = 0;
	        gpio_config(&io_conf);


}

