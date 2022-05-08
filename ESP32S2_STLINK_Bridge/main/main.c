#include "freertos/FreeRTOS.h"
//#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "main.h"
#include "esp_attr.h"




static void IRAM_ATTR Slave_Clk_ISR_Handler(void* arg);
void uart_init(void);
void uart_callback(void);


//void wifi_init_softap(void);

uart_port_t uart_num = UART_NUM_1;

uart_isr_handle_t uart_isr_handle;

uint8_t data[50];
int length = 0;

void app_main(void){


    gpio_init();
 vCommontask_StartApk();
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM  ); //create isr for gpio

    //add ISR hander for SWD slave CLK pin
	gpio_set_level(SWD_MASTER_CLK_Pin,1);

    gpio_isr_handler_add(SWD_SLAVE_CLK_Pin, Slave_Clk_ISR_Handler, (void*)SWD_SLAVE_CLK_Pin);


    uart_init();




//    wifi_init_softap();



    while (1)
    {
    	length = uart_read_bytes(uart_num, data, 3, portMAX_DELAY);

        uart_write_bytes(uart_num, (const char*)data, length);
   // vTaskDelay(1000);


    }
}



static void IRAM_ATTR Slave_Clk_ISR_Handler(void* arg)

{
	//gpio_set_level(DEBUG_PIN_1, 1);
	Swd_SlaveStateMachineShifter();
	//gpio_set_level(DEBUG_PIN_1, 0);

}

void uart_init(void)
{

	    uart_config_t uart_config = {
	        .baud_rate = 115200,
	        .data_bits = UART_DATA_8_BITS,
	        .parity = UART_PARITY_DISABLE,
	        .stop_bits = UART_STOP_BITS_1,
	        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	        .rx_flow_ctrl_thresh = 122,
	    };
	    // Configure UART parameters
	 uart_param_config(uart_num, &uart_config);

	    uart_set_pin(UART_NUM_1, 20, 21, 26, 33);

	    // Setup UART buffered IO with event queue
	    const int uart_buffer_size = (1024 * 2);
	    QueueHandle_t uart_queue;
	    // Install UART driver using an event queue here
	    uart_driver_install(UART_NUM_1, uart_buffer_size,
	                                            uart_buffer_size, 10, &uart_queue, 0);



}

void uart_callback(void)
{
	gpio_set_level(DEBUG_PIN_1,1);



	gpio_set_level(DEBUG_PIN_1,0);


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
	        io_conf.mode = GPIO_MODE_INPUT;
	        io_conf.pin_bit_mask = (1<<SWD_SLAVE_DATA_Pin) ;
	        io_conf.pull_down_en = 1; //changed from 1
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
	        io_conf.mode = GPIO_MODE_OUTPUT;
	        io_conf.pin_bit_mask = (1<<SWD_MASTER_DATA_Pin) ;
	        io_conf.pull_down_en = 0; //changed from 1
	        io_conf.pull_up_en = 1; //changed from 0
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

