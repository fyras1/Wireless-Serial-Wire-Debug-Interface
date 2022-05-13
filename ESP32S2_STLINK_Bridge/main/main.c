#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include <esp_log.h>


#include "main.h"
#include "esp_attr.h"


#define PORT 8001

#define EXAMPLE_ESP_WIFI_SSID      "bridge_master_AP"
#define EXAMPLE_ESP_WIFI_PASS      "123456789"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10000

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

void uart_init(void);
void uart_callback(void);
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

void socketClient() ;

void wifi_init_sta(void);


//void wifi_init_softap(void);
static EventGroupHandle_t s_wifi_event_group;

uart_port_t uart_num = UART_NUM_1;

uart_isr_handle_t uart_isr_handle;

uint8_t rxData[10];
uint8_t txData[10];
int length = 0;

int sock;

static int s_retry_num = 0;



void app_main(void){


    gpio_init();



    uart_init();

   nvs_flash_init();


    wifi_init_sta();


    socketClient();

//    wifi_init_softap();



    while (1)
    {

    	length = uart_read_bytes(uart_num, rxData, 9, portMAX_DELAY);

        gpio_set_level(DEBUG_PIN_1, 1);
    	gpio_set_level(DEBUG_PIN_1, 0);


    	send(sock,rxData,9,0);

        gpio_set_level(DEBUG_PIN_1, 1);
   	    	gpio_set_level(DEBUG_PIN_1, 0);

    	    	//vTaskDelay(1); // TESTING ONLY

    	recv(sock,txData,9,0);

    	gpio_set_level(DEBUG_PIN_1, 1);
    	   	    	gpio_set_level(DEBUG_PIN_1, 0);
//    		gpio_set_level(DEBUG_PIN_2, 1);
//    	    	gpio_set_level(DEBUG_PIN_2, 0);


        uart_write_bytes(uart_num, (const char*)txData, length);

        	gpio_set_level(DEBUG_PIN_1, 1);
            	gpio_set_level(DEBUG_PIN_1, 0);
   // vTaskDelay(1000);


    }

}

    /***************************** Socket Client ********************/
    void socketClient() {
    	 sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    	struct sockaddr_in serverAddress;
    	serverAddress.sin_family = AF_INET;
    	inet_pton(AF_INET, "192.168.2.1", &serverAddress.sin_addr.s_addr); //change ip here 192.168.2.1
    	serverAddress.sin_port = htons(PORT); //CHAGE

    	 connect(sock, (struct sockaddr *)&serverAddress, sizeof(struct sockaddr_in));


//    	char *data = "Hello world";
//    	rc = send(sock, data, strlen(data), 0);
//
//
//    	rc = close(sock);
//
//    	vTaskDelete(NULL);
    }







/************************* UART INIT ***************************/

void uart_init(void)
{

	    uart_config_t uart_config = {
	        .baud_rate = 115200, //CHANGE
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

/************************* UART CALLBACK***************************/

void uart_callback(void)
{
	//gpio_set_level(DEBUG_PIN_1,1);



	//gpio_set_level(DEBUG_PIN_1,0);


}


/************************* WIFI STATION INIT ***************************/

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();

  esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
   esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,

        },
    };
  esp_wifi_set_mode(WIFI_MODE_STA) ;
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config) ;
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE); // added new - need test


    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);


//    if (bits & WIFI_CONNECTED_BIT) {
//
//    } else if (bits & WIFI_FAIL_BIT) {
//
//    } else {
//
//    }
}

/************************* WIFI EVENT HANDLER  ***************************/

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	gpio_set_level(DEBUG_PIN_2, 1);
	   	gpio_set_level(DEBUG_PIN_2, 0);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;

        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;

        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}



/************************* GPIO INIT ***************************/
void gpio_init(void)
{


	    gpio_config_t io_conf = {};

	        /*GPIO PIN: SWD_SLAVE_CLK_Pin*/
	        io_conf.intr_type = GPIO_INTR_DISABLE;
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
	        io_conf.mode = GPIO_MODE_OUTPUT_OD;
	        io_conf.pin_bit_mask = (1<<DEBUG_PIN_2) ;
	        io_conf.pull_down_en = 0;
	        io_conf.pull_up_en = 1;
	        gpio_config(&io_conf);

	        /*DEBUG_PIN_3*/
	        io_conf.intr_type = GPIO_INTR_DISABLE;
	        io_conf.mode = GPIO_MODE_OUTPUT;
	        io_conf.pin_bit_mask = (1<<DEBUG_PIN_3) ;
	        io_conf.pull_down_en = 0;
	        io_conf.pull_up_en = 0;
	        gpio_config(&io_conf);


}

