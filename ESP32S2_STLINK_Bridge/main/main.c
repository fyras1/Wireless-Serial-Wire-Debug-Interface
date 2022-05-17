#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "esp_private/wifi.h"

#include "esp_now.h"



#include "main.h"
#include "esp_attr.h"


#define PORT 8001

#define CONFIG_ESPNOW_CHANNEL				1
#define ESPNOW_WIFI_MODE 					WIFI_MODE_STA
#define ESPNOW_WIFI_IF 						ESP_IF_WIFI_STA
#define MAX_ESPNOW_PACKET_SIZE				250



#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1


typedef struct {
	uint8_t isReady;
	uint8_t previousSend;
	uint8_t currentData;

} esnowQueObject;



esnowQueObject globalQue;
long espnowTimers[3];
example_espnow_send_param_t *send_param_var;

static esp_err_t example_espnow_init(void);

void uart_init(void);
void uart_callback(void);
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);
static void initWifi(void);
void wifi_init(void);
void add_peer(void);
static void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

static void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);

void socketClient() ;

void wifi_init_sta(void);

static void sendESPNOWData(void *pvParameter);

char* TAG="esp bridge ";
//void wifi_init_softap(void);
static EventGroupHandle_t s_wifi_event_group;

uart_port_t uart_num = UART_NUM_1;

uart_isr_handle_t uart_isr_handle;

uint8_t rxData[10];
uint8_t txData[10]={0x04,0,0,0x79,0xE7,0,0,0,0};
int length = 0;

int sock;

int a;

static int s_retry_num = 0;

uint8_t espnow_recv=0;

esp_now_peer_info_t peer;
static uint8_t peer_mac_addr[6] = { 0x7C, 0xDF, 0xA1, 0x54, 0x5C, 0x6C }; //BACLK BASE MAC: 7c:df:a1:54:5c:6c


void app_main(void){


    gpio_init();



    uart_init();

   nvs_flash_init();


    //wifi_init_sta();

 //  wifi_init();
   initWifi();
  // esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_AP, 1, WIFI_PHY_RATE_MCS7_SGI);
   example_espnow_init();

 /*  esp_now_init() ;
  esp_now_register_send_cb(send_cb) ;
   esp_now_register_recv_cb(recv_cb) ;

   add_peer();*/



   // socketClient();

 //  while(1){};

//    wifi_init_softap();



   // while (1)
   // {

    //	uart_read_bytes(uart_num, rxData, 9, portMAX_DELAY);

      //  gpio_set_level(DEBUG_PIN_1, 1);
    	//gpio_set_level(DEBUG_PIN_1, 0);


    	esp_now_send(peer.peer_addr, txData, 9);

    	  // while(!espnow_recv){} //wait until recv_cb is called

    	   espnow_recv=0;
    //}

    	//send(sock,rxData,9,0);

       // gpio_set_level(DEBUG_PIN_1, 1);
   	    //	gpio_set_level(DEBUG_PIN_1, 0);

    	    	//vTaskDelay(1); // TESTING ONLY

    	//recv(sock,txData,9,0);

    	//gpio_set_level(DEBUG_PIN_1, 1);
    	  // 	    	gpio_set_level(DEBUG_PIN_1, 0);
//    		gpio_set_level(DEBUG_PIN_2, 1);
//    	    	gpio_set_level(DEBUG_PIN_2, 0);


     //   uart_write_bytes(uart_num, (const char*)txData, length);

        //	gpio_set_level(DEBUG_PIN_1, 1);
           // 	gpio_set_level(DEBUG_PIN_1, 0);
   // vTaskDelay(1000);




}


/**********esp now init***********/
static esp_err_t example_espnow_init(void)
{
    example_espnow_send_param_t *send_param;

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(recv_cb) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    peer->channel = CONFIG_ESPNOW_CHANNEL;
    peer->ifidx = ESPNOW_WIFI_IF;
    peer->encrypt = false;
    memcpy(peer->peer_addr, peer_mac_addr, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK( esp_now_add_peer(peer) );
    //free(peer);

	//Add espnow node to reply to

	printf("Peer Added!\r\n");


    /* Initialize sending parameters. */
    send_param = malloc(sizeof(example_espnow_send_param_t));
    memset(send_param, 0, sizeof(example_espnow_send_param_t));
    send_param->unicast = true;
    send_param->broadcast = false;
    send_param->state = 0;
    send_param->magic = esp_random();
    send_param->count = 1;//CONFIG_ESPNOW_SEND_COUNT;
    send_param->delay = 0;//CONFIG_ESPNOW_SEND_DELAY;
    send_param->len = MAX_ESPNOW_PACKET_SIZE;//CONFIG_ESPNOW_SEND_LEN;
    send_param->buffer = malloc(send_param->len);
    memcpy(send_param->dest_mac, peer_mac_addr, ESP_NOW_ETH_ALEN);

	printf("\r\nESPNOW Set up complete!\r\n");
	xTaskCreate(sendESPNOWData, "sendESPNOWData", 2048, send_param, 4, NULL);

    return ESP_OK;
}


/**************************ESP SEND TASK*********************/
static void sendESPNOWData(void *pvParameter)
{
    int ret;
	unsigned short int sendCount=0, maxSends=1000, missCnt=0;
	long maxTimeOut = 1000000;
    printf("\r\n");
    globalQue.isReady=0;
    	globalQue.previousSend=255;
    	globalQue.currentData=0;
    /* Start sending ESPNOW data. */

	send_param_var = (example_espnow_send_param_t *)pvParameter;

	while(1)
	{
		//printf("\r\n\tCurrent sendCount\t%d\t", sendCount);
		//ESP_LOGI(TAG,"sending %d",sendCount);

		uart_read_bytes(uart_num, txData, 9, portMAX_DELAY);

		for(int i=0;i<9;i++)
		{
			send_param_var->buffer[i]=txData[i];
		}
		//send_param_var->buffer[0] = sendCount;
		if ((ret=esp_now_send(send_param_var->dest_mac, send_param_var->buffer, 9)) != ESP_OK)
		{
			printf(esp_err_to_name(ret));
			//example_espnow_deinit(send_param_var);
			vTaskDelete(NULL);
		}
		else
		{
			//ESP_LOGI(TAG,"length: ? : %d",send_param_var->len);
			//espnowTimers[0] = esp_timer_get_time();
			//wait for reply before trasmiting
			globalQue.previousSend = sendCount;
			while(!globalQue.isReady)
			{
				vTaskDelay(1);

				//espnowTimers[1]=esp_timer_get_time()-espnowTimers[0];
//				if(espnowTimers[1]>maxTimeOut)
//				{
//					globalQue.isReady=2;
//				}

			}
			if(globalQue.isReady==1)
			{
				//ESP_LOGI(TAG,"inside uart write ready 1");

				uart_write_bytes(uart_num, (const char*)rxData, 9);
				//ESP_LOGI(TAG,"end  uart write ready 1");

				//printf("RTT\t%lu microseconds", espnowTimers[1]);
			}
			else if(globalQue.isReady==2)
			{
				missCnt++;
				printf("TIMEOUT!");
			}
			globalQue.isReady=0;
			sendCount++;
		}
		vTaskDelay(5 / portTICK_RATE_MS);
	}
	printf("\r\n\r\n\t\t\tAvg RTT\t%lu\tSent\t%d\tMissed\t%d\r\n", espnowTimers[2]/(maxSends-missCnt), (maxSends-missCnt), missCnt);
	//example_espnow_deinit(send_param_var);
	vTaskDelete(NULL);
	printf("\r\n\tOut of sendESPNOWData\r\n");
}


/*************ESP NOW SEND CALLBACK***************/
static void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
   // ESP_LOGI(TAG, "DATA send CB , status: %d", status);



}


/*************ESP NOW RECEIVE CALLBACK***************/

static void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{

	//ESP_LOGI(TAG,"last received : %d",data[8]);
		for(int i=0;i<len;i++){
		  //  ESP_LOGI(TAG, "received : %d",data[i]);
		    rxData[i]=data[i];
		}
		//ESP_LOGI(TAG,"after loop : %d",data[8]);

		globalQue.isReady=1;
		//espnowTimers[1]=esp_timer_get_time()-espnowTimers[0];
	//	espnowTimers[2]+=espnowTimers[1];


	    //vTaskDelay(1000 / portTICK_RATE_MS);
       /* uart_write_bytes(uart_num, (const char*)rxData, 9);
    	uart_read_bytes(uart_num, rxData, 9, portMAX_DELAY);
    	esp_now_send(peer.peer_addr, rxData, 9);*/

    //espnow_recv=1;

}

/****************ESP ADD PEER************************/
void add_peer()
{

    peer.channel = 1;
    peer.ifidx = ESP_IF_WIFI_STA; //changed from sta;
    peer.encrypt = false;
    for(int i=0;i<6;i++){
    	peer.peer_addr[i]=peer_mac_addr[i];
    }
    esp_now_add_peer(&peer) ;

}

/************************WIFI INIT ESP NOW************************/
static void initWifi(void)
{
    esp_err_t ret;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	uint8_t cnt=0;

	//tcpip_adapter_init();
  //  ret = esp_event_loop_init(example_event_handler, NULL);
    ret = esp_wifi_init(&cfg);
    ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    ret = esp_wifi_set_mode( ESPNOW_WIFI_MODE );
    ret = esp_wifi_start();
    ret = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0);
	ret = esp_wifi_set_ps(WIFI_PS_NONE);
//	ret = esp_wifi_get_mac(ESPNOW_WIFI_IF, localMac);
	ESP_LOGI(TAG,"\r\n[%d]\r\n", ret);
	ESP_LOGI(TAG,"\r\nWIFI_MODE_STA MAC Address:\t");

	printf("\r\nEnabling Long range setting...[");
	ret = esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N);
	printf(esp_err_to_name(ret));
	printf("]\r\n");
	printf("Setting High Spees Mode...[");
//	ret = esp_wifi_internal_set_fix_rate(ESPNOW_WIFI_IF, 1, WIFI_PHY_RATE_MCS3_SGI);
	printf("%d]\r\n", ret);
}

 void wifi_init(void)
{
    esp_netif_init();
  //  esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
     esp_wifi_init(&cfg) ;
    esp_wifi_set_storage(WIFI_STORAGE_RAM) ;
     esp_wifi_set_mode(WIFI_MODE_STA) ; //changed from sta;
    esp_wifi_start();


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
	        .baud_rate = 921600, //CHANGE
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


/************************* WIFI EVENT HANDLER  ***************************/

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{

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

