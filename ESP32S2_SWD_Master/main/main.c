

#include "server_master.h"
#include "main.h"

 #include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
//#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_private/wifi.h"



#include "esp_now.h"






#include <string.h>

#include "esp_mac.h"
#include <lwip/sockets.h>


//#include "server_master.h"

//#include "main.h"

#define CONFIG_ESPNOW_CHANNEL				7
#define ESPNOW_WIFI_MODE 					WIFI_MODE_STA
#define ESPNOW_WIFI_IF 						ESP_IF_WIFI_STA
#define MAX_ESPNOW_PACKET_SIZE				250

char* TAG="esp master:";
esp_now_peer_info_t peer;
example_espnow_send_param_t *send_param;

static uint8_t peer_mac_addr[6] = { 0x7C, 0xDF, 0xA1, 0x51, 0x08, 0xDC }; //white base mac 7c:df:a1:51:08:dc


example_espnow_send_param_t *send_param_var;

static esp_err_t example_espnow_init(void);

struct sockaddr_in clientAddress;
struct sockaddr_in serverAddress;

uint8_t tcpRxBuff[9];
uint8_t tcpTxBuff[9];

notificationStruct slaveNotif,masterNotif;

TaskHandle_t esp_now_task_handle;


uint8_t received=0;

void gpio_init(void);

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}


static esp_err_t example_espnow_init(void);

static void initWifi(void);

void wifi_init_softap(void);
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data);

void wifi_init(void);
void add_peer(void);
static void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status);

static void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len);

void app_main(void)
{
	gpio_init();
    nvs_flash_init();
   // tcpip_adapter_init();
   // wifi_init_softap();

   // wifi_init();

    initWifi();

    example_espnow_init();


    //esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_AP, 1, WIFI_PHY_RATE_MCS7_SGI);


   // esp_now_init() ;
//   esp_now_register_send_cb(send_cb) ;
//    esp_now_register_recv_cb(recv_cb) ;
//
//    add_peer();




    //while(1){}







}


/**************************ESP SEND TASK*********************/
static void sendESPNOWData_Master(void *pvParameter)
{
    int ret;


    /* Start sending ESPNOW data. */

	send_param_var = (example_espnow_send_param_t *)pvParameter;

	while(1)
	{
		xTaskNotifyWait(0x00, 0xffffffff, NULL, portMAX_DELAY);

		received=0;

	 	   masterNotif.type=(notifTypeTypedef) tcpRxBuff[0];

	 	   masterNotif.value1=0;
	 	   masterNotif.value2=0;

	 	   masterNotif.value1=(tcpRxBuff[1]<<24) + (tcpRxBuff[2]<<16) +( tcpRxBuff[3]<<8) + (tcpRxBuff[4]);
	 	   masterNotif.value2=(tcpRxBuff[5]<<24) + (tcpRxBuff[6]<<16) + (tcpRxBuff[7]<<8) + (tcpRxBuff[8]);

	 	   master_func();

			tcpTxBuff[0]=(uint8_t) slaveNotif.type;

			tcpTxBuff[1]=(slaveNotif.value1>>24) & 0xFF;
			tcpTxBuff[2]=(slaveNotif.value1>>16) & 0xFF;
			tcpTxBuff[3]=(slaveNotif.value1>>8) & 0xFF;
			tcpTxBuff[4]=(slaveNotif.value1>>0) & 0xFF;

			tcpTxBuff[5]=(slaveNotif.value2>>24) & 0xFF;
			tcpTxBuff[6]=(slaveNotif.value2>>16) & 0xFF;
			tcpTxBuff[7]=(slaveNotif.value2>>8) & 0xFF;
			tcpTxBuff[8]=(slaveNotif.value2>>0) & 0xFF;

			gpio_set_level(DEBUG_PIN_1, 1);
			gpio_set_level(DEBUG_PIN_1, 0);
		esp_now_send(send_param->dest_mac, tcpTxBuff, 9);
	}
}

/***********************esp now master init**********************/

static esp_err_t example_espnow_init(void)
{

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(recv_cb) );

    /* Add broadcast peer information to peer list. */
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    memset(peer, 0, sizeof(esp_now_peer_info_t));

	//Add espnow node to reply to
	printf("\r\n\tAdd Remote node(peer) to local peer list...");
	peer->channel = CONFIG_ESPNOW_CHANNEL;
	peer->ifidx = ESPNOW_WIFI_IF;
	peer->encrypt = false;
	//memcpy(peer->lmk, CONFIG_ESPNOW_LMK, ESP_NOW_KEY_LEN);
	memcpy(peer->peer_addr, peer_mac_addr, ESP_NOW_ETH_ALEN);
	ESP_ERROR_CHECK( esp_now_add_peer(peer) );
	free(peer);
	printf("Remote Peer(the sender) Added!\r\n");


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

	//xTaskCreate(sendESPNOWData_Master, "sendESPNOWData", 2048, send_param, 20, NULL);
	xTaskCreate(sendESPNOWData_Master, "sendESPNOWData_Master", 2048, send_param, 20, &esp_now_task_handle);


	return ESP_OK;
}





/*************ESP NOW SEND CALLBACK***************/
static void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
   // ESP_LOGI(TAG, "DATA send CB , status: %d", status);





}


/*************ESP NOW RECEIVE CALLBACK***************/

static void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{


	gpio_set_level(DEBUG_PIN_1, 1);
	gpio_set_level(DEBUG_PIN_1, 0);
	//gpio_set_level(DEBUG_PIN_1, 1);
//	gpio_set_level(DEBUG_PIN_1, 0);

	//esp_now_send(send_param->dest_mac, data, len);

		for(int i=0;i<len;i++){
		   // ESP_LOGI(TAG, "received : %d",data[i]);
		    tcpRxBuff[i]=data[i];
		}

		BaseType_t xHigherPriorityTaskWoken=pdFALSE;

		xTaskNotifyFromISR(esp_now_task_handle,0,eNoAction,&xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR();


	    //vTaskDelay(1000 / portTICK_RATE_MS);
       // uart_write_bytes(uart_num, (const char*)rxData, 9);

   // espnow_recv=1;


}

/****************ESP ADD PEER************************/
void add_peer()
{

    peer.channel = 1;
    peer.ifidx = ESP_IF_WIFI_STA;
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
    //ret = esp_event_loop_init(example_event_handler, NULL);
    ret = esp_wifi_init(&cfg);
    ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    ret = esp_wifi_set_mode( ESPNOW_WIFI_MODE );
    ret = esp_wifi_start();
    ret = esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, 0);
	ret = esp_wifi_set_ps(WIFI_PS_NONE);
	//ret = esp_wifi_get_mac(ESPNOW_WIFI_IF, localMac);
	printf("\r\n[%d]\r\n", ret);
	printf("\r\nWIFI_MODE_STA MAC Address:\t");

	printf("\r\nEnabling Long range setting...[");
	ret = esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N); //WIFI_PROTOCOL_LR
	printf(esp_err_to_name(ret));
	printf("]\r\n");
	printf("Setting High Spees Mode...[");
	ret = esp_wifi_internal_set_fix_rate(ESPNOW_WIFI_IF, 1, WIFI_PHY_RATE_MCS7_SGI);
	printf("%d]\r\n", ret);
}

 void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
     esp_wifi_init(&cfg) ;
    esp_wifi_set_storage(WIFI_STORAGE_RAM) ;
     esp_wifi_set_mode(WIFI_MODE_AP) ;
     esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_start();


}





void wifi_init_softap(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_t* wifiAP = esp_netif_create_default_wifi_ap();


    esp_netif_ip_info_t ipInfo;
    IP4_ADDR(&ipInfo.ip, 192,168,2,1);
	IP4_ADDR(&ipInfo.gw, 192,168,2,1);
	IP4_ADDR(&ipInfo.netmask, 255,255,255,0);
	esp_netif_dhcps_stop(wifiAP);
	esp_netif_set_ip_info(wifiAP, &ipInfo);
	esp_netif_dhcps_start(wifiAP);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID, &wifi_event_handler,NULL, NULL);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = 16,
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,

        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    esp_wifi_set_ps(WIFI_PS_NONE); //added new




}


static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
//    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
//        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
//
//    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
//        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
//
//    }
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








