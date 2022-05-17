

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
char* TAG="esp master:";
esp_now_peer_info_t peer;

static uint8_t peer_mac_addr[6] = { 0x7C, 0xDF, 0xA1, 0x51, 0x08, 0xDD }; //white base mac 7c:df:a1:51:08:dc


struct sockaddr_in clientAddress;
struct sockaddr_in serverAddress;

uint8_t tcpRxBuff[9];
uint8_t tcpTxBuff[9];

notificationStruct slaveNotif,masterNotif;


void gpio_init(void);

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

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

    wifi_init();
    esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_AP, 1, WIFI_PHY_RATE_MCS7_SGI);


    esp_now_init() ;
   esp_now_register_send_cb(send_cb) ;
    esp_now_register_recv_cb(recv_cb) ;

    add_peer();



  /*  int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

       serverAddress.sin_family = AF_INET;
       	serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    	//serverAddress.sin_addr.s_addr = htonl(((u32_t)0x7f000002UL));
       	serverAddress.sin_port = htons(PORT_NUMBER);

       	bind(sock, (struct sockaddr *)&serverAddress, sizeof(serverAddress));

       	 listen(sock, 5);

*/
       	/* while (1)
       	 {
       		 socklen_t clientAddressLength = sizeof(clientAddress);
       		 int clientSock = accept(sock, (struct sockaddr *)&clientAddress, &clientAddressLength);

       		 while(1)
       		 {
       			gpio_set_level(DEBUG_PIN_1, 1);
       			gpio_set_level(DEBUG_PIN_1, 0);
       				 recv(clientSock, tcpRxBuff, 9, 0);
       				 gpio_set_level(DEBUG_PIN_1, 1);

       				// vTaskDelay(1);

       		 	   //    notificationStruct masterNotif;
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
              gpio_set_level(DEBUG_PIN_1, 0);
       				send(clientSock,tcpTxBuff,9,0);

       				gpio_set_level(DEBUG_PIN_1, 1);
       				gpio_set_level(DEBUG_PIN_1, 0);






       		 }
       	 }*/



}


/*************ESP NOW SEND CALLBACK***************/
static void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI(TAG, "DATA send CB , status: %d", status);



}


/*************ESP NOW RECEIVE CALLBACK***************/

static void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int len)
{

		for(int i=0;i<len;i++){
		    ESP_LOGI(TAG, "received : %d",data[i]);
		    tcpRxBuff[i]=data[i];
		}

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

		esp_now_send(peer.peer_addr, tcpTxBuff, len);
	    //vTaskDelay(1000 / portTICK_RATE_MS);
       // uart_write_bytes(uart_num, (const char*)rxData, 9);

   // espnow_recv=1;


}

/****************ESP ADD PEER************************/
void add_peer()
{

    peer.channel = 1;
    peer.ifidx = ESP_IF_WIFI_AP;
    peer.encrypt = false;
    for(int i=0;i<6;i++){
    	peer.peer_addr[i]=peer_mac_addr[i];
    }
    esp_now_add_peer(&peer) ;

}

/************************WIFI INIT ESP NOW************************/
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








