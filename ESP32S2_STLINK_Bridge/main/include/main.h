/*
 * main.h
 *
 *  Created on: May 1, 2022
 *      Author: firas
 */

#ifndef MAIN_INCLUDE_MAIN_H_
#define MAIN_INCLUDE_MAIN_H_

#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "slave.h"
#include "common.h"
#include "master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



#define SWD_SLAVE_CLK_Pin GPIO_NUM_2
#define SWD_SLAVE_DATA_Pin GPIO_NUM_4

#define SWD_MASTER_CLK_Pin GPIO_NUM_6
#define SWD_MASTER_DATA_Pin GPIO_NUM_8

#define DEBUG_PIN_1 GPIO_NUM_10
#define DEBUG_PIN_2 GPIO_NUM_12
#define DEBUG_PIN_3 GPIO_NUM_16




void gpio_init(void);


#endif /* MAIN_INCLUDE_MAIN_H_ */
