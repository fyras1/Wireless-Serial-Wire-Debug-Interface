/*
 * esp_master.h
 *
 *  Created on: May 1, 2022
 *      Author: firas
 */

#ifndef COMPONENTS_APK_INCLUDE_ESP_MASTER_H_
#define COMPONENTS_APK_INCLUDE_ESP_MASTER_H_

#include <stdio.h>

#include <stdint.h>
#include "driver/gpio.h"


#define SWD_SLAVE_CLK_Pin GPIO_NUM_2
#define SWD_SLAVE_DATA_Pin GPIO_NUM_4

#define SWD_MASTER_CLK_Pin GPIO_NUM_6
#define SWD_MASTER_DATA_Pin GPIO_NUM_8


#define DEBUG_PIN_1 GPIO_NUM_10
#define DEBUG_PIN_2 GPIO_NUM_12
#define DEBUG_PIN_3 GPIO_NUM_16



#define SWD_Select_Seq 0x79E7


 typedef enum {
	 REQUEST,
	 ACK,
	 DATA_FROM_MASTER,
	 DATA_FROM_ISR,
	 LINE_RESET_FULL,

	 DATA_WRITE_FINISH,
	 LINE_RESET_FROM_MASTER

 }notifTypeTypedef;

 typedef struct ns{
	 notifTypeTypedef type;
	 uint32_t value1;
	 uint32_t value2;
 } notificationStruct;

 extern notificationStruct slaveNotif,masterNotif;


 void master_func(void);
 void printRequest(uint32_t rq);
 void printReset(uint32_t SWDorJTAG_val);

 void swclk_reset(void);
 void swclk_set(void);
 void swclk_cycle(void);
 void swdio_WriteBit(uint8_t bit);
 void swdio_Read();
 void swdio_mode_input(void);
 void swdio_mode_output(void);
 void swdio_Write(uint32_t val , uint8_t len);

 uint8_t parity(uint32_t val);


  uint8_t readAck(void);
  void readData(void);

#endif /* COMPONENTS_APK_INCLUDE_ESP_MASTER_H_ */
