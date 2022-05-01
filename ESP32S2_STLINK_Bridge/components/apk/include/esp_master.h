/*
 * esp_master.h
 *
 *  Created on: May 1, 2022
 *      Author: firas
 */

#ifndef COMPONENTS_APK_INCLUDE_ESP_MASTER_H_
#define COMPONENTS_APK_INCLUDE_ESP_MASTER_H_



 void vMasterswd_Task(void * argument);
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
