#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "main.h"



void app_main(void){


    nvs_flash_init();

    gpio_config_t io_conf = {};

    //disable interrupt
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
        //set as output mode
        io_conf.mode = GPIO_MODE_INPUT;
        //bit mask of the pins that you want to set,e.g.GPIO18/19
        io_conf.pin_bit_mask = (1<<SWD_SLAVE_CLK_Pin) ;
        //disable pull-down mode
        io_conf.pull_down_en = 0;
        //disable pull-up mode
        io_conf.pull_up_en = 0;
        //configure GPIO with the given settings
        gpio_config(&io_conf);



    while (1)
    {
       uint8_t a=1;
    }
}

