/* Timer group-hardware timer example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "pca9555.h"

#include "driver/gpio.h"

#define GPIO_INPUT_IO     15
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO)
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

int isr_io_config()
{
	gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

	gpio_set_intr_type(GPIO_INPUT_IO, GPIO_INTR_NEGEDGE);

	//install gpio isr service
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    //hook isr handler for specific gpio pin

    gpio_isr_handler_add(GPIO_INPUT_IO, gpio_isr_handler, (void*) GPIO_INPUT_IO);
	return 0;
}

static void i2c_isr_gpio_task(void* arg)
{
    uint32_t io_num;
	uint16_t read_val = 0;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			read_val = pca9555_multi_read_ex();
			printf("read_val:%02x\r\n",read_val);
            //printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));       
		}
    }
}

static void i2c_w_task(void* arg)
{
	uint16_t v=0;
	for(int j=0;j<16;j++)
	{
		v|=0x0001<<j;
		pca9555_pin_write(v,v);
		printf("pca9555_pin_write : %x\r\n",v);
		vTaskDelay(10000 / portTICK_RATE_MS);
	}

}


void app_main()
{
	pca9555_init();
	isr_io_config();
	xTaskCreate(i2c_isr_gpio_task, "i2c_isr_gpio_task", 2048, NULL, 10, NULL);
	//xTaskCreate(i2c_w_task, "i2c_w_task", 2048, NULL, 10, NULL);
}

