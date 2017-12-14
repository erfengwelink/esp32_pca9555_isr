#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2c.h"

#include "pca9555.h"

#define I2C_RW_BYTES_LENGTH       2  /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define I2C_MAX_TIMEOUT_MS        200

#define I2C_MASTER_SCL_IO    13    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    14    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    200000     /*!< I2C master clock frequency */

#define ESP_SLAVE_ADDR 0x20         /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */


static const char* TAG = "PCA9555";
static uint16_t pca9555_addr  = ESP_SLAVE_ADDR;
static uint16_t pca9555_value = 0x0000;
static uint16_t pca9555_readonly_value = 0x0000;
static uint16_t pca9555_failed = 0;

/*
esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t ctrl, uint8_t* data_rd, size_t size)
{
	if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, ( pca9555_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ctrl, ACK_CHECK_EN);
	i2c_master_start(cmd);

	i2c_master_write_byte(cmd, ( pca9555_addr << 1 ) | READ_BIT, ACK_CHECK_EN);
	
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MAX_TIMEOUT_MS / portTICK_RATE_MS);
	printf(">>>>>>>>ret = %x| data_rd[0]:%x|data_rd[1]:%x\r\n",ret,data_rd[0],data_rd[1]);

	for(int i=0;i<15;i++)
		printf("%02x |",*(uint8_t*)(cmd+i));

    i2c_cmd_link_delete(cmd);
    return ret;
}
*/

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
	i2c_master_write_byte(cmd,0x00, ACK_CHECK_EN);

	i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	printf("ret 1 = %d\r\n",ret);    
	i2c_cmd_link_delete(cmd);
	//~~~~~~~~~~~~~
	cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
	if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    //esp_err_t 
	ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
	printf("ret 2 = %d\r\n",ret);    
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t ctrl,  uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( pca9555_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ctrl, ACK_CHECK_EN);
    //i2c_master_write_byte(cmd, data_wr[0], ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, I2C_MAX_TIMEOUT_MS / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint16_t pca9555_config(void)
{
	uint8_t w_data[I2C_RW_BYTES_LENGTH];
    esp_err_t err;
    uint16_t config_value = 0xffff; //
    int retry = 3;

	w_data[0] = (uint8_t)(0xFF & config_value);
	w_data[1] = (uint8_t)(0xFF & (config_value >> 8));

    do {
        err = i2c_master_write_slave(I2C_MASTER_NUM, REG_CONFIG_PORT0, w_data, I2C_RW_BYTES_LENGTH);
        if (err != ESP_OK) {
            retry--;
            vTaskDelay(100 / portTICK_PERIOD_MS); 
            ESP_LOGE(TAG, "%s configure %X failed, ret: %d", __func__, config_value, err);
            pca9555_failed = 1;
            if ((retry <= 0) && pca9555_failed) {
                vTaskDelay(5000 / portTICK_PERIOD_MS); 
                esp_restart();
            }
        } else {
            pca9555_failed = 0;
            break;
        }
    } while (retry);

    //ESP_LOGI(TAG, "%s, pca8575_config ok ", __func__);

    return config_value;
}


void pca9555_init()
{
	ESP_LOGI(TAG, "%s ...", __func__);
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
#if 1
    pca9555_config();
#endif
}

int  pca9555_error()
{
    return pca9555_failed;
}


uint16_t pca9555_multi_write_ex(uint16_t reg_value)
{
    uint8_t w_data[I2C_RW_BYTES_LENGTH];
    esp_err_t err;
    int retry = 3;

    w_data[0] = (uint8_t)(0xFF & reg_value);
    w_data[1] = (uint8_t)(0xFF & (reg_value >> 8));

#if 0
    if (pca9555_failed) {
        ESP_LOGE(TAG, "%s, i2c failed skipped pca9555", __func__);
        return 0;
    }
#endif
    do {
        err = i2c_master_write_slave(I2C_MASTER_NUM, REG_OUTPUT_PORT0, w_data, I2C_RW_BYTES_LENGTH);
        if (err != ESP_OK) {
            retry--;
            vTaskDelay(100 / portTICK_PERIOD_MS);
            ESP_LOGE(TAG, "%s failed, prev:%X, target: %X", __func__, pca9555_value, reg_value);
            pca9555_failed = 1;
            //esp_restart();
        } else {
            pca9555_failed = 0;
            break;
        }
    } while (retry);

	//ESP_LOGE(TAG, "%s, i2c failed", __func__);
    ESP_LOGI(TAG, "%s New:%X, OLD:%X", __func__, reg_value, pca9555_value);

    if (pca9555_failed == 0) {
        pca9555_value = reg_value;
    }
    return pca9555_value;
}

uint16_t pca9555_multi_read_ex(void)
{
	uint16_t ret;
    esp_err_t err;
	uint8_t r_data[I2C_RW_BYTES_LENGTH];
    int retry = 3;

#if 0
    if (pca9555_failed) {
        ESP_LOGE(TAG, "%s, i2c failed skipped pca9555", __func__);
        return 0;
    }
#endif
    do {
        err = i2c_master_read_slave(I2C_MASTER_NUM, /*REG_INPUT_PORT0,*/ r_data, I2C_RW_BYTES_LENGTH);
        if (err != ESP_OK) {
            retry--;
            vTaskDelay(100 / portTICK_PERIOD_MS);
            ESP_LOGE(TAG, "%s failed, prev:%X, target: %X", __func__, pca9555_value, pca9555_readonly_value);
            pca9555_failed = 1;
        } else {
            pca9555_failed = 0;
            break;
        }
    } while (retry);


	//ESP_LOGE(TAG, "%s, i2c failed", __func__);

	ret = r_data[1] << 8 | r_data[0];

    pca9555_readonly_value = ret;
	return ret;
}

uint16_t pca9555_multi_read_local(void)
{
    return pca9555_value;
}

uint16_t pca9555_multi_write(uint16_t pins, int val)
{
    //Write I2C multiple ports value
    //Wait for ACK to take effect.
    //Check I2C write result
    uint16_t i;
    uint16_t wrt_data = pca9555_value;
    uint16_t bitval = (val > 0) ? 1 : 0;
    uint16_t temp;

    for (i = 0; i < 16; i++) {
        temp = 0x1 << i;
        if (pins & temp) {
            if (val) wrt_data |= temp;
            else wrt_data &= ~temp;
        }
    }

    return pca9555_multi_write_ex(wrt_data);
}

uint16_t pca9555_multi_read(uint16_t pins)
{
	//Read I2C All value.
	//By default pins is 0xFFFF
	//TODO Check current value and update new value
	//pca8575_value = value;
	return pca9555_multi_read_ex() & pins;
}

uint16_t pca9555_pin_write(uint16_t pin, int val)
{
	//Check port & value valid
	//pca8575_value bit set or bit clear pin
	//call pca9555_multi_write()
	return pca9555_multi_write(pin, val);
}

int pca9555_pin_read(uint16_t pin)
{
	//Check port & value valid
	//pca8575_multi_read
	//bit get value
	return (pca9555_multi_read_ex() & pin) ? 1 : 0;
}



