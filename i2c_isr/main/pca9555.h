#ifndef __PCA_9555_H__
#define __PCA_9555_H__

#include <stdint.h>
#include <esp_err.h>

#define PCA_PIN_P00       0x0001
#define PCA_PIN_P01       0x0002
#define PCA_PIN_P02       0x0004
#define PCA_PIN_P03       0x0008
#define PCA_PIN_P04       0x0010
#define PCA_PIN_P05       0x0020
#define PCA_PIN_P06       0x0040
#define PCA_PIN_P07       0x0080
#define PCA_PIN_PC10      0x0100
#define PCA_PIN_PC11      0x0200
#define PCA_PIN_PC12      0x0400
#define PCA_PIN_PC13      0x0800
#define PCA_PIN_PC14      0x1000
#define PCA_PIN_PC15      0x2000
#define PCA_PIN_PC16      0x4000
#define PCA_PIN_PC17      0x8000
#define PCA_PIN_P_ALL     0x00FF
#define PCA_PIN_PC_ALL    0xFF00
#define PCA_PIN_ALL       0xFFFF
#define PCA_PIN_NULL      0x0000


#define IOMODE_OUTPUT  1
#define IOMODE_INPUT   0

#define REG_INPUT_PORT0      0
#define REG_INPUT_PORT1      1

#define REG_OUTPUT_PORT0     2
#define REG_OUTPUT_PORT1     3

#define REG_INVERT_PORT0     4
#define REG_INVERT_PORT1     5

#define REG_CONFIG_PORT0     6
#define REG_CONFIG_PORT1     7

#ifdef __cplusplus
extern "C" {
#endif


void pca9555_init();

int  pca9555_error();

uint16_t pca9555_multi_write_ex(uint16_t reg_value);

uint16_t pca9555_multi_read_ex(void);

uint16_t pca9555_multi_read_local(void);

uint16_t pca9555_multi_write(uint16_t pins, int val);

uint16_t pca9555_multi_read(uint16_t pins);

uint16_t pca9555_pin_write(uint16_t pin, int val);

uint16_t pca9555_pin_write(uint16_t pin, int val);

int pca9555_pin_read(uint16_t pin);


#ifdef __cplusplus
}
#endif


#endif
