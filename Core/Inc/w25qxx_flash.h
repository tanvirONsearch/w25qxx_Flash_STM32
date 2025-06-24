#ifndef DRIVER_W25QXX_INTERFACE_H
#define DRIVER_W25QXX_INTERFACE_H

#include "driver_w25qxx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define Flash_INITIAL_ADDRESS 0
#define ADDRS_SPACE_SLOT_SIZE 4

void cs_low(void);
void cs_high(void);

//low level APIs
uint8_t w25qxx_platform_init(w25qxx_handle_t *handle);

uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction, uint8_t instruction_line,
                                             uint32_t address, uint8_t address_line, uint8_t address_len,
                                             uint32_t alternate, uint8_t alternate_line, uint8_t alternate_len,
                                             uint8_t dummy, uint8_t *in_buf, uint32_t in_len,
                                             uint8_t *out_buf, uint32_t out_len, uint8_t data_line);

void w25qxx_interface_delay_ms(uint32_t ms);
void w25qxx_interface_delay_us(uint32_t us);
void w25qxx_interface_debug_print(const char *fmt, ...);

//high level APIs
uint8_t flash_init(void);
uint8_t flash_load_cache(void);
uint8_t flash_flush(void); // to set total cache to memory

uint8_t flash_read(uint16_t addr, uint32_t *data); // no length 32bit  // addrs is serial address
uint8_t flash_read_float(uint16_t addr, float *value);
uint8_t flash_read_burst(uint16_t start_addr, uint32_t *data, uint16_t count);

uint8_t flash_write(uint16_t addr, uint32_t value);
uint8_t flash_write_burst(uint16_t start_addr, uint32_t *data, uint16_t count);
uint8_t flash_write_float(uint16_t addr, float value);
uint8_t flash_cleanup(void);

//uint8_t flash_erase_sector(uint32_t addr);
//uint8_t flash_chip_erase(void);
////read burst with out length
////write burst with out length

#ifdef __cplusplus
}
#endif

#endif
