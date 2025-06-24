/*
 * driver_w25q_interface.c
 *
 *  Created on: Jun 17, 2025
 *      Author: Tanvir Rahman Sahed
 */
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <w25qxx_flash.h>
#include "flash_addrs.h"
#include<stdbool.h>
//caching buffer
static uint32_t flash_buffer[FLASH_ADR_MAX];  // holds all values read from flash
static bool is_updated = false;

//w25qxx drivers handle
static w25qxx_handle_t w25q;

// External handles from STM32CubeMX
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;


// --- CS Pin Setup (PA15) ---
#define W25QXX_CS_PORT GPIOA
#define W25QXX_CS_PIN  GPIO_PIN_15
#define W25QXX_SPI_HANDLE &hspi1
#define W25QXX_CS_ACTIVE_HIGH 0  // Set to 1 if CS is active high

/**
 * @brief Drive CS pin to active state
 * @note  Uses logic defined by W25QXX_CS_ACTIVE_HIGH macro
 */
void cs_low(void)
{
#if W25QXX_CS_ACTIVE_HIGH
    HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_SET);  // active high
#else
    HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_RESET); // active low
#endif
}

/**
 * @brief Drive CS pin to inactive state
 * @note  Uses logic defined by W25QXX_CS_ACTIVE_HIGH macro
 */
void cs_high(void)
{
#if W25QXX_CS_ACTIVE_HIGH
    HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_RESET);  // inactive = low
#else
    HAL_GPIO_WritePin(W25QXX_CS_PORT, W25QXX_CS_PIN, GPIO_PIN_SET);    // inactive = high
#endif
}

/**
 * @brief checks SPI Initialization
 * * @return    status code
 *            - 0 success
 *            - 1 error
 * @note  Uses logic defined by W25QXX_CS_ACTIVE_HIGH macro
 */
uint8_t w25qxx_interface_spi_qspi_init(void)
{
    return (HAL_SPI_Init(W25QXX_SPI_HANDLE) == HAL_OK) ? 0 : 1;
}

/**
 * @brief checks SPI de_Initialization
 * * @return    status code
 *            - 0 success
 *            - 1 error
 * @note  Uses logic defined by W25QXX_CS_ACTIVE_HIGH macro
 */
uint8_t w25qxx_interface_spi_qspi_deinit(void)
{
    return (HAL_SPI_DeInit(W25QXX_SPI_HANDLE) == HAL_OK) ? 0 : 1;
}


// --- Platform Initialization ---
/**
 * @brief     initialize the chip
 * @param[in] *handle pointer to a w25qxx handle structure
 * @return    status code
 *            - 0 success
 *            - 1 spi or qspi initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 *            - 4 get manufacturer device id failed
 *            - 5 enter qspi failed
 *            - 6 id is invalid
 *            - 7 reset failed
 *            - 8 set address mode failed
 * @note      low level init
 */
uint8_t w25qxx_platform_init(w25qxx_handle_t *handle)
{
    DRIVER_W25QXX_LINK_INIT(handle, w25qxx_handle_t);
    DRIVER_W25QXX_LINK_SPI_QSPI_INIT(handle, w25qxx_interface_spi_qspi_init);
    DRIVER_W25QXX_LINK_SPI_QSPI_DEINIT(handle, w25qxx_interface_spi_qspi_deinit);
    DRIVER_W25QXX_LINK_SPI_QSPI_WRITE_READ(handle, w25qxx_interface_spi_qspi_write_read);
    DRIVER_W25QXX_LINK_DELAY_MS(handle, w25qxx_interface_delay_ms);
    DRIVER_W25QXX_LINK_DELAY_US(handle, w25qxx_interface_delay_us);
    DRIVER_W25QXX_LINK_DEBUG_PRINT(handle, w25qxx_interface_debug_print);

    handle->spi_qspi = W25QXX_INTERFACE_SPI;
    handle->type = W25Q16;
    // Optional but good to define
    handle->address_mode = 3;  // 3-byte addressing (0x00 - 0xFFFFFF)
    handle->dummy = 8;         // Typical for fast read: 8 dummy clocks

    return w25qxx_init(handle);
}

// --- Required Interface Functions ---
/**
 * @brief     SPI/QSPI raw command interface for W25Qxx
 * @param[in] instruction         SPI instruction (0x00 if using in_buf instead)
 * @param[in] instruction_line    Not used in SPI, reserved for QSPI
 * @param[in] address             Address to use (if any)
 * @param[in] address_line        Not used in SPI
 * @param[in] address_len         Not used in SPI
 * @param[in] alternate           Not used in SPI
 * @param[in] alternate_line      Not used in SPI
 * @param[in] alternate_len       Not used in SPI
 * @param[in] dummy               Number of dummy bytes to send after command
 * @param[in] in_buf              Pointer to command buffer to transmit
 * @param[in] in_len              Length of in_buf
 * @param[out] out_buf            Pointer to receive buffer
 * @param[in] out_len             Number of bytes to receive
 * @param[in] data_line           Not used in SPI
 * @return    status code
 *            - 0 success
 *            - 1 TX error (in_buf or dummy)
 *            - 2 RX error (out_buf)
 * @note      This interface assumes SPI-only operation. QSPI fields are ignored.
 */
uint8_t w25qxx_interface_spi_qspi_write_read(uint8_t instruction,
                                             uint8_t instruction_line,
                                             uint32_t address,
                                             uint8_t address_line,
                                             uint8_t address_len,
                                             uint32_t alternate,
                                             uint8_t alternate_line,
                                             uint8_t alternate_len,
                                             uint8_t dummy,
                                             uint8_t *in_buf,
                                             uint32_t in_len,
                                             uint8_t *out_buf,
                                             uint32_t out_len,
                                             uint8_t data_line)
{
    HAL_StatusTypeDef status = HAL_OK;

    cs_low();

    /* transmit command/data buffer */
    if (in_buf && in_len > 0)
    {
        status = HAL_SPI_Transmit(&hspi1, in_buf, in_len, HAL_MAX_DELAY);
        if (status != HAL_OK)
        {
            cs_high();
            return 1;  // TX error (command/data)
        }
    }

    /* send dummy bytes if needed */
    if (dummy > 0)
    {
        uint8_t d = 0x00;
        for (uint32_t i = 0; i < dummy; i++)
        {
            status = HAL_SPI_Transmit(&hspi1, &d, 1, HAL_MAX_DELAY);
            if (status != HAL_OK)
            {
                cs_high();
                return 1;  // TX error (dummy)
            }
        }
    }

    /* receive data if requested */
    if (out_buf && out_len > 0)
    {
        status = HAL_SPI_Receive(&hspi1, out_buf, out_len, HAL_MAX_DELAY);
        if (status != HAL_OK)
        {
            cs_high();
            return 2;  // RX error
        }
    }

    cs_high();
    return 0;  // success
}



void w25qxx_interface_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
 * @brief     delays for passed us
 * @param[in] microseconds
 * @return    nothing
 * @note      none
 */
void w25qxx_interface_delay_us(uint32_t us)
{
    // Enable DWT CYCCNT if not already enabled
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;      // Enable trace
        DWT->CYCCNT = 0;                                     // Reset counter
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                 // Enable cycle counter
    }

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);

    while ((DWT->CYCCNT - start) < ticks);
}

/**
 * @brief     prints to uart2
 * @return    nothing
 * @note      none
 */
void w25qxx_interface_debug_print(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
}


// high level APIs
/**
 * @brief     initialize the chip
 * @return    status code
 *            - 0 success
 *            - 1 spi or qspi initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 *            - 4 get manufacturer device id failed
 *            - 5 enter qspi failed
 *            - 6 id is invalid
 *            - 7 reset failed
 *            - 8 set address mode failed
 * @note      none
 */
uint8_t flash_init(void){
	uint8_t a = w25qxx_platform_init(&w25q);
	if(!a){
		flash_load_cache();
		is_updated = false;
	}
	return a;
}

/**
 * @brief     resets update flag
 * @return    status code
 *            - 0 success
 *            - 1 read failed
 */
uint8_t reset_update_flag(void){
	is_updated = false;
	return is_updated;
}


// caching function
/**
 * @brief     Load all logical slots from flash into local RAM buffer
 * @return    status code
 *            - 0 success
 *            - 1 read failed
 */
uint8_t flash_load_cache(void)
{
   if (w25qxx_read(&w25q, Flash_INITIAL_ADDRESS, flash_buffer, FLASH_ADR_MAX * ADDRS_SPACE_SLOT_SIZE) != 0) // cahnge FLASH_SLOT_SIZE to user_data_size
        return 1;

    return 0;
}


//reads in local buffer one slot
uint8_t flash_read(uint16_t addr, uint32_t *data)
{
    if (addr >= FLASH_ADR_MAX)
        return 1;

    *data = flash_buffer[addr];
    return 0;
}


//reads multiple slots
uint8_t flash_read_burst(uint16_t start_addr, uint32_t *data, uint16_t count)
{
    if ((start_addr + count) > FLASH_ADR_MAX)
        return 1;

    for (uint16_t i = 0; i < count; i++)
    {
        data[i] = flash_buffer[start_addr + i];
    }
    return 0;
}

/**
 * @brief     Read a 32-bit float from flash (via cache)
 * @param[in] slot   The logical flash slot (must be > 0)
 * @param[out] value Pointer to store the float result
 * @return    status code
 *            - 0 success
 *            - 1 invalid slot or NULL pointer
 */
uint8_t flash_read_float(uint16_t addr, float *value)
{
    if (addr >= FLASH_ADR_MAX || value == NULL)
        return 1;

    uint32_t raw;
    if (flash_read(addr, &raw) != 0)
        return 1;

    memcpy(value, &raw, sizeof(float));
    return 0;
}


/**
 * @brief     Write a 32-bit value to the RAM buffer (not flash)
 * @param[in] slot   The flash logical slot
 * @param[in] value  The value to write
 * @return    status code
 *            - 0 success
 *            - 1 invalid slot
 */
uint8_t flash_write(uint16_t addr, uint32_t value)
{
    if (addr >= FLASH_ADR_MAX)
        return 1;

    flash_buffer[addr] = value;
    is_updated = true;
    return 0;
}


/**
 * @brief     Write multiple 32-bit values to RAM buffer
 * @param[in] start_slot  Start slot index
 * @param[in] values      Pointer to values
 * @param[in] count       Number of values
 * @return    status code
 *            - 0 success
 *            - 1 invalid range
 */
uint8_t flash_write_burst(uint16_t start_addr, uint32_t *data, uint16_t count)
{
    if ((start_addr + count) > FLASH_ADR_MAX)
        return 1;

    for (uint16_t i = 0; i < count; i++)
    {
        flash_buffer[start_addr + i] = data[i];
    }
    return 0;
}
/**
 * @brief     Write a 32-bit float to flash (via cache)
 * @param[in] slot   The logical flash slot (must be > 0)
 * @param[in] value  Float value to write
 * @return    status code
 *            - 0 success
 *            - 1 invalid slot
 */
uint8_t flash_write_float(uint16_t addr, float value)
{
    if (addr >= FLASH_ADR_MAX)
        return 1;

    uint32_t raw;
    memcpy(&raw, &value, sizeof(float));
    return flash_write(addr, raw);
}

/**
 * @brief     Write all RAM-buffered slots to flash memory
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 */
uint8_t flash_flush(void)
{
    if (w25qxx_write(&w25q, Flash_INITIAL_ADDRESS, flash_buffer,FLASH_ADR_MAX * ADDRS_SPACE_SLOT_SIZE) != 0)
        return 1;

    return 0;
}

/**
 * @brief     Conditionally flush the flash buffer if updated
 * @return    status code
 *            - 0 success (flushed or nothing to do)
 *            - 1 flush failed
 */
uint8_t flash_flush_if_updated(void)
{
    if (is_updated)
    {
        is_updated = 0;  // reset update flag
        return flash_flush();  // flush buffer to flash
    }
    else
    {
        return 0;  // nothing to flush, but not an error
    }
}


/**
 * @brief     Clean up the flash by resetting all slots to 0xFFFFFFFF
 *            and flushing the cleared buffer to flash.
 * @return    status code
 *            - 0 success
 *            - 1 flush failed
 */
uint8_t flash_cleanup(void)
{
    // Set user slots to 0xFFFFFFFF starting from slot 0
    memset(&flash_buffer[0], 0xFF, (FLASH_ADR_MAX) * ADDRS_SPACE_SLOT_SIZE);

    // Flush updated RAM buffer to flash
    return flash_flush();
}

///**
// * @brief     erase the 4k sector
// * @param[in] addr erase address
// * @return    status code
// *            - 0 success
// *            - 1 sector erase 4k failed
// *            - 2 handle is NULL
// *            - 3 handle is not initialized
// *            - 4 addr is invalid
// *            - 5 address mode is invalid
// *            - 6 sector erase 4k timeout
// * @note      must be 0/4k/8k.. addresses
// */
//uint8_t flash_erase_sector(uint32_t addr){
//	return w25qxx_sector_erase_4k(&w25q, addr);
//}
//
//
///**
// * @brief     erase the chip
// * @return    status code
// *            - 0 success
// *            - 1 chip erase failed
// *            - 2 handle is NULL
// *            - 3 handle is not initialized
// *            - 4 erase timeout
// * @note      significantly slower than sector erase
// */
//uint8_t flash_chip_erase(void){
//	return w25qxx_chip_erase(&w25q);
//}





