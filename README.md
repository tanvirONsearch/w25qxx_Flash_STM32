# STM32 W25Qxx Flash Library

A modular and portable library for interfacing Winbond W25Qxx SPI NOR flash memory with STM32 microcontrollers.  
Designed for STM32CubeIDE and tested on STM32F407VET6 using a W25Q16JV chip.

---

## 📦 Repository

📍 GitHub: [tanvirONsearch/w25qxx_Flash_STM32](https://github.com/tanvirONsearch/w25qxx_Flash_STM32)

---

## 🧩 Features

- ✅ SPI-based flash communication using STM32 HAL
- ✅ Support for W25Qxx series (tested on W25Q16JV)
- ✅ Portable HAL interface with CS polarity configuration
- ✅ Logical address slot model using enums
- ✅ 32-bit aligned read and write functions
- ✅ RAM-buffered write-back system
- ✅ Flash flush and cleanup functions
- ✅ Float read/write support using `memcpy`
- ✅ Optional reserved address protection (slot 0)
- ✅ STM32CubeIDE-compatible (C language)

---

## 🛠️ MCU Platform

- **Microcontroller:** STM32F407VET6
- **Board:** STM32F4xx-M development board
- **Interface:** SPI1 (customizable)
- **Flash chip:** W25Q16JV (32-bit addressing, 4KB sector size)

---

## 🔧 How It Works

- Flash slots are managed using a `flash_addr_t` enum.
- Each slot stores exactly one 32-bit value.
- The first slot (`FLASH_ADDR_RESERVED`) is reserved for internal use.
- A local RAM buffer (`flash_buffer[]`) mirrors flash content.
- All read/write calls access the RAM buffer.
- Use `flash_flush()` to persist data to flash.
- Use `flash_cleanup()` to erase all slots and reset the buffer.

---

## 📄 Example: Usage in `main.c`

```c
#include "flash_api.h"
#include "w25qxx_interface.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();

    if (flash_init() != 0)
    {
        // Init failed
        while (1);
    }

    // Set values
    flash_write(FLASH_ADDR_DEVICE_ID, 0x12345678);
    flash_write_float(FLASH_ADDR_FLAGS, 3.1416f);

    // Commit to flash
    flash_flush();

    // Read back
    uint32_t id;
    float flags;
    flash_read(FLASH_ADDR_DEVICE_ID, &id);
    flash_read_float(FLASH_ADDR_FLAGS, &flags);

    w25qxx_interface_debug_print("ID: 0x%08lX, FLAGS: %.4f\r\n", id, flags);

    while (1);
}
