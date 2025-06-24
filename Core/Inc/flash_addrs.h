#pragma once


typedef enum
{	FLASH_ADDR_RESERVED  = 0, //32 bit long
    FLASH_ADDR_DEVICE_ID = 1,
    FLASH_ADDR_LOG_COUNT = 2,
    FLASH_ADDR_FLAGS     = 3,
	kp,
	kd,
	ki,
	FLASH_ADR_MAX

    // ...
} flash_addr_t;
