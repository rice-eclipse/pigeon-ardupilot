#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_TORQEEDO_ENABLED
#define HAL_TORQEEDO_ENABLED !HAL_MINIMIZE_FEATURES && (BOARD_FLASH_SIZE > 1024) && !defined(HAL_BUILD_AP_PERIPH)
#endif
