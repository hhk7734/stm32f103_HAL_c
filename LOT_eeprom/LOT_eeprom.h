/*
 * MIT License
 * Copyright (c) 2019 Hyeonki Hong <hhk7734@gmail.com>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include "stm32f1xx_hal.h"

#define E2END ( FLASH_PAGE_SIZE - 1 )
#define FLASH_BANK_NUMBER FLASH_BANK_1
#define FLASH_END FLASH_BANK1_END
#define FLASH_BASE_ADDRESS \
    ( ( uint32_t )( ( FLASH_END + 1 ) - FLASH_PAGE_SIZE ) )

#ifdef __cplusplus
extern "C" {
#endif

uint8_t eeprom_read_byte( const uint32_t pos );
void    eeprom_write_byte( uint32_t pos, uint8_t value );

#ifdef __cplusplus
}
#endif
