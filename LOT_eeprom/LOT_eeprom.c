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

#include "LOT_eeprom.h"

#include <string.h>

static uint8_t eeprom_buffer[E2END + 1] = { 0 };

/**
 * @brief  This function copies the data from flash into the buffer
 */
static void eeprom_buffer_fill( void )
{
    memcpy( eeprom_buffer, ( uint8_t * )( FLASH_BASE_ADDRESS ), E2END + 1 );
}

/**
 * @brief  This function writes the buffer content into the flash
 */
static void eeprom_buffer_flush( void )
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t               offset      = 0;
    uint32_t               address     = FLASH_BASE_ADDRESS;
    uint32_t               address_end = FLASH_BASE_ADDRESS + E2END;
    uint32_t               pageError   = 0;
    uint64_t               data        = 0;

    /* ERASING page */
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks       = FLASH_BANK_NUMBER;
    EraseInitStruct.PageAddress = FLASH_BASE_ADDRESS;
    EraseInitStruct.NbPages     = 1;

    if( HAL_FLASH_Unlock() == HAL_OK )
    {
        __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_WRPERR
                                | FLASH_FLAG_PGERR );
        if( HAL_FLASHEx_Erase( &EraseInitStruct, &pageError ) == HAL_OK )
        {
            while( address <= address_end )
            {
                data = *(
                    ( uint64_t * )( ( uint8_t * )eeprom_buffer + offset ) );

                if( HAL_FLASH_Program(
                        FLASH_TYPEPROGRAM_DOUBLEWORD, address, data )
                    == HAL_OK )
                {
                    address += 8;
                    offset += 8;
                }
                else
                {
                    address = address_end + 1;
                }
            }
        }
        HAL_FLASH_Lock();
    }
}

/**
 * @brief  Function reads a byte from the eeprom buffer
 * @param  pos : address to read
 * @retval byte : data read from eeprom
 */
static uint8_t eeprom_buffered_read_byte( const uint32_t pos )
{
    return eeprom_buffer[pos];
}

/**
 * @brief  Function writes a byte to the eeprom buffer
 * @param  pos : address to write
 * @param  value : value to write
 */
static void eeprom_buffered_write_byte( uint32_t pos, uint8_t value )
{
    eeprom_buffer[pos] = value;
}

/**
 * @brief  Function reads a byte from emulated eeprom (flash)
 * @param  pos : address to read
 * @retval byte : data read from eeprom
 */
uint8_t eeprom_read_byte( const uint32_t pos )
{
    eeprom_buffer_fill();
    return eeprom_buffered_read_byte( pos );
}

/**
 * @brief  Function writes a byte to emulated eeprom (flash)
 * @param  pos : address to write
 * @param  value : value to write
 */
void eeprom_write_byte( uint32_t pos, uint8_t value )
{
    eeprom_buffered_write_byte( pos, value );
    eeprom_buffer_flush();
}
