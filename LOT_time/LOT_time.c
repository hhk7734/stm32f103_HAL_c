/**
 * @file LOT_time.c
 * @author Hyeon-ki, Hong (hhk7734@gmail.com)
 * @brief Functions to check and delay time.
 */

#include "LOT_time.h"

void HAL_Delay( uint32_t Delay )
{
    if( Delay < 10 )
    {
        delay_us( Delay * 1000 );
    }
    else
    {
        uint32_t start  = HAL_GetTick();
        uint32_t target = start + Delay;
        if( target > start )
        {
            while( HAL_GetTick() < target )
            {
            }
        }
        else
        {
            while( HAL_GetTick() > start || HAL_GetTick() < target )
            {
            }
        }
    }
}

uint32_t millis( void )
{
    return HAL_GetTick();
}

uint32_t micros( void )
{
    uint32_t ms;
    uint32_t count;

    do
    {
        ms    = HAL_GetTick();
        count = SysTick->VAL;
        __ASM volatile( "nop" );
        __ASM volatile( "nop" );
    } while( ms != HAL_GetTick() );

    return ( ( ms * 1000UL ) + ( SysTick->LOAD + 1 - count ) / COUNT_PER_MICROSECOND );
}

void delay_ms( uint32_t msec )
{
    HAL_Delay( msec );
}

void delay_us( volatile uint32_t usec )
{
    if( usec > 0 )
    {
        usec *= US_TO_CYCLE;
        --usec;

        __ASM volatile(
            "   mov r0, %[usec] \n\t"
            "1: subs r0, #1 \n\t"
            "   bhi 1b \n\t" ::[usec] "r"( usec )
            : "r0" );
    }
}