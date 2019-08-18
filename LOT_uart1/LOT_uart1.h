/**
 * @file LOT_uart1.h
 * @author Hyeon-ki, Hong (hhk7734@gmail.com)
 * @brief UART communication
 */

#ifndef _LOT_UART1_H_
#define _LOT_UART1_H_
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#define LOT_UART1_RX_BUF_SIZE 128
#define LOT_UART1_TX_BUF_SIZE 128

void uart1_init( void );
void uart1_rxne_isr( void );
void uart1_txe_isr( void );

void     uart1_putchar( const char c );
void     uart1_putstr( const char *str );
char     uart1_getchar( void );
uint16_t uart1_available( void );
void     uart1_flush( void );

__STATIC_INLINE void uart1_print_str( const char *str );
void                 uart1_print_u32( uint32_t num );
void                 uart1_print_s32( int32_t num );
void                 uart1_print_float( float num, uint16_t digits );

__STATIC_INLINE void uart1_print_str( const char *str )
{
    uart1_putstr( str );
}

#ifdef __cplusplus
}
#endif
#endif    // _LOT_UART1_H_
