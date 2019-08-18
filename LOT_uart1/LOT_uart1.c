/**
 * @file LOT_uart1.c
 * @author Hyeon-ki, Hong (hhk7734@gmail.com)
 * @brief UART communication
 */

#include "LOT_uart1.h"
#include <math.h>

volatile static uint16_t rx_buf_head                   = 0;
volatile static uint16_t rx_buf_tail                   = 0;
volatile static uint8_t  rx_buf[LOT_UART1_RX_BUF_SIZE] = { 0 };
volatile static uint16_t tx_buf_head                   = 0;
volatile static uint16_t tx_buf_tail                   = 0;
volatile static uint8_t  tx_buf[LOT_UART1_TX_BUF_SIZE] = { 0 };

void uart1_init( void )
{
    LL_USART_EnableIT_RXNE( USART1 );
}

void uart1_putchar( const char c )
{
    if( ( tx_buf_head == tx_buf_tail ) && LL_USART_IsActiveFlag_TXE( USART1 ) )
    {
        LL_USART_TransmitData8( USART1, c );
        return;
    }
    tx_buf[tx_buf_head] = c;
    tx_buf_head         = ( tx_buf_head + 1 ) % LOT_UART1_TX_BUF_SIZE;
    while( tx_buf_head == tx_buf_tail )
    {
    }
    LL_USART_EnableIT_TXE( USART1 );
}

void uart1_putstr( const char *str )
{
    while( *str )
    {
        uart1_putchar( *str++ );
    }
}

char uart1_getchar( void )
{
    if( rx_buf_head == rx_buf_tail )
        return 0;
    else
    {
        uint8_t buf = rx_buf[rx_buf_tail];
        rx_buf_tail = ( rx_buf_tail + 1 ) % LOT_UART1_RX_BUF_SIZE;
        return buf;
    }
}

uint16_t uart1_available( void )
{
    return ( LOT_UART1_RX_BUF_SIZE + rx_buf_head - rx_buf_tail ) % LOT_UART1_RX_BUF_SIZE;
}

void uart1_flush( void )
{
    rx_buf_head = 0;
    rx_buf_tail = 0;
    rx_buf[0]   = 0;
}

void uart1_rxne_isr( void )
{
    if( LL_USART_IsActiveFlag_RXNE( USART1 ) )
    {
        rx_buf[rx_buf_head] = LL_USART_ReceiveData8( USART1 );
        rx_buf_head         = ( rx_buf_head + 1 ) % LOT_UART1_RX_BUF_SIZE;
        if( rx_buf_head == rx_buf_tail )
        {
            rx_buf_tail = ( rx_buf_tail + 1 ) % LOT_UART1_RX_BUF_SIZE;
        }
    }
}

void uart1_txe_isr( void )
{
    if( LL_USART_IsActiveFlag_TXE( USART1 ) && LL_USART_IsEnabledIT_TXE( USART1 ) )
    {
        LL_USART_TransmitData8( USART1, tx_buf[tx_buf_tail] );
        tx_buf_tail = ( tx_buf_tail + 1 ) % LOT_UART1_TX_BUF_SIZE;
        if( tx_buf_head == tx_buf_tail )
        {
            LL_USART_DisableIT_TXE( USART1 );
        }
    }
}

void uart1_print_u32( uint32_t num )
{
    uint32_t temp;
    char     buf[11];
    char *   str = &buf[10];
    *str         = '\0';

    do
    {
        temp           = num / 10;
        char remainder = ( num - temp * 10 ) + '0';
        *--str         = remainder;
        num            = temp;
    } while( num );

    uart1_print_str( str );
}

void uart1_print_s32( int32_t num )
{
    if( num < 0 )
    {
        uart1_putchar( '-' );
        num = -num;
    }

    uart1_print_u32( ( uint32_t )num );
}

void uart1_print_float( float num, uint16_t digits )
{
    if( isnan( num ) )
    {
        uart1_print_str( "nan" );
        return;
    }
    else if( isinf( num ) )
    {
        uart1_print_str( "inf" );
        return;
    }
    else if( num > 4294967040.0 )
    {
        uart1_print_str( "ovf" );
        return;
    }
    else if( num < -4294967040.0 )
    {
        uart1_print_str( "ovf" );
        return;
    }

    if( num < 0.0 )
    {
        uart1_putchar( '-' );
        num = -num;
    }

    float rounding = 0.5;
    for( uint8_t i = 0; i < digits; ++i )
    {
        rounding /= 10.0;
    }
    num += rounding;

    uint32_t integer = ( uint32_t )num;
    uart1_print_u32( integer );
    uart1_putchar( '.' );

    float decimal = num - ( float )integer;

    while( digits-- )
    {
        decimal *= 10.0;
        uint8_t to_print = ( uint8_t )( decimal );
        uart1_putchar( to_print + '0' );
        decimal -= to_print;
    }
}