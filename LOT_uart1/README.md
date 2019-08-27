# STM32 LL UART(using interrupts)

## STM32CubeMX

  * Pinout & Configuration -> Connectivity -> USARTx
    * Mode
      * Mode: Asynchronous
    * Configuration
      * Parameter Settings
        * Baudrate
        * Word Length: 8 Bits
        * Parity: None
        * Stop Bits: 1
      * NVIC settings
        * USARTx global interrupt: Enabled
  * Project Manager
    * Code Generator -> Generate peripheral initialization as a pair of '.c/.h' files per peripheral: check
    * Advanced Settings -> USART -> USARTx -> LL

## Edit stm32xxx_it.c

```c
/* USER CODE BEGIN Includes */
#include "LOT_uart1.h"
/* USER CODE END Includes */

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
    uart1_rxne_isr();
    uart1_txe_isr();
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}
```

## Examples

```c
...

/* USER CODE BEGIN Includes */
#include "LOT_uart1.h"
/* USER CODE END Includes */

...

int main( void )
{
    ...

    /* USER CODE BEGIN 2 */
    uart1_init();

    uart1_putstr( "hello world!\r\n" );
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while( 1 )
    {
        if( uart1_available() )
        {
            uart1_putchar( uart1_getchar() );
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

...
```
