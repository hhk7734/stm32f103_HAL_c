/**
 * @file LOT_time.h
 * @author Hyeon-ki, Hong (hhk7734@gmail.com)
 * @brief Functions to check and delay time.
 */

#ifndef _LOT_TIME_H_
#define _LOT_TIME_H_
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define COTEX_TIMER_SYSTEM_DIV 1UL
#define SYSTEM_CORE_CLOCK 72000000UL
#define COUNT_PER_MICROSECOND ( SYSTEM_CORE_CLOCK / 1000000UL / COTEX_TIMER_SYSTEM_DIV )
#define ASSEMBLY_CYCLE 6UL
#define US_TO_CYCLE ( SYSTEM_CORE_CLOCK / 1000000UL / ASSEMBLY_CYCLE )

uint32_t millis( void );
uint32_t micros( void );
void     delay_ms( uint32_t msec );
void     delay_us( uint32_t usec );

#ifdef __cplusplus
}
#endif
#endif    // _LOT_TIME_H_
