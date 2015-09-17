/* =======================================================================
 * ps2d_xcvr_hal_avr.h
 *
 * Purpose:
 *  Implementation of the HAL for the PS/2 device subsystem for AVR 
 *  devices.
 *  Currently supports:
 *    ATTiny85
 *    ATMega88
 *
 * License:
 *  Copyright (c) 2015, Benjamin G. Rockwell
 *  All rights reserved.
 *  See LICENSE.txt for license details.
 * ----------------------------------------------------------------------- */ 


#ifndef PS2D_XCVR_HAL_AVR_H_
#define PS2D_XCVR_HAL_AVR_H_

#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "board.h"
#include "ps2d_xcvr_config.h"

static inline void Ps2dXcvrHal_BusTimerInit()
{
    /* Configure Timer0 to generate PS/2 clock */
    TCCR0A = (1 << WGM01); // CTC mode
    TCCR0B = 0x00; // Timer stopped

    /* Set timer compare values */
    OCR0A = PS2D_XCVR_PULSE_WIDTH;
    OCR0B = PS2D_XCVR_PULSE_WIDTH / 2;
    
    TCNT0 = 0;
}


static inline void Ps2dXcvrHal_BusTimerStart()
{

    /* Reset counter */
    TCNT0 = 0;

    /* Clear the interrupt flag */
    TIFR = (1<<OCF0A | 1 << OCF0B);

    /* Enable interrupt on match */
#if defined ATMEGA88
    TIMSK0 |= 1 << OCIE0A |  1 << OCIE0B;
#elif defined ATTINY85
    TIMSK |= 1 << OCIE0A |  1 << OCIE0B; 
#endif


    /* Start timer */
    TCCR0B = 1 << CS01; // Start timer at CLK/8



}

static inline void Ps2dXcvrHal_BusTimerStop()
{

    /* Stop timer */
    TCCR0B = 0 << CS01;

    /* Disable interrupt */
    TIMSK &= ~(1 << OCIE0A |  1 << OCIE0B); 

}
static inline void Ps2dXcvrHal_RtsInterruptInit()
{
#ifdef ATTINY85
    PCMSK = (1 << PCINT3);
#endif
}

static inline void Ps2dXcvrHal_RtsInterruptEnable()
{
#ifdef ATTINY85

    GIFR = (1 << PCIF); /* Clear pin change interrupt flag */
    GIMSK |= (1 << PCIE); /* Enable pin change interrupt */
#endif
}
static inline void Ps2dXcvrHal_RtsInterruptDisable()
{
#ifdef ATTINY85
    GIMSK &= ~(1 << PCIE); /* Disable pin change interrupt */
#endif

}

static inline bool Ps2dXcvrHal_ClockIsHigh()
{
    return (PS2D_CLOCK_PINS & (1 << PS2D_CLOCK_BIT));
}


static inline void Ps2dXcvrHal_ClockHigh()
{
    /* If current pin direction is output change to input */
    if (PS2D_CLOCK_DDR & (1 << PS2D_CLOCK_BIT))
        PS2D_CLOCK_DDR  &= ~(1<<PS2D_CLOCK_BIT);

    /* Turn on internal pullup to go to high-Z state */
    PS2D_CLOCK_PORT |=  (1<<PS2D_CLOCK_BIT); 
      
}
static inline void Ps2dXcvrHal_ClockLow()
{
    PS2D_CLOCK_PORT &= ~(1<<PS2D_CLOCK_BIT);
    PS2D_CLOCK_DDR  |=  (1<<PS2D_CLOCK_BIT);   
}

static inline void Ps2dXcvrHal_ClockInit()
{
}

static inline bool Ps2dXcvrHal_DataIsHigh()
{
    return (PS2D_DATA_PINS & (1 << PS2D_DATA_BIT));
}

static inline void Ps2dXcvrHal_DataHigh()
{
    /* If current pin direction is output change to input */
    if (PS2D_DATA_DDR & (1 << PS2D_DATA_BIT))
        PS2D_DATA_DDR  &= ~(1<<PS2D_DATA_BIT);

    /* Turn on internal pullup to go to high-Z state */
    PS2D_DATA_PORT |=  (1<<PS2D_DATA_BIT); 
}

static inline void Ps2dXcvrHal_DataLow()
{
    PS2D_DATA_PORT &= ~(1<<PS2D_DATA_BIT);
    PS2D_DATA_DDR  |=  (1<<PS2D_DATA_BIT);   
}

static inline void Ps2dXcvrHal_DataInit()
{
}

#define PS2D_XCVR_CLOCK_ISR() ISR(PS2D_CLOCK_INTERRUPT_VECTOR)
#define PS2D_XCVR_DATA_ISR() ISR(PS2D_DATA_INTERRUPT_VECTOR)
#define PS2D_XCVR_RTS_ISR() ISR(PS2D_RTS_INTERRUPT_VECTOR)

#endif /* PS2D_XCVR_HAL_AVR_H_ */