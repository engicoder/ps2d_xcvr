/* =======================================================================
 * ps2d_xcvr_hal.h
 *
 * Purpose:
 *  Declares the interface to the underlying hardware resources used by the 
 *  PS/2 device subsystem.
 *
 * License:
 *  Copyright (c) 2015, Benjamin G. Rockwell
 *  All rights reserved.
 *  See LICENSE.txt for license details.
 * ----------------------------------------------------------------------- */ 


#ifndef PS2D_XCVR_HAL_H_
#define PS2D_XCVR_HAL_H_

#include <stdbool.h>

/* Pulse width definitions */
#define PS2D_XCVR_PULSE_WIDTH 45U
#define PS2D_XCVR_MAX_PULSE_WIDTH 50U

/* Forward declaration of functions that must be implemented by the HAL */
static inline void Ps2dXcvrHal_BusTimerInit();
static inline void Ps2dXcvrHal_BusTimerStart();
static inline void Ps2dXcvrHal_BusTimerStop();

static inline void Ps2dXcvrHal_RtsInterruptEnable();
static inline void Ps2dXcvrHal_RtsInterruptDisable();

static inline bool Ps2dXcvrHal_ClockIsHigh();
static inline void Ps2dXcvrHal_ClockHigh();
static inline void Ps2dXcvrHal_ClockLow();
static inline void Ps2dXcvrHal_ClockInit();

static inline bool Ps2dXcvrHal_DataIsHigh();
static inline void Ps2dXcvrHal_DataHigh();
static inline void Ps2dXcvrHal_DataLow();
static inline void Ps2dXcvrHal_DataInit();


/* Include the appropriate HAL implementation */
#ifdef AVR
    #include "ps2d_xcvr_hal_avr.h"
#else
    #error No HAL implementation for Ps2 Device defined.
#endif

/* Ensure that the HAL implementation has defined the ISRs */
#if !defined(PS2D_XCVR_CLOCK_ISR)
    #error PS2D_XCVR_CLOCK_ISR not defined.
#endif

#if !defined(PS2D_XCVR_DATA_ISR)
    #error PS2D_XCVR_DATA_ISR not defined.
#endif

#if !defined(PS2D_XCVR_RTS_ISR)
    #error PS2D_XCVR_RTS_ISR not defined.
#endif

/* -----------------------------------------------------------------------
 * Description:
 *  Initializes the PS/2 bus timer used to generate PS/2 CLOCK signals and
 *  write and read data using the DATA line via the CLOCK and DATA ISRs.
 *
 * Returns:
 *  N/A
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_BusTimerInit()
{
    Ps2dXcvrHal_BusTimerInit();
}

/* -----------------------------------------------------------------------
 * Description:
 *  Starts the PS/2 bus timer.
 *
 * Returns:
 *  N/A
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_BusTimerStart()
{
    Ps2dXcvrHal_BusTimerStart();
}

/* -----------------------------------------------------------------------
 * Description:
 *  Stops the PS/2 bus timer. 
 *
 * Returns:
 *  N/A
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_BusTimerStop()
{
    Ps2dXcvrHal_BusTimerStop();
}

/* -----------------------------------------------------------------------
 * Description:
 *  Initializes the Request-To-Send (RTS) interrupt. 
 *
 * Returns:
 *  N/A
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */

static inline void Ps2dXcvr_RtsInterruptInit()
{
    Ps2dXcvrHal_RtsInterruptInit();
}

/* -----------------------------------------------------------------------
 * Description:
 *  Enables the Request-To-Send (RTS) interrupt. 
 *
 * Returns:
 *  N/A
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_RtsInterruptEnable()
{
    Ps2dXcvrHal_RtsInterruptEnable();
}

/* -----------------------------------------------------------------------
 * Description:
 *  Disables the Request-To-Send (RTS) interrupt. 
 *
 * Returns:
 *  N/A
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_RtsInterruptDisable()
{
    Ps2dXcvrHal_RtsInterruptDisable();
}


/* -----------------------------------------------------------------------
 * Description:
 *   Initializes the CLOCK line.
 *
 * Returns: 
 *   n/a
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_ClockInit()
{
    Ps2dXcvrHal_ClockInit();
}

/* -----------------------------------------------------------------------
 * Description:
 *   Determines if PS/2 CLOCK line is HIGH. This should only be called if
 *   the direction of the CLOCK line is INPUT.
 *
 * Returns: 
 *   n/a
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline bool Ps2dXcvr_ClockIsHigh()
{
    return Ps2dXcvrHal_ClockIsHigh();
}

/* -----------------------------------------------------------------------
 * Description:
 *   Sets the CLOCK line to HIGH by setting the pin direction to INPUT and
 *   enabling the internal pull-up resistor.

 *
 * Returns: 
 *   n/a
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_ClockHigh()
{
    Ps2dXcvrHal_ClockHigh();
}

/* -----------------------------------------------------------------------
 * Description:
 *   Sets the CLOCK line to LOW by setting the pin direction to OUTPUT and
 *   setting the pin to LOW.
 *
 * Returns: 
 *   n/a
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_ClockLow()
{
    Ps2dXcvrHal_ClockLow();
}


/* -----------------------------------------------------------------------
 * Description:
 *   Determines if PS/2 DATA line is HIGH. This should only be called if
 *   the direction of the DATA line is INPUT.
 *
 * Returns: 
 *   n/a
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline bool Ps2dXcvr_DataIsHigh()
{
    return Ps2dXcvrHal_DataIsHigh();
}

/* -----------------------------------------------------------------------
 * Description:
 *   Sets the DATA line to HIGH by setting the pin direction to INPUT and
 *   enabling the internal pull-up resistor.
 *
 * Returns: 
 *   n/a
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_DataHigh()
{
    Ps2dXcvrHal_DataHigh();
}

/* -----------------------------------------------------------------------
 * Description:
 *   Sets the DATA line to LOW by setting the pin direction to OUTPUT and
 *   setting the pin to LOW.
 *
 * Returns: 
 *   n/a
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_DataLow()
{
    Ps2dXcvrHal_DataLow();
}

/* -----------------------------------------------------------------------
 * Description:
 *   Initializes the DATA line.
 *
 * Returns: 
 *   n/a
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void Ps2dXcvr_DataInit()
{
    Ps2dXcvrHal_DataInit();
}





#endif /* PS2D_XCVR_HAL_H_ */