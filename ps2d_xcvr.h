/* =======================================================================
 * ps2d_xcvr.h
 *
 * Purpose:
 *  Transmit and receive data using the PS/2 device bus.
 *
 * License:
 *  Copyright (c) 2015, Benjamin G. Rockwell
 *  All rights reserved.
 *  See LICENSE.txt for license details.
 * --------------------------------------------------------------------- */ 
#include <stdint-gcc.h>
#include <stdbool.h>


#ifndef PS2D_TRANSEIVER_H_
#define PS2D_TRANSEIVER_H_

/* === Type Definitions ================================================ */
typedef enum
{
    PS2D_XCVR_RECV_DATA_READY       = (1 << 1),
    PS2D_XCVR_RECV_BUFFER_OVERFLOW  = (1 << 2),
    PS2D_XCVR_RECV_FRAME_ERROR      = (1 << 3),
    PS2D_XCVR_XMIT_INTERRUPTED      = (1 << 4),
    PS2D_XCVR_BUS_BUSY              = (1 << 7)

}Ps2dXcvrStatus;

/* === External Variables ============================================== */
extern volatile uint8_t _ps2dXcvrStatus;

/* === Forward declarations ============================================ */

/* -----------------------------------------------------------------------
* Description:
*  Initializes the PS/2 Transiever module be configuring the PS/2 bus
*  bus lines, waiting for a specified start up delay to expire and
*  setting up interrupt to listen for data from the host.
*
* Returns:
*  N/A
* . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
void Ps2dXcvr_Init();

/* -----------------------------------------------------------------------
* Description:
*  Attempts to begin transmission of the specified byte of data to the
*  PS/2 host using the PS/2 bus.
*
* Parameters:
*  data - the byte of data to be transmitted.
*
* Returns: bool
*  true  - if transmission was successfully initiated.
*  false - otherwise.
* . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
bool Ps2dXcvr_TransmitDataAsync(uint8_t data);

/* -----------------------------------------------------------------------
* Description:
*  Reads a byte of data that has been received from the PS/2 host.
*
* Parameters:
*  n/a
*
* Returns: uint8_t
*  The byte of data received.
* . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
uint8_t Ps2dXcvr_ReadReceivedData();

/* -----------------------------------------------------------------------
* Description:
*  Busy waits until the PS/2 bus is in the IDLE state.
*
* Parameters:
*  n/a
*
* Returns: 
*  n/a
* . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
void Ps2dXcvr_WaitBusIdle();


/* === Forward declarations ============================================ */

/* -----------------------------------------------------------------------
* Description:
*  Returns the status value of the PS/2 Device Transiever.
*
* Parameters:
*  n/a
*
* Returns: Ps2dXcvrStatus
*  The status value of the PS/2 Device Transiever.
* . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline Ps2dXcvrStatus GetStatus()
{
    return (Ps2dXcvrStatus)_ps2dXcvrStatus;
}

/* -----------------------------------------------------------------------
* Description:
*  Determines if a specified status flag is currently set.
*
* Parameters:
*  status - one or more of the Ps2dXcvrStatus status flags.
*
* Returns: bool
*  true  - if the specified status flag(s) is/are set.
*  false - otherwise.
* . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline bool StatusIsSet(Ps2dXcvrStatus status)
{
    return (bool)((uint8_t)_ps2dXcvrStatus & status);
}

/* -----------------------------------------------------------------------
* Description:
*  Determines if the Transiever is actively transmitting or receiving data
*  on the PS/2 bus.
*
* Parameters:
*  n/a
*
* Returns: bool
*  true  - if the Transiever is transmitting or receiving data.
*  false - otherwise.
* . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline bool Ps2dXcvr_BusActive()
{
    return StatusIsSet(PS2D_XCVR_BUS_BUSY);
}

/* -----------------------------------------------------------------------
* Description:
*  Determines if the Transiever has received data from the PS/2 host.
*
* Parameters:
*  n/a
*
* Returns: bool
*  true  - if the Transiever has received data.
*  false - otherwise.
* . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline bool Ps2dXcvr_DataReceived()
{
    return StatusIsSet(PS2D_XCVR_RECV_DATA_READY | PS2D_XCVR_RECV_BUFFER_OVERFLOW);
}


#endif /* PS2D_TRANSEIVER_H_ */