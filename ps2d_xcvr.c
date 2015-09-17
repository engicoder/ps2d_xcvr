/* =======================================================================
 * ps2d_xcvr.c
 *
 * Purpose:
 *  Transmit and receive data using the PS/2 device bus.
 *
 * License:
 *  Copyright (c) 2015, Benjamin G. Rockwell
 *  All rights reserved.
 *  See LICENSE.txt for license details.
 * ----------------------------------------------------------------------- */ 
#include "ps2d_xcvr.h"
#include "ps2d_xcvr_config.h"
#include "ps2d_xcvr_hal.h"
#include <util\atomic.h>

/* Declaration of the possible values for the Ps2dXcvr state */
typedef enum {
    IDLE, INHIBIT,
    RECV_START,   XMIT_START, 
    RECV_DATA0,   XMIT_DATA0, 
    RECV_DATA1,   XMIT_DATA1, 
    RECV_DATA2,   XMIT_DATA2, 
    RECV_DATA3,   XMIT_DATA3, 
    RECV_DATA4,   XMIT_DATA4,
    RECV_DATA5,   XMIT_DATA5, 
    RECV_DATA6,   XMIT_DATA6, 
    RECV_DATA7,   XMIT_DATA7, 
    RECV_PARITY,  XMIT_PARITY, 
    RECV_STOP,    XMIT_STOP, 
    RECV_ACK,     XMIT_COMPLETE,
    RECV_COMPLETE,
} Ps2dXcvrState;

typedef enum {
    LOW = 0x00,
    HIGH = 0x01,
} LineState;

/* Definition of values used by BusWaitIdle() function */
#define PS2D_XCVR_WAIT_IDLE_THRESHOLD 5 /* milliseconds */
#define PS2D_XCVR_WAIT_IDLE_INTERVAL 100 /* microseconds */




/* Provides status to the caller regarding the state of Ps2dXcvr */
volatile uint8_t _ps2dXcvrStatus = 0x00;

/* Tracks the internal state of Ps2dXcvr */
static volatile uint8_t _state;

/* Used to transfer data in an out of the interrupt driven 
 * transmission routines */
static volatile uint8_t _xmitBuffer;
static volatile uint8_t _xmitRegister;
static volatile uint8_t _recvBuffer;
static volatile uint8_t _recvRegister;
static volatile uint8_t _idleCount;

static volatile LineState _clockState = HIGH;  /* Used by ISR to specify expected state of clock line */

/* ------------------------------------------------------------------------
 *  Convenience function to set a flag in status. The specified status flag
 *  is added to the status, no previous status is cleared.
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void SetStatus(Ps2dXcvrStatus status)
{
    _ps2dXcvrStatus |= (uint8_t)status;
}


/* ------------------------------------------------------------------------
 *  Convenience function to clear a flag in status. The specified status 
 *  flag is removed from the status, but other status flags are undisturbed.
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
 static inline void ClearStatus(Ps2dXcvrStatus status)
{
    _ps2dXcvrStatus &= ~status;
}

/* ------------------------------------------------------------------------
 *  Convenience function to advance the state of Ps2dXcvr. Used limit direct
 *  manipulation of the state variable in unexpected ways.
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void AdvanceState()
{
    _state += 2;
}

/* ------------------------------------------------------------------------
 *  Convenience function to set the state of Ps2dXcvr. Used limit direct
 *  manipulation of the state variable in unexpected ways.
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline void SetState(Ps2dXcvrState state)
{
    if (state == IDLE)
        XMIT_DATA_PORT |= (1 << XMIT_DATA_BIT);
    else
        XMIT_DATA_PORT &= ~(1 << XMIT_DATA_BIT);

    _state = state;
}

/* ------------------------------------------------------------------------
 *  Convenience function to check the state of Ps2dXcvr. Used limit direct
 *  manipulation of the state variable in unexpected ways.
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline bool IsState(Ps2dXcvrState state)
{
    return _state == state;
}

/* ------------------------------------------------------------------------
 *  Convenience function to check the state of Ps2dXcvr and determine if 
 *  it is in one of the XMIT states. Used limit direct manipulation of the 
 *  state variable in unexpected ways.
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
static inline bool IsStateTransmitting()
{
    
    return (_state & 0x01 && _state != INHIBIT);
}


/* ------------------------------------------------------------------------
 *  Initialize the PS/2 Device Transiever
 *   - Initialize the bus timer and lines
 *   - Set state to IDLE
 *   - Initialize and enable the Request-To-Send interrupt
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
void Ps2dXcvr_Init()
{
    /* Initialize the bus timer and clock and data lines */
    Ps2dXcvr_BusTimerInit();
    Ps2dXcvr_ClockInit();
    Ps2dXcvr_DataInit();    

    Ps2dXcvr_ClockLow();
    Ps2dXcvr_DataHigh();

    _delay_ms(PS2D_XCR_STARTUP_DELAY);

    Ps2dXcvr_ClockHigh();
    Ps2dXcvr_DataHigh();

    Ps2dXcvr_RtsInterruptInit();
    
    SetState(IDLE);

    _ps2dXcvrStatus = 0x00;

    Ps2dXcvr_RtsInterruptEnable();
}

/* ------------------------------------------------------------------------
 *  Returns the last received byte of data to the user and resets state to 
 *  IDLE.
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
uint8_t Ps2dXcvr_ReadReceivedData()
{
    uint8_t data;
    data = _recvBuffer;
    ClearStatus(PS2D_XCVR_RECV_BUFFER_OVERFLOW | 
                         PS2D_XCVR_RECV_DATA_READY | 
                         PS2D_XCVR_RECV_FRAME_ERROR);
    return data;
}

 /* ------------------------------------------------------------------------
 * Initiate the transmission of a byte of data across the PS/2 bus.
 *   The PS/2 device drives the clock line for both sending and received.
 *    - Load the data to be send into the send register
 *    - Initialize the ISR state
 *    - Pull the data line low to tell the host we are sending
 *    - Start the clock interrupt which drives the clock line. 
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
bool Ps2dXcvr_TransmitDataAsync (uint8_t data)
{
    /* Must be in IDLE state to begin transmission */
    if(!IsState(IDLE))
    {
        return false;
    }

    Ps2dXcvr_RtsInterruptDisable();

    /* Check state again to avoid race condition where RTS interrupt fired
     * after the initial check and initiated the INHIBIT state */
    if(!IsState(IDLE))
    {
        return false;
    }

    _xmitBuffer = data;

    ClearStatus(PS2D_XCVR_XMIT_INTERRUPTED);
    SetStatus(PS2D_XCVR_BUS_BUSY);

    _xmitRegister = _xmitBuffer;

    /* Reset the data state and expected clock */
    SetState(XMIT_START);
    _clockState = LOW;

    Ps2dXcvr_BusTimerStart();

    return true;
}

 /* ------------------------------------------------------------------------
 * Performs busy waiting until the PS/2 bus has remained in the IDLE state
 *  for a designated period of time.
 *   The PS/2 device drives the clock line for both sending and received.
 *    - Load the data to be send into the send register
 *    - Initialize the ISR state
 *    - Pull the data line low to tell the host we are sending
 *    - Start the clock interrupt which drives the clock line. 
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
void Ps2dXcvr_WaitBusIdle()
{
    _idleCount = 0;
    Ps2dXcvr_DataHigh();
    Ps2dXcvr_ClockHigh();

    while (_idleCount < PS2D_XCVR_WAIT_IDLE_THRESHOLD * 1000/PS2D_XCVR_WAIT_IDLE_INTERVAL)
    {
        ATOMIC_BLOCK(ATOMIC_FORCEON)
        {
            _idleCount++;
        }
        _delay_us(PS2D_XCVR_WAIT_IDLE_INTERVAL);
    }

    return;
}



 /* ------------------------------------------------------------------------
 *  ISR for the PS/2 data line - transmits and receives a byte of data using
 *  the PS/2 bus.
 *   This ISR is triggered by the timer 1/2 pulse width after the clock
 *   line interrupt is triggered to change state of the clock line. 
 *   (i.e. in the middle of each clock pulse) The host latches/reads data 
 *   on the rising edge of the clock pulse, and the device does the same on 
 *   falling edge, so reading/writing data at the middle of pulse guarantees 
 *   the data line state will have settled when the clock line transition
 *   occurs.
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
PS2D_XCVR_DATA_ISR()
{
    static uint8_t parity = 0;
    static uint8_t bit = 0;

    if(_clockState != HIGH) 
    {
        return;
    }    

    bool dataLineHigh = Ps2dXcvr_DataIsHigh();
    uint8_t bitValue = 0x00;
    if (dataLineHigh)
    {
        bitValue = 0x01;
    }

    switch(_state)
    {
        case RECV_START:
            if (dataLineHigh)
            {
                SetStatus(PS2D_XCVR_RECV_FRAME_ERROR);
                goto STOP_BUS_TIMER_IDLE;
            }            
            _recvRegister = 0;
            bit = 0;
            parity = 0;                
            break;
        case RECV_DATA0:
        case RECV_DATA1:
        case RECV_DATA2:
        case RECV_DATA3:
        case RECV_DATA4:
        case RECV_DATA5:
        case RECV_DATA6:
        case RECV_DATA7:   
            {
                _recvRegister |= (bitValue << bit);
                parity += bitValue;
                bit++;
            }     
            break;
        case RECV_PARITY:   
            if (bitValue == parity % 2)
            {
                SetStatus(PS2D_XCVR_RECV_FRAME_ERROR);
                goto STOP_BUS_TIMER_IDLE;
            }
            break;
        case RECV_STOP:
            Ps2dXcvr_DataLow(); // Set ACK
            break;
        case RECV_ACK:
            Ps2dXcvr_DataHigh(); // Release ACK
            _delay_us(1);
            if (!Ps2dXcvr_DataIsHigh())
            {
                SetStatus(PS2D_XCVR_RECV_FRAME_ERROR);
                SetState(RECV_STOP);
            }
            else
            {
                if(!StatusIsSet(PS2D_XCVR_RECV_FRAME_ERROR))
                {
                    if (_ps2dXcvrStatus & PS2D_XCVR_RECV_DATA_READY)
                        SetStatus(PS2D_XCVR_RECV_BUFFER_OVERFLOW);
                    else
                        SetStatus(PS2D_XCVR_RECV_DATA_READY);
                    _recvBuffer = _recvRegister;
                }
                goto STOP_BUS_TIMER_IDLE;
            }
            break;
            
        case XMIT_START:
                Ps2dXcvr_DataLow();
                bit = 0;
                parity = 0;
            break;
        case XMIT_DATA0:
        case XMIT_DATA1:
        case XMIT_DATA2:
        case XMIT_DATA3:
        case XMIT_DATA4:
        case XMIT_DATA5:
        case XMIT_DATA6:
        case XMIT_DATA7:
            {
                if (_xmitRegister & ( 1 << bit)) 
                {
                    Ps2dXcvr_DataHigh();
                    parity++;
                }     
                else 
                    Ps2dXcvr_DataLow();
                bit++;
            }                        
            break;
        case XMIT_PARITY: // Odd Parity - Set data low (0) if odd number of 1's
            if (parity % 2)
                Ps2dXcvr_DataLow();
            else
                Ps2dXcvr_DataHigh();
            break;
        case XMIT_STOP:
            Ps2dXcvr_DataHigh();
            break;
        case XMIT_COMPLETE:
            /* Host may hold clock low while processing data. If clock
             * is low, set INHIBIT state and let RTS ISR handle wait */
            if (!Ps2dXcvr_ClockIsHigh())
            {
                goto STOP_BUS_TIMER_INHIBIT;
            }
            
            goto STOP_BUS_TIMER_IDLE;
            break;
    }    

    AdvanceState();
    return;

  STOP_BUS_TIMER_IDLE:
    SetState(IDLE);
    goto STOP_BUS_TIMER;
  STOP_BUS_TIMER_INHIBIT:
    SetState(INHIBIT);
    goto STOP_BUS_TIMER;
    
  STOP_BUS_TIMER:
    Ps2dXcvr_BusTimerStop();
    ClearStatus(PS2D_XCVR_BUS_BUSY);
    Ps2dXcvr_RtsInterruptEnable();

    return;
}

/* ------------------------------------------------------------------------
 *  ISR for the PS/2 clock line - generates clock signal
 *    This ISR is triggered by the timer once for each transition of the 
 *    clock line. 
 *    - The value of the clock state variable dictates the transition 
 *      to occur.
 *    - If transmitting, check for host Request-To-Send.
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
PS2D_XCVR_CLOCK_ISR()
{

    if (_clockState == HIGH)
    {
        /* If we are transmitting and the clock line should be HIGH.
         * If the clock line is not HIGH, the host is requesting to send;
         * Abort transmission. */
        if (IsStateTransmitting() && !IsState(XMIT_COMPLETE) && !Ps2dXcvr_ClockIsHigh())
        {
            /* Stop transmission and set INTERRUPTED status */
            Ps2dXcvr_BusTimerStop();
            SetState(IDLE);
            ClearStatus(PS2D_XCVR_BUS_BUSY);
            SetStatus(PS2D_XCVR_XMIT_INTERRUPTED);

            /* Enable the Request-To-Send interrupt */
            Ps2dXcvr_RtsInterruptEnable();

            /* Release the data line (clock line is already released) */
            Ps2dXcvr_DataHigh();

            return;
        }

        Ps2dXcvr_ClockLow();
        _clockState = LOW;
        return;
    }
    
    _clockState = HIGH;
    Ps2dXcvr_ClockHigh();

    return;
}

/* ------------------------------------------------------------------------
 *  ISR to detect the host Request-To-Send (RTS) condition.
 *    The host initiates a Request-To-Send by pulling the clock line low 
 *    for 100 microseconds to cancel any transmission in progress and inhibit 
 *    further transmissions. It then pulls the data line low and releases 
 *    the clock line. 
 *    The Request-To-Send (RTS) ISR is triggered when the host pulls the 
 *    data line low.
 * . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . */
PS2D_XCVR_RTS_ISR()
{
    _idleCount = 0;

#ifdef ATTINY85
    /* On the ATTiny85, we are using the pin change interrupt that is 
     * triggered when the CLOCK line changes state.  This means 
     * that we could arrive in this ISR on either the rising or falling edge.
     * If the CLOCK is low, the host is inhibiting activity on the bus, set
     * state to INHIBIT. Otherwise the host has released the CLOCK line, and
     * the DATA line must be checked for the RTS condition.
     * i.e. when the CLOCK line is high */
    if (!Ps2dXcvr_ClockIsHigh())
    {   
        SetState(INHIBIT);
        return;
    }
    
    /* If the host releases the CLOCK and the DATA line is low, the 
     * Request-To-Send condition exists. Check to see if DATA is low.
     * If not, reset state to IDLE and return. */
    if (Ps2dXcvr_DataIsHigh())
    {
        SetState(IDLE);
        return;
    }
#endif


    /* Initialize the receive state */
    SetState(RECV_START);
    _clockState = HIGH;

    SetStatus(PS2D_XCVR_BUS_BUSY);

    /* Disable the Request-To-Send interrupt while receiving */
    Ps2dXcvr_RtsInterruptDisable();

    /* Start the timer that drives the clock and data interrupts */
    Ps2dXcvr_BusTimerStart();

}