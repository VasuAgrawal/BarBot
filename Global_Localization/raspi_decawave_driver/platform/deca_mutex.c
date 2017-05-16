/*******************************************************************************
 * @file	deca_mutex.c                                                       *
 * @brief	IRQ interface / mutex implementation                               *
 *                                                                             *
 * Raspberry Pi specific IRQ interface. The deca_mutex is used in the          *
 * platform-specific IRQ handler to mask SPI transactions when an IRQ is       *
 * fired.                                                                      *
 *                                                                             *
 * @author Vivek Sridhar <vivek4830@gmail.com>                                 *
 ******************************************************************************/

#include "deca_device_api.h"

/**
 * The code implementing the deca_mutex uses a simple flag that is set or unset
 * depending on whether the mutex is locked or unlocked. The SPI transaction
 * code locks and unlocks the mutex, to prevent us from receiving IRQs and
 * possibly conducting additional SPI transactions while another transaction is
 * ongoing. In the platform-specific IRQ, we check if this flag is set or not.
 */
extern int DECA_MUTEX_FLAG;

/**
 * @function decamutexon
 *
 * This function sets the flag to 0 so the IRQ is not handled, since a SPI
 * transaction is going on.
 */
decaIrqStatus_t decamutexon(void)           
{
    // Sets the mutex flag so dwt_isr() is not fired
    DECA_MUTEX_FLAG = 0;
}

/**
 * @function decamutexoff
 *
 * This function sets the flag to 1 so IRQs can be handled again. It is called
 * after the SPI transaction is completed.
 */
void decamutexoff(decaIrqStatus_t s)
{
    // Sets the mutex flag so dwt_isr() is fired again
    DECA_MUTEX_FLAG = 1;
}
