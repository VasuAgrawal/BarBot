/*******************************************************************************
 * @file    deca_sleep.c                                                       *
 * @brief   Platform specific sleep implementation                             *
 *                                                                             *
 * This file contains the sleep implementation for the raspberry pi, which     *
 * simply calls the unix sleep function.                                       *
 *                                                                             *
 * @author Vivek Sridhar <vivek4830@gmail.com>                                 *
 ******************************************************************************/

#include <unistd.h>

#include "deca_device_api.h"

/**
 * @function deca_sleep
 *
 * Sleeps for the provdied number of milliseconds
 */
void deca_sleep(unsigned int time_ms) 
{
    usleep(time_ms * 1000);
}
