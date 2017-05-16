/*******************************************************************************
 * @file raspi_init.h                                                          *
 * @brief Declaration of platform-specific initialization functions            *
 *                                                                             *
 * @author Vivek Sridhar <vivek4830@gmail.com>                                 *
 ******************************************************************************/

#ifndef _RASPI_INIT_H_
#define _RASPI_INIT_H_

// IRQ pin from the Decwave chip. Change based on hardware configuration
#define DWM_INTERRUPT_PIN 22

// Reset pin of the Decawave chip. Change based on hardware configuration
#define DWM_RESET_PIN 27

void reset_decawave();

void raspi_decawave_init();

#endif /* _RASPI_INIT_H_ */
