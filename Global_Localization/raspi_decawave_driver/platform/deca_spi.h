/*******************************************************************************
 * @file    deca_spi.h                                                         *
 * @brief   Platform-specific SPI functions                                    *
 *                                                                             *
 * The initialization of the functions that are called by our platform         *
 * specific initialization routine.                                            * 
 *                                                                             *
 * @author Vivek Sridhar <vivek4830@gmail.com>                                 *
 ******************************************************************************/

#ifndef _DECA_SPI_H_
#define _DECA_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "deca_types.h"

// Max number of bytes in a message header
#define DECA_MAX_SPI_HEADER_LENGTH (3)

int openspi(void);

int closespi(void);

#ifdef __cplusplus
}
#endif

#endif /* _DECA_SPI_H_ */
