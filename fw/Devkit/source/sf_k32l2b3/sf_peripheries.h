/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * File: sf_peripheries.h
 *
 * This file implements functions for SPI and GPIO operations required by SIGFOX
 * driver. It is closely related to this demo example.
 */

#ifndef SF_PERIPHERIES_H_
#define SF_PERIPHERIES_H_

#include "sf.h"

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @addtogroup function_group
 * @{
 */

/*!
 * @brief Implementation of assert.
 *
 * @param x - True if everything is OK.
 */
void SF_MCU_Assert(bool x);

/*!
 * @brief This function performs SPI blocking transfer.
 *
 * @param drvInstance Instance of SIGFOX driver.
 * @param txBuffer    Pointer to data buffer to be sent. It is NULL for ACK SPI
 *                    frames.
 * @param rxBuffer    Pointer to data buffer for received data. It is NULL for
 *                    CMD SPI frames.
 * @param dataSize Number of bytes to be transmitted.
 *
 * @return sf_status_t Returns sfStatusSuccess on success, sfStatusSpiError
 *         otherwise.
 */
sf_status_t SF_MCU_TransferSpi(uint8_t drvInstance, uint8_t txBuffer[],
        uint8_t rxBuffer[], uint16_t dataSize);

/*!
 * @brief Writes logic 0 or 1 to the CS pin.
 *
 * @param drvInstance Instance of SIGFOX driver.
 * @param value - Zero or one to be set to CS pin.
 */
void SF_MCU_WriteCsPin(uint8_t drvInstance, uint8_t value);

/*!
 * @brief Reads logic value of ACK pin.
 *
 * @param drvInstance Instance of SIGFOX driver.
 *
 * @return Zero value for logic zero, one for the logic one.
 */
uint32_t SF_MCU_ReadAckPin(uint8_t drvInstance);
/*! @} */

#endif /* SF_PERIPHERIES_H_ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
