/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * File: sf_peripheries.c
 *
 * This file implements functions for SPI and GPIO operations required by SIGFOX
 * driver. It is closely related to this demo example.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "sf_peripheries.h"

#include "fsl_gpio.h"
#include "fsl_spi.h"

#include "peripherals.h"
#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_MCU_Assert
 * Description   : User implementation of assert.
 *
 *END**************************************************************************/
void SF_MCU_Assert(bool x)
{
    assert(x);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_MCU_TransferSpi
 * Description   : This function performs SPI blocking transfer.
 *
 *END**************************************************************************/
sf_status_t SF_MCU_TransferSpi(uint8_t drvInstance, uint8_t txBuffer[],
    uint8_t rxBuffer[], uint16_t dataSize)
{
    assert(drvInstance == 0);

    status_t error;
    spi_transfer_t xfer;

    xfer.txData = txBuffer;
    xfer.rxData = rxBuffer;
    xfer.dataSize = dataSize;

    error = SPI_MasterTransferBlocking(SF_SPI_PERIPHERAL, &xfer);

    return (error == kStatus_Success) ? sfStatusSuccess : sfStatusSpiError;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_MCU_WriteCsPin
 * Description   :
 *
 *END**************************************************************************/
void SF_MCU_WriteCsPin(uint8_t drvInstance, uint8_t value)
{
    assert(drvInstance == 0);

    GPIO_PinWrite(BOARD_INITPINS_SF_CS_GPIO, BOARD_INITPINS_SF_CS_PIN, value);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_MCU_ReadAckPin
 * Description   : Reads logic value of ACK pin.
 *
 *END**************************************************************************/
uint32_t SF_MCU_ReadAckPin(uint8_t drvInstance)
{
    assert(drvInstance == 0);

    return GPIO_PinRead(BOARD_INITPINS_SF_ACK_GPIO, BOARD_INITPINS_SF_ACK_PIN);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
