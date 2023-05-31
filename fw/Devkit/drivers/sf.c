/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file sf.c
 *
 * SIGFOX driver supporting boards based on OL2385 with FW version:
 * 2.3.0.26.11.20.20.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "sf.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/

/*! Timeout in [us] used to wait for an ACK SPI frame. Intended for SPI commands
 * causing transmission to SIGFOX network (TX_Frame, Bit_Frame, OOB_TX_Frame).
 * The TRX_Frame command has a higher timeout (see macro
 * SF_ACK_TRANS_RECV_TIMEOUT_US).
 * Note: Waiting for an acknowledge frame can take several seconds. */
#define SF_ACK_TRANS_TIMEOUT_US         20000000U

/*! Timeout in [us] used to wait for an ACK in case of TRX_Frame SPI command. */
#define SF_ACK_TRANS_RECV_TIMEOUT_US    70000000U

/*! Timeout in [us] used to wait for an ACK in case of SPI commands which do not
 * cause any transmission to the SIGFOX network (e.g. Get_Sigfox_ID,
 * Get_Initial_PAC, etc.). */
#define SF_ACK_NOTRANS_TIMEOUT_US       1000000U

/*! Waiting for ACK level is divided into smaller steps. This macro defines
 * value of this step in [us]. */
#define SF_TIMEOUT_STEP_US              20U

/*!
 * @brief This macro returns true when CMD frame has a payload (data except
 * length and command code fields).
 * Otherwise it returns false.
 * @param cmd Command number (see sf_spi_cmd_t enumeration).
 */
#define SF_HAS_CMD_PLD(cmd) \
    (((cmd) == sfSpiCmdTxFrame) || ((cmd) == sfSpiCmdTrxFrame) ||            \
     ((cmd) == sfSpiCmdBitFrame) || ((cmd) == sfSpiCmdOobTxFrame) ||         \
     ((cmd) == sfSpiCmdStartCont) || ((cmd) == sfSpiCmdTestMode) ||          \
     ((cmd) == sfSpiCmdStaticFreqCal) || ((cmd) == sfSpiCmdTempFreqCal) ||   \
     ((cmd) == sfSpiCmdGetLastRssi) || ((cmd) == sfSpiCmdRssiCalib) ||       \
     ((cmd) == sfSpiCmdSetRcSyncPer) || ((cmd) == sfSpiCmdSendTestFrame) ||  \
     ((cmd) == sfSpiCmdRecvTestFrame) || ((cmd) == sfSpiCmdChangeRc) || \
     ((cmd) == sfSpiCmdSetCustomRc) || ((cmd) == sfSpiCmdSetHwConfig) ||        \
     ((cmd) == sfSpiCmdSetTxConfig))

/*!
 * @brief This macro returns true when an acknowledgement is expected for a
 * given SPI command. Otherwise, false is returned.
 *
 * @param cmd Command number (see sf_spi_cmd_t enumeration).
 */
#define SF_HAS_CMD_ACK(cmd) \
    (((cmd) != sfSpiCmdWakeUp) && ((cmd) != sfSpiCmdSleep) && \
     ((cmd) != sfSpiCmdStartCont))

/*******************************************************************************
 * Prototypes of internal functions
 ******************************************************************************/

/*
 * @brief Extracts 4-byte unsigned integer from the uint8_t array (little endian).
 *
 * @param buffer uint8_t array.
 *
 * @return Extracted number.
 */
static uint32_t SF_StringLsbToUint32(const uint8_t* const buffer);

/*
 * @brief Stores 4-byte unsigned integer to the uint8_t array (little endian).
 *
 * @param n      Number to be stored in the array.
 * @param buffer uint8_t array.
 */
static void SF_Uint32ToStringLsb(const uint32_t n, uint8_t* const buffer);

/*
 * @brief Stores 4-byte signed integer to the uint8_t array (little endian).
 *
 * @param n      Number to be stored in the array.
 * @param buffer uint8_t array.
 */
static inline void SF_Int32ToStringLsb(const int32_t n, uint8_t* const buffer);

/*
 * @brief Creates the SPI frame according to the defined format.
 *
 * @param cmd        Command code.
 * @param payloadLen Size of the frame payload in bytes.
 * @param payload    Payload of the frame. Can be NULL if the command has no
 *                   payload or the payload will be added later.
 * @param frame      Pointer where the resulting frame will be stored. Length
 *                   of the array must at least '2U + payloadLen'.
 */
static void SF_PackFrame(const sf_spi_cmd_t cmd, const uint8_t payloadLen,
    const uint8_t* const payload, uint8_t frame[]);

/*
 * @brief Parses the error codes from a received ACK SPI frame.
 *
 * @param drvData   Driver run-time data.
 * @param spiRcvBuf Pointer to a SPI buffer containing received data.
 *
 * @return Status result of the function (sfStatusSuccess on success).
 */
static sf_status_t SF_ParseErrorCodes(sf_drv_data_t *drvData,
    const uint8_t *spiRcvBuf);

/*
 * @brief Parses payload from a received SPI ACK frame.
 *
 * @param rxFrame        Pointer to a SPI buffer containing received data.
 * @param payload        Pointer where a resulting payload will be stored.
 * @param payloadBufSize Size of the payload buffer in bytes.
 *
 * @return Status result of the function (sfStatusSuccess on success).
 */
static sf_status_t SF_ParsePayload(const uint8_t *rxFrame,
    sf_msg_payload_t *payload, uint8_t payloadBufSize);

/*
 * @brief Waits until ACK pin has a desired value.
 *
 * @param drvData  Driver run-time data.
 * @param expValue Desired level of the pin (1u - log. 1, 0u - log. 0).
 * @param tmoutUs  Timeout in [us].
 *
 * @return Status result of the function (sfStatusSuccess on success).
 */
static sf_status_t SF_WaitAckLevel(sf_drv_data_t *drvData, uint32_t expValue,
    uint32_t timeout);

/*
 * @brief Sends a command via SPI; MISO data is ignored.
 *
 * @param drvData  Driver run-time data.
 * @param txBuffer Pointer to data buffer to be sent.
 * @param dataSize Number of bytes to be transmitted.
 *
 * @return Status result of the function (sfStatusSuccess on success).
 */
static sf_status_t SF_SendCmdFrame(sf_drv_data_t *drvData, uint8_t txBuffer[],
    uint16_t dataSize);

/*
 * @brief Receives ACK frame via SPI.
 *
 * Note that it sends dummy data at MOSI.
 *
 * @param drvData    Driver run-time data.
 * @param rxBuffer   Pointer to data buffer for received data.
 * @param bufferSize RxBuffer size (maximal length of the ACK frame).
 * @param tmoutUs    Timeout in [us] when waiting for ACK pin to become LOW.
 *
 * @return Status result of the function (sfStatusSuccess on success).
 */
static sf_status_t SF_ReceiveAckFrame(sf_drv_data_t *drvData,
    uint8_t rxBuffer[], uint16_t dataSize, uint32_t tmoutUs);

/*
 * @brief Creates a SPI frame, sends a SPI command to the device,
 * receives an acknowledgement (if any) and stores an error code and state value
 * from the acknowledgement.
 *
 * @param drvData    Driver run-time data.
 * @param sendLen    Length of a frame to be sent (in bytes).
 * @param recvLen    Length of a frame to be received (in bytes).
 * @param sendBuffer Pointer to a SPI send buffer.
 * @param recvBuffer Pointer to a SPI receive buffer.
 * @param tmoutUs    Timeout in [us]. It is applied when waiting for the low
 *                   level of ACK pin.
 *
 * @return Status result of the function (sfStatusSuccess on success).
 */
static sf_status_t SF_SendCommandInt(sf_drv_data_t *drvData, uint8_t sendLen,
    uint8_t recvLen, uint8_t *sendBuffer, uint8_t *recvBuffer,
    uint32_t tmoutUs);

/*!
 * @brief Sends and receives data via SPI and checks if an acknowledgement has
 * an expected size.
 *
 * @param drvData    Driver run-time data.
 * @param sendLen    Length of a frame to be sent (in bytes).
 * @param recvLen    Length of a frame to be received (in bytes).
 * @param sendBuffer Pointer to a SPI send buffer.
 * @param recvBuffer Pointer to a SPI receive buffer.
 * @param tmoutUs    Timeout in ms. It is applied when waiting for the low level
 *                   of ACK pin.
 *
 * @return Status result of the function (sfStatusSuccess on success).
 */
static sf_status_t SF_SendAndCheck(sf_drv_data_t *drvData, uint8_t sendLen,
    uint8_t recvLen, uint8_t *sendBuffer, uint8_t *recvBuffer,
    uint32_t tmoutUs);

/*******************************************************************************
 * Internal functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_StringLsbToUint32
 * Description   : Extracts 4-byte unsigned integer from the uint8_t array
 *                 (little endian).
 *
 *END**************************************************************************/
static uint32_t SF_StringLsbToUint32(const uint8_t* const buffer)
{
    return (((uint32_t) buffer[3]) << 24U) |
           (((uint32_t) buffer[2]) << 16U) |
           (((uint32_t) buffer[1]) << 8U) |
            ((uint32_t) buffer[0]);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_Uint32ToStringLsb
 * Description   : Stores 4-byte unsigned integer to the uint8_t array
 *                 (little endian).
 *
 *END**************************************************************************/
static void SF_Uint32ToStringLsb(const uint32_t n, uint8_t* const buffer)
{
    buffer[0] = (uint8_t)(n);
    buffer[1] = (uint8_t)(n >> 8U);
    buffer[2] = (uint8_t)(n >> 16U);
    buffer[3] = (uint8_t)(n >> 24U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_Int32ToStringLsb
 * Description   : Stores 4-byte signed integer to the uint8_t array
 *                 (little endian).
 *
 *END**************************************************************************/
static inline void SF_Int32ToStringLsb(const int32_t n, uint8_t* const buffer)
{
    SF_Uint32ToStringLsb((uint32_t)n, buffer);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_PackFrame
 * Description   : Creates the SPI frame according to the defined format.
 *
 *END**************************************************************************/
static void SF_PackFrame(const sf_spi_cmd_t cmd, const uint8_t payloadLen,
    const uint8_t* const payload, uint8_t frame[])
{
    *(frame + SF_CMD_LENGTH_OF) = payloadLen + SF_CMD_HEADER_B;
    *(frame + SF_CMD_CMD_OF) = (uint8_t)cmd;

    if ((payloadLen > 0U) && (payload != NULL))
    {
        /* Store the payload. */
        memcpy((void*)(frame + SF_CMD_PAYLOAD_OF), (void*)payload, (size_t)payloadLen);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_ParseErrorCodes
 * Description   : Parses the error codes from a received ACK SPI frame.
 *
 *END**************************************************************************/
static sf_status_t SF_ParseErrorCodes(sf_drv_data_t *drvData,
    const uint8_t *spiRcvBuf)
{
    sf_status_t status = sfStatusSuccess;

    if ((*(spiRcvBuf + SF_ACK_LENGTH_OF)) < 3U)
    {
        status = sfStatusSpiAckLength;
    }

    if (status == sfStatusSuccess)
    {
        /* Store the error code and state from the ACK frame. */
        drvData->errorCode = (sf_spi_error_t)(*(spiRcvBuf + SF_ACK_ERROR_OF));
        drvData->manufError = (sf_spi_manuf_err_t)(*(spiRcvBuf + SF_ACK_MANUF_ERR_OF));

        /* Note: the user can obtain a values of the error codes with use
         * of the SF_GetErrorCode and SF_GetManufErrorCode functions. */
        if ((drvData->errorCode != sfErrNone) || (drvData->manufError != sfManufErrNone))
        {
            status = sfStatusCmdFailed;
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_ParsePayload
 * Description   : Parses payload from a received SPI ACK frame.
 *
 *END**************************************************************************/
static sf_status_t SF_ParsePayload(const uint8_t *rxFrame,
    sf_msg_payload_t *payload, uint8_t payloadBufSize)
{
    sf_status_t status = sfStatusSuccess;

    /* Preconditions. */
    SF_MCU_Assert(rxFrame != NULL);
    SF_MCU_Assert(payload != NULL);
    SF_MCU_Assert((payload->payload) != NULL);

    /* Get payload length. */
    payload->payloadLen = (uint8_t)(*(rxFrame + SF_ACK_LENGTH_OF));

    /* Subtract size of the ACK header. */
    payload->payloadLen -= SF_ACK_HEADER_B;

    if (payload->payloadLen > payloadBufSize)
    {
        status = sfStatusSpiAckLength;
    }

    if ((status == sfStatusSuccess) && (payload->payloadLen > 0U))
    {
        memcpy(payload->payload, rxFrame + SF_ACK_PAYLOAD_OF, payload->payloadLen);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_WaitAckLevel
 * Description   : Waits until ACK pin has a desired value.
 *
 *END**************************************************************************/
static sf_status_t SF_WaitAckLevel(sf_drv_data_t *drvData, uint32_t expValue,
    uint32_t timeout)
{
    volatile uint32_t pinVal = 0U;     /* Current ACK pin level. */
    uint32_t elapsedTime = 0U;         /* Current time in [us]. */

    pinVal = SF_MCU_ReadAckPin(drvData->drvInstance);
    while ((pinVal != expValue) && (timeout > elapsedTime))
    {
        /* Wait a small portion of time and check the pin level again. */
        SF_MCU_WaitUs(SF_TIMEOUT_STEP_US);
        elapsedTime += SF_TIMEOUT_STEP_US;
        pinVal = SF_MCU_ReadAckPin(drvData->drvInstance);
    }

    return (elapsedTime < timeout) ? sfStatusSuccess : sfStatusSpiTimeout;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SendCmdFrame
 * Description   : Sends a command via SPI; MISO data is ignored.
 *
 *END**************************************************************************/
static sf_status_t SF_SendCmdFrame(sf_drv_data_t *drvData, uint8_t txBuffer[],
    uint16_t dataSize)
{
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(txBuffer != NULL);
    SF_MCU_Assert(dataSize > 0U);

    /* Wait until ACK pin is high to be sure that the previous transmission is
     * complete. */
    status = SF_WaitAckLevel(drvData, 1U, SF_ACK_NOTRANS_TIMEOUT_US);
    if (status != sfStatusSuccess)
    {
        status = sfStatusDeviceNotReady;
    }
    else
    {
        /* Set CS pin low. */
        SF_MCU_WriteCsPin(drvData->drvInstance, 0);

        /* Wait until ACK pin goes low. */
        status = SF_WaitAckLevel(drvData, 0U, SF_ACK_NOTRANS_TIMEOUT_US);
    }

    if (status == sfStatusSuccess)
    {
        /* Send data (ignore data at MISO). */
        status = SF_MCU_TransferSpi(drvData->drvInstance, txBuffer, NULL, dataSize);
    }

    if (status == sfStatusSuccess)
    {
        /* Wait until ACK goes high. */
        status = SF_WaitAckLevel(drvData, 1U, SF_ACK_NOTRANS_TIMEOUT_US);
    }

    /* Set CS pin high (even if there is an error). */
    SF_MCU_WriteCsPin(drvData->drvInstance, 1);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_ReceiveAckFrame
 * Description   : Receives ACK frame via SPI. Note that it sends dummy data.
 *
 *END**************************************************************************/
static sf_status_t SF_ReceiveAckFrame(sf_drv_data_t *drvData,
    uint8_t rxBuffer[], uint16_t dataSize, uint32_t tmoutUs)
{
    sf_status_t status;
    uint8_t lengthByte, i;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(rxBuffer != NULL);
    SF_MCU_Assert(dataSize > 0U);

    /* Wait until ACK pin goes low. It takes very long time for commands causing
     * SIGFOX communication (e.g. TX_Frame or TRX_Frame) in contrast to other
     * commands (e.g. Get_Sigfox_ID) taking a few milliseconds. */
    status = SF_WaitAckLevel(drvData, 0U, tmoutUs);
    if (status == sfStatusSuccess)
    {
        /* Set CS pin low. */
        SF_MCU_WriteCsPin(drvData->drvInstance, 0);

        /* OL2385 needs some time to activate its MISO pin. */
        SF_MCU_WaitUs(10);

        /* Receive the first byte with length of the frame. */
        status = SF_MCU_TransferSpi(drvData->drvInstance, NULL, rxBuffer, 1U);
    }

    if (status == sfStatusSuccess)
    {
        /* Receive the rest of the ACK frame. */
        lengthByte = *(rxBuffer + SF_ACK_LENGTH_OF);
        if (lengthByte == 0U)
        {
            /* Corrupted frame. */
            status = sfStatusSpiAckLength;
        }
        else if (lengthByte > dataSize)
        {
            /* Size of the receive buffer is smaller than data to be received.
             * Receive (and do not store the rest of the frame in order OL2385
             * is ready for another communication. */
            for (i = 0U; i < (lengthByte - 1); i++)
            {
                SF_MCU_TransferSpi(drvData->drvInstance, NULL, rxBuffer, 1U);
            }

            status = sfStatusSpiAckLength;
        }
        else
        {
            /* Receive the rest of the frame. */
            status = SF_MCU_TransferSpi(drvData->drvInstance, NULL, rxBuffer + 1, lengthByte - 1);
        }
    }

    if (status == sfStatusSuccess)
    {
        /* Wait until ACK goes high. */
        status = SF_WaitAckLevel(drvData, 1U, SF_ACK_NOTRANS_TIMEOUT_US);
    }

    /* Set CS pin high (even if there was an error). */
    SF_MCU_WriteCsPin(drvData->drvInstance, 1);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SendCommandInt
 * Description   : Creates an SPI frame, sends it to the device, receives an
 *                 acknowledgement (if any) and stores an error code
 *                 and state value from the acknowledgement.
 *
 *END**************************************************************************/
static sf_status_t SF_SendCommandInt(sf_drv_data_t *drvData, uint8_t sendLen,
    uint8_t recvLen, uint8_t *sendBuffer, uint8_t *recvBuffer, uint32_t tmoutUs)
{
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(sendLen > 0U);
    SF_MCU_Assert(sendBuffer != NULL);
    SF_MCU_Assert((recvLen == 0U) || (recvBuffer != NULL));

    status = SF_SendCmdFrame(drvData, sendBuffer, sendLen);
    if (status != sfStatusSuccess)
    {
        return status;
    }

    if (recvLen > 0U)
    {
        /* Receive and process an ACK frame. */
        status = SF_ReceiveAckFrame(drvData, recvBuffer, recvLen, tmoutUs);
        if (status == sfStatusSuccess)
        {   /* Store the error code and state from the ACK frame. */
            status = SF_ParseErrorCodes(drvData, recvBuffer);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SendAndCheck
 * Description   : Sends and receives data via SPI and checks if an
 *                 acknowledgement has an expected size.
 *
 *END**************************************************************************/
static sf_status_t SF_SendAndCheck(sf_drv_data_t *drvData, uint8_t sendLen,
    uint8_t recvLen, uint8_t *sendBuffer, uint8_t *recvBuffer, uint32_t tmoutUs)
{
    sf_status_t status = SF_SendCommandInt(drvData, sendLen, recvLen, sendBuffer,
                                           recvBuffer, tmoutUs);

    /* Check the response. */
    if (status == sfStatusSuccess)
    {
        if (recvBuffer[SF_ACK_LENGTH_OF] != recvLen)
        {
            /* An acknowledgement does not have an expected length. */
            status = sfStatusSpiAckLength;
        }
    }

    return status;
}

/******************************************************************************
 * API
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_Init
 * Description   : Initializes internal driver structure and (optionally)
 *                 configures the HW configuration of the OL2385 device.
 *
 *END**************************************************************************/
sf_status_t SF_Init(sf_drv_data_t *drvData, sf_hw_config_t *hwConfig)
{
    sf_status_t status = sfStatusSuccess;
    sf_hw_config_t readHwConfig; /* HW configuration read from the device. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    drvData->manufError = sfManufErrNone;
    drvData->errorCode = sfErrNone;

    if (hwConfig != NULL)
    {
        /* The number of OL2385 EROM write cycles is limited. Read the current
         * configuration first and call the SF_SetHwConfig function only
         * if necessary. */
        status = SF_GetHwConfig(drvData, &readHwConfig);
        if ((status == sfStatusSuccess) &&
            ((readHwConfig.externalPaUsed != hwConfig->externalPaUsed) ||
             (readHwConfig.oscType != hwConfig->oscType)))
        {
            status = SF_SetHwConfig(drvData, hwConfig);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetErrorCode
 * Description   : Returns 'error code' parsed from the last ACK SPI frame
 *                 received from the OL2385 device.
 *
 *END**************************************************************************/
sf_spi_error_t SF_GetErrorCode(sf_drv_data_t *drvData)
{
    SF_MCU_Assert(drvData != NULL);

    return drvData->errorCode;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetManufErrorCode
 * Description   : Returns 'manuf. error code' parsed from the last ACK SPI
 *                 frame received from the OL2385 device.
 *
 *END**************************************************************************/
sf_spi_manuf_err_t SF_GetManufErrorCode(sf_drv_data_t *drvData)
{
    SF_MCU_Assert(drvData != NULL);

    return drvData->manufError;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SendCommandBlock
 * Description   : Sends a command to the device and waits for
 *                 an acknowledgement (if any).
 *
 *END**************************************************************************/
sf_status_t SF_SendCommandBlock(sf_drv_data_t *drvData, sf_spi_cmd_t cmd,
    const sf_msg_payload_t *cmdPayload, sf_msg_payload_t *ackPayload,
    uint8_t ackPaylBufLen, uint32_t timeout)
{
    uint8_t spiSndBuf[SF_SPI_MSG_MAX_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SPI_MSG_MAX_B]; /* SPI receive buffer. */
    uint8_t recvSize = 0U;               /* Maximal size the ACK frame. */
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert((cmd >= ((sf_spi_cmd_t)SF_CMD_FIRST_ID)) && (cmd <= ((sf_spi_cmd_t)SF_CMD_LAST_ID)));
    SF_MCU_Assert(cmdPayload != NULL);
    SF_MCU_Assert(cmdPayload->payloadLen <= SF_CMD_PAYLOAD_MAX_B);

    /* Payload pointer must be non-null when the command contains a payload. */
    SF_MCU_Assert((SF_HAS_CMD_PLD(cmd) && (cmdPayload->payload != NULL)) ||
            (SF_HAS_CMD_PLD(cmd) == false));

    /* When a command has an acknowledgement, the actual ACK frame size is
     * defined by the first byte of the ACK and updated later. */
    recvSize = (SF_HAS_CMD_ACK(cmd)) ? SF_SPI_MSG_MAX_B : 0U;

    SF_PackFrame(cmd, cmdPayload->payloadLen, cmdPayload->payload, spiSndBuf);
    status = SF_SendCommandInt(drvData, cmdPayload->payloadLen + SF_CMD_HEADER_B,
            recvSize, spiSndBuf, spiRcvBuf, timeout);

    if ((status == sfStatusSuccess) && (recvSize != 0U) && (ackPayload != NULL))
    {
        /* Store the payload and its length. */
        status = SF_ParsePayload(spiRcvBuf, ackPayload, ackPaylBufLen);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SendCommandNonBlock
 * Description   : Sends a command to the device. It does not wait for an
 *                 acknowledgement.
 *
 *END**************************************************************************/
sf_status_t SF_SendCommandNonBlock(sf_drv_data_t *drvData, sf_spi_cmd_t cmd,
    const sf_msg_payload_t *sendPayload)
{
    uint8_t spiSndBuf[SF_CMD_SPI_MSG_MAX_B]; /* SPI send buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert((cmd >= ((sf_spi_cmd_t)SF_CMD_FIRST_ID)) && (cmd <= ((sf_spi_cmd_t)SF_CMD_LAST_ID)));
    SF_MCU_Assert(sendPayload != NULL);
    SF_MCU_Assert(sendPayload->payloadLen <= SF_CMD_PAYLOAD_MAX_B);

    /* Pack the command. */
    SF_PackFrame(cmd, sendPayload->payloadLen, sendPayload->payload, spiSndBuf);
    return SF_SendCmdFrame(drvData, spiSndBuf, sendPayload->payloadLen + SF_CMD_HEADER_B);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_IsAckFrameReady
 * Description   : Checks if the SIGFOX device is ready to send an
 *                 acknowledgement.
 *
 *END**************************************************************************/
bool SF_IsAckFrameReady(sf_drv_data_t *drvData)
{
    SF_MCU_Assert(drvData != NULL);

    return (SF_MCU_ReadAckPin(drvData->drvInstance) == 0U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_ReadAckFrameNonBlock
 * Description   : Receives an acknowledge frame.
 *
 *END**************************************************************************/
sf_status_t SF_ReadAckFrameNonBlock(sf_drv_data_t *drvData,
    sf_msg_payload_t *recvPayload, uint8_t recvBufferSize)
{
    uint8_t spiRcvBuf[SF_ACK_SPI_MSG_MAX_B]; /* SPI receive buffer. */
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(recvPayload != NULL);
    SF_MCU_Assert(recvPayload->payload != NULL);

    /* Receive and process an ACK frame. */
    status = SF_ReceiveAckFrame(drvData, spiRcvBuf,
            SF_ACK_SPI_MSG_MAX_B, SF_ACK_TRANS_RECV_TIMEOUT_US);

    /* Store items from the ACK frame. */
    if (status == sfStatusSuccess)
    {
        status = SF_ParseErrorCodes(drvData, spiRcvBuf);
    }
    if (status == sfStatusSuccess)
    {
        status = SF_ParsePayload(spiRcvBuf, recvPayload, recvBufferSize);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_WakeUp
 * Description   : Wakes OL2385 from the low power mode (POWER_OFF2) up.
 *
 *END**************************************************************************/
sf_status_t SF_WakeUp(sf_drv_data_t *drvData)
{
    uint8_t spiSndBuf[SF_WAKEUP_CMD_B]; /* SPI send buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdWakeUp, SF_WAKEUP_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_WAKEUP_CMD_B, SF_WAKEUP_ACK_B,
            spiSndBuf, NULL, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_Sleep
 * Description   : This function puts the device into power off mode with use of
 *                 the "Send To Sleep" command.
 *
 *END**************************************************************************/
sf_status_t SF_Sleep(sf_drv_data_t *drvData)
{
    uint8_t spiSndBuf[SF_SLEEP_CMD_B]; /* SPI send buffer. */

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSleep, SF_SLEEP_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_SLEEP_CMD_B, SF_SLEEP_ACK_B,
            spiSndBuf, NULL, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SetHwConfig
 * Description   : Sets the HW configuration.
 *
 *END**************************************************************************/
sf_status_t SF_SetHwConfig(sf_drv_data_t *drvData, sf_hw_config_t *cfg)
{
    uint8_t spiSndBuf[SF_SET_HW_CFG_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SET_HW_CFG_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(cfg != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSetHwConfig, SF_SET_HW_CFG_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_SET_HW_CFG_CMD_CONFIG_LSB_OF] = (cfg->externalPaUsed) ? 1U : 0U;
    spiSndBuf[SF_SET_HW_CFG_CMD_CONFIG_LSB_OF] |= (((uint8_t)cfg->oscType) << 1u);
    spiSndBuf[SF_SET_HW_CFG_CMD_CONFIG_MSB_OF] = 0U;

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_SET_HW_CFG_CMD_B, SF_SET_HW_CFG_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetHwConfig
 * Description   : Reads the HW configuration stored in OL2385.
 *
 *END**************************************************************************/
sf_status_t SF_GetHwConfig(sf_drv_data_t *drvData, sf_hw_config_t *cfg)
{
    sf_status_t status;
    uint8_t spiSndBuf[SF_GET_HW_CFG_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_HW_CFG_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(cfg != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetHwConfig, SF_GET_HW_CFG_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_GET_HW_CFG_CMD_B, SF_GET_HW_CFG_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        cfg->externalPaUsed = (bool)(spiRcvBuf[SF_GET_HW_CFG_ACK_CONFIG_LSB_OF] & 0x01U);
        cfg->oscType = (sf_osc_type_t)((spiRcvBuf[SF_GET_HW_CFG_ACK_CONFIG_LSB_OF] >> 1) & 0x01U);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_ChangeRc
 * Description   : Initializes the OL2385 for one of the defined standards
 *                 (RC1, RC2, RC3A, RC3C, RC4, RC5, RC6 or RC7).
 *
 *END**************************************************************************/
sf_status_t SF_ChangeRc(sf_drv_data_t *drvData, sf_rc_enum_t rc)
{
    uint8_t spiSndBuf[SF_CHANGE_RC_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_CHANGE_RC_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert((rc == sfRc1) || (rc == sfRc2) || (rc == sfRc3a) ||
                  (rc == sfRc3c) || (rc == sfRc4) || (rc == sfRc5) ||
                  (rc == sfRc6) || (rc == sfRc7));

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdChangeRc, SF_CHANGE_RC_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_CHANGE_RC_CMD_RC_OF] = (uint8_t)rc;

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_CHANGE_RC_CMD_B, SF_CHANGE_RC_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SetCustomRc
 * Description   : Sets a specific radio configuration.
 *
 *END**************************************************************************/
sf_status_t SF_SetCustomRc(sf_drv_data_t *drvData, const sf_rc_t *rc)
{
    uint8_t spiSndBuf[SF_SET_CUSTOM_RC_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SET_CUSTOM_RC_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(rc != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSetCustomRc, SF_SET_CUSTOM_RC_CMD_PLD_B, NULL, spiSndBuf);

    spiSndBuf[SF_SET_CUSTOM_RC_CMD_SA_OF] = (uint8_t)rc->spectrumAccess;
    spiSndBuf[SF_SET_CUSTOM_RC_CMD_MOD_OF] = (uint8_t)rc->modulation;
    SF_Uint32ToStringLsb(rc->openTxFrequency, &spiSndBuf[SF_SET_CUSTOM_RC_CMD_TX_FREQ_OF]);
    SF_Uint32ToStringLsb(rc->openRxFrequency, &spiSndBuf[SF_SET_CUSTOM_RC_CMD_RX_FREQ_OF]);
    SF_Uint32ToStringLsb(rc->macroChannelWidth, &spiSndBuf[SF_SET_CUSTOM_RC_CMD_MCW_OF]);
    SF_Uint32ToStringLsb(rc->specificRc.openCsFrequency, &spiSndBuf[SF_SET_CUSTOM_RC_CMD_OCS_FREQ_OF]);
    SF_Uint32ToStringLsb(rc->specificRc.openCsBandwidth, &spiSndBuf[SF_SET_CUSTOM_RC_CMD_OCS_BW_OF]);
    spiSndBuf[SF_SET_CUSTOM_RC_CMD_CS_THRES_OF] = *((uint8_t *)(&(rc->specificRc.csThreshold)));

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_SET_CUSTOM_RC_CMD_B, SF_SET_CUSTOM_RC_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SaveRcToErom
 * Description   : Saves the current radio configuration to non-volatile EROM
 *                 memory.
 *
 *END**************************************************************************/
sf_status_t SF_SaveRcToErom(sf_drv_data_t *drvData)
{
    uint8_t spiSndBuf[SF_SAVE_RC_EROM_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SAVE_RC_EROM_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSaveRcToErom, SF_SAVE_RC_EROM_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_SAVE_RC_EROM_CMD_B, SF_SAVE_RC_EROM_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetRc
 * Description   : Reads the current radio configuration.
 *
 *END**************************************************************************/
sf_status_t SF_GetRc(sf_drv_data_t *drvData, sf_rc_t *rc)
{
    sf_status_t status;
    uint8_t spiSndBuf[SF_GET_RC_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_RC_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(rc != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetRc, SF_GET_RC_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_GET_RC_CMD_B, SF_GET_RC_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        rc->spectrumAccess = (sf_spectrum_access_t)(spiRcvBuf[SF_GET_RC_ACK_SA_OF]);
        rc->modulation = (sf_modulation_t)(spiRcvBuf[SF_GET_RC_ACK_MOD_OF]);
        rc->openTxFrequency = SF_StringLsbToUint32(spiRcvBuf + SF_GET_RC_ACK_TX_FREQ_OF);
        rc->openRxFrequency = SF_StringLsbToUint32(spiRcvBuf + SF_GET_RC_ACK_RX_FREQ_OF);
        rc->macroChannelWidth = SF_StringLsbToUint32(spiRcvBuf + SF_GET_RC_ACK_MCW_OF);
        rc->specificRc.openCsFrequency = SF_StringLsbToUint32(spiRcvBuf + SF_GET_RC_ACK_OCS_FREQ_OF);
        rc->specificRc.openCsBandwidth = SF_StringLsbToUint32(spiRcvBuf + SF_GET_RC_ACK_OCS_BW_OF);
        rc->specificRc.csThreshold = *((int8_t *)(spiRcvBuf + SF_GET_RC_ACK_CS_THRES_OF));
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SetTxConfig
 * Description   : Updates the TX configuration including specific variables for
 *                 FH and LBT standards (so-called config words).
 *
 *END**************************************************************************/
sf_status_t SF_SetTxConfig(sf_drv_data_t *drvData, sf_tx_config_t *cfg)
{
    uint8_t spiSndBuf[SF_SET_TX_CFG_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SET_TX_CFG_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(cfg != NULL);
    SF_MCU_Assert((cfg->txRepeat == sfTxRepeat1) || (cfg->txRepeat == sfTxRepeat3));
    SF_MCU_Assert((cfg->paType == sfPa0) || (cfg->paType == sfPa14));

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSetTxConfig, SF_SET_TX_CFG_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_SET_TX_CFG_CMD_TX_STEPS_OF] = cfg->txAttSteps;
    spiSndBuf[SF_SET_TX_CFG_CMD_TX_CFG_OF] = (cfg->timerEnable) ? SF_SET_TX_CFG_CMD_TX_CFG_TMREN_MASK : 0U;
    spiSndBuf[SF_SET_TX_CFG_CMD_TX_CFG_OF] |= (cfg->payloadEncEn) ? SF_SET_TX_CFG_CMD_TX_CFG_PEE_MASK : 0U;
    spiSndBuf[SF_SET_TX_CFG_CMD_TX_CFG_OF] |= (((uint8_t)(cfg->txRepeat)) << 2) & SF_SET_TX_CFG_CMD_TX_CFG_TXR_MASK;
    spiSndBuf[SF_SET_TX_CFG_CMD_TX_CFG_OF] |= (((uint8_t)(cfg->paType)) << 4) & SF_SET_TX_CFG_CMD_TX_CFG_PAT_MASK;
    SF_Uint32ToStringLsb(cfg->configWords[0], &spiSndBuf[SF_SET_TX_CFG_CMD_WORD0_OF]);
    SF_Uint32ToStringLsb(cfg->configWords[1], &spiSndBuf[SF_SET_TX_CFG_CMD_WORD1_OF]);
    SF_Uint32ToStringLsb(cfg->configWords[2], &spiSndBuf[SF_SET_TX_CFG_CMD_WORD2_OF]);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_SET_TX_CFG_CMD_B, SF_SET_TX_CFG_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SaveTxConfigToErom
 * Description   : Saves the current TX configuration to EROM memory.
 *
 *END**************************************************************************/
sf_status_t SF_SaveTxConfigToErom(sf_drv_data_t *drvData)
{
    uint8_t spiSndBuf[SF_SAVE_TX_CFG_EROM_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SAVE_TX_CFG_EROM_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSaveTxCfgToErom, SF_SAVE_TX_CFG_EROM_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_SAVE_TX_CFG_EROM_CMD_B, SF_SAVE_TX_CFG_EROM_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetTxConfig
 * Description   : Reads the current TX configuration.
 *
 *END**************************************************************************/
sf_status_t SF_GetTxConfig(sf_drv_data_t *drvData, sf_tx_config_t *cfg)
{
    sf_status_t status;
    uint8_t spiSndBuf[SF_GET_TX_CFG_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_TX_CFG_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(cfg != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetTxConfig, SF_GET_TX_CFG_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_GET_TX_CFG_CMD_B, SF_GET_TX_CFG_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        cfg->txAttSteps = spiRcvBuf[SF_GET_TX_CFG_ACK_TX_STEPS_OF];
        cfg->timerEnable = (spiRcvBuf[SF_GET_TX_CFG_ACK_TX_CFG_OF] & SF_GET_TX_CFG_ACK_TX_CFG_TMREN_MASK) ? true : false;
        cfg->payloadEncEn = (spiRcvBuf[SF_GET_TX_CFG_ACK_TX_CFG_OF] & SF_GET_TX_CFG_ACK_TX_CFG_PEE_MASK) ? true : false;
        cfg->txRepeat = (sf_tx_repeat_t)((spiRcvBuf[SF_GET_TX_CFG_ACK_TX_CFG_OF] & SF_GET_TX_CFG_ACK_TX_CFG_TXR_MASK) >> 2U);
        cfg->paType = (sf_pa_type_t)((spiRcvBuf[SF_GET_TX_CFG_ACK_TX_CFG_OF] & SF_GET_TX_CFG_ACK_TX_CFG_PAT_MASK) >> 4U);

        cfg->configWords[0] = SF_StringLsbToUint32(&spiRcvBuf[SF_GET_TX_CFG_ACK_WORD0_OF]);
        cfg->configWords[1] = SF_StringLsbToUint32(&spiRcvBuf[SF_GET_TX_CFG_ACK_WORD1_OF]);
        cfg->configWords[2] = SF_StringLsbToUint32(&spiRcvBuf[SF_GET_TX_CFG_ACK_WORD2_OF]);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SendFrame
 * Description   : Sends a standard SIGFOX frame with customer payload (up to
 *                 12 bytes) to the SIGFOX network.
 *
 *END**************************************************************************/
sf_status_t SF_SendFrame(sf_drv_data_t *drvData,
    const sf_msg_payload_t *sendPayload)
{
    uint8_t spiSndBuf[SF_TX_FRAME_CMD_B_MAX]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_TX_FRAME_ACK_B];     /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(sendPayload != NULL);
    SF_MCU_Assert(sendPayload->payloadLen > 0U);
    SF_MCU_Assert(sendPayload->payload != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdTxFrame, sendPayload->payloadLen, sendPayload->payload, spiSndBuf);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_CMD_HEADER_B + sendPayload->payloadLen, SF_TX_FRAME_ACK_B,
                           spiSndBuf, spiRcvBuf, SF_ACK_TRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SendReceiveFrame
 * Description   : Sends a standard SIGFOX frame with the customer payload (up
 *                 to 12 bytes) and triggers a reception of a SIGFOX message.
 *
 *END**************************************************************************/
sf_status_t SF_SendReceiveFrame(sf_drv_data_t *drvData,
    const sf_msg_payload_t *sendPayload, sf_msg_payload_t *recvPayload)
{
    uint8_t spiSndBuf[SF_TRX_FRAME_CMD_B_MAX]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_TRX_FRAME_ACK_B];     /* SPI receive buffer. */
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(recvPayload != NULL);
    SF_MCU_Assert(sendPayload != NULL);
    SF_MCU_Assert(sendPayload->payloadLen > 0U);
    SF_MCU_Assert(sendPayload->payload != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdTrxFrame, sendPayload->payloadLen, sendPayload->payload, spiSndBuf);

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_CMD_HEADER_B + sendPayload->payloadLen, SF_TRX_FRAME_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_TRANS_RECV_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        recvPayload->payloadLen = spiRcvBuf[SF_ACK_LENGTH_OF] - SF_ACK_HEADER_B;
        if (recvPayload->payloadLen > 0U)
        {
            memcpy(recvPayload->payload, spiRcvBuf + SF_ACK_PAYLOAD_OF, recvPayload->payloadLen);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SendBit
 * Description   : Sends only one bit to the SIGFOX network and (optionally)
 *                 initiates a downlink.
 *
 *END**************************************************************************/
sf_status_t SF_SendBit(sf_drv_data_t *drvData, bool sentBit,
    sf_msg_payload_t *recvPayload)
{
    uint8_t spiSndBuf[SF_BIT_FRAME_CMD_B];      /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_BIT_FRAME_ACK_DOWN_B]; /* SPI receive buffer. */
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdBitFrame, SF_BIT_FRAME_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_BIT_FRAME_CMD_CONFIG_OF] =
            sentBit ? SF_BIT_FRAME_CMD_CONFIG_BIT_MASK : 0x00U;
    spiSndBuf[SF_BIT_FRAME_CMD_CONFIG_OF] |=
            (recvPayload != NULL) ? SF_BIT_FRAME_CMD_CONFIG_DOWNLINK_MASK : 0x00U;

    /* Send the command. */
    if (recvPayload == NULL)
    {
        status = SF_SendAndCheck(drvData, SF_BIT_FRAME_CMD_B, SF_BIT_FRAME_ACK_NO_DOWN_B,
                                 spiSndBuf, spiRcvBuf, SF_ACK_TRANS_TIMEOUT_US);
    }
    else
    {
        status = SF_SendAndCheck(drvData, SF_BIT_FRAME_CMD_B, SF_BIT_FRAME_ACK_DOWN_B,
                                 spiSndBuf, spiRcvBuf, SF_ACK_TRANS_RECV_TIMEOUT_US);

        /* Process the response. */
        if (status == sfStatusSuccess)
        {
            recvPayload->payloadLen = spiRcvBuf[SF_ACK_LENGTH_OF] - SF_ACK_HEADER_B;
            if (recvPayload->payloadLen > 0U)
            {
                memcpy(recvPayload->payload, spiRcvBuf + SF_ACK_PAYLOAD_OF, recvPayload->payloadLen);
            }
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SendOobFrame
 * Description   : Sends an OOB SIGFOX frame.
 *
 *END**************************************************************************/
sf_status_t SF_SendOobFrame(sf_drv_data_t *drvData, sf_oob_type_t type)
{
    uint8_t spiSndBuf[SF_OOB_TX_FRAME_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_OOB_TX_FRAME_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdOobTxFrame, SF_OOB_TX_FRAME_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_OOB_TX_FRAME_CMD_TYPE_OF] = (uint8_t)type;

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_OOB_TX_FRAME_CMD_B, SF_OOB_TX_FRAME_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_TRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetDeviceVersion
 * Description   : Reads the version of OL2385 chip and OL2385 Sigfox firmware.
 *
 *END**************************************************************************/
sf_status_t SF_GetDeviceVersion(sf_drv_data_t *drvData, uint8_t hwVer[],
    uint8_t swVer[])
{
    sf_status_t status;
    uint8_t spiSndBuf[SF_GET_DEV_VER_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_DEV_VER_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetDevVer, SF_GET_DEV_VER_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command.
     * SF_SendAndCheck() function is not used here in order to reach a backwards
     * compatibility of SF_GetDeviceVersion() with older versions of OL2385 FW
     * which had a different header of the ACK frame. */
    status = SF_SendCmdFrame(drvData, spiSndBuf, SF_GET_DEV_VER_CMD_B);

    /* Receive the ACK frame. */
    if (status == sfStatusSuccess)
    {
        status = SF_ReceiveAckFrame(drvData, spiRcvBuf, SF_GET_DEV_VER_ACK_B, SF_ACK_NOTRANS_TIMEOUT_US);
    }

    /* Process the header of ACK frame. */
    if (status == sfStatusSuccess)
    {
    	/* Store the error code and state from the ACK frame. */
        status = SF_ParseErrorCodes(drvData, spiRcvBuf);

        /* Older OL2385 FWs send a OL2385 state byte (with a value of 2u)
         * instead of Manuf. error code as the 3rd byte of ACK frame. */
        if ((status == sfStatusCmdFailed) &&
            (drvData->errorCode == sfErrNone) &&
            ((uint8_t)drvData->manufError == 2u))
        {
            status = sfStatusSuccess;
        }

        /* Check the frame length. */
        if ((status == sfStatusSuccess) &&
            (spiRcvBuf[SF_ACK_LENGTH_OF] != SF_GET_DEV_VER_ACK_B))
        {
            /* An acknowledgement does not have an expected length. */
            status = sfStatusSpiAckLength;
        }
    }

    /* Process the payload of ACK frame. */
    if (status == sfStatusSuccess)
    {
        if (hwVer != NULL)
        {
            memcpy(hwVer, spiRcvBuf + SF_GET_DEV_VER_ACK_HW_OF, SF_GET_DEV_VER_ACK_HW_B);
        }

        if (swVer != NULL)
        {
            memcpy(swVer, spiRcvBuf + SF_GET_DEV_VER_ACK_SW_OF, SF_GET_DEV_VER_ACK_SW_B);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetSigfoxId
 * Description   : Reads the Sigfox ID.
 *
 *END**************************************************************************/
sf_status_t SF_GetSigfoxId(sf_drv_data_t *drvData, uint8_t id[])
{
    uint8_t spiSndBuf[SF_GET_ID_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_ID_ACK_B]; /* SPI receive buffer. */
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(id != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetSigId, SF_GET_ID_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_GET_ID_CMD_B, SF_GET_ID_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        memcpy(id, spiRcvBuf + SF_GET_ID_ACK_ID_OF, SF_GET_ID_ACK_ID_B);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetInitialPac
 * Description   : Reads the initial device PAC.
 *
 *END**************************************************************************/
sf_status_t SF_GetInitialPac(sf_drv_data_t *drvData, uint8_t pac[])
{
    sf_status_t status;
    uint8_t spiSndBuf[SF_GET_INIT_PAC_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_INIT_PAC_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(pac != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetInitPac, SF_GET_INIT_PAC_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_GET_INIT_PAC_CMD_B, SF_GET_INIT_PAC_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        memcpy(pac, spiRcvBuf + SF_GET_INIT_PAC_ACK_PAC_OF, SF_GET_INIT_PAC_ACK_PAC_B);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetSigfoxLibVersion
 * Description   : Reads the Sigfox library version.
 *
 *END**************************************************************************/
sf_status_t SF_GetSigfoxLibVersion(sf_drv_data_t *drvData, uint8_t ver[])
{
    sf_status_t status;
    uint8_t spiSndBuf[SF_GET_LIB_VER_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_LIB_VER_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(ver != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetLibVer, SF_GET_LIB_VER_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_GET_LIB_VER_CMD_B, SF_GET_LIB_VER_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        memcpy(ver, spiRcvBuf + SF_GET_LIB_VER_ACK_VER_OF, SF_GET_LIB_VER_ACK_VER_B);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetSigfoxAddonVersion
 * Description   : Reads the Sigfox Addon version.
 *
 *END**************************************************************************/
sf_status_t SF_GetSigfoxAddonVersion(sf_drv_data_t *drvData, uint8_t ver[])
{
    sf_status_t status;
    uint8_t spiSndBuf[SF_GET_ADDON_VER_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_ADDON_VER_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(ver != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetAddonVer, SF_GET_ADDON_VER_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_GET_ADDON_VER_CMD_B, SF_GET_ADDON_VER_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        memcpy(ver, spiRcvBuf + SF_GET_ADDON_VER_ACK_VER_OF, SF_GET_ADDON_VER_ACK_VER_B);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetLbtInfo
 * Description   : Reads info on sent frame depending on the mode you are using.
 *
 *END**************************************************************************/
sf_status_t SF_GetLbtInfo(sf_drv_data_t *drvData, uint8_t *returnedInfo)
{
    uint8_t spiSndBuf[SF_GET_LBT_INFO_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_LBT_INFO_ACK_B]; /* SPI receive buffer. */
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(returnedInfo != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetLbtInfo, SF_GET_LBT_INFO_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_GET_LBT_INFO_CMD_B, SF_GET_LBT_INFO_ACK_B,
            spiSndBuf,
            spiRcvBuf,
            SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        *returnedInfo = spiRcvBuf[SF_GET_LBT_INFO_ACK_INFO_OF];
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SetRcSyncPeriod
 * Description   : Sets the period for transmission of RC Sync frame.
 *
 *END**************************************************************************/
sf_status_t SF_SetRcSyncPeriod(sf_drv_data_t *drvData, uint16_t period)
{
    uint8_t spiSndBuf[SF_SET_RC_SYNC_PER_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SET_RC_SYNC_PER_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSetRcSyncPer, SF_SET_RC_SYNC_PER_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_SET_RC_SYNC_PER_CMD_LSB_OF] = (uint8_t)(period & 0xFF);
    spiSndBuf[SF_SET_RC_SYNC_PER_CMD_MSB_OF] = (uint8_t)((period >> 8) & 0xFF);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_SET_RC_SYNC_PER_CMD_B, SF_SET_RC_SYNC_PER_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_CalibrateFreqStatic
 * Description   : Calibrates the TX/RX frequency.
 *
 *END**************************************************************************/
sf_status_t SF_CalibrateFreqStatic(sf_drv_data_t *drvData, int32_t compensation)
{
    uint8_t spiSndBuf[SF_STATIC_FREQ_CAL_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_STATIC_FREQ_CAL_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdStaticFreqCal, SF_STATIC_FREQ_CAL_CMD_PLD_B, NULL, spiSndBuf);
    SF_Int32ToStringLsb(compensation, &spiSndBuf[SF_STATIC_FREQ_CAL_CMD_COMP_OF]);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_STATIC_FREQ_CAL_CMD_B, SF_STATIC_FREQ_CAL_ACK_B,
            spiSndBuf,
            spiRcvBuf,
            SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_UpdateTempFreqCalTable
 * Description   : Updates the temperature-frequency calibration table.
 *
 *END**************************************************************************/
sf_status_t SF_UpdateTempFreqCalTable(sf_drv_data_t *drvData,
    const int8_t *table)
{
    uint8_t spiSndBuf[SF_TEMP_FREQ_CAL_CMD_UPD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_TEMP_FREQ_CAL_ACK_UPD_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(table != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdTempFreqCal, SF_TEMP_FREQ_CAL_CMD_UPD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_TEMP_FREQ_CAL_CMD_SUBCMD_OF] = SF_TEMP_FREQ_CAL_CMD_SUBCMD_UPDATE;
    memcpy(&spiSndBuf[SF_TEMP_FREQ_CAL_CMD_TEMP_OF], (uint8_t*)table, SF_TEMP_FREQ_CAL_TBL_SIZE);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_TEMP_FREQ_CAL_CMD_UPD_B, SF_TEMP_FREQ_CAL_ACK_UPD_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_ReadTempFreqCalTable
 * Description   : Reads the temperature-frequency calibration table.
 *
 *END**************************************************************************/
sf_status_t SF_ReadTempFreqCalTable(sf_drv_data_t *drvData,
    int8_t *table)
{
    uint8_t spiSndBuf[SF_TEMP_FREQ_CAL_CMD_READ_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_TEMP_FREQ_CAL_ACK_READ_B]; /* SPI receive buffer. */
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(table != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdTempFreqCal, SF_TEMP_FREQ_CAL_CMD_READ_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_TEMP_FREQ_CAL_CMD_SUBCMD_OF] = SF_TEMP_FREQ_CAL_CMD_SUBCMD_READ;

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_TEMP_FREQ_CAL_CMD_READ_B, SF_TEMP_FREQ_CAL_ACK_READ_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        memcpy((uint8_t*)table, &spiRcvBuf[SF_TEMP_FREQ_CAL_ACK_TEMP_OF], SF_TEMP_FREQ_CAL_TBL_SIZE);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SetDefaultTempFreqCalTable
 * Description   : Sets a default temperature-frequency calibration table
 *                 (no temperature calibration).
 *
 *END**************************************************************************/
sf_status_t SF_SetDefaultTempFreqCalTable(sf_drv_data_t *drvData)
{
    uint8_t spiSndBuf[SF_TEMP_FREQ_CAL_CMD_DEF_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_TEMP_FREQ_CAL_ACK_DEF_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdTempFreqCal, SF_TEMP_FREQ_CAL_CMD_DEF_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_TEMP_FREQ_CAL_CMD_SUBCMD_OF] = SF_TEMP_FREQ_CAL_CMD_SUBCMD_SET_DEFAULT;

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_TEMP_FREQ_CAL_CMD_DEF_B, SF_TEMP_FREQ_CAL_ACK_DEF_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_CalibrateRSSI
 * Description   : Calibrates the RSSI measurements.
 *
 *END**************************************************************************/
sf_status_t SF_CalibrateRssi(sf_drv_data_t *drvData, uint16_t offset)
{
    uint8_t spiSndBuf[SF_RSSI_CALIB_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_RSSI_CALIB_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdRssiCalib, SF_RSSI_CALIB_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_RSSI_CALIB_CMD_RSSI_LSB_OF] = (uint8_t)(offset & 0xFF);
    spiSndBuf[SF_RSSI_CALIB_CMD_RSSI_MSB_OF] = (uint8_t)((offset >> 8) & 0xFF);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_RSSI_CALIB_CMD_B, SF_RSSI_CALIB_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_StartContTransmission
 * Description   : Starts a continuous transmission.
 *
 *END**************************************************************************/
sf_status_t SF_StartContTransmission(sf_drv_data_t *drvData, uint32_t frequency,
    sf_modulation_t modulation)
{
    uint8_t spiSndBuf[SF_START_CONT_CMD_B]; /* SPI send buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdStartCont, SF_START_CONT_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_START_CONT_CMD_MOD_OF] = (uint8_t)modulation;
    SF_Uint32ToStringLsb(frequency, &spiSndBuf[SF_START_CONT_CMD_FREQ_OF]);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_START_CONT_CMD_B, SF_START_CONT_ACK_B,
            spiSndBuf, NULL, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_StopContTransmission
 * Description   : Stops the continuous transmission.
 *
 *END**************************************************************************/
sf_status_t SF_StopContTransmission(sf_drv_data_t *drvData)
{
    uint8_t spiSndBuf[SF_STOP_CONT_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_STOP_CONT_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdStopCont, SF_STOP_CONT_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_STOP_CONT_CMD_B, SF_STOP_CONT_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_TestMode
 * Description   : Sends a command to execute Sigfox test procedures.
 *
 *END**************************************************************************/
sf_status_t SF_TestMode(sf_drv_data_t *drvData, sf_rc_enum_t rc,
    sf_test_mode_t testMode, uint32_t timeout)
{
    uint8_t spiSndBuf[SF_TEST_MODE_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_TEST_MODE_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdTestMode, SF_TEST_MODE_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_TEST_MODE_CMD_RC_OF] = (uint8_t)rc;
    spiSndBuf[SF_TEST_MODE_CMD_MODE_OF] = (uint8_t)testMode;

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_TEST_MODE_CMD_B, SF_TEST_MODE_ACK_B,
            spiSndBuf, spiRcvBuf, timeout);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SetPrivateKey
 * Description   : Switches to the private key which should be used for normal
 *                 RF transmission.
 *
 *END**************************************************************************/
sf_status_t SF_SetPrivateKey(sf_drv_data_t *drvData)
{
    uint8_t spiSndBuf[SF_SWITCH_PRIVATE_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SWITCH_PRIVATE_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSetPrivateKey, SF_SWITCH_PRIVATE_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_SWITCH_PRIVATE_CMD_B, SF_SWITCH_PRIVATE_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SetPublicKey
 * Description   : Switches to the public key (e.g. for the protocol tests).
 *
 *END**************************************************************************/
sf_status_t SF_SetPublicKey(sf_drv_data_t *drvData)
{
    uint8_t spiSndBuf[SF_SWITCH_PUBLIC_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SWITCH_PUBLIC_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSetPublicKey, SF_SWITCH_PUBLIC_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_SWITCH_PUBLIC_CMD_B, SF_SWITCH_PUBLIC_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SetCertificationId
 * Description   : Temporary sets the Sigfox certification ID (0xFEDCBA98).
 *
 *END**************************************************************************/
sf_status_t SF_SetCertificationId(sf_drv_data_t *drvData)
{
    uint8_t spiSndBuf[SF_SET_CERT_ID_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SET_CERT_ID_ACK_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSetCertId, SF_SET_CERT_ID_CMD_PLD_B, NULL, spiSndBuf);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_SET_CERT_ID_CMD_B, SF_SET_CERT_ID_ACK_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_SendTestFrame
 * Description   : Builds a Sigfox Frame with the customer payload and sends it
 *                 at a specified frequency.
 *
 *END**************************************************************************/
sf_status_t SF_SendTestFrame(sf_drv_data_t *drvData, uint32_t frequency,
    bool initDownlink, const sf_msg_payload_t *payload)
{
    uint8_t spiSndBuf[SF_SEND_TEST_FR_CMD_B_MAX]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_SEND_TEST_FR_ACK_B];     /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(payload != NULL);
    SF_MCU_Assert((payload->payloadLen > 0U) && (payload->payload != NULL));

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdSendTestFrame, 5U + payload->payloadLen, NULL, spiSndBuf);
    SF_Uint32ToStringLsb(frequency, &spiSndBuf[SF_SEND_TEST_FR_CMD_FREQ_OF]);
    spiSndBuf[SF_SEND_TEST_FR_CMD_DOWN_OF] = (uint8_t)initDownlink;
    memcpy(spiSndBuf + SF_SEND_TEST_FR_CMD_PAYL_OF, payload->payload, payload->payloadLen);

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_CMD_HEADER_B + 5U + payload->payloadLen,
            SF_SEND_TEST_FR_ACK_B,
            spiSndBuf,
            spiRcvBuf,
            SF_ACK_TRANS_TIMEOUT_US);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_ReceiveTestFrame
 * Description   : Waits for a valid downlink frame during timeout time and
 *                 returns the received data.
 *
 *END**************************************************************************/
sf_status_t SF_ReceiveTestFrame(sf_drv_data_t *drvData, uint32_t frequency,
    bool auth, uint8_t *buffer, uint8_t timeout, int8_t *rssi)
{
    uint8_t spiSndBuf[SF_MAX(SF_RECV_TEST_FR_CMD_AOFF_B, SF_RECV_TEST_FR_CMD_AON_B)]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_MAX(SF_RECV_TEST_FR_ACK_AOFF_B, SF_RECV_TEST_FR_ACK_AON_B)]; /* SPI receive buffer. */
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(buffer != NULL);
    SF_MCU_Assert(rssi != NULL);

    if (auth)
    {
        /* AUTHENTICATION_ON */
    	/* Pack the command. */
        SF_PackFrame(sfSpiCmdRecvTestFrame, SF_RECV_TEST_FR_CMD_AON_PLD_B, NULL, spiSndBuf);
        SF_Uint32ToStringLsb(frequency, &spiSndBuf[SF_RECV_TEST_FR_CMD_FREQ_OF]);
        spiSndBuf[SF_RECV_TEST_FR_CMD_TOUT_OF] = timeout;

        /* Send the command. */
        status = SF_SendAndCheck(drvData, SF_RECV_TEST_FR_CMD_AON_B,
                SF_RECV_TEST_FR_ACK_AON_B,
                spiSndBuf,
                spiRcvBuf,
                /* Add several seconds to the timeout for Sigfox lib. */
                ((uint32_t)timeout + 3U) * 1000000u);

        /* Process the response. */
        if (status == sfStatusSuccess)
        {
            memcpy(buffer, spiRcvBuf + SF_RECV_TEST_FR_ACK_BUFF_OF, SF_RECV_TEST_FR_BUFF_B);
            *rssi = *((int8_t *)(spiRcvBuf + SF_RECV_TEST_FR_ACK_RSSI_OF));
        }
    }
    else
    {
        /* AUTHENTICATION_OFF */
    	/* Pack the command. */
        SF_PackFrame(sfSpiCmdRecvTestFrame, SF_RECV_TEST_FR_CMD_AOFF_PLD_B, NULL, spiSndBuf);
        SF_Uint32ToStringLsb(frequency, &spiSndBuf[SF_RECV_TEST_FR_CMD_FREQ_OF]);
        spiSndBuf[SF_RECV_TEST_FR_CMD_TOUT_OF] = timeout;
        memcpy(spiSndBuf + SF_RECV_TEST_FR_CMD_BUFF_OF, buffer, SF_RECV_TEST_FR_BUFF_B);

        /* Send the command. */
        status = SF_SendAndCheck(drvData, SF_RECV_TEST_FR_CMD_AOFF_B,
                SF_RECV_TEST_FR_ACK_AOFF_B,
                spiSndBuf,
                spiRcvBuf,
                /* Add several seconds to the timeout for Sigfox lib. */
                ((uint32_t)timeout + 3U) * 1000000u);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetLastRssi
 * Description   : Returns the recently computed RSSI.
 *
 *END**************************************************************************/
sf_status_t SF_GetLastRssi(sf_drv_data_t *drvData, int8_t *rssi)
{
    uint8_t spiSndBuf[SF_GET_LAST_RSSI_CMD_B];      /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_LAST_RSSI_ACK_LAST_B]; /* SPI receive buffer. */
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(rssi != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetLastRssi, SF_GET_LAST_RSSI_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_GET_LAST_RSSI_CMD_SUBCMD_OF] = SF_GET_LAST_RSSI_CMD_SUBCMD_LAST_RSSI;

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_GET_LAST_RSSI_CMD_B, SF_GET_LAST_RSSI_ACK_LAST_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        *rssi = (int8_t)(spiRcvBuf[SF_GET_LAST_RSSI_ACK_RSSI_OF]);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_GetLast30Rssi
 * Description   : Returns the RSSI for last 30 frames reported from RX GFSK
 *                 test or Receive_Test_Frame command.
 *
 *END**************************************************************************/
sf_status_t SF_GetLast30Rssi(sf_drv_data_t *drvData, int8_t *rssiBuffer)
{
    uint8_t spiSndBuf[SF_GET_LAST_RSSI_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_LAST_RSSI_ACK_LAST30_B]; /* SPI receive buffer. */
    sf_status_t status;

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);
    SF_MCU_Assert(rssiBuffer != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetLastRssi, SF_GET_LAST_RSSI_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_GET_LAST_RSSI_CMD_SUBCMD_OF] = SF_GET_LAST_RSSI_CMD_SUBCMD_LAST_30_RSSI;

    /* Send the command. */
    status = SF_SendAndCheck(drvData, SF_GET_LAST_RSSI_CMD_B, SF_GET_LAST_RSSI_ACK_LAST30_B,
                             spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);

    /* Process the response. */
    if (status == sfStatusSuccess)
    {
        memcpy(rssiBuffer,
               spiRcvBuf + SF_GET_LAST_RSSI_ACK_30_RSSI_OF,
               SF_GET_LAST_RSSI_ACK_30_RSSI_B);
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_ResetRssiBuffer
 * Description   : Resets buffers read by SF_GetLastRssi and SF_GetLast30Rssi
 *                 functions.
 *
 *END**************************************************************************/
sf_status_t SF_ResetRssiBuffer(sf_drv_data_t *drvData)
{
    uint8_t spiSndBuf[SF_GET_LAST_RSSI_CMD_B]; /* SPI send buffer. */
    uint8_t spiRcvBuf[SF_GET_LAST_RSSI_ACK_RESET_B]; /* SPI receive buffer. */

    /* Preconditions. */
    SF_MCU_Assert(drvData != NULL);

    /* Pack the command. */
    SF_PackFrame(sfSpiCmdGetLastRssi, SF_GET_LAST_RSSI_CMD_PLD_B, NULL, spiSndBuf);
    spiSndBuf[SF_GET_LAST_RSSI_CMD_SUBCMD_OF] = SF_GET_LAST_RSSI_CMD_SUBCMD_RESET;

    /* Send the command. */
    return SF_SendAndCheck(drvData, SF_GET_LAST_RSSI_CMD_B, SF_GET_LAST_RSSI_ACK_RESET_B,
            spiSndBuf, spiRcvBuf, SF_ACK_NOTRANS_TIMEOUT_US);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
