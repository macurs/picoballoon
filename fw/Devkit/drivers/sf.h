/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file sf.h
 *
 * SIGFOX driver supporting boards based on OL2385 with FW version:
 * 2.3.0.26.11.20.20.
 */

#ifndef SOURCE_SF_H_
#define SOURCE_SF_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/

#include "sf_ol2385.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*******************************************************************************
 * Defines
 ******************************************************************************/
/*!
 * @addtogroup sf_enum_group
 * Enumerations common for all models.
 * @{
 */
/*! @brief Specific error codes for this driver. */
typedef enum
{
    /*!< No error. */
    sfStatusSuccess = 0U,
    /*!< Device is not ready for the SPI communication (ACK pin is LOW). */
    sfStatusDeviceNotReady = 1U,
    /*!< Error in SPI transmission (coming from SF_MCU_TransferSpi function). */
    sfStatusSpiError = 2U,
    /*!< SPI communication timeout expired. */
    sfStatusSpiTimeout = 3U,
    /*!< Size of the receive buffer is smaller than the size of ACK frame to be
     * received. */
    sfStatusSpiAckLength = 4U,
    /*!< Command execution failed. An error was parsed from the ACK SPI frame.
     * Use the SF_GetErrorCode and SF_GetManufErrorCode functions to obtain
     * the error codes from the last ACK frame. */
    sfStatusCmdFailed = 5U
} sf_status_t;

/*! @brief SPI command codes. */
typedef enum
{
    sfSpiCmdWakeUp          = SF_CMD_WAKEUP_ID,           /*!< Wake_Up command. */
    sfSpiCmdSleep           = SF_CMD_SLEEP_ID,            /*!< Sleep command. */
    sfSpiCmdTxFrame         = SF_CMD_TX_FRAME_ID,         /*!< TX_Frame command. */
    sfSpiCmdBitFrame        = SF_CMD_BIT_FRAME_ID,        /*!< Bit_Frame command. */
    sfSpiCmdOobTxFrame      = SF_CMD_OOB_TX_FRAME_ID,     /*!< OOB_TX_Frame command. */
    sfSpiCmdTrxFrame        = SF_CMD_TRX_FRAME_ID,        /*!< TRX_Frame command. */
    sfSpiCmdStartCont       = SF_CMD_START_CONT_ID,       /*!< Start_Continuous_Transmission command. */
    sfSpiCmdStopCont        = SF_CMD_STOP_CONT_ID,        /*!< Stop_Continuous_Transmission command. */
    sfSpiCmdGetDevVer       = SF_CMD_GET_DEV_VER_ID,      /*!< Get_Device_Version command. */
    sfSpiCmdTestMode        = SF_CMD_TEST_MODE_ID,        /*!< Test_Mode command. */
    sfSpiCmdSetPrivateKey   = SF_CMD_SWITCH_PRIVATE_ID,   /*!< Switch_To_Private_Key command. */
    sfSpiCmdSetPublicKey    = SF_CMD_SWITCH_PUBLIC_ID,    /*!< Switch_To_Public_Key command. */
    sfSpiCmdGetLbtInfo      = SF_CMD_GET_LBT_INFO_ID,     /*!< Get LBT info  command. */
    sfSpiCmdStaticFreqCal   = SF_CMD_STATIC_FREQ_CAL_ID,  /*!< Static_Frequency_Calibration command. */
    sfSpiCmdTempFreqCal     = SF_CMD_TEMP_FREQ_CAL_ID,    /*!< Temperature_Frequency_Calibration command. */
    sfSpiCmdGetLastRssi     = SF_CMD_GET_LAST_RSSI_ID,    /*!< Get_Last_RSSI command. */
    sfSpiCmdRssiCalib       = SF_CMD_RSSI_CALIB_ID,       /*!< RSSI_Calibration command. */
    sfSpiCmdGetRc           = SF_CMD_GET_RC_ID,           /*!< Get_RC command. */
    sfSpiCmdSetCustomRc     = SF_CMD_SET_CUSTOM_RC_ID,    /*!< Set_Custom_RC command. */
    sfSpiCmdGetSigId        = SF_CMD_GET_ID_ID,           /*!< Get_Sigfox_ID command. */
    sfSpiCmdGetInitPac      = SF_CMD_GET_INIT_PAC_ID,     /*!< Get_Initial_Sigfox_PAC command. */
    sfSpiCmdGetLibVer       = SF_CMD_GET_LIB_VER_ID,      /*!< Get_Sigfox_Lib_Version command. */
    sfSpiCmdSetRcSyncPer    = SF_CMD_SET_RC_SYNC_PER_ID,  /*!< Set RC Sync Period command */
    sfSpiCmdSendTestFrame   = SF_CMD_SEND_TEST_FR_ID,     /*!< Send_Test_Frame command. */
    sfSpiCmdRecvTestFrame   = SF_CMD_RECV_TEST_FR_ID,     /*!< Receive_Test_Frame command. */
    sfSpiCmdSetCertId       = SF_CMD_SET_CERT_ID_ID,      /*!< Set_Certification_ID command. */
    sfSpiCmdGetAddonVer     = SF_CMD_GET_ADDON_VER_ID,    /*!< Get_Sigfox_Addon_Version command. */
    sfSpiCmdSetHwConfig     = SF_CMD_SET_HW_CFG_ID,       /*!< Set_HW_Configuration command. */
    sfSpiCmdGetHwConfig     = SF_CMD_GET_HW_CFG_ID,       /*!< Get_HW_Configuration command. */
    sfSpiCmdChangeRc        = SF_CMD_CHANGE_RC_ID,        /*!< Change_RC command. */
    sfSpiCmdSaveRcToErom    = SF_CMD_SAVE_RC_EROM_ID,     /*!< Save_RC_To_EROM command. */
    sfSpiCmdSetTxConfig     = SF_CMD_SET_TX_CFG_ID,       /*!< Set_TX_Configuration command. */
    sfSpiCmdSaveTxCfgToErom = SF_CMD_SAVE_TX_CFG_EROM_ID, /*!< Save_TX_Config_To_EROM command. */
    sfSpiCmdGetTxConfig     = SF_CMD_GET_TX_CFG_ID        /*!< Get_TX_Configuration command. */
} sf_spi_cmd_t;

/*! @brief Error codes received in SPI acknowledgement frames. */
typedef enum
{
    sfErrNone                           = 0x00U, /*!< No error. */
    sfErrUnknownSpiCommand              = 0x01U, /*!< Unknown SPI command. */
    sfErrWrongSpiParameter              = 0x02U, /*!< Wrong parameter passed in the CMD SPI frame payload. */
    sfErrCommandLengthNotOk             = 0x03U, /*!< Command length not OK. */

    sfErrApiOpen                        = 0x10U, /*!< Error occurs during the opening of the Sigfox Library: Check the manuf. error code. */
    sfErrApiOpenState                   = 0x11U, /*!< State is not IDLE, library should be closed before. */
    sfErrApiOpenGetNvmemMemoryOverlap   = 0x12U, /*!< MCU_API_get_nv_mem overlap the memory. */
    sfErrApiOpenRcPtr                   = 0x13U, /*!< RC pointer is NULL. */
    sfErrApiOpenOpenMacroChannelWidth   = 0x14U, /*!< Macro channel width is not authorized by Sigfox Lib, check your RC configuration. */

    sfErrApiCloseFree                   = 0x20U, /*!< Error occurs during the closing of the Sigfox Library: error on MCU_API_free. */
    sfErrApiCloseState                  = 0x21U, /*!< Error occurs during the closing of the Sigfox Library: error on library state. */

    sfErrApiSendFrameDataLength         = 0x30U, /*!< Customer data length > 12 Bytes. */
    sfErrApiSendFrameResponsePtr        = 0x31U, /*!< Response data pointer NULL in case of downlink. */
    sfErrApiSendFrameDelayOobAck        = 0x32U, /*!< Error on MANUF_API_delay w/ SFX_DLY_OOB_ACK (Downlink). */
    sfErrApiSendFrameDataPtr            = 0x33U, /*!< Customer data pointer NULL. */

    sfErrApiSendBitResponsePtr          = 0x34U, /*!< Response data pointer NULL in case of downlink. */
    sfErrApiSendOobType                 = 0x35U, /*!< Wrong enum value for the OOB type. */

    sfErrApiSetStdConfigCsConfig        = 0x40U, /*!< Error on the carrier sense configuration - check the config words. */
    sfErrApiSetStdConfigFhChannels      = 0x41U, /*!< Config word empty whereas they should configure channels for Frequency Hopping. */

    sfErrApiSendTestFrameDeviceId       = 0x50U, /*!< Device Id that should be used in SIGFOX_API_send_test_frame function must be 0xFEDCBA98. */
    sfErrApiSendTestFrameState          = 0x51U, /*!< State is not READY - Should open the library. */
    sfErrApiSendTestFrameDataLength     = 0x52U, /*!< Customer data length > 12 Bytes. */
    sfErrApiSendTestFrameDataPtr        = 0x53U, /*!< Customer data pointer NULL. */
    sfErrApiSendTestStoreNvm            = 0x54U, /*!< Error occurs during the NVM Storage: Check the manuf. error code to get the error. */

    sfErrApiReceiveTestFrameDeviceId    = 0x55U, /*!< Device Id that should be used in SIGFOX_API_send_test_frame function must be 0xFEDCBA98. */
    sfErrApiReceiveTestFrameState       = 0x56U, /*!< State is not READY - Should open the library. */

    sfErrApiStartContTrans              = 0x57U, /*!< Error occurs during the start continuous transmission: Check the manuf. error code. */
    sfErrApiStartContTransState         = 0x58U, /*!< State is not idle, library should be closed before. */
    sfErrApiStopContTrans               = 0x59U, /*!< Error occurs during the stop continuous transmission: Check the manuf. error code. */
    sfErrApiStopContTransState          = 0x5AU, /*!< State is not TX, function SIGFOX_API_start_continuous_tranmission has to be called before. */

    sfErrApiGetInitialPac               = 0x5BU, /*!< Error occurs when trying to retrieve the PAC: Check the manuf. error code. */
    sfErrApiGetVersion                  = 0x5CU, /*!< Error occurs when trying to retrieve the version: Check the manuf. error code. */
    sfErrApiGetVersionWrongType         = 0x5DU, /*!< Error occurs when trying to retrieve the version: Wrong version type - see the enum sfx_version_type_t. */
    sfErrApiSwitchPublicKey             = 0x5EU, /*!< Error occurs when switching device key: State is not READY - Should open the library. */

    sfErrIntExecuteComSeqState          = 0x60U, /*!< State is not READY, library should be opened before. */
    sfErrIntExecuteComSeqNvmMessage     = 0x61U, /*!< Error occurs during the NVM storage used for uplink transmission: Check the manuf error code. */
    sfErrIntExecuteComSeqNvmAck         = 0x62U, /*!< Error occurs during the NVM storage used for ack transmission: Check the manuf error code. */
    sfErrIntExecuteComSeqNvmRcsync      = 0x63U, /*!< Error occurs during the NVM storage used for rc sync transmission: Check the manuf error code. */
    sfErrIntExecuteComSeqDelayOobAck    = 0x64U, /*!< Error occurs when setting the delay between downlink and ack: Check the manuf error code. */

    sfErrIntProcUlStartTimerFhInDl      = 0x70U, /*!< Error when calling MCU_API_timer_start for FH: Check the manuf error code. */
    sfErrIntProcUlWaitForEndTimerFhInDl = 0x71U, /*!< Error when calling MCU_API_timer_stop for FH: Check the manuf. error code. */
    sfErrIntProcUlTimerFh               = 0x72U, /*!< Error when starting the timer after the first frame to respect the FCC regulation - timer_enable config set to 1. */
    sfErrIntProcUlWaitForEndTimerFh     = 0x73U, /*!< Error when stopping the timer after the first frame. */
    sfErrIntProcUlDelayInterframe       = 0x74U, /*!< Error when executing the interframe delay: Check the manuf. error code. */
    sfErrIntProcUlTimerDonwlink         = 0x75U, /*!< Error when starting the timer after the first frame to prepare the downlink. */
    sfErrIntProcUlCsRetry               = 0x76U, /*!< Error when executing the Carrier Sense for the first frame: Check the manuf. error code. to get the error. */
    sfErrIntProcUlCsRetryStartTimer     = 0x77U, /*!< Error Carrier Sense for the first frame on start timer: Check the manuf. error code. */
    sfErrIntProcUlCsRetryStopTimer      = 0x78U, /*!< Error Carrier Sense for the first frame on stop timer: Check the manuf. error code. */
    sfErrIntProcUlCsRetryDelayAttempt   = 0x79U, /*!< Error on executing the delay between several attempts of the first frame: Check the manuf. error code. */
    sfErrIntProcUlCsRepet               = 0x7AU, /*!< Error Carrier Sense for frame 2 and 3: Check the manuf. error code. */
    sfErrIntProcUlCsRepetStartTimer     = 0x7BU, /*!< Error Carrier Sense for starting the timer for CS on frame 2 and 3: Check the manuf. error code. */
    sfErrIntProcUlCsRepetStopTimer      = 0x7CU, /*!< Error Carrier Sense for stopping the timer for CS on frame 2 and 3: Check the manuf. error code. */
    sfErrIntProcUlCsRepetStopTimer2     = 0x7DU, /*!< Error Carrier Sense for stopping the timer for CS on frame 2 and 3: Check the manuf. error code. */
    sfErrIntProcUlCsTimeout             = 0x7EU, /*!< Error Carrier Sense unsuccessful. */

    sfErrIntBuildFrameSe                = 0x90U, /*!< Error occurs when building the frame with a SE: Check the manuf. error code. */
    sfErrIntBuildFrame                  = 0x91U, /*!< Error occurs when building the frame: Check the manuf. error code. */
    sfErrIntBuildFrameOobService        = 0x92U, /*!< Error occurs when building the OOB Frame in MCU_API_get_voltage_temperature: Check the manuf. error code. */
    sfErrIntBuildFrameOobDownlinkAck    = 0x93U, /*!< Error occurs when building the OOB downlink frame in MCU_API_get_voltage_temperature: Check the manuf. error code. */
    sfErrIntBuildFrameOobRepeaterStatus = 0x94U, /*!< Error occurs when building the OOB REPEATER_FRAME in REPEATER_API_get_voltage: Check the manuf. error code. */
    sfErrIntBuildFrameOobRcSync         = 0x95U, /*!< Error occurs when building the OOB RCSYNC frame: Check the manuf. error code. */
    sfErrIntBuildFramePayloadCrypted    = 0x96U, /*!< Error occurs when building the encrypted frame: Check the manuf. error code. */

    sfErrIntSendSingleFrame             = 0x97U, /*!< Error when sending a frame: Check the manuf. error code. */
    sfErrIntProcessDownlink             = 0x98U, /*!< Error when starting the downlink: In MCU_API_timer_wait_for_end: Check the manuf. error code. */

    sfErrIntGetDeviceId                 = 0x99U, /*!< Error when retrieving the device ID: Check the manuf. error code. */
    sfErrIntGetRcvFrames                = 0x9AU, /*!< Error occurs when trying to receive frames: Check the manuf. error code. */
    sfErrIntGetRcvFramesTimeout         = 0x9BU, /*!< Timeout on frame reception. */
    sfErrIntGetRcvFramesWaitNotExecuted = 0x9CU, /*!< State return by the RF_API_wait_frame is downlink reception not executed. */

    sfErrIntGetDeviceInfo               = 0x9DU, /*!< Error when retrieving the device info: Check the manuf. error code. */
    sfErrIntGetDeviceInfoCrc            = 0x9EU, /*!< Error when checking the validty of the device info - CRC is bad. */
    sfErrIntGetDeviceInfoCertificate    = 0x9FU, /*!< Error when checking the validty of the device info - Certificate is not the appropriate one. */

    sfErrApiSetRcSyncPeriod             = 0xB0U, /*!< Set RC Sync frame transmission period failed. */
    sfErrApiSetRcSyncPeriodValue        = 0xB1U, /*!< Error in the RC Sync period value. */

    sfErrRfProtocolTimerStart           = 0xC0U, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_TIMER_START. */
    sfErrRfProtocolReportTest           = 0xC1U, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_REPORT_TEST. */
    sfErrRfProtocolSleepSendBit1        = 0xC2U, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_SLEEP_SEND_BIT_1. */
    sfErrRfProtocolSleepSendBit2        = 0xC3U, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_SLEEP_SEND_BIT_2. */
    sfErrRfProtocolSleepSendOob         = 0xC4U, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_SLEEP_SEND_OUTOFBAND. */
    sfErrRfProtocolSleepSendRepStatus   = 0xC5U, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_SLEEP_SEND_REPEATER_STATUS. */
    sfErrRfProtocolSleepSendFrame       = 0xC6U, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_SLEEP_SEND_FRAME. */
    sfErrRfProtocolTxDbpsk              = 0xC7U, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_TX_DBPSK. */
    sfErrRfProtocolDelaySensi           = 0xC8U, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_DELAY_SENSI. */
    sfErrRfProtocolDelayTxSynth         = 0xC9U, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_DELAY_TX_SYNTH. */
    sfErrRfProtocolWrongRcEnum          = 0xCAU, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_WRONG_RC_ENUM. */
    sfErrRfProtocolGetPayloadEncrFlag   = 0xCBU, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_GET_PAYLOAD_ENCRYPTION_FLAG. */
    sfErrRfProtocolTestModeUnknown      = 0xCCU, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_TEST_MODE_UNKNOWN. */
    sfErrRfProtocolSleepSendRcSync      = 0xCDU, /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_SLEEP_SEND_RC_SYNC. */
    sfErrRfProtocolNvm                  = 0xD0U  /*!< Addon: Return code SFX_RF_PROTOCOL_ERR_API_NVM. */
} sf_spi_error_t;

/*! @brief Manuf. error codes received in SPI acknowledgement frames. */
typedef enum
{
    sfManufErrNone                 = 0x00U, /*!< No error. */

    sfManufErrMcuMalloc            = 0x11U, /*!< Error on MCU_API_malloc. */
    sfManufErrMcuFree              = 0x12U, /*!< Error on MCU_API_free. */
    sfManufErrMcuVoltTemp          = 0x13U, /*!< Error on MCU_API_get_voltage_temperature. */
    sfManufErrMcuDly               = 0x14U, /*!< Error on MCU_API_delay. */
    sfManufErrMcuAes               = 0x15U, /*!< Error on MCU_API_aes_128_cbc_encrypt. */
    sfManufErrMcuGetnvmem          = 0x16U, /*!< Error on MCU_API_get_nv_mem. */
    sfManufErrMcuSetnvmem          = 0x17U, /*!< Error on MCU_API_set_nv_mem. */
    sfManufErrMcuTimerStart        = 0x18U, /*!< Error on MCU_API_timer_start. */
    sfManufErrMcuTimerStartCs      = 0x19U, /*!< Error on MCU_API_timer_start_carrier_sense. */
    sfManufErrMcuTimerStopCs       = 0x1AU, /*!< Error on MCU_API_timer_stop_carrier_sense. */
    sfManufErrMcuTimerStop         = 0x1BU, /*!< Error on MCU_API_timer_stop. */
    sfManufErrMcuTimerEnd          = 0x1CU, /*!< Error on MCU_API_timer_wait_for_end. */
    sfManufErrMcuTestReport        = 0x1DU, /*!< Error on MCU_API_report_test_result. */
    sfManufErrMcuGetVersion        = 0x1EU, /*!< Error on MCU_API_get_version. */
    sfManufErrMcuGetIdPaylEncrFlag = 0x1FU, /*!< Error on MCU_API_get_device_id_and_payload_encryption_flag. */
    sfManufErrMcuGetPac            = 0x20U, /*!< Error on MCU_API_get_initial_pac. */

    sfManufErrRfInit               = 0x30U, /*!< Error on RF_API_init. */
    sfManufErrRfSend               = 0x31U, /*!< Error on RF_API_send. */
    sfManufErrRfChangeFreq         = 0x32U, /*!< Error on RF_API_change_frequency. */
    sfManufErrRfStop               = 0x33U, /*!< Error on RF_API_stop. */
    sfManufErrRfWaitFrame          = 0x34U, /*!< Error on RF_API_wait_frame. */
    sfManufErrRfWaitClearChannel   = 0x35U, /*!< Error on RF_API_wait_for_clear_channel. */
    sfManufErrRfStartContTrans     = 0x36U, /*!< Error on RF_API_start_continuous_transmission. */
    sfManufErrRfStopContTrans      = 0x37U, /*!< Error on RF_API_stop_continuous_transmission. */
    sfManufErrRfGetVersion         = 0x38U  /*!< Error on RF_API_get_version. */
} sf_spi_manuf_err_t;

/*! @brief Oscillator type. */
typedef enum
{
    sfOsc276Tcxo = 0U,     /*!< 27.6 MHz TCXO (recommended). Used in:
                                - Innocomm SN10-11 and SN10-12 modules or
                                - OM2385/SF001EU, OM2385/SF001US kits. */
    sfOsc552Xtal = 1U      /*!< 55.2 MHz XTAL. Used in the OM2385/SF001 kit. */
} sf_osc_type_t;

/*! @brief Amount of TX transmissions that will be invoken by SF_SendFrame,
 * SF_SendBit and SF_SendReceiveFrame functions. */
typedef enum
{
    sfTxRepeat1 = 0U,     /*!< One TX transmission. */
    sfTxRepeat3 = 2U      /*!< Three TX transmissions. */
} sf_tx_repeat_t;

/*! @brief Selection of internal PA type used in the device. */
typedef enum
{
    sfPa14 = 0U,     /*!< 14 dBm PA. */
    sfPa0  = 1U      /*!< 0 dBm PA. */
} sf_pa_type_t;

/*! @brief List of standard Radio Configurations. */
typedef enum
{
    /*!< RC1: Spectrum access: Duty Cycle
	 *        Uplink frequency: 868130000 Hz
	 *        Downlink frequency: 869525000 Hz
	 *        Macro channel width: 192000 Hz
	 *        Uplink modulation: 100 bps DBPSK */
    sfRc1   = 0U,
    /*!< RC2: Spectrum access: Frequency Hopping
	 *        Uplink frequency: 902200000 Hz
	 *        Downlink frequency: 905200000 Hz
	 *        Macro channel width: 192000 Hz
	 *        Uplink modulation: 600 bps DBPSK */
    sfRc2   = 1U,
    /*!< RC3A: Spectrum access: Listen Before Talk
	 *         Uplink frequency: 923200000 Hz
	 *         Downlink frequency: 922200000 Hz
	 *         Macro channel width: 36000 Hz
	 *         Uplink modulation: 100 bps DBPSK
	 *         Carrier sense center frequency: 923200000 Hz
	 *         Carrier sense bandwidth: 200000 Hz
	 *         LBT threshold: -80 dBm */
	sfRc3a  = 2U,
    /*!< RC3C: Spectrum access: Listen Before Talk
	 *         Uplink frequency: 923200000 Hz
	 *         Downlink frequency: 922200000 Hz
	 *         Macro channel width: 192000 Hz
	 *         Uplink modulation: 100 bps DBPSK
	 *         Carrier sense center frequency: 923200000 Hz
	 *         Carrier sense bandwidth: 200000 Hz
	 *         LBT threshold: -80 dBm */
    sfRc3c  = 3U,
    /*!< RC4: Spectrum access: Frequency Hopping
	 *        Uplink frequency: 902200000 Hz
	 *        Downlink frequency: 922300000 Hz
	 *        Macro channel width: 192000 Hz
	 *        Uplink modulation: 600 bps DBPSK */
    sfRc4   = 4U,
    /*!< RC5: Spectrum access: Listen Before Talk
	 *        Uplink frequency: 923300000 Hz
	 *        Downlink frequency: 922300000 Hz
	 *        Macro channel width: 192000 Hz
	 *        Uplink modulation: 100 bps DBPSK
	 *        Carrier sense center frequency: 923300000 Hz
	 *        Carrier sense bandwidth: 200000 Hz
	 *        LBT threshold: -65 dBm */
    sfRc5   = 5U,
    /*!< RC6: Spectrum access: Duty Cycle
	 *        Uplink frequency: 865200000 Hz
	 *        Downlink frequency: 866300000 Hz
	 *        Macro channel width: 192000 Hz
	 *        Uplink modulation: 100 bps DBPSK */
    sfRc6   = 6U,
    /*!< RC7: Spectrum access: Duty Cycle
	 *        Uplink frequency: 868800000 Hz
	 *        Downlink frequency: 869100000 Hz
	 *        Macro channel width: 192000 Hz
	 *        Uplink modulation: 100 bps DBPSK */
	sfRc7   = 7U,
} sf_rc_enum_t;

/*! @brief Out of Band frame types that can be send by the user applications. */
typedef enum
{
    sfOobService = 0U,     /*!< Contains device temperature, battery voltage
                               (VDD Idle and VDD Tx) and RSSI. Can be used every
                               24 hours maximum or never (if the application has
                               some energy critical constraints). */
    sfOobRcSync  = 1U,    /*!< If payload encryption is supported and activated,
                               this OOB can be used to synchronize the device
                               and the backend. */
} sf_oob_type_t;

/*! @brief Data type for Spectrum Access. */
typedef enum
{
    sfSpectrumAccessFH   = 1U,  /*!< Frequency Hopping. */
    sfSpectrumAccessLBT  = 2U,  /*!< Listen Before Talk. */
    sfSpectrumAccessDC   = 4U,  /*!< Duty Cycle. */
} sf_spectrum_access_t;

/*! @brief Uplink Modulation type and baudrate. */
typedef enum
{
    sfNoModulation       = 0U, /*!< No modulation: Signal is a pure carrier. */
    sfDbpsk100bps        = 1U, /*!< DBPSK Modulation with 100 bps baudrate. */
    sfDbpsk600bps        = 2U, /*!< DBPSK Modulation with 600 bps baudrate. */
} sf_modulation_t;

/*! @brief Uplink Modulation type and baudrate. */
typedef enum
{
    sfPaCurveRam         = 0U, /*!< Current PA curve (stored in RAM). */
    sfPaCurve100bps      = 1U, /*!< Default PA curve for 100 bps baudrate (stored in EROM). */
    sfPaCurve600bps      = 2U, /*!< Default PA curve for 600 bps baudrate (stored in EROM). */
} sf_pa_curve_t;

/*! @brief Selection of a test mode used to verify the protocol, RF and sensitivity. */
typedef enum
{
    sfTestModeTxBpsk              = 0U,   /*!< Only BPSK with Synchro Bit + Synchro frame + PN sequence: No hopping centered on the TX_frequency. */
    sfTestModeTxProtocol          = 1U,   /*!< With full protocol with AES key defined at SIGFOX_API_open call: Send all SIGFOX protocol frames available with hopping. */
    sfTestModeRxProtocol          = 2U,   /*!< With full protocol with AES key defined at SIGFOX_API_open call: Send SIGFOX protocol frames w/ initiate_downlink_flag = SFX_TRUE. */
    sfTestModeRxGfsk              = 3U,   /*!< With known pattern with SB + SF + Pattern on RX_Frequency defined at SIGFOX_API_open function: Internaly compare received frame <=> known pattern and call sfx_test_report(). */
    sfTestModeRxSensi             = 4U,   /*!< Do uplink + downlink frame with AES key defined at SIGFOX_API_open call but specific shorter timings. */
    sfTestModeTxSynth             = 5U,   /*!< Do 9 uplink frames to measure frequency synthesis step. */
    sfTestModeTxFreqDistribution  = 6U,   /*!< Call all Sigfox frames of all types and size on all the Sigfox Band. */
    sfTestModeTxBit               = 11U,  /*!< Call twice the Sigfox frames (payload 1 bit only). */
    sfTestModePublicKey           = 12U,  /*!< Execute the public key test - all the frames of the protocol needs to be sent. */
    sfTestModeNvm                 = 13U   /*!< Execute the NVM test. */
} sf_test_mode_t;
/*! @} */

/*!
 * @addtogroup sf_struct_group
 * Structures common for all models.
 * @{
 */
/*! @brief HW configuration of the system based on OL2385. */
typedef struct
{
    sf_osc_type_t oscType;          /*!< Oscillator type. */
    bool externalPaUsed;            /*!< True if external PA is used. */
} sf_hw_config_t;

/*! @brief TX configuration. */
typedef struct
{
    /*!< Config words:
     *   - DC (Duty Cycle): Config Words unused.
     *   - LBT (Listen Before Talk): Carrier Sense feature for the first frame
     *     can be configured.
     *       - configWords[0] : Number of attempts to send the first frame
     *           [has to be greater or equal to 1]
     *       - Config_words[1] : Maximum carrier sense sliding window (in ms)
     *           [has to be greater than 6 ms (CS_MIN_DURATION_IN_MS + 1)]
     *       - Config_words[2] :
     *           . bit 8   : Set the value to 1 to indicate that the device will
     *                       use the full operational radio band (192kHz).
     *           . bit 7-3 : Number of Carrier Sense attempts.
     *           . bit 2-0 : Number of frames sent.
     *   - FH (Frequency Hopping): Config words to enable/disable 192KHz macro
     *     channels authorized for transmission. Each macro channel is separated
     *     from another of 300 kHz. At least 9 macro channel must be enabled to
     *     ensure the minimum of 50 FCC channels (9*6 = 54).
     *     WARNING : SF_SetTxConfig should be called each time your FCC
     *     configuration will not be applied.
     *
     *   configWords recommended in the Sigfox library:
     *   - RC2 (Short message): {0x00000001, 0x00000000, 0x00000000}
     *   - RC2 (Long message): {0x000001FF, 0x00000000, 0x00000000}
     *   - RC4 (Short message): {0x00000000, 0x40000000, 0x00000000}
     *   - RC4 (Long message): {0x00000000, 0xF0000000, 0x0000001F}
     *   - RC3A, RC3B, RC5: {0x00000003, 0x00001388, 0x00000000}
     *
     *   The example below shows you the Long Message configuration for RC2.
     *   Example: To enable Macro channel 1 to 9, that is to say 902.2MHz to
     *   904.6MHz with 902.2MHz as main Macro channel, you must set:
     *     Config_words[0] = [0x000001FF]
     *     Config_words[1] = [0x00000000]
     *     Config_words[2] = [0x00000000] */
    uint32_t configWords[3];
    /*!< Timer enable feature for Frequency Hopping. */
    bool timerEnable;
    /*!< Payload encryption: True for enable, False for disable. */
    bool payloadEncEn;
    /*!< TX Repeat Number: The amount of TX transmissions in SF_SendFrame,
     * SF_SendReceiveFrame and SF_SendBit functions. */
    sf_tx_repeat_t txRepeat;
    /*!< Amount of 0.25 dB steps that will be subtracted from the maximal TX power. */
    uint8_t txAttSteps;
    /*!< Selection of internal PA type used in the device. */
    sf_pa_type_t paType;
} sf_tx_config_t;

/*! @brief This data structure is used by the SIGFOX driver (it is the first
 * parameter of most SIGFOX functions). */
typedef struct
{
    uint8_t drvInstance;            /*!< SIGFOX driver instance. Passed to the external
                                         functions defined by the user. */
    sf_spi_error_t errorCode;       /*!< The 'error code' received in the last
                                         acknowledgement SPI frame. */
    sf_spi_manuf_err_t manufError;  /*!< The 'manuf. error frame' received in
                                         the last acknowledgement SPI frame. */
} sf_drv_data_t;

/*! @brief This structure represents the payload of SPI frames. */
typedef struct
{
    uint8_t payloadLen;             /*!< The size of the payload in bytes. */
    uint8_t *payload;               /*!< Pointer to the frame payload. */
} sf_msg_payload_t;

/*! @brief Radio configuration substruture for specific radio configurations. */
typedef struct
{
    uint32_t openCsFrequency; /*!< Carrier sense center frequency in [Hz].
                                   Can be equal to uplink center frequency. */
    uint32_t openCsBandwidth; /*!< Carrier sense bandwidth in [Hz] to apply
                                   carrier sensing. */
    int8_t csThreshold;       /*!< LBT threshold in [dBm] defined in the
                                   standards related to the RC. */
} sf_rc_specific_t;

/*! @brief Radio configuration structure. */
typedef struct
{
    uint32_t openTxFrequency;            /*!< Uplink frequency in [Hz]. This is not
                                              necessary the Transmitter center frequency
                                              as it may depends on the values
                                              set in config words (function SF_SetTxConfig). */
    uint32_t openRxFrequency;            /*!< Downlink frequency in [Hz]. */
    uint32_t macroChannelWidth;          /*!< Macro channel width in [Hz]
                                              (= SIGFOX Operational radio band. */
    sf_modulation_t modulation;          /*!< Uplink modulation and baudrate. */
    sf_spectrum_access_t spectrumAccess; /*!< Spectrum access. */
    sf_rc_specific_t specificRc;         /*!< Specific radio configuration for LBT. */
} sf_rc_t;
/*! @} */

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @addtogroup sf_functions_group
 * @{
 */

/*!
 * @brief Initializes internal driver structure and (optionally) configures the
 * HW configuration of the OL2385 device.
 *
 * If parameter hwConfig is not NULL, OL2385 is woken up by this function!
 *
 * @param drvData  Driver run-time data.
 * @param hwConfig HW configuration. Can be NULL if there is no need to
 *                 change it. If not NULL, OL2385 device is woken up.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_Init(sf_drv_data_t *drvData, sf_hw_config_t *hwConfig);

/*!
 * @brief Returns 'error code' parsed from the last ACK SPI frame received from
 * the OL2385 device.
 *
 * It is useful to use this function when any of the SF_* functions returns
 * sfStatusCmdFailed, i.e. when there was an error in performing the SPI
 * command inside the SPI firmware. Some of the sf_spi_error_t errors can
 * further require to read the 'manuf. error code' using SF_GetManufErrorCode
 * for more details.
 *
 * @param drvData Driver run-time data.
 *
 * @return An error code received in the last SPI ACK frame.
 */
sf_spi_error_t SF_GetErrorCode(sf_drv_data_t *drvData);

/*!
 * @brief Returns 'manuf. error code' parsed from the last ACK SPI frame
 * received from the OL2385 device.
 *
 * Note that the main error code from OL2385 is 'error code' and can be
 * retrieved by SF_GetErrorCode(). However, several error codes provide
 * more detailed information via 'manuf. error code'.
 *
 * @param drvData Driver run-time data.
 *
 * @return A manuf. error code received in the last SPI ACK frame.
 */
sf_spi_manuf_err_t SF_GetManufErrorCode(sf_drv_data_t *drvData);

/*!
 * @brief Sends a command to the device and waits for an acknowledgement
 * (if any).
 *
 * @param drvData       Driver run-time data.
 * @param cmd           The frame command number.
 * @param cmdPayload    Pointer to a payload to be sent. The user can set
 *                      payload data (sendPayload->payload) to NULL when the
 *                      size is zero.
 * @param ackPayload    Pointer where a resulting payload is stored. It can be
 *                      NULL if the payload is not desired or present in the
 *                      ACK frame. The user can use SF_HAS_CMD_ACK to check if
 *                      a command has an ACK frame. Size of the array
 *                      (ackPayload->payload) must be sufficient
 *                      (see SF_ACK_PAYLOAD_MAX_B macro). The size variable
 *                      (ackPayload->payloadLen) will be set by this function
 *                      and indicates the number of actually received bytes.
 * @param ackPaylBufLen Size of the receive buffer (ackPayload->payload) in
 *                      bytes.
 * @param timeout       Timeout for receiving the SPI ACK frame in [us].
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SendCommandBlock(sf_drv_data_t *drvData, sf_spi_cmd_t cmd,
    const sf_msg_payload_t *cmdPayload, sf_msg_payload_t *ackPayload,
    uint8_t ackPaylBufLen, uint32_t timeout);

/*!
 * @brief Sends a command to the device without waiting for an acknowledgement.
 *
 * The user can use SF_HAS_CMD_ACK macro to figure out if a command
 * has ACK. Then he must utilize SF_IsAckFrameReady and SF_ReadAckFrameNonBlock
 * functions to receive the ACK SPI frame.
 *
 * @param drvData     Driver run-time data.
 * @param cmd         The frame command number.
 * @param sendPayload Pointer to a frame payload to be sent. The user can set
 *                    payload data (sendPayload->payload) to NULL when the size
 *                    is zero.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SendCommandNonBlock(sf_drv_data_t *drvData, sf_spi_cmd_t cmd,
    const sf_msg_payload_t *sendPayload);

/*!
 * @brief Checks if the SIGFOX device is ready to send an acknowledgement. It
 * checks the ACK pin value, which is active low when the device is ready.
 *
 * The user can use SF_HAS_CMD_ACK macro to check if a command has an
 * acknowledgement.
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
bool SF_IsAckFrameReady(sf_drv_data_t *drvData);

/*!
 * @brief Receives an acknowledge frame.
 *
 * It should be used in conjunction with SF_SendCommandNonBlock and
 * SF_IsAckFrameReady functions. The user can use SF_HAS_CMD_ACK macro to check
 * if a command has an acknowledgement.
 *
 * @param drvData        Driver run-time data.
 * @param recvPayload    Pointer where a resulting payload will be stored. Size
 *                       of recvPayload->payload array must be sufficient
 *                       (see SF_ACK_PAYLOAD_MAX_B macro). The size variable
 *                       (recvPayload->payloadLen) will be set by this function
 *                       and indicates the number of actually received bytes.
 * @param recvBufferSize Size of the payload buffer (recvPayload->payload)
 *                       in bytes.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_ReadAckFrameNonBlock(sf_drv_data_t *drvData,
    sf_msg_payload_t *recvPayload, uint8_t recvBufferSize);

/*!
 * @brief Wakes OL2385 from the low power mode (POWER_OFF2) up.
 *
 * By default, the device goes immediately to low power sleep mode after each
 * reset (software device reset, power on-off reset or battery-on reset) or
 * after explicit SF_Sleep function. The CS pin of SPI interface is the wake-up
 * pin and has to be driven low by MCU to wake the device up. Therefore, any
 * command (function) can be used to wake the device up, not only SF_WakeUp.
 *
 * The device is automatically initialized with:
 *  - HW configuration stored in non-volatile EROM memory
 *    (see SF_SetHwConfig function).
 *  - Radio configuration stored in non-volatile EROM memory
 *    (see SF_SaveRcToErom function).
 *  - TX configuration stored in non-volatile EROM memory
 *    (see SF_SaveTxConfigToErom function).
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_WakeUp(sf_drv_data_t *drvData);

/*!
 * @brief Sends OL2385 to low power mode.
 *
 * Even if you use some transistor or swich for OL2385 power lines to save the
 * energy, it is mandatory to send this function to OL2385 in order the device
 * can store required data to EROM. The CS pin of SPI interface is than the
 * wake-up pin (see the SF_WakeUp function).
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_Sleep(sf_drv_data_t *drvData);

/*!
 * @brief Sets the HW configuration.
 *
 * The configuration is stored in non-volatile EROM memory so that you don't
 * have to call this function after each OL2385 wake-up.
 *
 * @param drvData Driver run-time data.
 * @param cfg     Pointer to the HW configuration.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SetHwConfig(sf_drv_data_t *drvData, sf_hw_config_t *cfg);

/*!
 * @brief Reads the HW configuration stored in OL2385.
 *
 * @param drvData Driver run-time data.
 * @param cfg     Pointer where the HW configuration will be stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetHwConfig(sf_drv_data_t *drvData, sf_hw_config_t *cfg);

/*!
 * @brief Initializes the OL2385 for one of the defined standards (RC1, RC2,
 * RC3A, RC3C, RC4, RC5, RC6 or RC7).
 *
 * If you want to save the RC to OL2385's EROM, use the SF_SaveRcToErom function.
 *
 * SF_ChangeRc automatically sets the config words for the following zones
 * to OL2385's RAM (not to EROM). If you want to use other config words, use
 * also the SF_SetTxConfig function. If you want to save updated config words
 * to OL2385's EROM, use the SF_SaveTxConfigToErom function.
 *
 * RC2: {0x00000001, 0x00000000, 0x00000000} - Default config words for Short
 *      messages.
 * RC4: {0x00000000, 0x40000000, 0x00000000} - Default config words for Short
 *      messages.
 * RC3A, RC3B, RC5: {0x00000003, 0x00001388, 0x00000000}
 * RC1, RC6, RC7: No needed for config words => TX configuration is not modified
 *                by this function.
 *
 * @param drvData Driver run-time data.
 * @param rc      Radio configuration to be configured.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_ChangeRc(sf_drv_data_t *drvData, sf_rc_enum_t rc);

/*!
 * @brief Sets a custom radio configuration.
 *
 * For FH and LBT, it is mandatory to call SF_SetTxConfig after this function.
 *
 * @param drvData Driver run-time data.
 * @param rc      Radio configuration.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SetCustomRc(sf_drv_data_t *drvData, const sf_rc_t *rc);

/*!
 * @brief Saves the current radio configuration (set by SF_ChangeRc or
 * SF_SetCustomRc commands) to non-volatile EROM memory.
 *
 * Configuration in EROM memory is used as the default one after the OL2385
 * power-up or wake-up.
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SaveRcToErom(sf_drv_data_t *drvData);

/*!
 * @brief Reads the current radio configuration.
 *
 * @param drvData Driver run-time data.
 * @param rc      Pointer where the Radio configuration will be stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetRc(sf_drv_data_t *drvData, sf_rc_t *rc);

/*!
 * @brief Updates the TX configuration including specific variables for FH and
 * LBT standards (so-called config words).
 *
 * For FH and LBT, it is mandatory to call this function after the
 * SF_SetCustomRc function to set the config words. Optionally, this command can
 * be called also after the SF_ChangeRc function.
 *
 * Note that this TX configuration is stored in RAM only and is no more
 * available after the device enters the low power mode (SF_Sleep function).
 * You must to send this command after the Wake-up/Power-up again or to save
 * this configuration to EROM using the SF_SaveTxConfigToErom function.
 *
 * @param drvData Driver run-time data.
 * @param cfg     TX configuration.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SetTxConfig(sf_drv_data_t *drvData, sf_tx_config_t *cfg);

/*!
 * @brief Saves the current TX configuration (set by SF_SetTxConfig command) to
 * EROM memory.
 *
 * The configuration in EROM memory is used as the default one after the OL2385
 * power-up and wake-up.
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SaveTxConfigToErom(sf_drv_data_t *drvData);

/*!
 * @brief Reads the current TX configuration.
 *
 * @param drvData Driver run-time data.
 * @param cfg     Pointer where the TX configuration will be stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetTxConfig(sf_drv_data_t *drvData, sf_tx_config_t *cfg);

/*!
 * @brief Sends a standard SIGFOX frame with customer payload (up to 12 bytes)
 * to the SIGFOX network.
 *
 * @param drvData     Driver run-time data.
 * @param sendPayload Pointer to a payload to be sent.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SendFrame(sf_drv_data_t *drvData,
    const sf_msg_payload_t *sendPayload);

/*!
 * @brief Sends a standard SIGFOX frame with the customer payload (up to 12
 * bytes) and triggers a reception of a SIGFOX message.
 *
 * @param drvData     Driver run-time data.
 * @param sendPayload Pointer to a frame payload to be sent.
 * @param recvPayload Pointer where the payload from a received ACK frame will
 *                    be stored. The buffer recvPayload->payload must have
 *                    a sufficient size (see #SF_TRX_FRAME_ACK_PLD_B).
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SendReceiveFrame(sf_drv_data_t *drvData,
    const sf_msg_payload_t *sendPayload, sf_msg_payload_t *recvPayload);

/*!
 * @brief Sends only one bit to the SIGFOX network and (optionally) initiates
 * a downlink.
 *
 * If recvPayload parameter is not NULL, reception of a SIGFOX message is
 * triggered.
 *
 * - In uplink (recvPayload is NULL):
 *     - Send uplink frames (1 or 3; depending on the TX Repeat Number in TX
 *       Configuration - see SF_SetTxConfig function)
 * - In downlink (recvPayload is not NULL):
 *     - Send uplink frames (1 or 3; depending on the TX Repeat Number in TX
 *       Configuration - see SF_SetTxConfig function)
 *     - Receive downlink frame
 *     - Send out of band frame (Voltage, temperature and RSSI)
 *
 * @param drvData     Driver run-time data.
 * @param sentBit     Bit to be sent.
 * @param recvPayload If no downlink should be initiated, keep NULL. Otherwise,
 *                    pointer where the downlink message will be stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SendBit(sf_drv_data_t *drvData, bool sentBit,
    sf_msg_payload_t *recvPayload);

/*!
 * @brief Sends an OOB SIGFOX frame.
 *
 * Data is composed of information about the chip itself (Voltage, Temperature).
 * This function must be called by application every 24 hours maximum (or never
 * if application has some energy critical constraints) with the sfOobService
 * OOB type.
 *
 * If Payload encryption is supported and activated, the user can synchronize
 * the device and the backend with sending sfOobRcSync OOB type.
 *
 * @param drvData Driver run-time data.
 * @param type    Type of the OOB message.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SendOobFrame(sf_drv_data_t *drvData, sf_oob_type_t type);

/*!
 * @brief Reads the version of OL2385 chip and OL2385 Sigfox firmware.
 *
 * @param drvData Driver run-time data.
 * @param hwVer   Array where 8-byte HW version will be stored. Can be NULL.
 * @param swVer   Array where 7-byte SW version will be stored. Can be NULL.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetDeviceVersion(sf_drv_data_t *drvData, uint8_t hwVer[],
    uint8_t swVer[]);

/*!
 * @brief Reads the Sigfox ID.
 *
 * @param drvData Driver run-time data.
 * @param id      Array where 4-byte Sigfox ID will be stored.
 *                Sigfox ID 00112233 is stored as follows:
 *                id[0]=0x00U, id[1]=0x11U, id[2]=0x22U, id[3]=0x33U.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetSigfoxId(sf_drv_data_t *drvData, uint8_t id[]);

/*!
 * @brief Reads the initial device PAC.
 *
 * Note that the device PAC changes during the device registration. After that,
 * the PAC read by this function will be always obsolete.
 *
 * @param drvData Driver run-time data.
 * @param pac     Array where 8-byte initial PAC will be stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetInitialPac(sf_drv_data_t *drvData, uint8_t pac[]);

/*!
 * @brief Reads the Sigfox library version.
 *
 * @param drvData Driver run-time data.
 * @param ver     Array where 11-byte version (ASCII) will be stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetSigfoxLibVersion(sf_drv_data_t *drvData, uint8_t ver[]);

/*!
 * @brief Reads the Sigfox Addon version.
 *
 * @param drvData Driver run-time data.
 * @param ver     Array where 7-byte version (ASCII) will be stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetSigfoxAddonVersion(sf_drv_data_t *drvData, uint8_t ver[]);

/*!
 * @brief Reads info on sent frame depending on the mode you are using:
 * - In DC and FH:
 *      *returnedInfo = 0.
 * - In LBT :
 *      *returnedInfo = bit[7-3]: Carrier Sense attempts and
 *                      bit[2-0]: Number of frames sent
 *
 * @param drvData      Driver run-time data.
 * @param returnedInfo Pointer where the info byte will be stored.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetLbtInfo(sf_drv_data_t *drvData, uint8_t *returnedInfo);

/*!
 * @brief Sets the period for transmission of RC Sync frame.
 *
 * By default, when payload is encrypted, a RC Sync frame is transmitted by the
 * device every 4096 messages transmissions (i.e. when the sequence number loops
 * to 0) to 're-synchronize' the device with the backend (from a payload
 * encryption point of view). This transmission period could be reduced through
 * this command. Then, a RC Sync frame will be transmitted every 'RC Sync
 * period' transmissions of a 'normal' frame. The value 0 corresponds to the
 * default behavior, i.e. a RC Sync frame transmitted every 4096 messages
 * transmissions. As sequence number is on 12 bits, setting a rcSyncPeriod
 * with a value more than (4095 - 1) will return an error.
 *
 * @param drvData      Driver run-time data.
 * @param rcSyncPeriod RC Sync period.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SetRcSyncPeriod(sf_drv_data_t *drvData, uint16_t rcSyncPeriod);

/*!
 * @brief Calibrates the TX/RX frequency.
 *
 * The compensation passed by this function is added to the LO when doing a RF
 * TX or RX. Use a positive value to increase the frequency and negative value
 * to decrease the frequency. The value is stored in non-volatile EROM memory so
 * that you don't need to call this command after each OL2385 wake-up.
 *
 * @param drvData      Driver run-time data.
 * @param compensation Frequency compensation in [Hz].
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_CalibrateFreqStatic(sf_drv_data_t *drvData, int32_t compensation);

/*!
 * @brief Updates the temperature-frequency calibration table.
 *
 * @param drvData Driver run-time data.
 * @param table   Drift table containing signed integer values (see
 *                #SF_TEMP_FREQ_CAL_TBL_SIZE).
 *
 *                table[0] contains signed PPM for -40 Celsius degrees
 *                table[1] contains signed PPM for -35 Celsius degrees
 *                ...
 *                table[25] contains signed PPM for 85 Celsius degrees
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_UpdateTempFreqCalTable(sf_drv_data_t *drvData,
    const int8_t *table);

/*!
 * @brief Reads the temperature-frequency calibration table.
 *
 * @param drvData Driver run-time data.
 * @param table   Pointer where the resulting drift table is stored. The array
 *                must have sufficient size (see #SF_TEMP_FREQ_CAL_TBL_SIZE).
 *
 *                table[0] will contain signed PPM for -40 Celsius degrees
 *                table[1] will contain signed PPM for -35 Celsius degrees
 *                ...
 *                table[25] will contain signed PPM for 85 Celsius degrees.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_ReadTempFreqCalTable(sf_drv_data_t *drvData,
    int8_t *table);

/*!
 * @brief Sets a default temperature-frequency calibration table
 * (no temperature calibration).
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SetDefaultTempFreqCalTable(sf_drv_data_t *drvData);

/*!
 * @brief Calibrates the RSSI measurements.
 *
 * Updates the RSSI calibration value stored in EROM which the
 * RSSI_CORRECTION_OFFSET register is initialized after each wake-up by.
 * You can use the SF_Sleep and SF_WakeUp functions to apply the value.
 *
 * @param drvData Driver run-time data.
 * @param offset  Value of the RSSI_CORRECTION_OFFSET register.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_CalibrateRssi(sf_drv_data_t *drvData, uint16_t offset);

/*!
 * @brief Starts a continuous transmission.
 *
 * After calling this function, you are allowed to call only the
 * SF_StopContTransmission function!
 *
 * @param drvData    Driver run-time data.
 * @param frequency  Frequency of the transmission.
 * @param modulation Modulation type and baudrate.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_StartContTransmission(sf_drv_data_t *drvData, uint32_t frequency,
    sf_modulation_t modulation);

/*!
 * @brief Stops the continuous transmission.
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_StopContTransmission(sf_drv_data_t *drvData);

/*!
 * @brief Sends a command to execute Sigfox test procedures.
 *
 * @param drvData	Driver run-time data.
 * @param rc		Radio configuration.
 * @param testMode	Test mode selection.
 * @param timeout	Timeout for receiving the SPI ACK frame in [us].
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_TestMode(sf_drv_data_t *drvData, sf_rc_enum_t rc,
    sf_test_mode_t testMode, uint32_t timeout);

/*!
 * @brief Switches to the private key which should be used for normal RF
 * transmission.
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SetPrivateKey(sf_drv_data_t *drvData);

/*!
 * @brief Switches to the public key (e.g. for the protocol tests).
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SetPublicKey(sf_drv_data_t *drvData);

/*!
 * @brief Temporary sets the Sigfox certification ID (0xFEDCBA98).
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SetCertificationId(sf_drv_data_t *drvData);

/*!
 * @brief Builds a Sigfox Frame with the customer payload and sends it at
 * a specified frequency.
 *
 * This function can ONLY be used with Certification ID (0xFEDCBA98) otherwise
 * an issue occurs.
 *
 * @param drvData      Driver run-time data.
 * @param frequency    TX frequency.
 * @param initDownlink Initiate downling flag.
 * @param payload      Pointer to a payload to be sent.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_SendTestFrame(sf_drv_data_t *drvData, uint32_t frequency,
    bool initDownlink, const sf_msg_payload_t *payload);

/*!
 * @brief Waits for a valid downlink frame during timeout time and returns the
 * received data.
 *
 * This function can ONLY be used with Certification ID (0xFEDCBA98) otherwise
 * an issue occurs.
 *
 * @param drvData   Driver run-time data.
 * @param frequency RX frequency.
 * @param auth      Use true for AUTHENTICATION_ON and false for
 *                  AUTHENTICATION_OFF.
 * @param buffer    Depends of the Authentication mode:
 *                  - if AUTHENTICATION_OFF: Buffer is used as input to check
 *                  the bit stream of the received frame; Results (OK/FALSE +
 *                  RSSI) can be later read by the SF_GetLast30Rssi function.
 *                  - if AUTHENTICATION_ON: Buffer is used as output to get the
 *                  received payload.
 * @param timeout   Timeout in [s] for the reception of a valid downlink frame.
 * @param rssi      Pointer to memory where RSSI value of the received frame
 *                  will be stored (only valid for AUTHENTICATION_ON).
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_ReceiveTestFrame(sf_drv_data_t *drvData, uint32_t frequency,
    bool auth, uint8_t *buffer, uint8_t timeout, int8_t *rssi);

/*!
 * @brief Returns the recently computed RSSI.
 *
 * @param drvData Driver run-time data.
 * @param rssi    Pointer where the measured RSSI will be stored. Note that
 *                a value of 127 is returned when no computation has been
 *                performed yet.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetLastRssi(sf_drv_data_t *drvData, int8_t *rssi);

/*!
 * @brief Returns the RSSI for last 30 frames reported from RX GFSK test
 * (SF_TestMode function) or SF_ReceiveTestFrame function
 * (w/ AUTHENTICATION_OFF).
 *
 * @param drvData     Driver run-time data.
 * @param rssiBuffer  Pointer where the measured RSSIs will be stored. The array
 *                    must a have sufficient size (see #SF_GET_LAST_RSSI_ACK_30_RSSI_B).
 *                    The value at [0] is the oldest one. If no RSSI computation
 *                    has been carried out, value of 127 is used. If the test
 *                    result was not ok, value of 126 is used at the appropriate
 *                    index.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_GetLast30Rssi(sf_drv_data_t *drvData, int8_t *rssiBuffer);

/*!
 * @brief Resets buffers read by SF_GetLastRssi and SF_GetLast30Rssi functions.
 *
 * @param drvData Driver run-time data.
 *
 * @return Status result of the function (kStatus_Success on success).
 */
sf_status_t SF_ResetRssiBuffer(sf_drv_data_t *drvData);

/*******************************************************************************
 * Platform specific functions
 ******************************************************************************/

/*!
 * @brief User implementation of assert.
 *
 * @param x True if everything is OK.
 */
extern void SF_MCU_Assert(bool x);

/*!
 * @brief Waits for specified amount of milliseconds. This function needs to be
 * implemented for specified MCU by the user.
 *
 * @param delay Number of milliseconds to wait.
 */
extern void SF_MCU_WaitMs(uint16_t delay);

/*!
 * @brief Waits for specified amount of microseconds. This function needs to be
 * implemented for specified MCU by the user.
 *
 * @param delay Number of microseconds to wait.
 */
extern void SF_MCU_WaitUs(uint16_t delay);

/*!
 * @brief Writes logic 0 or 1 to the CS pin. This function needs to be
 * implemented by the user.
 *
 * @param drvInstance Instance of SIGFOX driver.
 * @param value       Zero or one to be set to CS pin.
 */
extern void SF_MCU_WriteCsPin(uint8_t drvInstance, uint8_t value);

/*!
 * @brief Reads logic value of ACK pin. This function needs to be implemented
 * by the user.
 *
 * @param drvInstance Instance of SIGFOX driver.
 *
 * @return Zero value for logic zero, one for the logic one.
 */
extern uint32_t SF_MCU_ReadAckPin(uint8_t drvInstance);

/*!
 * @brief This function performs SPI blocking transfer. It needs to be
 * implemented for specified MCU by the user.
 *
 * @param drvInstance Instance of SIGFOX driver.
 * @param txBuffer    Pointer to data buffer to be sent. It is NULL for ACK SPI
 *                    frames.
 * @param rxBuffer    Pointer to data buffer for received data. It is NULL for
 *                    CMD SPI frames.
 * @param dataSize    Number of bytes to be transmitted.
 *
 * @return sf_status_t Returns sfStatusSuccess on success, sfStatusSpiError
 *         otherwise.
 */
extern sf_status_t SF_MCU_TransferSpi(uint8_t drvInstance, uint8_t txBuffer[],
        uint8_t rxBuffer[], uint16_t dataSize);
/*! @} */

#endif /* SOURCE_SF_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
