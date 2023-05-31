/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file sf_ol2385.h
 *
 * Definitions of SPI frames and commands for OL2385 with FW version:
 * 2.3.0.26.11.20.20.
 */

#ifndef SOURCE_SF_OL2385_H_
#define SOURCE_SF_OL2385_H_

/*******************************************************************************
 * General definitions
 ******************************************************************************/

/*!
 * @name Max. and min. up-link/down-link frequency.
 * @{
 */
/* Min. up-link/down-link frequency in Hz. */
#define SF_FREQ_MIN_HZ           850000000U
/* Max. up-link/down-link frequency in Hz. */
#define SF_FREQ_MAX_HZ           960000000U
/*! @} */

/*! @name Min/max macros */
/* @{ */
#define SF_MIN(a, b) ((a) < (b) ? (a) : (b))
#define SF_MAX(a, b) ((a) > (b) ? (a) : (b))
/* @} */

/*******************************************************************************
 * Definitions of SPI frames
 ******************************************************************************/
/*!
 * @name SPI command frame
 * Definitions common for all CMD frames.
 * @{
 */
/*! Size of the SPI command frame header (including length and command fields)
 * in bytes. */
#define SF_CMD_HEADER_B             2U
/*! Max. payload size in bytes. */
#define SF_CMD_PAYLOAD_MAX_B        SF_TEMP_FREQ_CAL_CMD_UPD_PLD_B
/*! Max. command frame size in bytes. */
#define SF_CMD_SPI_MSG_MAX_B        (SF_CMD_HEADER_B + SF_CMD_PAYLOAD_MAX_B)
/*! Offset of the 'Length' byte in the command frame. */
#define SF_CMD_LENGTH_OF            0U
/*! Offset of the 'Command code' byte in the command frame. */
#define SF_CMD_CMD_OF               1U
/*! Offset of the 'Payload' field in the command frame. */
#define SF_CMD_PAYLOAD_OF           2U
/*! @} */

/*!
 * @name SPI command frame
 * The min. and max. value of the command code.
 * @{
 */
#define SF_CMD_FIRST_ID             SF_CMD_WAKEUP_ID
#define SF_CMD_LAST_ID              SF_CMD_GET_TX_CFG_ID
/*! @} */

/*!
 * @name SPI acknowledgement frame
 * Definitions common for all ACK frames.
 */
/*! @{ */
/*! Size of the SPI ACK frame header (including length and error code fields)
 * in bytes. */
#define SF_ACK_HEADER_B             3U
/*! Max. payload size in bytes (Update_PA_Curve returns the longest payload). */
#define SF_ACK_PAYLOAD_MAX_B        SF_GET_LAST_RSSI_ACK_LAST30_PLD_B
/*! Max. ACK frame size in bytes. */
#define SF_ACK_SPI_MSG_MAX_B        (SF_ACK_HEADER_B + SF_ACK_PAYLOAD_MAX_B)
/*! Offset of the 'Length' byte in the ACK frame. */
#define SF_ACK_LENGTH_OF            0U
/*! Offset of the 'Error code' byte in the ACK frame. */
#define SF_ACK_ERROR_OF             1U
/*! Offset of the 'Manuf. error' byte in the ACK frame. */
#define SF_ACK_MANUF_ERR_OF         2U
/*! Offset of the 'Payload' field in the ACK frame. */
#define SF_ACK_PAYLOAD_OF           3U
/*! @} */

/*! Max. SPI frame size in bytes. */
#define SF_SPI_MSG_MAX_B            (SF_MAX(SF_CMD_SPI_MSG_MAX_B, SF_ACK_SPI_MSG_MAX_B))

/*******************************************************************************
 * Definitions of SPI commands
 ******************************************************************************/
/*!
 * @name Wake_Up SPI command
 *
 * This command wakes-up the device.
 * @{
 */
/*! Wake_Up command code. */
#define SF_CMD_WAKEUP_ID                1U
/*! Size of the Wake_Up command frame payload in bytes. */
#define SF_WAKEUP_CMD_PLD_B             0U
/*! Size of the Wake_Up command frame frame in bytes. */
#define SF_WAKEUP_CMD_B                 (SF_CMD_HEADER_B + SF_WAKEUP_CMD_PLD_B)
/*! Size of the Wake_Up acknowledgement payload in bytes. */
#define SF_WAKEUP_ACK_PLD_B             0U
/*! Size of the Wake_Up acknowledgement in bytes (no ACK). */
#define SF_WAKEUP_ACK_B                 0U
/*! @} */

/*!
 * @name Sleep SPI command
 *
 * This command puts the device into the sleep mode.
 * @{
 */
/*! Sleep command code. */
#define SF_CMD_SLEEP_ID                 3U
/*! Size of the Sleep command frame payload in bytes. */
#define SF_SLEEP_CMD_PLD_B              0U
/*! Size of the Sleep command frame in bytes. */
#define SF_SLEEP_CMD_B                  (SF_CMD_HEADER_B + SF_SLEEP_CMD_PLD_B)
/*! Size of the Sleep acknowledgement payload in bytes. */
#define SF_SLEEP_ACK_PLD_B              0U
/*! Size of the Sleep acknowledgement in bytes (no ACK). */
#define SF_SLEEP_ACK_B                  0U
/*! @} */

/*!
 * @name TX_Frame SPI command
 *
 * This command sends a standard SIGFOX frame with a customer payload
 * (up to 12 bytes).
 * @{
 */
/*! TX_Frame command code. */
#define SF_CMD_TX_FRAME_ID              4U
/*! Max. size of the TX_Frame command frame payload in bytes. */
#define SF_TX_FRAME_CMD_PLD_B_MAX       12U
/*! Max. size of the TX_Frame command frame in bytes. */
#define SF_TX_FRAME_CMD_B_MAX           (SF_CMD_HEADER_B + SF_TX_FRAME_CMD_PLD_B_MAX)
/*! Size of the TX_Frame acknowledgement payload in bytes. */
#define SF_TX_FRAME_ACK_PLD_B           0U
/*! Size of the TX_Frame acknowledgement in bytes. */
#define SF_TX_FRAME_ACK_B               (SF_ACK_HEADER_B + SF_TX_FRAME_ACK_PLD_B)
/*! @} */

/*!
 * @name Bit_Frame SPI command
 *
 * This command sends a standard SIGFOX frame with null customer payload.
 * Data is composed of one boolean only. Downlink can be initiated.
 * @{
 */
/*! Bit_Frame command code. */
#define SF_CMD_BIT_FRAME_ID             5U
/*! Size of the Bit_Frame command frame payload in bytes. */
#define SF_BIT_FRAME_CMD_PLD_B          1U
/*! Size of the Bit_Frame command frame in bytes. */
#define SF_BIT_FRAME_CMD_B              (SF_CMD_HEADER_B + SF_BIT_FRAME_CMD_PLD_B)
/*! Size of the Bit_Frame acknowledgement payload in bytes (when no downlink initiated). */
#define SF_BIT_FRAME_ACK_PLD_NO_DOWN_B  0U
/*! Size of the Bit_Frame acknowledgement payload in bytes (when downlink initiated). */
#define SF_BIT_FRAME_ACK_PLD_DOWN_B     8U
/*! Size of the Bit_Frame acknowledgement in bytes (when no downlink initiated). */
#define SF_BIT_FRAME_ACK_NO_DOWN_B      (SF_ACK_HEADER_B + SF_BIT_FRAME_ACK_PLD_NO_DOWN_B)
/*! Size of the Bit_Frame acknowledgement in bytes (when downlink initiated). */
#define SF_BIT_FRAME_ACK_DOWN_B         (SF_ACK_HEADER_B + SF_BIT_FRAME_ACK_PLD_DOWN_B)

/*! Offset of the 'Config' byte in the CMD frame. */
#define SF_BIT_FRAME_CMD_CONFIG_OF      (SF_CMD_PAYLOAD_OF + 0U)
/*! Mask of the 'Sent Bit' field in 'Config' byte. */
#define SF_BIT_FRAME_CMD_CONFIG_BIT_MASK       0x01U
/*! Mask of the 'Initiate Downlink flag' field in 'Config' byte. */
#define SF_BIT_FRAME_CMD_CONFIG_DOWNLINK_MASK  0x02U
/*! @} */

/*!
 * @name OOB_TX_Frame SPI command
 *
 * Sends an out of band SIGFOX frame.
 * @{
 */
/*! OOB_TX_Frame command code. */
#define SF_CMD_OOB_TX_FRAME_ID          6U
/*! Size of the OOB_TX_Frame command frame payload in bytes. */
#define SF_OOB_TX_FRAME_CMD_PLD_B       1U
/*! Size of the OOB_TX_Frame command frame in bytes. */
#define SF_OOB_TX_FRAME_CMD_B           (SF_CMD_HEADER_B + SF_OOB_TX_FRAME_CMD_PLD_B)
/*! Size of the OOB_TX_Frame acknowledgement payload in bytes. */
#define SF_OOB_TX_FRAME_ACK_PLD_B       0U
/*! Size of the OOB_TX_Frame acknowledgement in bytes. */
#define SF_OOB_TX_FRAME_ACK_B           (SF_ACK_HEADER_B + SF_OOB_TX_FRAME_ACK_PLD_B)

/*! Offset of the 'OOB Type' byte in the CMD frame. */
#define SF_OOB_TX_FRAME_CMD_TYPE_OF     (SF_CMD_PAYLOAD_OF + 0U)
/*! @} */

/*!
 * @name TRX_Frame SPI command
 *
 * This command sends a standard SIGFOX frame with a customer payload
 * (up to 12 bytes) and initiates a downlink.
 * @{
 */
/*! TRX_Frame command code. */
#define SF_CMD_TRX_FRAME_ID             7U
/*! Max. size of the TRX_Frame command frame payload in bytes. */
#define SF_TRX_FRAME_CMD_PLD_B_MAX      12U
/*! Max. size of the TRX_Frame command frame in bytes. */
#define SF_TRX_FRAME_CMD_B_MAX          (SF_CMD_HEADER_B + SF_TRX_FRAME_CMD_PLD_B_MAX)
/*! Size of the TRX_Frame acknowledgement payload in bytes. */
#define SF_TRX_FRAME_ACK_PLD_B          8U
/*! Size of the TRX_Frame acknowledgement in bytes. */
#define SF_TRX_FRAME_ACK_B              (SF_ACK_HEADER_B + SF_TRX_FRAME_ACK_PLD_B)
/*! @} */

/*!
 * @name Start_Continous_Transmission SPI command
 *
 * This command starts a continuous transmission with the passed frequency.
 * @{
 */
/*! Start_Continous_Transmission command code. */
#define SF_CMD_START_CONT_ID            11U
/*! Size of the Start_Continous_Transmission command frame payload in bytes. */
#define SF_START_CONT_CMD_PLD_B         5U
/*! Size of the Start_Continous_Transmission command frame in bytes. */
#define SF_START_CONT_CMD_B             (SF_CMD_HEADER_B + SF_START_CONT_CMD_PLD_B)
/*! Size of the Start_Continous_Transmission acknowledgement payload in bytes. */
#define SF_START_CONT_ACK_PLD_B         0U
/*! Size of the Start_Continous_Transmission acknowledgement in bytes. */
#define SF_START_CONT_ACK_B             0U

/*! Offset of the 'Modulation' byte in the CMD frame. */
#define SF_START_CONT_CMD_MOD_OF        (SF_CMD_PAYLOAD_OF + 0U)
/*! Offset of the 'Frequency' word in the CMD frame. */
#define SF_START_CONT_CMD_FREQ_OF       (SF_CMD_PAYLOAD_OF + 1U)
/*! @} */

/*!
 * @name Stop_Continuous_Transmission SPI command
 *
 * This command stops the continuous transmission.
 * @{
 */
/*! Stop_Continous_Transmission command code. */
#define SF_CMD_STOP_CONT_ID             51U
/*! Size of the Stop_Continous_Transmission command frame payload in bytes. */
#define SF_STOP_CONT_CMD_PLD_B          0U
/*! Size of the Stop_Continous_Transmission command frame in bytes. */
#define SF_STOP_CONT_CMD_B              (SF_CMD_HEADER_B + SF_STOP_CONT_CMD_PLD_B)
/*! Size of the Stop_Continous_Transmission acknowledgement payload in bytes. */
#define SF_STOP_CONT_ACK_PLD_B          0U
/*! Size of the Stop_Continous_Transmission acknowledgement in bytes. */
#define SF_STOP_CONT_ACK_B              (SF_ACK_HEADER_B + SF_STOP_CONT_ACK_PLD_B)
/*! @} */

/*!
 * @name Get_Device_Version SPI command
 *
 * This command gets the version of the OL2385 chip and software version of
 * Sigfox firmware inside OL2385.
 * @{
 */
/*! Get_Device_Version command code. */
#define SF_CMD_GET_DEV_VER_ID           13U
/*! Size of the Get_Device_Version command frame payload in bytes. */
#define SF_GET_DEV_VER_CMD_PLD_B        0U
/*! Size of the Get_Device_Version command frame in bytes. */
#define SF_GET_DEV_VER_CMD_B            (SF_CMD_HEADER_B + SF_GET_DEV_VER_CMD_PLD_B)
/*! Size of the Get_Device_Version acknowledgement payload in bytes. */
#define SF_GET_DEV_VER_ACK_PLD_B        15U
/*! Size of the Get_Device_Version acknowledgement in bytes. */
#define SF_GET_DEV_VER_ACK_B            (SF_ACK_HEADER_B + SF_GET_DEV_VER_ACK_PLD_B)

/*! Size of the 'Hardware version' field in the ACK frame. */
#define SF_GET_DEV_VER_ACK_HW_B         8U
/*! Offset of the 'Hardware version' field in the ACK frame. */
#define SF_GET_DEV_VER_ACK_HW_OF        (SF_ACK_PAYLOAD_OF + 0U)
/*! Size of the 'Software version' field in the ACK frame. */
#define SF_GET_DEV_VER_ACK_SW_B         7U
/*! Offset of the 'Software version' field in the ACK frame. */
#define SF_GET_DEV_VER_ACK_SW_OF        (SF_ACK_PAYLOAD_OF + 8U)
/*! @} */

/*!
 * @name Test_Mode SPI command
 *
 * This command executes the test modes needed for the Sigfox RF and Protocol
 * Tests.
 * @{
 */
/*! Test_Mode command code. */
#define SF_CMD_TEST_MODE_ID             20U
/*! Size of the Test_Mode command frame payload in bytes. */
#define SF_TEST_MODE_CMD_PLD_B          2U
/*! Size of the Test_Mode command frame in bytes. */
#define SF_TEST_MODE_CMD_B              (SF_CMD_HEADER_B + SF_TEST_MODE_CMD_PLD_B)
/*! Size of the Test_Mode acknowledgement payload in bytes. */
#define SF_TEST_MODE_ACK_PLD_B          0U
/*! Size of the Test_Mode acknowledgement in bytes. */
#define SF_TEST_MODE_ACK_B              (SF_ACK_HEADER_B + SF_TEST_MODE_ACK_PLD_B)

/*! Offset of the RC byte in the CMD frame. */
#define SF_TEST_MODE_CMD_RC_OF          (SF_CMD_PAYLOAD_OF + 0U)
/*! Offset of the 'Test Mode' byte in the CMD frame. */
#define SF_TEST_MODE_CMD_MODE_OF        (SF_CMD_PAYLOAD_OF + 1U)
/*! @} */

/*!
 * @name Switch_To_Private_Key SPI command
 *
 * This command switches to the private key which is used for the normal
 * RF transmission.
 * @{
 */
/*! Switch_To_Private_Key command code. */
#define SF_CMD_SWITCH_PRIVATE_ID        25U
/*! Size of the Switch_To_Private_Key command frame payload in bytes. */
#define SF_SWITCH_PRIVATE_CMD_PLD_B     0U
/*! Size of the Switch_To_Private_Key command frame in bytes. */
#define SF_SWITCH_PRIVATE_CMD_B         (SF_CMD_HEADER_B + SF_SWITCH_PRIVATE_CMD_PLD_B)
/*! Size of the Switch_To_Private_Key acknowledgement payload in bytes. */
#define SF_SWITCH_PRIVATE_ACK_PLD_B     0U
/*! Size of the Switch_To_Private_Key acknowledgement in bytes. */
#define SF_SWITCH_PRIVATE_ACK_B         (SF_ACK_HEADER_B + SF_SWITCH_PRIVATE_ACK_PLD_B)
/*! @} */

/*!
 * @name Switch_To_Public_Key SPI command
 *
 * This command switches to the public key (e.g. for the protocol tests).
 * @{
 */
/*! Switch_To_Public_Key command code. */
#define SF_CMD_SWITCH_PUBLIC_ID         26U
/*! Size of the Switch_To_Public_Key command frame payload in bytes. */
#define SF_SWITCH_PUBLIC_CMD_PLD_B      0U
/*! Size of the Switch_To_Public_Key command frame in bytes. */
#define SF_SWITCH_PUBLIC_CMD_B          (SF_CMD_HEADER_B + SF_SWITCH_PUBLIC_CMD_PLD_B)
/*! Size of the Switch_To_Public_Key acknowledgement payload in bytes. */
#define SF_SWITCH_PUBLIC_ACK_PLD_B      0U
/*! Size of the Switch_To_Public_Key acknowledgement in bytes. */
#define SF_SWITCH_PUBLIC_ACK_B          (SF_ACK_HEADER_B + SF_SWITCH_PUBLIC_ACK_PLD_B)
/*! @} */

/*!
 * @name Get_LBT_Info SPI command
 *
 * Gets the FCC Macro channel usage
 * @{
 */
/*! Get_LBT_Info command code. */
#define SF_CMD_GET_LBT_INFO_ID          32U
/*! Size of the Get_LBT_Info command frame payload in bytes. */
#define SF_GET_LBT_INFO_CMD_PLD_B       0U
/*! Size of the Get_LBT_Info command frame in bytes. */
#define SF_GET_LBT_INFO_CMD_B           (SF_CMD_HEADER_B + SF_GET_LBT_INFO_CMD_PLD_B)
/*! Size of the Get_LBT_Info acknowledgement payload in bytes. */
#define SF_GET_LBT_INFO_ACK_PLD_B       1U
/*! Size of the Get_LBT_Info acknowledgement in bytes. */
#define SF_GET_LBT_INFO_ACK_B           (SF_ACK_HEADER_B + SF_GET_LBT_INFO_ACK_PLD_B)

/*! Offset of the 'Returned_info' byte in the ACK frame. */
#define SF_GET_LBT_INFO_ACK_INFO_OF     (SF_ACK_PAYLOAD_OF + 0U)

/*! Mask of the 'Number of sent frames' field in 'Returned_info' byte. */
#define SF_GET_LBT_INFO_ACK_INFO_SENT_FRAMES_MASK  0x07U
/*! Shift of the 'Number of sent frames' field in 'Returned_info' byte. */
#define SF_GET_LBT_INFO_ACK_INFO_SENT_FRAMES_SHIFT 0U
/*! Mask of the 'Carrier sense attempts' field in 'Returned_info' byte. */
#define SF_GET_LBT_INFO_ACK_INFO_CS_ATTEPTS_MASK   0xF8U
/*! Shift of the 'Carrier sense attempts' field in 'Returned_info' byte. */
#define SF_GET_LBT_INFO_ACK_INFO_CS_ATTEPTS_SHIFT  3U
/*! @} */

/*!
 * @name Static_Frequency_Calibration SPI command
 *
 * The compensation passed by this command is added to the LO when doing
 * a RF TX or RX.
 * @{
 */
/*! Static_Frequency_Calibration command code. */
#define SF_CMD_STATIC_FREQ_CAL_ID       34U
/*! Size of the "Static frequency calibration" command frame payload in bytes. */
#define SF_STATIC_FREQ_CAL_CMD_PLD_B    4U
/*! Size of the "Static frequency calibration" command frame in bytes. */
#define SF_STATIC_FREQ_CAL_CMD_B        (SF_CMD_HEADER_B + SF_STATIC_FREQ_CAL_CMD_PLD_B)
/*! Size of the "Static frequency calibration" acknowledgement payload in bytes. */
#define SF_STATIC_FREQ_CAL_ACK_PLD_B    0U
/*! Size of the "Static frequency calibration" acknowledgement in bytes. */
#define SF_STATIC_FREQ_CAL_ACK_B        (SF_ACK_HEADER_B + SF_STATIC_FREQ_CAL_ACK_PLD_B)

/*! Offset of the 'Compensation (Hz)' word in the CMD frame. */
#define SF_STATIC_FREQ_CAL_CMD_COMP_OF  (SF_CMD_PAYLOAD_OF + 0U)
/*! @} */

/*!
 * @name Temperature_Frequency_Calibration SPI command
 *
 *  This command will execute according to subcommand value specified in 3rd byte:
 *   - 0u: Reading the current temperature frequency table (crystal dependent
 *         drift table)
 *   - 1u: Updating the table
 *   - 2u: Setting the default table (no temperature compensation)
 * @{
 */
/*! Temperature_Frequency_Calibration command code. */
#define SF_CMD_TEMP_FREQ_CAL_ID         35U
/*! Size of the Temperature_Frequency_Calibration command frame payload in bytes (Update). */
#define SF_TEMP_FREQ_CAL_CMD_UPD_PLD_B  27U
/*! Size of the Temperature_Frequency_Calibration command frame payload in bytes (Read). */
#define SF_TEMP_FREQ_CAL_CMD_READ_PLD_B 1U
/*! Size of the Temperature_Frequency_Calibration command frame payload in bytes (Set default). */
#define SF_TEMP_FREQ_CAL_CMD_DEF_PLD_B  1U
/*! Size of the Temperature_Frequency_Calibration command frame in bytes (Update). */
#define SF_TEMP_FREQ_CAL_CMD_UPD_B      (SF_CMD_HEADER_B + SF_TEMP_FREQ_CAL_CMD_UPD_PLD_B)
/*! Size of the Temperature_Frequency_Calibration command frame in bytes (Read). */
#define SF_TEMP_FREQ_CAL_CMD_READ_B     (SF_CMD_HEADER_B + SF_TEMP_FREQ_CAL_CMD_READ_PLD_B)
/*! Size of the Temperature_Frequency_Calibration command frame in bytes (Set default). */
#define SF_TEMP_FREQ_CAL_CMD_DEF_B      (SF_CMD_HEADER_B + SF_TEMP_FREQ_CAL_CMD_DEF_PLD_B)
/*! Size of the Temperature_Frequency_Calibration acknowledgement payload in bytes (Update). */
#define SF_TEMP_FREQ_CAL_ACK_UPD_PLD_B  0U
/*! Size of the Temperature_Frequency_Calibration acknowledgement payload in bytes (Read). */
#define SF_TEMP_FREQ_CAL_ACK_READ_PLD_B 26U
/*! Size of the Temperature_Frequency_Calibration acknowledgement payload in bytes (Set default). */
#define SF_TEMP_FREQ_CAL_ACK_DEF_PLD_B  0U
/*! Size of the Temperature_Frequency_Calibration acknowledgement in bytes (Update). */
#define SF_TEMP_FREQ_CAL_ACK_UPD_B      (SF_ACK_HEADER_B + SF_TEMP_FREQ_CAL_ACK_UPD_PLD_B)
/*! Size of the Temperature_Frequency_Calibration acknowledgement in bytes (Read). */
#define SF_TEMP_FREQ_CAL_ACK_READ_B     (SF_ACK_HEADER_B + SF_TEMP_FREQ_CAL_ACK_READ_PLD_B)
/*! Size of the Temperature_Frequency_Calibration acknowledgement in bytes (Set default). */
#define SF_TEMP_FREQ_CAL_ACK_DEF_B      (SF_ACK_HEADER_B + SF_TEMP_FREQ_CAL_ACK_DEF_PLD_B)

/*! Offset of the 'Subcommand' byte in the CMD frame. */
#define SF_TEMP_FREQ_CAL_CMD_SUBCMD_OF  (SF_CMD_PAYLOAD_OF + 0U)
/*! Offset of the 'Temperature calibration table' field in the CMD frame
 * (if subcommand is 1u). */
#define SF_TEMP_FREQ_CAL_CMD_TEMP_OF    (SF_CMD_PAYLOAD_OF + 1U)

/*! Offset of the 'Temperature calibration table' field in the ACK frame
 * (if subcommand was 0u). */
#define SF_TEMP_FREQ_CAL_ACK_TEMP_OF    (SF_ACK_PAYLOAD_OF + 0U)

/*! Read table sub-command. */
#define SF_TEMP_FREQ_CAL_CMD_SUBCMD_READ          0U
/*! Update table sub-command. */
#define SF_TEMP_FREQ_CAL_CMD_SUBCMD_UPDATE        1U
/*! Set default table sub-command. */
#define SF_TEMP_FREQ_CAL_CMD_SUBCMD_SET_DEFAULT   2U

/*! Size of the temperature calibration table in bytes. */
#define SF_TEMP_FREQ_CAL_TBL_SIZE                 26U
/*! @} */

/*!
 * @name Get_Last_RSSI SPI command
 *
 *  This command will execute according to subcommand value specified in 3rd byte:
 *   - 0u: Returns the recently computed RSSI measured by OL2385. If no RSSI
 *         computation has been carried out, it delivers 127 (0x7F).
 *   - 1u: Returns the RSSI for last 30 frames reported from RX GFSK test
 *         (Test_Mode command) or Receive_Test_Frame command
 *         (w/ AUTHENTICATION_OFF). RSSI value at index [3] is the oldest one,
 *         value at [32] the latest one. If no RSSI computation has been carried
 *         out, value 127 is used. If the test status was not ok, value 126 is
 *         returned at the appropriate index.
 *   - 2u: Resets the internal variable used for subcommand 0u and buffer used
 *         for subcommand 1u.
 * @{
 */
/*! Get_Last_RSSI command code. */
#define SF_CMD_GET_LAST_RSSI_ID           36U
/*! Size of the Get_Last_RSSI command frame payload in bytes. */
#define SF_GET_LAST_RSSI_CMD_PLD_B        1U
/*! Size of the Get_Last_RSSI command frame in bytes. */
#define SF_GET_LAST_RSSI_CMD_B            (SF_CMD_HEADER_B + SF_GET_LAST_RSSI_CMD_PLD_B)
/*! Size of the Get_Last_RSSI acknowledgement payload in bytes (Last RSSI). */
#define SF_GET_LAST_RSSI_ACK_LAST_PLD_B   1U
/*! Size of the Get_Last_RSSI acknowledgement payload in bytes (Last 30). */
#define SF_GET_LAST_RSSI_ACK_LAST30_PLD_B 30U
/*! Size of the Get_Last_RSSI acknowledgement payload in bytes (Reset). */
#define SF_GET_LAST_RSSI_ACK_RESET_PLD_B  0U
/*! Size of the Get_Last_RSSI acknowledgement in bytes (Last RSSI). */
#define SF_GET_LAST_RSSI_ACK_LAST_B       (SF_ACK_HEADER_B + SF_GET_LAST_RSSI_ACK_LAST_PLD_B)
/*! Size of the Get_Last_RSSI acknowledgement in bytes (Last 30). */
#define SF_GET_LAST_RSSI_ACK_LAST30_B     (SF_ACK_HEADER_B + SF_GET_LAST_RSSI_ACK_LAST30_PLD_B)
/*! Size of the Get_Last_RSSI acknowledgement in bytes (Reset). */
#define SF_GET_LAST_RSSI_ACK_RESET_B      (SF_ACK_HEADER_B + SF_GET_LAST_RSSI_ACK_RESET_PLD_B)

/*! Offset of the sub-command in the CMD frame. */
#define SF_GET_LAST_RSSI_CMD_SUBCMD_OF  (SF_CMD_PAYLOAD_OF + 0U)

/*! Delivers the recently computed RSSI measured by OL2385. */
#define SF_GET_LAST_RSSI_CMD_SUBCMD_LAST_RSSI       0U
/*! Returns the RSSI for last 30 frames reported from RX GFSK test (SF_TestMode function)
 * or SF_ReceiveTestFrame function (w/ AUTHENTICATION_OFF). */
#define SF_GET_LAST_RSSI_CMD_SUBCMD_LAST_30_RSSI    1U
/*! Resets the internal variable used for previous subcommands. */
#define SF_GET_LAST_RSSI_CMD_SUBCMD_RESET           2U

/*! Offset of the 'RSSI' signed byte in the ACK frame (if subcommand was 0u). */
#define SF_GET_LAST_RSSI_ACK_RSSI_OF          (SF_ACK_PAYLOAD_OF + 0U)
/*! Offset of the 'RSSI' array of signed bytes in the ACK frame
 * (if subcommand was 1u). */
#define SF_GET_LAST_RSSI_ACK_30_RSSI_OF       (SF_ACK_PAYLOAD_OF + 0U)
/*! Size of the 'RSSI' array of signed bytes in the ACK frame
 * (if subcommand was 1u). */
#define SF_GET_LAST_RSSI_ACK_30_RSSI_B        30U
/*! @} */

/*!
 * @name RSSI_Calibration SPI command
 *
 * This command updates the RSSI calibration value stored in EROM, which the
 * OL2385's RSSI_CORRECTION_OFFSET register is initialized after each wake-up by.
 * @{
 */
/*! RSSI_Calibration command code. */
#define SF_CMD_RSSI_CALIB_ID            38U
/*! Size of the RSSI_Calibration  command frame payload in bytes. */
#define SF_RSSI_CALIB_CMD_PLD_B         2U
/*! Size of the RSSI_Calibration  command frame in bytes. */
#define SF_RSSI_CALIB_CMD_B             (SF_CMD_HEADER_B + SF_RSSI_CALIB_CMD_PLD_B)
/*! Size of the RSSI_Calibration  acknowledgement payload in bytes. */
#define SF_RSSI_CALIB_ACK_PLD_B         0U
/*! Size of the RSSI_Calibration  acknowledgement in bytes. */
#define SF_RSSI_CALIB_ACK_B             (SF_ACK_HEADER_B + SF_RSSI_CALIB_ACK_PLD_B)

/*! Offset of the LSB byte of 'RSSI calibration value' word in the CMD frame. */
#define SF_RSSI_CALIB_CMD_RSSI_LSB_OF   (SF_CMD_PAYLOAD_OF + 0U)
/*! Offset of the MSB byte of 'RSSI calibration value' word in the CMD frame. */
#define SF_RSSI_CALIB_CMD_RSSI_MSB_OF   (SF_CMD_PAYLOAD_OF + 1U)
/*! @} */

/*!
 * @name Get_Sigfox_ID SPI command
 *
 * This command gets the Sigfox ID.
 * @{
 */
/*! Get_Sigfox_ID command code. */
#define SF_CMD_GET_ID_ID                45U
/*! Size of the Get_Sigfox_ID command frame payload in bytes. */
#define SF_GET_ID_CMD_PLD_B             0U
/*! Size of the Get_Sigfox_ID command frame in bytes. */
#define SF_GET_ID_CMD_B                 (SF_CMD_HEADER_B + SF_GET_ID_CMD_PLD_B)
/*! Size of the Get_Sigfox_ID acknowledgement payload in bytes. */
#define SF_GET_ID_ACK_PLD_B             4U
/*! Size of the Get_Sigfox_ID acknowledgement in bytes. */
#define SF_GET_ID_ACK_B                 (SF_ACK_HEADER_B + SF_GET_ID_ACK_PLD_B)


/*! Offset of the 'Sigfox ID' field in the ACK frame. */
#define SF_GET_ID_ACK_ID_OF             (SF_ACK_PAYLOAD_OF + 0U)
/*! Size of the 'Sigfox ID' field in the ACK frame. */
#define SF_GET_ID_ACK_ID_B              4U
/*! @} */

/*!
 * @name Get_Initial_PAC SPI command
 *
 * This command gets the initial PAC of the device.
 * @{
 */
/*! Get Get_Initial_PAC command code.  */
#define SF_CMD_GET_INIT_PAC_ID          46U
/*! Size of the Get_Initial_PAC command frame payload in bytes. */
#define SF_GET_INIT_PAC_CMD_PLD_B       0U
/*! Size of the Get_Initial_PAC command frame in bytes. */
#define SF_GET_INIT_PAC_CMD_B           (SF_CMD_HEADER_B + SF_GET_INIT_PAC_CMD_PLD_B)
/*! Size of the Get_Initial_PAC acknowledgement payload in bytes. */
#define SF_GET_INIT_PAC_ACK_PLD_B       8U
/*! Size of the Get_Initial_PAC acknowledgement in bytes. */
#define SF_GET_INIT_PAC_ACK_B           (SF_ACK_HEADER_B + SF_GET_INIT_PAC_ACK_PLD_B)

/*! Offset of the 'Initial PAC' field in the ACK frame. */
#define SF_GET_INIT_PAC_ACK_PAC_OF      (SF_ACK_PAYLOAD_OF + 0U)
/*! Size of the 'Initial PAC' field in the ACK frame. */
#define SF_GET_INIT_PAC_ACK_PAC_B       8U
/*! @} */

/*!
 * @name Get_Sigfox_Lib_Version SPI command
 *
 * This command gets Sigfox library version.
 * @{
 */
/*! Get_Sigfox_Lib_Version command code.  */
#define SF_CMD_GET_LIB_VER_ID           47U
/*! Size of the Get_Sigfox_Lib_Version command frame payload in bytes. */
#define SF_GET_LIB_VER_CMD_PLD_B        0U
/*! Size of the Get_Sigfox_Lib_Version command frame in bytes. */
#define SF_GET_LIB_VER_CMD_B            (SF_CMD_HEADER_B + SF_GET_LIB_VER_CMD_PLD_B)
/*! Size of the Get_Sigfox_Lib_Version acknowledgement payload in bytes. */
#define SF_GET_LIB_VER_ACK_PLD_B        11U
/*! Size of the Get_Sigfox_Lib_Version acknowledgement in bytes. */
#define SF_GET_LIB_VER_ACK_B            (SF_ACK_HEADER_B + SF_GET_LIB_VER_ACK_PLD_B)

/*! Offset of the Sigfox library version field in the ACK frame. */
#define SF_GET_LIB_VER_ACK_VER_OF       (SF_ACK_PAYLOAD_OF + 0U)
/*! Size of the Sigfox library version in the ACK frame. */
#define SF_GET_LIB_VER_ACK_VER_B        11U
/*! @} */

/*!
 * @name Set_RC_Sync_Period SPI command
 *
 * This command sets the RC sync period.
 * @{
 */
/*! Set_RC_Sync_Period command code.  */
#define SF_CMD_SET_RC_SYNC_PER_ID       48U
/*! Size of the Set_RC_Sync_Period command frame payload in bytes. */
#define SF_SET_RC_SYNC_PER_CMD_PLD_B    2U
/*! Size of the Set_RC_Sync_Period command frame in bytes. */
#define SF_SET_RC_SYNC_PER_CMD_B        (SF_CMD_HEADER_B + SF_SET_RC_SYNC_PER_CMD_PLD_B)
/*! Size of the Set_RC_Sync_Period acknowledgement payload in bytes. */
#define SF_SET_RC_SYNC_PER_ACK_PLD_B    0U
/*! Size of the Set_RC_Sync_Period acknowledgement in bytes. */
#define SF_SET_RC_SYNC_PER_ACK_B        (SF_ACK_HEADER_B + SF_SET_RC_SYNC_PER_ACK_PLD_B)

/*! Offset of the LSB byte of 'Period' word in the CMD frame. */
#define SF_SET_RC_SYNC_PER_CMD_LSB_OF   (SF_CMD_PAYLOAD_OF + 0U)
/*! Offset of the MSB byte of 'Period' word in the CMD frame. */
#define SF_SET_RC_SYNC_PER_CMD_MSB_OF   (SF_CMD_PAYLOAD_OF + 1U)
/*! @} */

/*!
 * @name Get_RC SPI command
 *
 * This command gets the current radio configuration (parameters which were used
 * for last opening of the Sigfox library).
 * @{
 */
/*! Get_RC command code.  */
#define SF_CMD_GET_RC_ID                39U
/*! Size of the Get_RC command frame payload in bytes. */
#define SF_GET_RC_CMD_PLD_B             0U
/*! Size of the Get_RC command frame in bytes. */
#define SF_GET_RC_CMD_B                 (SF_CMD_HEADER_B + SF_GET_RC_CMD_PLD_B)
/*! Size of the Get_RC acknowledgement payload in bytes. */
#define SF_GET_RC_ACK_PLD_B             23U
/*! Size of the Get_RC acknowledgement in bytes. */
#define SF_GET_RC_ACK_B                 (SF_ACK_HEADER_B + SF_GET_RC_ACK_PLD_B)

/*! Offset of the 'Spectrum_access byte in the ACK frame. */
#define SF_GET_RC_ACK_SA_OF             (SF_ACK_PAYLOAD_OF + 0U)
/*! Offset of the 'Modulation' byte in the ACK frame. */
#define SF_GET_RC_ACK_MOD_OF            (SF_ACK_PAYLOAD_OF + 1U)
/*! Offset of the 'Open_tx_frequency' word in the ACK frame. */
#define SF_GET_RC_ACK_TX_FREQ_OF        (SF_ACK_PAYLOAD_OF + 2U)
/*! Offset of the 'Open_rx_frequency' word in the ACK frame. */
#define SF_GET_RC_ACK_RX_FREQ_OF        (SF_ACK_PAYLOAD_OF + 6U)
/*! Offset of the 'Macro_channel_width¨' word in the ACK frame. */
#define SF_GET_RC_ACK_MCW_OF            (SF_ACK_PAYLOAD_OF + 10U)
/*! Offset of the 'Open_cs_frequency word in the ACK frame. */
#define SF_GET_RC_ACK_OCS_FREQ_OF       (SF_ACK_PAYLOAD_OF + 14U)
/*! Offset of the 'Open_cs_bandwidth' word in the ACK frame. */
#define SF_GET_RC_ACK_OCS_BW_OF         (SF_ACK_PAYLOAD_OF + 18U)
/*! Offset of the 'Cs_threshold' signed byte in the ACK frame. */
#define SF_GET_RC_ACK_CS_THRES_OF       (SF_ACK_PAYLOAD_OF + 22U)
/*! @} */

/*!
 * @name Set_Custom_RC SPI command
 *
 * The Change_RC command opens the Sigfox library with parameters dedicated to
 * the individual zones (RC1 - RC7). This command allows to open the Sigfox
 * library with parameters passed by this command.
 * @{
 */
/*! Set_Custom_RC command code.  */
#define SF_CMD_SET_CUSTOM_RC_ID         40U
/*! Size of the Set_Custom_RC command frame payload in bytes. */
#define SF_SET_CUSTOM_RC_CMD_PLD_B      23U
/*! Size of the Set_Custom_RC command frame in bytes. */
#define SF_SET_CUSTOM_RC_CMD_B          (SF_CMD_HEADER_B + SF_SET_CUSTOM_RC_CMD_PLD_B)
/*! Size of the Set_Custom_RC acknowledgement payload in bytes. */
#define SF_SET_CUSTOM_RC_ACK_PLD_B      0U
/*! Size of the Set_Custom_RC acknowledgement in bytes. */
#define SF_SET_CUSTOM_RC_ACK_B          (SF_ACK_HEADER_B + SF_SET_CUSTOM_RC_ACK_PLD_B)

/*! Offset of the 'Spectrum_access' byte in the CMD frame. */
#define SF_SET_CUSTOM_RC_CMD_SA_OF         (SF_CMD_PAYLOAD_OF + 0U)
/*! Offset of the 'Modulation' byte in the CMD frame. */
#define SF_SET_CUSTOM_RC_CMD_MOD_OF        (SF_CMD_PAYLOAD_OF + 1U)
/*! Offset of the 'Open_tx_frequency' word in the CMD frame. */
#define SF_SET_CUSTOM_RC_CMD_TX_FREQ_OF    (SF_CMD_PAYLOAD_OF + 2U)
/*! Offset of the 'Open_rx_frequency' word in the CMD frame. */
#define SF_SET_CUSTOM_RC_CMD_RX_FREQ_OF    (SF_CMD_PAYLOAD_OF + 6U)
/*! Offset of the 'Macro_channel_width' word in the CMD frame. */
#define SF_SET_CUSTOM_RC_CMD_MCW_OF        (SF_CMD_PAYLOAD_OF + 10U)
/*! Offset of the 'Open_cs_frequency' word in the CMD frame. */
#define SF_SET_CUSTOM_RC_CMD_OCS_FREQ_OF   (SF_CMD_PAYLOAD_OF + 14U)
/*! Offset of the 'Open_cs_bandwidth' word in the CMD frame. */
#define SF_SET_CUSTOM_RC_CMD_OCS_BW_OF     (SF_CMD_PAYLOAD_OF + 18U)
/*! Offset of the 'Cs_threshold' signed byte in the CMD frame. */
#define SF_SET_CUSTOM_RC_CMD_CS_THRES_OF   (SF_CMD_PAYLOAD_OF + 22U)
/*! @} */

/*!
 * @name Send_Test_Frame SPI command
 *
 * This command builds a Sigfox Frame with the customer payload and sends it at
 * a specified frequency. Use this command ONLY with Certification ID
 * (0xFEDCBA98) otherwise an issue occurs.
 * @{
 */
/*! Send_Test_Frame command code. */
#define SF_CMD_SEND_TEST_FR_ID          49U
/*! Max. size of the Send_Test_Frame command frame payload in bytes. */
#define SF_SEND_TEST_FR_CMD_PLD_B_MAX   17U
/*! Max. size of the Send_Test_Frame command frame in bytes. */
#define SF_SEND_TEST_FR_CMD_B_MAX       (SF_CMD_HEADER_B + SF_SEND_TEST_FR_CMD_PLD_B_MAX)
/*! Size of the Send_Test_Frame acknowledgement payload in bytes. */
#define SF_SEND_TEST_FR_ACK_PLD_B       0U
/*! Size of the Send_Test_Frame acknowledgement in bytes. */
#define SF_SEND_TEST_FR_ACK_B           (SF_ACK_HEADER_B + SF_SEND_TEST_FR_ACK_PLD_B)

/*! Offset of the 'Frequency' word in the CMD frame. */
#define SF_SEND_TEST_FR_CMD_FREQ_OF    (SF_CMD_PAYLOAD_OF + 0U)
/*! Offset of the 'Init Downlink' byte in the CMD frame. */
#define SF_SEND_TEST_FR_CMD_DOWN_OF    (SF_CMD_PAYLOAD_OF + 4U)
/*! Offset of the 'Payload' array in the CMD frame. */
#define SF_SEND_TEST_FR_CMD_PAYL_OF    (SF_CMD_PAYLOAD_OF + 5U)
/*! @} */

/*!
 * @name Receive_Test_Frame SPI command
 *
 * This command waits for a valid downlink frame during timeout time and returns
 * the received data. Use this command ONLY with Certification ID (0xFEDCBA98)
 * otherwise an issue occurs.
 * @{
 */
/*! Receive_Test_Frame command code.  */
#define SF_CMD_RECV_TEST_FR_ID          50U
/*! Size of the Receive_Test_Frame command frame payload in bytes (AUTHENTICATION_OFF). */
#define SF_RECV_TEST_FR_CMD_AOFF_PLD_B  20U
/*! Size of the Receive_Test_Frame command frame payload in bytes (AUTHENTICATION_ON). */
#define SF_RECV_TEST_FR_CMD_AON_PLD_B   5U
/*! Size of the Receive_Test_Frame command frame in bytes  (AUTHENTICATION_OFF). */
#define SF_RECV_TEST_FR_CMD_AOFF_B      (SF_CMD_HEADER_B + SF_RECV_TEST_FR_CMD_AOFF_PLD_B)
/*! Size of the Receive_Test_Frame command frame in bytes  (AUTHENTICATION_ON). */
#define SF_RECV_TEST_FR_CMD_AON_B       (SF_CMD_HEADER_B + SF_RECV_TEST_FR_CMD_AON_PLD_B)
/*! Size of the Receive_Test_Frame acknowledgement payload in bytes (AUTHENTICATION_OFF). */
#define SF_RECV_TEST_FR_ACK_AOFF_PLD_B  0U
/*! Size of the Receive_Test_Frame acknowledgement payload in bytes (AUTHENTICATION_ON). */
#define SF_RECV_TEST_FR_ACK_AON_PLD_B   16U
/*! Size of the Receive_Test_Frame acknowledgement in bytes (AUTHENTICATION_OFF). */
#define SF_RECV_TEST_FR_ACK_AOFF_B      (SF_ACK_HEADER_B + SF_RECV_TEST_FR_ACK_AOFF_PLD_B)
/*! Size of the Receive_Test_Frame acknowledgement in bytes (AUTHENTICATION_ON). */
#define SF_RECV_TEST_FR_ACK_AON_B       (SF_ACK_HEADER_B + SF_RECV_TEST_FR_ACK_AON_PLD_B)

/*! Offset of the 'Frequency' word in the CMD frame. */
#define SF_RECV_TEST_FR_CMD_FREQ_OF     (SF_CMD_PAYLOAD_OF + 0U)
/*! Offset of the 'Init Downlink' byte in the CMD frame. */
#define SF_RECV_TEST_FR_CMD_TOUT_OF     (SF_CMD_PAYLOAD_OF + 4U)
/*! Offset of the 'Buffer' array in the CMD frame (AUTHENTICATION_OFF only). */
#define SF_RECV_TEST_FR_CMD_BUFF_OF     (SF_CMD_PAYLOAD_OF + 5U)

/*! Offset of the 'RSSI' byte in the ACK frame (AUTHENTICATION_ON only). */
#define SF_RECV_TEST_FR_ACK_RSSI_OF     (SF_ACK_PAYLOAD_OF + 0U)
/*! Offset of the 'Buffer' array in the CMD frame (AUTHENTICATION_ON only). */
#define SF_RECV_TEST_FR_ACK_BUFF_OF     (SF_ACK_PAYLOAD_OF + 1U)

/*! Size of the 'Buffer' array in CMD and ACK frames. */
#define SF_RECV_TEST_FR_BUFF_B          15U
/*! @} */

/*!
 * @name Set_Certification_ID SPI command
 *
 * This command temporary sets the Sigfox certification ID (0xFEDCBA98).
 * @{
 */
/*! Set_Certification_ID command code. */
#define SF_CMD_SET_CERT_ID_ID           52U
/*! Size of the Set_Certification_ID command frame payload in bytes. */
#define SF_SET_CERT_ID_CMD_PLD_B        0U
/*! Size of the Set_Certification_ID command frame in bytes. */
#define SF_SET_CERT_ID_CMD_B            (SF_CMD_HEADER_B + SF_SET_CERT_ID_CMD_PLD_B)
/*! Size of the Set_Certification_ID acknowledgement payload in bytes. */
#define SF_SET_CERT_ID_ACK_PLD_B        0U
/*! Size of the Set_Certification_ID acknowledgement in bytes. */
#define SF_SET_CERT_ID_ACK_B            (SF_ACK_HEADER_B + SF_SET_CERT_ID_ACK_PLD_B)
/*! @} */

/*!
 * @name Get_Sigfox_Addon_Version SPI command
 *
 * This command gets the Sigfox Addon version.
 * @{
 */
/*! Get_Sigfox_Addon_Version command code.  */
#define SF_CMD_GET_ADDON_VER_ID         53U
/*! Size of the Get_Sigfox_Addon_Version command frame payload in bytes. */
#define SF_GET_ADDON_VER_CMD_PLD_B      0U
/*! Size of the Get_Sigfox_Addon_Version command frame in bytes. */
#define SF_GET_ADDON_VER_CMD_B          (SF_CMD_HEADER_B + SF_GET_ADDON_VER_CMD_PLD_B)
/*! Size of the Get_Sigfox_Addon_Version acknowledgement payload in bytes. */
#define SF_GET_ADDON_VER_ACK_PLD_B      7U
/*! Size of the Get_Sigfox_Addon_Version acknowledgement in bytes. */
#define SF_GET_ADDON_VER_ACK_B          (SF_ACK_HEADER_B + SF_GET_ADDON_VER_ACK_PLD_B)

/*! Offset of the 'Sigfox library version' field in the ACK frame. */
#define SF_GET_ADDON_VER_ACK_VER_OF     (SF_ACK_PAYLOAD_OF + 0U)
/*! Size of the 'Sigfox library version' field in the ACK frame. */
#define SF_GET_ADDON_VER_ACK_VER_B      7U
/*! @} */

/*!
 * @name Set_HW_Configuration SPI command
 *
 * The HW configuration stored in OL2385 by default does not have to match your
 * HW solution. Use this command to set the HW configuration.
 * @{
 */
/*! Set_HW_Configuration command code.  */
#define SF_CMD_SET_HW_CFG_ID            54U
/*! Size of the Set_HW_Configuration command frame payload in bytes. */
#define SF_SET_HW_CFG_CMD_PLD_B         2U
/*! Size of the Set_HW_Configuration command frame in bytes. */
#define SF_SET_HW_CFG_CMD_B             (SF_CMD_HEADER_B + SF_SET_HW_CFG_CMD_PLD_B)
/*! Size of the Set_HW_Configuration acknowledgement payload in bytes. */
#define SF_SET_HW_CFG_ACK_PLD_B         0U
/*! Size of the Set_HW_Configuration acknowledgement in bytes. */
#define SF_SET_HW_CFG_ACK_B             (SF_ACK_HEADER_B + SF_SET_HW_CFG_ACK_PLD_B)

/*! Offset of the LSB byte of the 'Config' word in the CMD frame. */
#define SF_SET_HW_CFG_CMD_CONFIG_LSB_OF (SF_CMD_PAYLOAD_OF + 0U)
/*! Offset of the MSB byte of the 'Config' word in the CMD frame. */
#define SF_SET_HW_CFG_CMD_CONFIG_MSB_OF (SF_CMD_PAYLOAD_OF + 1U)
/*! @} */

/*!
 * @name Get_HW_Configuration SPI command
 *
 * This command returns the HW configuration stored in EROM.
 * @{
 */
/*! Get_HW_Configuration command code.  */
#define SF_CMD_GET_HW_CFG_ID            55U
/*! Size of the Get_HW_Configuration command frame payload in bytes. */
#define SF_GET_HW_CFG_CMD_PLD_B         0U
/*! Size of the Get_HW_Configuration command frame in bytes. */
#define SF_GET_HW_CFG_CMD_B             (SF_CMD_HEADER_B + SF_GET_HW_CFG_CMD_PLD_B)
/*! Size of the Get_HW_Configuration acknowledgement payload in bytes. */
#define SF_GET_HW_CFG_ACK_PLD_B         2U
/*! Size of the Get_HW_Configuration acknowledgement in bytes. */
#define SF_GET_HW_CFG_ACK_B             (SF_ACK_HEADER_B + SF_GET_HW_CFG_ACK_PLD_B)

/*! Offset of the LSB byte of the 'Config' word in the ACK frame. */
#define SF_GET_HW_CFG_ACK_CONFIG_LSB_OF (SF_ACK_PAYLOAD_OF + 0U)
/*! Offset of the MSB byte of the 'Config' word in the ACK frame. */
#define SF_GET_HW_CFG_ACK_CONFIG_MSB_OF (SF_ACK_PAYLOAD_OF + 1U)
/*! @} */

/*!
 * @name Change_RC SPI command
 *
 * Initializes the OL2385 for one of the defined standards (RC1, RC2, RC3A,
 * RC3C, RC4, RC5, RC6 or RC7) or for the default radio configuration stored
 * in EROM.
 * @{
 */
/*! Change_RC command code.  */
#define SF_CMD_CHANGE_RC_ID             56U
/*! Size of the Change_RC command frame payload in bytes. */
#define SF_CHANGE_RC_CMD_PLD_B          1U
/*! Size of the Change_RC command frame in bytes. */
#define SF_CHANGE_RC_CMD_B              (SF_CMD_HEADER_B + SF_CHANGE_RC_CMD_PLD_B)
/*! Size of the Change_RC acknowledgement payload in bytes. */
#define SF_CHANGE_RC_ACK_PLD_B          0U
/*! Size of the Change_RC acknowledgement in bytes. */
#define SF_CHANGE_RC_ACK_B              (SF_ACK_HEADER_B + SF_CHANGE_RC_ACK_PLD_B)

/*! Offset of the 'RC' byte in the CMD frame. */
#define SF_CHANGE_RC_CMD_RC_OF          (SF_CMD_PAYLOAD_OF + 0U)
/*! @} */

/*!
 * @name Save_RC_To_EROM SPI command
 *
 * Saves the current radio configuration to non-volatile EROM memory.
 * Configuration in EROM memory is used as the default one after the OL2385
 * power-up or wake-up.
 * @{
 */
/*! Save_RC_To_EROM command code. */
#define SF_CMD_SAVE_RC_EROM_ID          57U
/*! Size of the Save_RC_To_EROM command frame payload in bytes. */
#define SF_SAVE_RC_EROM_CMD_PLD_B       0U
/*! Size of the Save_RC_To_EROM command frame in bytes. */
#define SF_SAVE_RC_EROM_CMD_B           (SF_CMD_HEADER_B + SF_SAVE_RC_EROM_CMD_PLD_B)
/*! Size of the Save_RC_To_EROM acknowledgement payload in bytes. */
#define SF_SAVE_RC_EROM_ACK_PLD_B       0U
/*! Size of the Save_RC_To_EROM acknowledgement in bytes. */
#define SF_SAVE_RC_EROM_ACK_B           (SF_ACK_HEADER_B + SF_SAVE_RC_EROM_ACK_PLD_B)
/*! @} */

/*!
 * @name Set_TX_Configuration SPI command
 *
 * This command updates the TX confiuration including specific variables for
 * FH and LBT standard (so-called config words). For FH and LBT, it is mandatory
 * to call this command after the Set_Custom_RC command to set the config words.
 * Optionally, this command can be called also after the Change_RC command.
 * @{
 */
/*! Set_TX_Configuration command code.  */
#define SF_CMD_SET_TX_CFG_ID            58U
/*! Size of the Set_TX_Configuration command frame payload in bytes. */
#define SF_SET_TX_CFG_CMD_PLD_B         14U
/*! Size of the Set_TX_Configuration command frame in bytes. */
#define SF_SET_TX_CFG_CMD_B             (SF_CMD_HEADER_B + SF_SET_TX_CFG_CMD_PLD_B)
/*! Size of the Set_TX_Configuration acknowledgement payload in bytes. */
#define SF_SET_TX_CFG_ACK_PLD_B         0U
/*! Size of the Set_TX_Configuration acknowledgement in bytes. */
#define SF_SET_TX_CFG_ACK_B             (SF_ACK_HEADER_B + SF_SET_TX_CFG_ACK_PLD_B)

/*! Offset of the 'TX Attenuation steps' byte in the CMD frame. */
#define SF_SET_TX_CFG_CMD_TX_STEPS_OF   (SF_CMD_PAYLOAD_OF + 0U)
/*! Offset of the 'TX Config' byte in the CMD frame. */
#define SF_SET_TX_CFG_CMD_TX_CFG_OF     (SF_CMD_PAYLOAD_OF + 1U)
/*! Offset of the 'Config_words[0]' word in the CMD frame. */
#define SF_SET_TX_CFG_CMD_WORD0_OF      (SF_CMD_PAYLOAD_OF + 2U)
/*! Offset of the 'Config_words[1]' word in the CMD frame. */
#define SF_SET_TX_CFG_CMD_WORD1_OF      (SF_CMD_PAYLOAD_OF + 6U)
/*! Offset of the 'Config_words[2]' word in the CMD frame. */
#define SF_SET_TX_CFG_CMD_WORD2_OF      (SF_CMD_PAYLOAD_OF + 10U)

/*! Mask of the 'Timer enable' field in 'TX Config' byte. */
#define SF_SET_TX_CFG_CMD_TX_CFG_TMREN_MASK     0x01U
/*! Mask of the 'Payload encryption enabled' field in 'TX Config' byte. */
#define SF_SET_TX_CFG_CMD_TX_CFG_PEE_MASK       0x02U
/*! Mask of the 'TX repeat' field in 'TX Config' byte. */
#define SF_SET_TX_CFG_CMD_TX_CFG_TXR_MASK       0x0CU
/*! Mask of the 'PA type' field in 'TX Config' byte. */
#define SF_SET_TX_CFG_CMD_TX_CFG_PAT_MASK       0x10U
/*! @} */

/*!
 * @name Save_TX_Config_To_EROM SPI command
 *
 * This command saves the current TX configuration to EROM memory. Configuration
 * in EROM memory is used as the default one after the OL2385 power-up and
 * wake-up.
 * @{
 */
/*! Save_TX_Config_To_EROM command code. */
#define SF_CMD_SAVE_TX_CFG_EROM_ID      59U
/*! Size of the Save_TX_Config_To_EROM command frame payload in bytes. */
#define SF_SAVE_TX_CFG_EROM_CMD_PLD_B   0U
/*! Size of the Save_TX_Config_To_EROM command frame in bytes. */
#define SF_SAVE_TX_CFG_EROM_CMD_B       (SF_CMD_HEADER_B + SF_SAVE_TX_CFG_EROM_CMD_PLD_B)
/*! Size of the Save_TX_Config_To_EROM acknowledgement payload in bytes. */
#define SF_SAVE_TX_CFG_EROM_ACK_PLD_B   0U
/*! Size of the Save_TX_Config_To_EROM acknowledgement in bytes. */
#define SF_SAVE_TX_CFG_EROM_ACK_B       (SF_ACK_HEADER_B + SF_SAVE_TX_CFG_EROM_ACK_PLD_B)
/*! @} */

/*!
 * @name Get_TX_Configuration SPI command
 *
 * This command reads the current TX configuration.
 * @{
 */
/*! Get_TX_Configuration command code.  */
#define SF_CMD_GET_TX_CFG_ID            60U
/*! Size of the Get_TX_Configuration command frame payload in bytes. */
#define SF_GET_TX_CFG_CMD_PLD_B         0U
/*! Size of the Get_TX_Configuration command frame in bytes. */
#define SF_GET_TX_CFG_CMD_B             (SF_CMD_HEADER_B + SF_GET_TX_CFG_CMD_PLD_B)
/*! Size of the Get_TX_Configuration acknowledgement payload in bytes. */
#define SF_GET_TX_CFG_ACK_PLD_B         14U
/*! Size of the Get_TX_Configuration acknowledgement in bytes. */
#define SF_GET_TX_CFG_ACK_B             (SF_ACK_HEADER_B + SF_GET_TX_CFG_ACK_PLD_B)

/*! Offset of the 'TX Attenuation steps' byte in the ACK frame. */
#define SF_GET_TX_CFG_ACK_TX_STEPS_OF   (SF_ACK_PAYLOAD_OF + 0U)
/*! Offset of the 'TX Config' byte in the ACK frame. */
#define SF_GET_TX_CFG_ACK_TX_CFG_OF     (SF_ACK_PAYLOAD_OF + 1U)
/*! Offset of the 'Config_words[0]' word in the ACK frame. */
#define SF_GET_TX_CFG_ACK_WORD0_OF      (SF_ACK_PAYLOAD_OF + 2U)
/*! Offset of the 'Config_words[1]' word in the ACK frame. */
#define SF_GET_TX_CFG_ACK_WORD1_OF      (SF_ACK_PAYLOAD_OF + 6U)
/*! Offset of the 'Config_words[2]' word in the ACK frame. */
#define SF_GET_TX_CFG_ACK_WORD2_OF      (SF_ACK_PAYLOAD_OF + 10U)

/*! Mask of the 'Timer enable' field in 'TX Config' byte. */
#define SF_GET_TX_CFG_ACK_TX_CFG_TMREN_MASK     0x01U
/*! Mask of the 'Payload encryption enabled' field in 'TX Config' byte. */
#define SF_GET_TX_CFG_ACK_TX_CFG_PEE_MASK       0x02U
/*! Mask of the 'TX repeat' field in 'TX Config' byte. */
#define SF_GET_TX_CFG_ACK_TX_CFG_TXR_MASK       0x0CU
/*! Mask of the 'PA type' field in 'TX Config' byte. */
#define SF_GET_TX_CFG_ACK_TX_CFG_PAT_MASK       0x10U

/*! @} */

#endif /* SOURCE_SF_OL2385_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
