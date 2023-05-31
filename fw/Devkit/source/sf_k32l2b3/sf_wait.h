/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file sf_wait.h
 *
 * This file implements functions for busy waiting for SF driver.
 */

#ifndef SF_WAIT_H_
#define SF_WAIT_H_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>

/*******************************************************************************
 * API
 ******************************************************************************/
/*!
 * @addtogroup function_group
 * @{
 */ 

/*!
 * @brief Waits for specified amount of milliseconds.
 *
 * @param delay - Number of milliseconds to wait.
 */
void SF_MCU_WaitMs(uint16_t delay);

/*!
 * @brief Waits for specified amount of microseconds.
 *
 * @param delay - Number of microseconds to wait.
 */
void SF_MCU_WaitUs(uint16_t delay);
/*! @} */

#endif /* SF_WAIT_H_ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
