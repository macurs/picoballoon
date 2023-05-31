/*
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file sf_wait.c
 *
 * This file implements functions for busy waiting for SF driver.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "sf_wait.h"
#include "fsl_clock.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!< Gets needed cycles for specified delay in milliseconds, calculation is
 * based on core clock frequency. */
#define SF_MCU_GET_CYCLES_FOR_MS(ms, freq) (((freq) / 1000U) * (ms))

/*!< Gets needed cycles for specified delay in microseconds, calculation is
 * based on core clock frequency. */
#define SF_MCU_GET_CYCLES_FOR_US(us, freq) (((freq) / 1000U) * (us) / 1000U)

/*******************************************************************************
 * Prototypes of internal functions
 ******************************************************************************/

/*!
 * @brief Waits for exact number of cycles which can be expressed as multiple of 4.
 *
 * MOV - 1 cycle
 * SUB - 1 cycle
 * BNE - 1 cycle or 2 cycles if jump is realized
 *
 * Output list (empty) - which registers are output and how to map them to C code.
 * Input list (Cycles) - which registers are input and how to map them to C code.
 * Clobber list (r0, r1, cc) - which registers might have changed during
 * execution of asm code (compiler will have to reload them).
 *
 * @param Cycles | Number of cycles to wait.
 */
inline static void SF_MCU_WaitForMul4Cycles(uint32_t cycles);

/*******************************************************************************
 * Internal functions
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_MCU_WaitForMul4Cycles
 * Description   : Waits for exact number of cycles which can be expressed as
 *                 multiple of 4.
 *
 *END**************************************************************************/
inline static void SF_MCU_WaitForMul4Cycles(uint32_t cycles)
{
#if defined(__CC_ARM)                            /* For ARM Compiler */
    __asm volatile{ nop
                loop:
                    subs cycles, cycles, #4
                    nop
                    bne loop};
#elif defined(__thumb__) && !defined(__thumb2__) /* Thumb instruction set only */
    __asm("mov r0, %[cycles] \n\t" 
          "0: \n\t"                
          "sub r0, #4 \n\t"      
          "nop \n\t"             
          "bne 0b \n\t"            
          :
          : [cycles] "r" (cycles) 
          : "r0", "r1", "cc");
#else /* Thumb2 or A32 instruction set */
    __asm("movs r0, %[cycles] \n"
          "0: \n"
          "subs r0, r0, #4 \n"
          "nop \n\t"
          "bne 0b \n"
          : 
          : [cycles] "r" (cycles) 
          : "r0", "r1", "cc");
#endif
}

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_MCU_WaitMs
 * Description   : Waits for specified amount of milliseconds.
 *
 *END**************************************************************************/
void SF_MCU_WaitMs(uint16_t delay)
{
    uint32_t cycles = (uint32_t) SF_MCU_GET_CYCLES_FOR_MS(1U, CLOCK_GetCoreSysClkFreq());

    /* Advance to multiple of 4. */
    cycles = cycles & 0xFFFFFFFCU;

    for (; delay > 0U; delay--) {
        SF_MCU_WaitForMul4Cycles(cycles);
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SF_MCU_WaitUs
 * Description   : Waits for specified amount of microseconds.
 *
 *END**************************************************************************/
void SF_MCU_WaitUs(uint16_t delay)
{
    uint32_t cycles = (uint32_t) SF_MCU_GET_CYCLES_FOR_US(delay, CLOCK_GetCoreSysClkFreq());

    /* Advance to next multiple of 4. Value 0x04U ensures that the number
     * is not zero. */
    cycles = (cycles & 0xFFFFFFFCU) | 0x04U;
    SF_MCU_WaitForMul4Cycles(cycles);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
