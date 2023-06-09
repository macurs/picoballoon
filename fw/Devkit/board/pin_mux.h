/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*! @name PORTD2 (number 59), SF_ACK
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_SF_ACK_FGPIO FGPIOD             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_SF_ACK_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_SF_ACK_GPIO_PIN_MASK (1U << 2U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_SF_ACK_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SF_ACK_PIN 2U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SF_ACK_PIN_MASK (1U << 2U)      /*!<@brief PORT pin mask */
                                                       /* @} */

/*! @name PORTD4 (number 61), SF_CS
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_SF_CS_FGPIO FGPIOD             /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_SF_CS_GPIO GPIOD               /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_SF_CS_GPIO_PIN_MASK (1U << 4U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_SF_CS_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SF_CS_PIN 4U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SF_CS_PIN_MASK (1U << 4U)      /*!<@brief PORT pin mask */
                                                      /* @} */

/*! @name PORTD5 (number 62), SF_SCLK
  @{ */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_SF_SCLK_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SF_SCLK_PIN 5U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SF_SCLK_PIN_MASK (1U << 5U)      /*!<@brief PORT pin mask */
                                                        /* @} */

/*! @name PORTD6 (number 63), SF_MOSI
  @{ */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_SF_MOSI_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SF_MOSI_PIN 6U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SF_MOSI_PIN_MASK (1U << 6U)      /*!<@brief PORT pin mask */
                                                        /* @} */

/*! @name PORTD7 (number 64), SF_MISO
  @{ */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_SF_MISO_PORT PORTD               /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_SF_MISO_PIN 7U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_SF_MISO_PIN_MASK (1U << 7U)      /*!<@brief PORT pin mask */
                                                        /* @} */

/*! @name PORTE31 (number 19), LED2
  @{ */

/* Symbols to be used with GPIO driver */
#define BOARD_INITPINS_LED2_FGPIO FGPIOE              /*!<@brief FGPIO peripheral base pointer */
#define BOARD_INITPINS_LED2_GPIO GPIOE                /*!<@brief GPIO peripheral base pointer */
#define BOARD_INITPINS_LED2_GPIO_PIN_MASK (1U << 31U) /*!<@brief GPIO pin mask */

/* Symbols to be used with PORT driver */
#define BOARD_INITPINS_LED2_PORT PORTE                /*!<@brief PORT peripheral base pointer */
#define BOARD_INITPINS_LED2_PIN 31U                   /*!<@brief PORT pin number */
#define BOARD_INITPINS_LED2_PIN_MASK (1U << 31U)      /*!<@brief PORT pin mask */
                                                      /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitLEDsPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
