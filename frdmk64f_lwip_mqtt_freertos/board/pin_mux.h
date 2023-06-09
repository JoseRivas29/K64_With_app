/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

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

#define SOPT5_UART0TXSRC_UART_TX 0x00u /*!<@brief UART 0 transmit data source select: UART0_TX pin */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif


#define BOARD_SW2_GPIO	GPIOC
#define BOARD_SW2_PORT	PORTC
#define BOARD_SW2_PIN	6U

#define BOARD_SW3_GPIO	GPIOA
#define BOARD_SW3_PORT	PORTA
#define BOARD_SW3_PIN	4U

#define BOARD_LED_BLUE_GPIO	GPIOB
#define BOARD_LED_BLUE_PORT	PORTB
#define BOARD_LED_BLUE_PIN	21U

#define BOARD_LED_RED_GPIO	GPIOB
#define BOARD_LED_RED_PORT	PORTB
#define BOARD_LED_RED_PIN	22U

#define BOARD_LED_GREEN_GPIO	GPIOE
#define BOARD_LED_GREEN_PORT	PORTE
#define BOARD_LED_GREEN_PIN	26U




/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
