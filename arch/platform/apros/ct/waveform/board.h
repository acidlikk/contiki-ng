/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/** \addtogroup apros-tag
 * @{
 *
 * \defgroup apros-ct-waveform-peripherals Peripherals
 *
 * Defines related to the CC2650 Apros CT Sensor
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 *
 * This file can be used as the basis to configure other boards using the
 * CC13xx/CC26xx code as their basis.
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the Apros
 * CC2650 CT Waveform Sensor(SK7255)
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_H_
#define BOARD_H_
/*---------------------------------------------------------------------------*/
#include "ioc.h"
/*---------------------------------------------------------------------------*/
/**
 * \name LED HAL configuration
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define LEDS_CONF_COUNT                 4
#define LEDS_CONF_RED                   1
#define LEDS_CONF_YELLOW                2
#define LEDS_CONF_GREEN                 4
#define LEDS_CONF_ORANGE                8
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_18
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name UART IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_UART_RX        IOID_UNUSED
#define BOARD_IOID_UART_TX        IOID_UNUSED
#define BOARD_IOID_UART_RTS       IOID_UNUSED
#define BOARD_IOID_UART_CTS       IOID_UNUSED
#define BOARD_UART_RX             (1 << BOARD_IOID_UART_RX)
#define BOARD_UART_TX             (1 << BOARD_IOID_UART_TX)
#define BOARD_UART_RTS            (1 << BOARD_IOID_UART_RTS)
#define BOARD_UART_CTS            (1 << BOARD_IOID_UART_CTS)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name User IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_TPS_SW         IOID_1
#define BOARD_IOID_TS_SW          IOID_4
#define BOARD_IOID_TPL_DONE       IOID_7
#define BOARD_IOID_CHA_ENABLE     IOID_12
#define BOARD_IOID_TPL_DRV        IOID_14
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Remaining pins
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_DIO0           IOID_0
#define BOARD_IOID_DIO2           IOID_2
#define BOARD_IOID_DIO3           IOID_3
#define BOARD_IOID_DIO5           IOID_5
#define BOARD_IOID_DIO6           IOID_6
#define BOARD_IOID_DIO8           IOID_8
#define BOARD_IOID_DIO9           IOID_9
#define BOARD_IOID_DIO10          IOID_10
#define BOARD_IOID_DIO11          IOID_11
#define BOARD_IOID_DIO13          IOID_13
#define BOARD_IOID_DIO15          IOID_15
#define BOARD_IOID_DIO19          IOID_19
#define BOARD_IOID_DIO20          IOID_20
#define BOARD_IOID_DIO21          IOID_21
#define BOARD_IOID_DIO22          IOID_22
#define BOARD_IOID_DIO23          IOID_23
#define BOARD_IOID_DIO24          IOID_24
#define BOARD_IOID_DIO25          IOID_25
#define BOARD_IOID_DIO26          IOID_26
#define BOARD_IOID_DIO27          IOID_27
#define BOARD_IOID_DIO28          IOID_28
#define BOARD_IOID_DIO30          IOID_30

#define BOARD_UNUSED_PINS { \
    BOARD_IOID_DIO0, BOARD_IOID_DIO2, BOARD_IOID_DIO3, BOARD_IOID_DIO5, \
    BOARD_IOID_DIO6, BOARD_IOID_DIO8, BOARD_IOID_DIO9, BOARD_IOID_DIO10, \
    BOARD_IOID_DIO11, BOARD_IOID_DIO13, BOARD_IOID_DIO15, BOARD_IOID_DIO19, \
    BOARD_IOID_DIO20, BOARD_IOID_DIO21, BOARD_IOID_DIO22, BOARD_IOID_DIO23, \
    BOARD_IOID_DIO24, BOARD_IOID_DIO25, BOARD_IOID_DIO26, BOARD_IOID_DIO27, \
    BOARD_IOID_DIO28, BOARD_IOID_DIO30, \
    IOID_UNUSED \
  }
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "APROS CT WAVEFORM SENSOR"
/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
