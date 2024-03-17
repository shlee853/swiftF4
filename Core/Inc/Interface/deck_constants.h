/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * deck_contants.h - Arduino-compatible constant definition
 */

#ifndef __DECK_CONSTANTS_H__
#define __DECK_CONSTANTS_H__

// For true and false
#include <stdbool.h>

#include "stm32f4xx.h"

#define LOW 0x0
#define HIGH 0x1

#define INPUT           0x0
#define OUTPUT          0x1
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0x3

#define RCC_AHB1Periph_GPIOA             ((uint32_t)0x00000001)
#define RCC_AHB1Periph_GPIOB             ((uint32_t)0x00000002)
#define RCC_AHB1Periph_GPIOC             ((uint32_t)0x00000004)
#define RCC_AHB1Periph_GPIOD             ((uint32_t)0x00000008)
#define RCC_AHB1Periph_GPIOE             ((uint32_t)0x00000010)
#define RCC_AHB1Periph_GPIOF             ((uint32_t)0x00000020)
#define RCC_AHB1Periph_GPIOG             ((uint32_t)0x00000040)
#define RCC_AHB1Periph_GPIOH             ((uint32_t)0x00000080)
#define RCC_AHB1Periph_GPIOI             ((uint32_t)0x00000100)
#define RCC_AHB1Periph_GPIOJ             ((uint32_t)0x00000200)
#define RCC_AHB1Periph_GPIOK             ((uint32_t)0x00000400)

#define RCC_APB1Periph_TIM2              ((uint32_t)0x00000001)
#define RCC_APB1Periph_TIM3              ((uint32_t)0x00000002)
#define RCC_APB1Periph_TIM4              ((uint32_t)0x00000004)
#define RCC_APB1Periph_TIM5              ((uint32_t)0x00000008)
#define RCC_APB1Periph_TIM6              ((uint32_t)0x00000010)
#define RCC_APB1Periph_TIM7              ((uint32_t)0x00000020)
#define RCC_APB1Periph_TIM12             ((uint32_t)0x00000040)
#define RCC_APB1Periph_TIM13             ((uint32_t)0x00000080)
#define RCC_APB1Periph_TIM14             ((uint32_t)0x00000100)

#define RCC_APB1Periph_I2C1              ((uint32_t)0x00200000)
#define RCC_APB1Periph_I2C2              ((uint32_t)0x00400000)
#define RCC_APB1Periph_I2C3              ((uint32_t)0x00800000)

#define RCC_AHB1Periph_DMA1              ((uint32_t)0x00200000)
#define RCC_AHB1Periph_DMA2              ((uint32_t)0x00400000)


#define DBGMCU_TIM2_STOP             ((uint32_t)0x00000001)
#define DBGMCU_TIM3_STOP             ((uint32_t)0x00000002)
#define DBGMCU_TIM4_STOP             ((uint32_t)0x00000004)
#define DBGMCU_TIM5_STOP             ((uint32_t)0x00000008)
#define DBGMCU_TIM6_STOP             ((uint32_t)0x00000010)
#define DBGMCU_TIM7_STOP             ((uint32_t)0x00000020)
#define DBGMCU_TIM12_STOP            ((uint32_t)0x00000040)
#define DBGMCU_TIM13_STOP            ((uint32_t)0x00000080)
#define DBGMCU_TIM14_STOP            ((uint32_t)0x00000100)


/*
 * The Deck port GPIO pins, as named by the deck port / expansion port / expansion breakout documentation on bitcraze.io.
 * Sequenced according to the deckGPIOMapping struct, so that these can be used for lookup in the struct.
 */

// Type used to identify a pin in the deck API.
// id is used as an index in the deckGPIOMapping array
typedef struct {uint8_t id;} deckPin_t;

extern const deckPin_t DECK_GPIO_RX1;
extern const deckPin_t DECK_GPIO_TX1;
extern const deckPin_t DECK_GPIO_SDA;
extern const deckPin_t DECK_GPIO_SCL;
extern const deckPin_t DECK_GPIO_IO1;
extern const deckPin_t DECK_GPIO_IO2;
extern const deckPin_t DECK_GPIO_IO3;
extern const deckPin_t DECK_GPIO_IO4;
extern const deckPin_t DECK_GPIO_TX2;
extern const deckPin_t DECK_GPIO_RX2;
extern const deckPin_t DECK_GPIO_SCK;
extern const deckPin_t DECK_GPIO_MISO;
extern const deckPin_t DECK_GPIO_MOSI;


typedef const struct {
  uint32_t periph;
  GPIO_TypeDef* port;
  uint16_t pin;
  int8_t adcCh; /* -1 means no ADC available for this pin. */
} deckGPIOMapping_t;

extern deckGPIOMapping_t deckGPIOMapping[];

#endif
