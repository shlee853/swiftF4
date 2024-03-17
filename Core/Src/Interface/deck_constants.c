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
 * deck_constants.c - Constants for the Deck API
 */

#include "deck.h"

/* Mapping between Deck Pin number, real GPIO and ADC channel */
deckGPIOMapping_t deckGPIOMapping[13] = {
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_PIN_11, .adcCh=-1},            /* RX1 */
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_PIN_10, .adcCh=-1},            /* TX1 */
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_PIN_7,  .adcCh=-1},            /* SDA */
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_PIN_6,  .adcCh=-1},            /* SCL */
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_PIN_8,  .adcCh=-1},            /* IO1 */
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_PIN_5,  .adcCh=-1},            /* IO2 */
  {.periph= RCC_AHB1Periph_GPIOB, .port= GPIOB, .pin=GPIO_PIN_4,  .adcCh=-1},            /* IO3 */
  {.periph= RCC_AHB1Periph_GPIOC, .port= GPIOC, .pin=GPIO_PIN_12, .adcCh=-1},            /* IO4 */
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_PIN_2,  .adcCh=ADC_CHANNEL_2}, /* TX2 */
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_PIN_3,  .adcCh=ADC_CHANNEL_3}, /* RX2 */
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_PIN_5,  .adcCh=ADC_CHANNEL_5}, /* SCK */
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_PIN_6,  .adcCh=ADC_CHANNEL_6}, /* MISO */
  {.periph= RCC_AHB1Periph_GPIOA, .port= GPIOA, .pin=GPIO_PIN_7,  .adcCh=ADC_CHANNEL_7}, /* MOSI */
};

// Pin definitions
const deckPin_t DECK_GPIO_RX1 = {.id=0};
const deckPin_t DECK_GPIO_TX1 = {.id=1};
const deckPin_t DECK_GPIO_SDA = {.id=2};
const deckPin_t DECK_GPIO_SCL = {.id=3};
const deckPin_t DECK_GPIO_IO1 = {.id=4};
const deckPin_t DECK_GPIO_IO2 = {.id=5};
const deckPin_t DECK_GPIO_IO3 = {.id=6};
const deckPin_t DECK_GPIO_IO4 = {.id=7};
const deckPin_t DECK_GPIO_TX2 = {.id=8};
const deckPin_t DECK_GPIO_RX2 = {.id=9};
const deckPin_t DECK_GPIO_SCK = {.id=10};
const deckPin_t DECK_GPIO_MISO = {.id=11};
const deckPin_t DECK_GPIO_MOSI = {.id=12};
