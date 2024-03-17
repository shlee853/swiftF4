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
 * digital.c - Deck-API digital IO implementation
 */

#include "deck.h"

#include "stm32f4xx.h"

void pinMode(const deckPin_t pin, const uint32_t mode)
{
  RCC_AHB1PeriphClockCmd(deckGPIOMapping[pin.id].periph, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure = {0};

  GPIO_InitStructure.Pin = deckGPIOMapping[pin.id].pin;
  GPIO_InitStructure.Mode = (mode == OUTPUT) ? GPIO_MODE_AF_PP:GPIO_MODE_INPUT;
  if (mode == INPUT_PULLUP) GPIO_InitStructure.Pull = GPIO_PULLUP;
  if (mode == INPUT_PULLDOWN) GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(deckGPIOMapping[pin.id].port, &GPIO_InitStructure);
}

void digitalWrite(const deckPin_t pin, const uint32_t val)
{
  GPIO_PinState action = GPIO_PIN_RESET;
  if (val) {
    action = GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(deckGPIOMapping[pin.id].port, deckGPIOMapping[pin.id].pin, action);
}

int digitalRead(const deckPin_t pin)
{
  int val = HAL_GPIO_ReadPin(deckGPIOMapping[pin.id].port, deckGPIOMapping[pin.id].pin);
  return (val==GPIO_PIN_SET)?HIGH:LOW;
}
