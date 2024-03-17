/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * buzzer.c - Functions for interfacing with decks with buzzers
 */
#define DEBUG_MODULE "BUZZER"

#include "stm32f4xx.h"
#include <string.h>
#include <stdbool.h>
#include "buzzer.h"


extern TIM_HandleTypeDef htim3;




void buzzerInit()
{
	 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	 TIM3->PSC= 0;
}

bool buzzerTest()
{
  return true;
}

void buzzerOff()
{
  TIM3->PSC= 0;
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

void buzzerOn(uint32_t freq)
{
	TIM3->PSC = (1000000-1)/freq;
}

