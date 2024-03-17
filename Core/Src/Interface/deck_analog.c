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
 * deck_analog.c - Arduino-compatible analog input implementation
 */

#include "deck.h"

#include "stm32f4xx.h"
#include "stm32f4xx_hal_gpio.h"

static  uint32_t  stregResolution;
static  uint32_t  adcRange;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

void adcInit(void)
{

	HAL_ADC_MspDeInit(&hadc1);

	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc1.Init.ScanConvMode = DISABLE;
	  hadc1.Init.ContinuousConvMode = ENABLE;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.NbrOfConversion = 1;
	  hadc1.Init.DMAContinuousRequests = ENABLE;
	  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }

	ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
  	   Error_Handler();
    }

	HAL_ADC_MspInit(&hadc1);

	__HAL_RCC_DMA2_CLK_ENABLE();
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);


}

static uint16_t analogReadChannel(uint8_t channel)
{

  ADC_ChannelConfTypeDef sConfig;

  sConfig.Channel = channel;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  sConfig.Rank = 1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the conversion */
  HAL_ADC_Start(&hadc1);

  /* Wait until conversion completion */
  while(HAL_ADC_GetState(&hadc1) == RESET);

  /* Get the conversion value */
  return HAL_ADC_GetValue(&hadc1);
}

uint16_t analogRead(const deckPin_t pin)
{
  assert_param(deckGPIOMapping[pin.id].adcCh > -1);



  /* Now set the GPIO pin to analog mode. */

  /* Enable clock for the peripheral of the pin.*/
  RCC_AHB1PeriphClockCmd(deckGPIOMapping[pin.id].periph, ENABLE);

  /* Populate structure with RESET values. */
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Initialise the GPIO pin to analog mode. */
  GPIO_InitStructure.Pin   = deckGPIOMapping[pin.id].pin;
  GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;

  /* TODO: Any settling time before we can do ADC after init on the GPIO pin? */
  LL_GPIO_Init(deckGPIOMapping[pin.id].port, &GPIO_InitStructure);

  /* Read the appropriate ADC channel. */
  return analogReadChannel((uint8_t)deckGPIOMapping[pin.id].adcCh);
}

void analogReference(uint8_t type)
{
  /*
   * TODO: We should probably support the Arduino EXTERNAL type here.
   * TODO: Figure out which voltage reference to compensate with.
   */
  assert_param(type == 0 /* DEFAULT */);
}

void analogReadResolution(uint8_t bits)
{

  assert_param((bits >= 6) && (bits <= 12));

  adcRange = 1 << bits;
  switch (bits)
  {
    case 12: stregResolution = ADC_RESOLUTION_12B; break;
    case 10: stregResolution = ADC_RESOLUTION_10B; break;
    case 8:  stregResolution = ADC_RESOLUTION_8B; break;
    case 6:  stregResolution = ADC_RESOLUTION_6B; break;
    default: stregResolution = ADC_RESOLUTION_12B; break;
  }

  /* Init ADC2 witch new resolution */
  hadc1.Init.Resolution = stregResolution;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

}

float analogReadVoltage(const deckPin_t pin)
{
  float voltage;

  voltage = analogRead(pin) * VREF / adcRange;

  return voltage;
}
