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
 * deck_spi.c - Deck-API SPI communication implementation
 */

#include "deck.h"

/*ST includes */
#include "stm32f4xx.h"
//#include "stm32f4xx_dma.h"
#include "config.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "cfassert.h"
#include "config.h"
#include "nvicconf.h"

#define SPI                     SPI1
#define SPI_CLK                 RCC_APB2Periph_SPI1
#define SPI_CLK_INIT            RCC_APB2PeriphClockCmd
#define SPI_IRQ_HANDLER         SPI1_IRQHandler
#define SPI_IRQn                SPI1_IRQn

#define SPI_DMA_IRQ_PRIO        (NVIC_HIGH_PRI)
#define SPI_DMA                 DMA2
#define SPI_DMA_CLK             RCC_AHB1Periph_DMA2
#define SPI_DMA_CLK_INIT        RCC_AHB1PeriphClockCmd

#define SPI_TX_DMA_STREAM       DMA2_Stream5
#define SPI_TX_DMA_IRQ          DMA2_Stream5_IRQn
#define SPI_TX_DMA_IRQHandler   DMA2_Stream5_IRQHandler
#define SPI_TX_DMA_CHANNEL      DMA_Channel_3
#define SPI_TX_DMA_FLAG_TCIF    DMA_FLAG_TCIF5

#define SPI_RX_DMA_STREAM       DMA2_Stream0
#define SPI_RX_DMA_IRQ          DMA2_Stream0_IRQn
#define SPI_RX_DMA_IRQHandler   DMA2_Stream0_IRQHandler
#define SPI_RX_DMA_CHANNEL      DMA_Channel_3
#define SPI_RX_DMA_FLAG_TCIF    DMA_FLAG_TCIF0

#define SPI_SCK_PIN             GPIO_Pin_5
#define SPI_SCK_GPIO_PORT       GPIOA
#define SPI_SCK_GPIO_CLK        RCC_AHB1Periph_GPIOA
#define SPI_SCK_SOURCE          GPIO_PinSource5
#define SPI_SCK_AF              GPIO_AF_SPI1

#define SPI_MISO_PIN            GPIO_Pin_6
#define SPI_MISO_GPIO_PORT      GPIOA
#define SPI_MISO_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define SPI_MISO_SOURCE         GPIO_PinSource6
#define SPI_MISO_AF             GPIO_AF_SPI1

#define SPI_MOSI_PIN            GPIO_Pin_7
#define SPI_MOSI_GPIO_PORT      GPIOA
#define SPI_MOSI_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define SPI_MOSI_SOURCE         GPIO_PinSource7
#define SPI_MOSI_AF             GPIO_AF_SPI1


#define DUMMY_BYTE         0xA5

static bool isInit = false;

static SemaphoreHandle_t txComplete;
static SemaphoreHandle_t rxComplete;
static SemaphoreHandle_t spiMutex;

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

static void spiDMAInit();
static void spiConfigureWithSpeed(uint16_t baudRatePrescaler);


void spiBegin(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // binary semaphores created using xSemaphoreCreateBinary() are created in a state
  // such that the semaphore must first be 'given' before it can be 'taken'
  txComplete = xSemaphoreCreateBinary();
  rxComplete = xSemaphoreCreateBinary();
  spiMutex = xSemaphoreCreateMutex();

  __HAL_RCC_SPI1_CLK_ENABLE();

   __HAL_RCC_GPIOA_CLK_ENABLE();
   /**SPI1 GPIO Configuration
   PA5     ------> SPI1_SCK
   PA6     ------> SPI1_MISO
   PA7     ------> SPI1_MOSI
   */
   GPIO_InitStruct.Pin = E_SPI2_SCK_Pin|E_SPI2_MISO_Pin|E_SPI2_MOSI_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;

#ifdef DECK_SPI_MODE3
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
#else
  GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
#endif
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  /*!< SPI DMA Initialization */
  spiDMAInit();

  /*!< SPI configuration */
  spiConfigureWithSpeed(SPI_BAUDRATE_2MHZ);

  isInit = true;
}

static void spiDMAInit()
{
  DMA_InitTypeDef  DMA_InitStructure;

  /* Configure DMA Initialization Structure */
  hdma_spi1_rx.Instance = DMA2_Stream0;
  hdma_spi1_rx.Init.Channel = DMA_CHANNEL_3;
  hdma_spi1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi1_rx.Init.Mode = DMA_NORMAL;
  hdma_spi1_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_spi1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&hspi1,hdmarx,hdma_spi1_rx);

  /* SPI1_TX Init */
  hdma_spi1_tx.Instance = DMA2_Stream3;
  hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
  hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi1_tx.Init.Mode = DMA_NORMAL;
  hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&hspi1,hdmatx,hdma_spi1_tx);


  // Configure interrupts

  HAL_NVIC_SetPriority(SPI_TX_DMA_IRQ, NVIC_SPI_PRI, 0);
  HAL_NVIC_EnableIRQ(SPI_TX_DMA_IRQ);

  HAL_NVIC_SetPriority(SPI_RX_DMA_IRQ, NVIC_SPI_PRI, 0);
  HAL_NVIC_EnableIRQ(SPI_RX_DMA_IRQ);

}

static void spiConfigureWithSpeed(uint16_t baudRatePrescaler)
{
  SPI_InitTypeDef  SPI_InitStructure;


  SPI_InitStructure.Mode = SPI_MODE_MASTER;
  SPI_InitStructure.Direction = SPI_DIRECTION_2LINES;
  SPI_InitStructure.DataSize = SPI_DATASIZE_8BIT;
  SPI_InitStructure.CLKPolarity = SPI_POLARITY_HIGH;
  SPI_InitStructure.CLKPhase = SPI_PHASE_2EDGE;
  SPI_InitStructure.NSS = SPI_NSS_SOFT;
  SPI_InitStructure.FirstBit = SPI_FIRSTBIT_MSB;
  SPI_InitStructure.TIMode = SPI_TIMODE_DISABLE;
  SPI_InitStructure.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  SPI_InitStructure.CRCPolynomial = 10;
  SPI_InitStructure.BaudRatePrescaler = baudRatePrescaler;


#ifdef DECK_SPI_MODE3
  SPI_InitStructure.CLKPolarity = SPI_POLARITY_LOW;
  SPI_InitStructure.CLKPhase = SPI_PHASE_1EDGE;
#else
  SPI_InitStructure.CLKPolarity = SPI_POLARITY_HIGH;
  SPI_InitStructure.CLKPhase = SPI_PHASE_2EDGE;
#endif

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }




}

bool spiTest(void)
{
  return isInit;
}

bool spiExchange(size_t length, const uint8_t * data_tx, uint8_t * data_rx)
{
  ASSERT_DMA_SAFE(data_tx);
  ASSERT_DMA_SAFE(data_rx);

  // DMA already configured, just need to set memory addresses
  SPI_TX_DMA_STREAM->M0AR = (uint32_t)data_tx;
  SPI_TX_DMA_STREAM->NDTR = length;

  SPI_RX_DMA_STREAM->M0AR = (uint32_t)data_rx;
  SPI_RX_DMA_STREAM->NDTR = length;

  // Enable SPI DMA Interrupts
  DMA_ITConfig(SPI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_ITConfig(SPI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);

  // Clear DMA Flags
//  DMA_ClearFlag(SPI_TX_DMA_STREAM, DMA_FLAG_FEIF5|DMA_FLAG_DMEIF5|DMA_FLAG_TEIF5|DMA_FLAG_HTIF5|DMA_FLAG_TCIF5);
//  DMA_ClearFlag(SPI_RX_DMA_STREAM, DMA_FLAG_FEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_TEIF0|DMA_FLAG_HTIF0|DMA_FLAG_TCIF0);

  // Enable DMA Streams
  DMA_Cmd(SPI_TX_DMA_STREAM,ENABLE);
  DMA_Cmd(SPI_RX_DMA_STREAM,ENABLE);

  // Enable SPI DMA requests
  SPI_I2S_DMACmd(SPI, 2, ENABLE);
  SPI_I2S_DMACmd(SPI, 1, ENABLE);

  // Enable peripheral
  SPI_Cmd(SPI, ENABLE);

  // Wait for completion
  bool result = (xSemaphoreTake(txComplete, portMAX_DELAY) == pdTRUE)
             && (xSemaphoreTake(rxComplete, portMAX_DELAY) == pdTRUE);

  // Disable peripheral
  SPI_Cmd(SPI, DISABLE);
  return result;
}

void spiBeginTransaction(uint16_t baudRatePrescaler)
{
  xSemaphoreTake(spiMutex, portMAX_DELAY);
  spiConfigureWithSpeed(baudRatePrescaler);
}

void spiEndTransaction()
{
  xSemaphoreGive(spiMutex);
}
/*
void __attribute__((used)) SPI_TX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(SPI_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(SPI_TX_DMA_STREAM, SPI_TX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(SPI_TX_DMA_STREAM,SPI_TX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(SPI, 2, DISABLE);

  // Disable streams
  DMA_Cmd(SPI_TX_DMA_STREAM,DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(txComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}

void __attribute__((used)) SPI_RX_DMA_IRQHandler(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(SPI_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(SPI_RX_DMA_STREAM, SPI_RX_DMA_FLAG_TCIF);

  // Clear stream flags
  DMA_ClearFlag(SPI_RX_DMA_STREAM,SPI_RX_DMA_FLAG_TCIF);

  // Disable SPI DMA requests
  SPI_I2S_DMACmd(SPI, 1, DISABLE);

  // Disable streams
  DMA_Cmd(SPI_RX_DMA_STREAM,DISABLE);

  // Give the semaphore, allowing the SPI transaction to complete
  xSemaphoreGiveFromISR(rxComplete, &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken)
  {
    portYIELD();
  }
}
*/
