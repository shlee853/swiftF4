/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie 2.0 NRF Firmware
 * Copyright (c) 2014, Bitcraze AB, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 *
 * i2c_drv.c - i2c driver implementation
 *
 * @note
 * For some reason setting CR1 reg in sequence with
 * I2C_AcknowledgeConfig(I2C_SENSORS, ENABLE) and after
 * I2C_GenerateSTART(I2C_SENSORS, ENABLE) sometimes creates an
 * instant start->stop condition (3.9us long) which I found out with an I2C
 * analyzer. This fast start->stop is only possible to generate if both
 * start and stop flag is set in CR1 at the same time. So i tried setting the CR1
 * at once with I2C_SENSORS->CR1 = (I2C_CR1_START | I2C_CR1_ACK | I2C_CR1_PE) and the
 * problem is gone. Go figure...
 */

// Standard includes.
#include <string.h>
#include <stdbool.h>
// Scheduler include files.
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"
// Application includes.
#include "i2c_drv.h"
#include "config.h"
#include "nvicconf.h"

#include "autoconf.h"
#include "cfassert.h"

#include "deck_constants.h"

//DEBUG
#ifdef I2CDRV_DEBUG_LOG_EVENTS
#include "usec_time.h"
#endif

// Definitions of sensors I2C bus
#define I2C_DEFAULT_SENSORS_CLOCK_SPEED             400000

// Definition of eeprom and deck I2C buss
#define I2C_DEFAULT_DECK_CLOCK_SPEED                400000

// Misc constants.
#define I2C_NO_BLOCK				    0
#define I2C_SLAVE_ADDRESS7      0x30
#define I2C_MAX_RETRIES         2
#define I2C_MESSAGE_TIMEOUT     M2T(1000)

// Helpers to unlock bus
#define I2CDEV_CLK_TS (10)
static void gpioWaitForHigh(GPIO_TypeDef *gpio, uint16_t pin, uint16_t timeout_us)
{
  uint64_t start = usecTimestamp();
  while (HAL_GPIO_ReadPin(gpio, pin) == GPIO_PIN_RESET && usecTimestamp() - start <= timeout_us)
  {
  }
}


#ifdef I2CDRV_DEBUG_LOG_EVENTS
// Debug variables
uint32_t eventDebug[1024][2];
uint32_t eventPos = 0;
#endif

/* Private functions */
/**
 * Low level i2c init funciton
 */
static void i2cdrvInitBus(I2cDrv* i2c);
/**
 * Low level dma init funciton
 */
static void i2cdrvDmaSetupBus(I2cDrv* i2c);
/**
 * Start the i2c transfer
 */
static void i2cdrvStartTransfer(I2cDrv *i2c);
/**
 * Try to restart a hanged buss
 */
static void i2cdrvTryToRestartBus(I2cDrv* i2c);
/**
 * Unlocks the i2c bus if needed.
 */
static void i2cdrvdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA);
/**
 * Clear DMA stream
 */
static void i2cdrvClearDMA(I2cDrv* i2c);
/**
 * Event interrupt service routine
 */
static void i2cdrvEventIsrHandler(I2cDrv* i2c);
/**
 * Error interrupt service routine
 */
static void i2cdrvErrorIsrHandler(I2cDrv* i2c);
/**
 * DMA interrupt service routine
 */
static void i2cdrvDmaIsrHandler(I2cDrv* i2c);

// Cost definitions of busses
static const I2cDef sensorBusDef =
{
  .i2cPort            = I2C1,
  .i2cPerif           = RCC_APB1Periph_I2C1,
  .i2cEVIRQn          = I2C1_EV_IRQn,
  .i2cERIRQn          = I2C1_ER_IRQn,
  .i2cClockSpeed      = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
  .gpioSCLPerif       = RCC_AHB1Periph_GPIOB,
  .gpioSCLPort        = GPIOB,
  .gpioSCLPin         = IMU_SCL_Pin,
  .gpioSCLPinSource   = 8,
  .gpioSDAPerif       = RCC_AHB1Periph_GPIOB,
  .gpioSDAPort        = GPIOB,
  .gpioSDAPin         = IMU_SDA_Pin,
  .gpioSDAPinSource   = 9,
  .gpioAF             = GPIO_AF4_I2C1,
  .dmaPerif           = RCC_AHB1Periph_DMA1,
  .dmaChannel         = DMA_CHANNEL_1,
  .dmaRxStream        = DMA1_Stream0,
  .dmaRxIRQ           = DMA1_Stream0_IRQn,
  .dmaRxTCFlag        = DMA_FLAG_TCIF0_4,
  .dmaRxTEFlag        = DMA_FLAG_TEIF0_4,
};

I2cDrv sensorsBus =
{
  .def                = &sensorBusDef,
};

static const I2cDef deckBusDef =
{
  .i2cPort            = I2C3,
  .i2cPerif           = RCC_APB1Periph_I2C3,
  .i2cEVIRQn          = I2C3_EV_IRQn,
  .i2cERIRQn          = I2C3_ER_IRQn,
  .i2cClockSpeed      = I2C_DEFAULT_DECK_CLOCK_SPEED,
  .gpioSCLPerif       = RCC_AHB1Periph_GPIOA,
  .gpioSCLPort        = GPIOA,
  .gpioSCLPin         = GPIO_PIN_8,
  .gpioSCLPinSource   = 8,
  .gpioSDAPerif       = RCC_AHB1Periph_GPIOB,
  .gpioSDAPort        = GPIOB,
  .gpioSDAPin         = GPIO_PIN_4,
  .gpioSDAPinSource   = 4,
  .gpioAF             = GPIO_AF4_I2C3,
  .dmaPerif           = RCC_AHB1Periph_DMA1,
  .dmaChannel         = DMA_CHANNEL_3,
#ifdef CONFIG_DECK_USD_USE_ALT_PINS_AND_SPI
  .dmaRxStream        = DMA1_Stream5,
  .dmaRxIRQ           = DMA1_Stream5_IRQn,
  .dmaRxTCFlag        = DMA_FLAG_TCIF1_5,
  .dmaRxTEFlag        = DMA_FLAG_TEIF1_5,
#else
  .dmaRxStream        = DMA1_Stream2,
  .dmaRxIRQ           = DMA1_Stream2_IRQn,
  .dmaRxTCFlag        = DMA_FLAG_TCIF2_6,
  .dmaRxTEFlag        = DMA_FLAG_TEIF2_6,
#endif
};



I2cDrv deckBus =
{
  .def                = &deckBusDef,
};


extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

static void i2cdrvStartTransfer(I2cDrv *i2c)
{
  ASSERT_DMA_SAFE(i2c->txMessage.buffer);

  if (i2c->txMessage.direction == i2cRead)
  {
	HAL_I2C_Mem_Read_DMA(&hi2c1, i2c->txMessage.slaveAddress<<1, i2c->txMessage.internalAddress, 1, i2c->txMessage.buffer, i2c->txMessage.messageLength);
  }
  else
  {
	HAL_I2C_Mem_Write_DMA(&hi2c1, i2c->txMessage.slaveAddress<<1, i2c->txMessage.internalAddress, 1, i2c->txMessage.buffer, i2c->txMessage.messageLength);
  }

}

static void i2cTryNextMessage(I2cDrv* i2c)
{
  i2c->def->i2cPort->CR1 = (I2C_CR1_STOP | I2C_CR1_PE);
 // I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
}

static void i2cNotifyClient(I2cDrv* i2c)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(i2c->isBusFreeSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void i2cdrvTryToRestartBus(I2cDrv* i2c)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /**I2C1 GPIO Configuration
  PB8     ------> I2C1_SCL
  PB9     ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = IMU_SCL_Pin|IMU_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  i2cdrvdevUnlockBus(i2c->def->gpioSCLPort, i2c->def->gpioSDAPort, i2c->def->gpioSCLPin, i2c->def->gpioSDAPin);


  /* I2C1 clock enable */
  __HAL_RCC_I2C1_CLK_ENABLE();

  // I2C_SENSORS configuration
  hi2c1.Instance = i2c->def->i2cPort;
  hi2c1.Init.ClockSpeed =  i2c->def->i2cClockSpeed;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;


  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }


  hdma_i2c1_rx.Instance = DMA1_Stream0;
  hdma_i2c1_rx.Init.Channel = DMA_CHANNEL_1;
  hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
  hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;



  if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&hi2c1, hdmarx, hdma_i2c1_rx);



  /* I2C1_TX Init */
  hdma_i2c1_tx.Instance = DMA1_Stream1;
  hdma_i2c1_tx.Init.Channel = DMA_CHANNEL_0;
  hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
  hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&hi2c1,hdmatx,hdma_i2c1_tx);

  /* I2C1 interrupt Init */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
/* USER CODE BEGIN I2C1_MspInit 1 */

}


static void i2cdrvInitBus(I2cDrv* i2c)
{
  i2cdrvTryToRestartBus(i2c);

  i2c->isBusFreeSemaphore = xSemaphoreCreateBinaryStatic(&i2c->isBusFreeSemaphoreBuffer);
  i2c->isBusFreeMutex = xSemaphoreCreateMutexStatic(&i2c->isBusFreeMutexBuffer);
}

static void i2cdrvdevUnlockBus(GPIO_TypeDef* portSCL, GPIO_TypeDef* portSDA, uint16_t pinSCL, uint16_t pinSDA)
{

  HAL_GPIO_WritePin(portSDA, pinSDA, GPIO_PIN_SET);

  /* Check SDA line to determine if slave is asserting bus and clock out if so */
  while(HAL_GPIO_ReadPin(portSDA, pinSDA) == GPIO_PIN_RESET)
  {
    /* Set clock high */
	  HAL_GPIO_WritePin(portSCL, pinSCL, GPIO_PIN_SET);
    /* Wait for any clock stretching to finish. */
    gpioWaitForHigh(portSCL, pinSCL, 10 * 1000);
    usDelay(I2CDEV_CLK_TS);

    /* Generate a clock cycle */
    HAL_GPIO_WritePin(portSCL, pinSCL, GPIO_PIN_RESET);
    usDelay(I2CDEV_CLK_TS);
    HAL_GPIO_WritePin(portSCL, pinSCL, GPIO_PIN_SET);
    usDelay(I2CDEV_CLK_TS);
  }

  /* Generate a start then stop condition */
  HAL_GPIO_WritePin(portSCL, pinSCL, GPIO_PIN_SET);
  usDelay(I2CDEV_CLK_TS);
  HAL_GPIO_WritePin(portSDA, pinSDA, GPIO_PIN_RESET);
  usDelay(I2CDEV_CLK_TS);
  HAL_GPIO_WritePin(portSCL, pinSCL, GPIO_PIN_RESET);
  usDelay(I2CDEV_CLK_TS);

  /* Set data and clock high and wait for any clock stretching to finish. */
  HAL_GPIO_WritePin(portSDA, pinSDA, GPIO_PIN_SET);
  HAL_GPIO_WritePin(portSCL, pinSCL, GPIO_PIN_SET);
  gpioWaitForHigh(portSCL, pinSCL, 10 * 1000);
  /* Wait for data to be high */
  gpioWaitForHigh(portSDA, pinSDA, 10 * 1000);
}

//-----------------------------------------------------------

void i2cdrvInit(I2cDrv* i2c)
{
  i2cdrvInitBus(i2c);
}

void i2cdrvCreateMessage(I2cMessage *message,
                      uint8_t  slaveAddress,
                      I2cDirection  direction,
                      uint32_t length,
                      const uint8_t *buffer)
{
  ASSERT_DMA_SAFE(buffer);

  message->slaveAddress = slaveAddress;
  message->direction = direction;
  message->isInternal16bit = false;
  message->internalAddress = I2C_NO_INTERNAL_ADDRESS;
  message->messageLength = length;
  message->status = i2cAck;
  message->buffer = (uint8_t *)buffer;
  message->nbrOfRetries = I2C_MAX_RETRIES;
}

void i2cdrvCreateMessageIntAddr(I2cMessage *message,
                             uint8_t  slaveAddress,
                             bool IsInternal16,
                             uint16_t intAddress,
                             I2cDirection  direction,
                             uint32_t length,
                             const uint8_t  *buffer)
{
  ASSERT_DMA_SAFE(buffer);

  message->slaveAddress = slaveAddress;
  message->direction = direction;
  message->isInternal16bit = IsInternal16;
  message->internalAddress = intAddress;
  message->messageLength = length;
  message->status = i2cAck;
  message->buffer = (uint8_t *)buffer;
  message->nbrOfRetries = I2C_MAX_RETRIES;
}

bool i2cdrvMessageTransfer(I2cDrv* i2c, I2cMessage* message)
{
  bool status = false;

  xSemaphoreTake(i2c->isBusFreeMutex, portMAX_DELAY); // Protect message data
  // Copy message
  memcpy((char*)&i2c->txMessage, (char*)message, sizeof(I2cMessage));
  // We can now start the ISR sending this message.
  i2cdrvStartTransfer(i2c);
  // Wait for transaction to be done

  xSemaphoreTake(i2c->isBusFreeSemaphore, 1);

  if (i2c->txMessage.status == i2cAck)
  {
    status = true;
  }



//  vTaskDelay(M2T(50));
/*
  if (xSemaphoreTake(i2c->isBusFreeSemaphore, I2C_MESSAGE_TIMEOUT) == pdTRUE)
  {
    if (i2c->txMessage.status == i2cAck)
    {
      status = true;
    }
  }
  else
  {
    i2cdrvClearDMA(i2c);
    i2cdrvTryToRestartBus(i2c);
    //TODO: If bus is really hanged... fail safe
  }
*/

  xSemaphoreGive(i2c->isBusFreeMutex);

  return status;
}


static void i2cdrvEventIsrHandler(I2cDrv* i2c)
{
  uint16_t SR1;
  uint16_t SR2;

  // read the status register first
  SR1 = i2c->def->i2cPort->SR1;

#ifdef I2CDRV_DEBUG_LOG_EVENTS
  // Debug code
  eventDebug[eventPos][0] = usecTimestamp();
  eventDebug[eventPos][1] = SR1;
  if (++eventPos == 1024)
  {
    eventPos = 0;
  }
#endif

//  HAL_I2C_EV_IRQHandler(&hi2c1);
  i2cNotifyClient(i2c);


/*
  // Start bit event
  if (SR1 & I2C_SR1_SB)
  {
    i2c->messageIndex = 0;

    if(i2c->txMessage.direction == i2cWrite ||
       i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
    {
//      I2C_Send7bitAddress(i2c->def->i2cPort, i2c->txMessage.slaveAddress << 1, I2C_Direction_Transmitter);
    }
    else
    {
//      I2C_AcknowledgeConfig(i2c->def->i2cPort, ENABLE);
//      I2C_Send7bitAddress(i2c->def->i2cPort, i2c->txMessage.slaveAddress << 1, I2C_Direction_Receiver);
    }
  }
  // Address event
  else if (SR1 & I2C_SR1_ADDR)
  {
    if(i2c->txMessage.direction == i2cWrite ||
       i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
    {
      SR2 = i2c->def->i2cPort->SR2;                               // clear ADDR
      // In write mode transmit is always empty so can send up to two bytes
      if (i2c->txMessage.internalAddress != I2C_NO_INTERNAL_ADDRESS)
      {
        if (i2c->txMessage.isInternal16bit)
        {
  //        I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0xFF00) >> 8);
  //        I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0x00FF));
        }
        else
        {
  //        I2C_SendData(i2c->def->i2cPort, (i2c->txMessage.internalAddress & 0x00FF));
        }
        i2c->txMessage.internalAddress = I2C_NO_INTERNAL_ADDRESS;
      }
  //    I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, ENABLE);        // allow us to have an EV7
    }
    else // Reading, start DMA transfer
    {
      if(i2c->txMessage.messageLength == 1)
      {
        I2C_AcknowledgeConfig(i2c->def->i2cPort, DISABLE);
      }
      else
      {
        I2C_DMALastTransferCmd(i2c->def->i2cPort, ENABLE); // No repetitive start
      }
      // Disable buffer I2C interrupts
      I2C_ITConfig(i2c->def->i2cPort, I2C_IT_EVT | I2C_IT_BUF, DISABLE);
      // Enable the Transfer Complete interrupt
      DMA_ITConfig(i2c->def->dmaRxStream, DMA_IT_TC | DMA_IT_TE, ENABLE);
      I2C_DMACmd(i2c->def->i2cPort, ENABLE); // Enable before ADDR clear

      // Workaround to enable DMA for Renode simulation.
      // The I2C uses the DMA for reading, but the Renode implementation lacks some functionality
      // and for a message to be read the DMA needs to be enabled manually.
      // Without setting this bit the I2C reading fails.
      // With added functionality it should be possible to remove.
      DMA_Cmd(i2c->def->dmaRxStream, ENABLE); // Workaround

      __DMB();                         // Make sure instructions (clear address) are in correct order
      SR2 = i2c->def->i2cPort->SR2;    // clear ADDR
    }
  }
  // Byte transfer finished
  else if (SR1 & I2C_SR1_BTF)
  {
    SR2 = i2c->def->i2cPort->SR2;
    if (SR2 & I2C_SR2_TRA) // In write mode?
    {
      if (i2c->txMessage.direction == i2cRead) // internal address read
      {
        i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE); // Generate start
      }
      else
      {
        i2cNotifyClient(i2c);
        // Are there any other messages to transact? If so stop else repeated start.
        i2cTryNextMessage(i2c);
      }
    }
    else // Reading. Shouldn't happen since we use DMA for reading.
    {
      ASSERT(1);
      i2c->txMessage.buffer[i2c->messageIndex++] = I2C_ReceiveData(i2c->def->i2cPort);
      if(i2c->messageIndex == i2c->txMessage.messageLength)
      {
        i2cNotifyClient(i2c);
        // Are there any other messages to transact?
        i2cTryNextMessage(i2c);
      }
    }
    // A second BTF interrupt might occur if we don't wait for it to clear.
    // TODO Implement better method.
    while (i2c->def->i2cPort->CR1 & 0x0100) { ; }
  }
  // Byte received
  else if (SR1 & I2C_SR1_RXNE) // Should not happen when we use DMA for reception.
  {
    i2c->txMessage.buffer[i2c->messageIndex++] = I2C_ReceiveData(i2c->def->i2cPort);
    if(i2c->messageIndex == i2c->txMessage.messageLength)
    {
      I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);   // disable RXE to get BTF
    }
  }
  // Byte ready to be transmitted
  else if (SR1 & I2C_SR1_TXE)
  {
    if (i2c->txMessage.direction == i2cRead)
    {
      // Disable TXE to flush and get BTF to switch to read.
      // Switch must be done in BTF or strange things happen.
      I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
    }
    else
    {
      I2C_SendData(i2c->def->i2cPort, i2c->txMessage.buffer[i2c->messageIndex++]);
      if(i2c->messageIndex == i2c->txMessage.messageLength)
      {
        // Disable TXE to allow the buffer to flush and get BTF
        I2C_ITConfig(i2c->def->i2cPort, I2C_IT_BUF, DISABLE);
        // If an instruction is not here an extra byte gets sent, don't know why...
        // Is is most likely timing issue but STM32F405 I2C peripheral is bugged so
        // this is the best solution so far.
        __DMB();
      }
    }
  }

  */


}


static void i2cdrvErrorIsrHandler(I2cDrv* i2c)
{

//	HAL_I2C_ER_IRQHandler(&hi2c1);
	/*
  if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_AF))
  {
    if(i2c->txMessage.nbrOfRetries-- > 0)
    {
      // Retry by generating start
      i2c->def->i2cPort->CR1 = (I2C_CR1_START | I2C_CR1_PE);
    }
    else
    {
      // Failed so notify client and try next message if any.
      i2c->txMessage.status = i2cNack;
      i2cNotifyClient(i2c);
      i2cTryNextMessage(i2c);
    }
    I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_AF);
  }
  if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_BERR))
  {
      I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_BERR);
  }
  if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_OVR))
  {
      I2C_ClearFlag(i2c->def->i2cPort, I2C_FLAG_OVR);
  }
  if (I2C_GetFlagStatus(i2c->def->i2cPort, I2C_FLAG_ARLO))
  {
      I2C_ClearFlag(i2c->def->i2cPort,I2C_FLAG_ARLO);
  }

  */
}

static void i2cdrvClearDMA(I2cDrv* i2c)
{
/*	  __HAL_DMA_DISABLE(i2c->def->dmaRxStream);
	  __HAL_DMA_CLEAR_FLAG(i2c->def->dmaRxStream, i2c->def->dmaRxTCFlag);
	//  I2C_DMACmd(i2c->def->i2cPort, DISABLE);
	//  I2C_DMALastTransferCmd(i2c->def->i2cPort, DISABLE);
	  __HAL_DMA_DISABLE_IT(i2c->def->dmaRxStream, DMA_IT_TC | DMA_IT_TE);

	  */
}

static void i2cdrvDmaRxIsrHandler(I2cDrv* i2c)
{


	if(__HAL_DMA_GET_FLAG(&hdma_i2c1_rx,DMA_FLAG_TCIF0_4)) {
//		HAL_DMA_IRQHandler(&hdma_i2c1_rx);
		//	    i2cNotifyClient(i2c);

	}

}

static void i2cdrvDmaTxIsrHandler(I2cDrv* i2c)
{

	int status = __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_i2c1_tx);
	if(status) {
//		HAL_DMA_IRQHandler(&hdma_i2c1_tx);
//	    i2cNotifyClient(i2c);
	}


}



#ifdef CONFIG_DECK_USD_USE_ALT_PINS_AND_SPI
void __attribute__((used)) DMA1_Stream5_IRQ_Callback(void)
#else
void __attribute__((used)) DMA1_Stream0_IRQ_Callback(void)
#endif
{
	i2cdrvDmaRxIsrHandler(&sensorsBus);
}


void __attribute__((used)) I2C1_ER_IRQ_Callback(void)
{
  i2cdrvErrorIsrHandler(&sensorsBus);
}

void __attribute__((used)) I2C1_EV_IRQ_Callback(void)
{
  i2cdrvEventIsrHandler(&sensorsBus);
}

void __attribute__((used)) DMA1_Stream1_IRQ_Callback(void)
{
	i2cdrvDmaTxIsrHandler(&sensorsBus);
}

