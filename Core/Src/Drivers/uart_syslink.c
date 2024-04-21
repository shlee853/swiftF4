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
 * uart_syslink.c - Uart syslink to nRF51 and raw access functions
 */
#include <stdint.h>
#include <string.h>

/*ST includes */
#include "stm32f4xx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"
#include "autoconf.h"
#include "uart_syslink.h"
#include "crtp.h"
#include "cfassert.h"
#include "config.h"
#include "queuemonitor.h"
#include "static_mem.h"
#include "led.h"

#define DEBUG_MODULE "U-SLK"
#include "debug.h"

#define UARTSLK_DATA_TIMEOUT_MS 1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((uint32_t)0x00000001)

#define UARTSLK_CLKSUM_SIZE   2

static bool isInit = false;

static xSemaphoreHandle waitUntilSendDone;
static StaticSemaphore_t waitUntilSendDoneBuffer;
static xSemaphoreHandle uartBusy;
static StaticSemaphore_t uartBusyBuffer;
static xQueueHandle syslinkPacketDelivery;
STATIC_MEM_QUEUE_ALLOC(syslinkPacketDelivery, 8, sizeof(SyslinkPacket));
static bool syslinkPacketDeliveryReadyToReceive = false;

#ifdef CONFIG_SYSLINK_DMA
static uint8_t dmaRXBuffer[64];
static DMA_InitTypeDef DMA_InitStructureShareRX;
#endif
static uint8_t dmaTXBuffer[64];
static uint8_t *outDataIsr;
static uint8_t dataIndexIsr;
static uint8_t dataSizeIsr;
static bool    isUartDmaInitialized;
static DMA_InitTypeDef DMA_InitStructureShareTX;
static uint16_t initialDMACount;
static uint16_t remainingDMACount;
static bool     dmaIsPaused;

static volatile SyslinkPacket slp = {0};
static volatile SyslinkRxState rxState = waitForFirstStart;
static volatile uint8_t dataIndex = 0;
static volatile uint8_t cksum[UARTSLK_CLKSUM_SIZE] = {0};
static void uartslkHandleDataFromISR(uint8_t c, BaseType_t * const pxHigherPriorityTaskWoken);

static void uartslkPauseDma();
static void uartslkResumeDma();

// Debug probe
static uint32_t dmaPausedCounter;
static uint32_t dmaTxStreamPausedCounter;
static uint32_t dmaResumedCounter;
static uint32_t dmaTxStreamResumedCounter;
static bool dmaNrfFlowControlBufferFull;
static uint32_t dmaSendWhileNrfBufferFull;


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;

/**
  * Configures the UART DMA. Mainly used for FreeRTOS trace
  * data transfer.
  */
void uartslkDmaInit(void)
{

#ifdef CONFIG_SYSLINK_DMA

	HAL_UART_MspInit(&huart1);

#endif

isUartDmaInitialized = true;

}

void uartslkInit(void)
{
  // initialize the FreeRTOS structures first, to prevent null pointers in interrupts
  waitUntilSendDone = xSemaphoreCreateBinaryStatic(&waitUntilSendDoneBuffer); // initialized as blocking
  uartBusy = xSemaphoreCreateBinaryStatic(&uartBusyBuffer); // initialized as blocking
  xSemaphoreGive(uartBusy); // but we give it because the uart isn't busy at initialization

  syslinkPacketDelivery = STATIC_MEM_QUEUE_CREATE(syslinkPacketDelivery);
  DEBUG_QUEUE_MONITOR_REGISTER(syslinkPacketDelivery);


  uartslkDmaInit();

  // Setting up TXEN pin (NRF flow control)
  // 향후 외부핀 PA4 연결 필요


#if defined(UARTSLK_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.Mode = UART_MODE_TX;

#else

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.Mode = UART_MODE_TX_RX;

#endif

  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;


  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

  isInit = true;
}

bool uartslkTest(void)
{
  return isInit;
}

void uartslkPauseRx(void)
{
  NVIC_DisableIRQ(UARTSLK_IRQ);
}

void uartslkResumeRx(void)
{
  rxState = waitForFirstStart;
  NVIC_EnableIRQ(UARTSLK_IRQ);
}

void uartslkEnableIncoming()
{
  syslinkPacketDeliveryReadyToReceive = true;
}

void uartslkGetPacketBlocking(SyslinkPacket* packet)
{
  xQueueReceive(syslinkPacketDelivery, packet, portMAX_DELAY);
}

void uartslkSendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
#ifdef UARTSLK_SPINLOOP_FLOWCTRL
    while(GPIO_ReadInputDataBit(UARTSLK_TXEN_PORT, UARTSLK_TXEN_PIN) == Bit_SET);
#endif
    while (!(huart1.Instance->SR & UART_FLAG_TXE));
    huart1.Instance->DR = (data[i] & 0x00FF);
  }
}

void uartslkSendDataIsrBlocking(uint32_t size, uint8_t* data)
{
  xSemaphoreTake(uartBusy, portMAX_DELAY);
  outDataIsr = data;
  dataSizeIsr = (uint8_t)size;
  dataIndexIsr = 1;
  uartslkSendData(1, &data[0]);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
  xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
  outDataIsr = 0;
  xSemaphoreGive(uartBusy);
}

int uartslkPutchar(int ch)
{
    uartslkSendData(1, (uint8_t *)&ch);

    return (unsigned char)ch;
}

void uartslkSendDataDmaBlocking(uint32_t size, uint8_t* data)
{
  if (dmaNrfFlowControlBufferFull) {
    dmaSendWhileNrfBufferFull++;
  }

  if (isUartDmaInitialized)
  {
    xSemaphoreTake(uartBusy, portMAX_DELAY);
    // Wait for DMA to be free
    while(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY);

    //Copy data in DMA buffer
    memcpy(dmaTXBuffer, data, size);
    initialDMACount = (uint16_t)size;

    if(HAL_UART_Transmit_DMA(&huart1, dmaTXBuffer, size)!=HAL_OK){
    	DEBUG_PRINT("DMA transfer failed\n");
    }
    xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
    xSemaphoreGive(uartBusy);

  }
}


#ifdef CONFIG_SYSLINK_DMA
static void uartslkReceiveDMA(uint32_t size)
{
  if (isUartDmaInitialized)
  {
    // Wait for DMA to be free
    while(HAL_DMA_GetState(&hdma_usart1_rx) != HAL_DMA_STATE_READY);

    if(HAL_UART_Receive_DMA(&huart1, dmaRXBuffer, size)!=HAL_OK){
    	DEBUG_PRINT("DMA receive failed\n");
    }


//    while(DMA_GetCmdStatus(UARTSLK_DMA_RX_STREAM) != DISABLE);
    // Reload new DMA stream by loading number of bytes to be transferred
//    UARTSLK_DMA_RX_STREAM->NDTR = size;
    // Enable the Transfer Complete interrupt
//    DMA_ITConfig(UARTSLK_DMA_RX_STREAM, DMA_IT_TC, ENABLE);
    // Enable USART DMA RX Requests
//    USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Rx, ENABLE);
    // Clear transfer complete
//    USART_ClearFlag(UARTSLK_TYPE, USART_FLAG_TC);
    // Disable USART RX interrupt
//    USART_ITConfig(UARTSLK_TYPE, USART_IT_RXNE, DISABLE);
    // Enable DMA USART RX Stream
//    DMA_Cmd(UARTSLK_DMA_RX_STREAM, ENABLE);
  }
}



/*
static void uartslkPauseDma()
{
  dmaNrfFlowControlBufferFull = true;
  dmaPausedCounter++;
  if (DMA_GetCmdStatus(UARTSLK_DMA_TX_STREAM) == ENABLE)
  {
    // Disable transfer complete interrupt
    DMA_ITConfig(UARTSLK_DMA_TX_STREAM, DMA_IT_TC, DISABLE);
    // Disable stream to pause it
    DMA_Cmd(UARTSLK_DMA_TX_STREAM, DISABLE);
    // Wait for it to be disabled
    while(DMA_GetCmdStatus(UARTSLK_DMA_TX_STREAM) != DISABLE);
    // Disable transfer complete
    DMA_ClearITPendingBit(UARTSLK_DMA_TX_STREAM, UARTSLK_DMA_TX_FLAG_TCIF);
    // Read remaining data count
    remainingDMACount = DMA_GetCurrDataCounter(UARTSLK_DMA_TX_STREAM);
    dmaIsPaused = true;
    dmaTxStreamPausedCounter++;
  }
}

static void uartslkResumeDma()
{
  dmaNrfFlowControlBufferFull = false;
  dmaResumedCounter++;
  if (dmaIsPaused)
  {
    // Update DMA counter
    DMA_SetCurrDataCounter(UARTSLK_DMA_TX_STREAM, remainingDMACount);
    // Update memory read address
    UARTSLK_DMA_TX_STREAM->M0AR = (uint32_t)&dmaTXBuffer[initialDMACount - remainingDMACount];
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UARTSLK_DMA_TX_STREAM, DMA_IT_TC, ENABLE);
    // Clear transfer complete
    USART_ClearFlag(UARTSLK_TYPE, USART_FLAG_TC);
    // Enable DMA USART TX Stream
    DMA_Cmd(UARTSLK_DMA_TX_STREAM, ENABLE);
    dmaIsPaused = false;
    dmaTxStreamResumedCounter++;
  }
}
*/

static void uartslkDmaTXIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
//  DMA_ITConfig(UARTSLK_DMA_TX_STREAM, DMA_IT_TC, DISABLE);
//  DMA_ClearITPendingBit(UARTSLK_DMA_TX_STREAM, UARTSLK_DMA_TX_FLAG_TCIF);
//  USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Tx, DISABLE);
//  DMA_Cmd(UARTSLK_DMA_TX_STREAM, DISABLE);

  remainingDMACount = 0;
  xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
}






static void uartslkDmaRXIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
//  DMA_ITConfig(UARTSLK_DMA_RX_STREAM, DMA_IT_TC, DISABLE);
//  DMA_ClearITPendingBit(UARTSLK_DMA_RX_STREAM, UARTSLK_DMA_RX_FLAG_TCIF);
//  USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Rx, DISABLE);
//  DMA_Cmd(UARTSLK_DMA_RX_STREAM, DISABLE);
  // Enable USART RX interrupt
//  USART_ITConfig(UARTSLK_TYPE, USART_IT_RXNE, ENABLE);

  for (int i = 0; i < slp.length; i++)
  {
    slp.data[i] = dmaRXBuffer[i];
    cksum[0] += dmaRXBuffer[i];
    cksum[1] += cksum[0];
  }
  // Check checksum and send to queue
  if ((cksum[0] == dmaRXBuffer[slp.length]) &&
      (cksum[1] == dmaRXBuffer[slp.length + 1]))
  {
    // Post the packet to the queue if there's room
    if (!xQueueIsQueueFullFromISR(syslinkPacketDelivery))
    {
      if (syslinkPacketDeliveryReadyToReceive)
      {
        xQueueSendFromISR(syslinkPacketDelivery, (void *)&slp, &xHigherPriorityTaskWoken);
      }
    }
    else if(!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
    {
      // Only assert if debugger is not connected
      ASSERT(0); // Queue overflow
    }
  }
  else
  { // Checksum error
    if(!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
    {
      // Only assert if debugger is not connected
      ASSERT(0);
    }
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif

void uartslkHandleDataFromISR(uint8_t c, BaseType_t * const pxHigherPriorityTaskWoken)
{
  switch (rxState)
  {
  case waitForFirstStart:
    rxState = (c == SYSLINK_START_BYTE1) ? waitForSecondStart : waitForFirstStart;
    break;
  case waitForSecondStart:
    rxState = (c == SYSLINK_START_BYTE2) ? waitForType : waitForFirstStart;
    break;
  case waitForType:
    cksum[0] = c;
    cksum[1] = c;
    slp.type = c;
    rxState = waitForLength;
    break;
  case waitForLength:
    if (c <= SYSLINK_MTU)
    {
      slp.length = c;
      cksum[0] += c;
      cksum[1] += cksum[0];
      dataIndex = 0;
#ifdef CONFIG_SYSLINK_DMA
      if (c >= 1)
      {
        rxState = waitForFirstStart;
        // For efficiency receive using DMA
        uartslkReceiveDMA(slp.length + UARTSLK_CLKSUM_SIZE);
      }
      else // zero length
      {
        rxState = waitForChksum1;
      }
#else
      rxState = (c > 0) ? waitForData : waitForChksum1;
#endif
    }
    else
    {
      rxState = waitForFirstStart;
    }
    break;
  case waitForData:
    slp.data[dataIndex] = c;
    cksum[0] += c;
    cksum[1] += cksum[0];
    dataIndex++;
    if (dataIndex == slp.length)
    {
      rxState = waitForChksum1;
    }
    break;
  case waitForChksum1:
    if (cksum[0] == c)
    {
      rxState = waitForChksum2;
    }
    else
    {
      rxState = waitForFirstStart; //Checksum error
      if(!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
      {
        // Only assert if debugger is not connected
        ASSERT(0);
      }
    }
    break;
  case waitForChksum2:
    if (cksum[1] == c)
    {
      // Post the packet to the queue if there's room
      if (!xQueueIsQueueFullFromISR(syslinkPacketDelivery))
      {
        if (syslinkPacketDeliveryReadyToReceive)
        {
          xQueueSendFromISR(syslinkPacketDelivery, (void *)&slp, pxHigherPriorityTaskWoken);
        }
      }
      else if(!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk))
      {
        // Only assert if debugger is not connected
        ASSERT(0); // Queue overflow
      }
    }
    else
    {
      rxState = waitForFirstStart; //Checksum error
      ASSERT(0);
    }
    rxState = waitForFirstStart;
    break;
  default:
    ASSERT(0);
    break;
  }
}



void uartslkIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // the following if statement replaces:
  // if (USART_GetITStatus(UARTSLK_TYPE, USART_IT_RXNE) == SET)
  // we do this check as fast as possible to minimize the chance of an overrun,
  // which occasionally cause problems and cause packet loss at high CPU usage
  if ((UARTSLK_TYPE->SR & (1<<5)) != 0) // if the RXNE interrupt has occurred
  {
    uint8_t rxDataInterrupt = (uint8_t)(UARTSLK_TYPE->DR & 0xFF);
    uartslkHandleDataFromISR(rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
  else if (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE) == SET)
  {
    if (outDataIsr && (dataIndexIsr < dataSizeIsr))
    {
//      USART_SendData(&huart1, outDataIsr[dataIndexIsr] & 0x00FF);
      UARTSLK_TYPE->DR = (outDataIsr[dataIndexIsr] & 0x00FF & (uint16_t)0x01FF);
      dataIndexIsr++;
    }
    else
    {
    	__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
//      USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, DISABLE);
      xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
    }
  }
  else
  {

    asm volatile ("" : "=m" (UARTSLK_TYPE->SR) : "r" (UARTSLK_TYPE->SR)); // force non-optimizable reads
    asm volatile ("" : "=m" (UARTSLK_TYPE->DR) : "r" (UARTSLK_TYPE->DR)); // of these two registers
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void uartslkTxenFlowctrlIsr()
{
//  EXTI_ClearFlag(UARTSLK_TXEN_EXTI);
  __HAL_GPIO_EXTI_CLEAR_FLAG(UARTSLK_TXEN_EXTI);
  if (HAL_GPIO_ReadPin(UARTSLK_TXEN_PORT, UARTSLK_TXEN_PIN) == GPIO_PIN_SET)
  {
//    uartslkPauseDma();
    ledSet(LED_GREEN_R, 1);
  }
  else
  {
//    uartslkResumeDma();
    ledSet(LED_GREEN_R, 0);
  }
}


/*
void __attribute__((used)) EXTI4_Callback(void)
{
  uartslkTxenFlowctrlIsr();
}
*/
void __attribute__((used)) USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
//  uartslkIsr();
}

void __attribute__((used)) DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  uartslkDmaTXIsr();
}

#ifdef CONFIG_SYSLINK_DMA
void __attribute__((used)) DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
//  uartslkDmaRXIsr();
}
#endif




void uartSyslinkDumpDebugProbe() {
  DEBUG_PRINT("STM dmaPausedCounter: %ld\n",dmaPausedCounter);
  DEBUG_PRINT("STM dmaTxStreamPausedCounter: %ld\n", dmaTxStreamPausedCounter);
  DEBUG_PRINT("STM dmaResumedCounter: %ld\n", dmaResumedCounter);
  DEBUG_PRINT("STM dmaTxStreamResumedCounter: %ld\n", dmaTxStreamResumedCounter);
  DEBUG_PRINT("STM dmaSendWhileNrfBufferFull: %ld\n", dmaSendWhileNrfBufferFull);
}
