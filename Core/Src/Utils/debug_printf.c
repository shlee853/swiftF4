
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "debug.h"
#include "static_mem.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define QUEUE_LENGTH 64
xQueueHandle uartqueue;
STATIC_MEM_QUEUE_ALLOC(uartqueue, QUEUE_LENGTH, sizeof(uint8_t));

xSemaphoreHandle uartBusy;
StaticSemaphore_t uartBusyBuffer;
xSemaphoreHandle waitUntilSendDone;
StaticSemaphore_t waitUntilSendDoneBuffer;
//	static DMA_HandleTypeDef DMA_InitStructureShare;
extern DMA_HandleTypeDef hdma_usart6_tx;
static uint8_t dmaBuffer[64];
static bool    isUartDmaInitialized;
static uint32_t initialDMACount;



static bool isInit = false;
static bool hasOverrun = false;




extern UART_HandleTypeDef huart6;

//int _write(int file, char* p, int len)
int UART_PRINTF(int file, char* p, int len)
{
		if(HAL_UART_Transmit(&huart6, p, len , 10 )!= 0)
			//		while(!LL_USART_IsActiveFlag_TXE(USART6));0
			//		usDelay(100);	// 문자 1개 출력당 약 100us 소요, Float, int형 차이 없음
			return -1;
		return len;
}

/*

int _write(int32_t file, uint8_t *ptr, int32_t len) {
    for(int32_t i = 0; i < len; ++i) {
    	ITM_SendChar(*ptr++); }
    return len;
}
*/


void uartInit(void) {

#ifdef CONFIG_DEBUG_PRINT_ON_UART
	uartqueue = STATIC_MEM_QUEUE_CREATE(uartqueue);
	isInit = true;
  //  uartSendDataDmaBlocking(36, (uint8_t *)" Testing UART1 DMA and it is working\n");
#endif

}



void uartDmaInit(void)
{


#ifdef ENABLE_UART_DMA

	  // initialize the FreeRTOS structures first, to prevent null pointers in interrupts
	  waitUntilSendDone = xSemaphoreCreateBinaryStatic(&waitUntilSendDoneBuffer); // initialized as blocking
	  uartBusy = xSemaphoreCreateBinaryStatic(&uartBusyBuffer); // initialized as blocking
	  xSemaphoreGive(uartBusy); // but we give it because the uart isn't busy at initialization
	  xSemaphoreGive(waitUntilSendDone);

	  __HAL_RCC_DMA2_CLK_ENABLE();
	  HAL_DMA_Init(&hdma_usart6_tx);
	  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

	  isUartDmaInitialized = true;
#endif


}


void uartSendDataDmaBlocking(uint32_t size, uint8_t* data)
{
  if (isUartDmaInitialized)
  {
    xSemaphoreTake(uartBusy, portMAX_DELAY);
    // Wait for DMA to be free
//    while(HAL_DMA_GetState(&hdma_usart6_tx) != HAL_DMA_STATE_READY);
    //Copy data in DMA buffer
    memcpy(dmaBuffer, data, size);
    initialDMACount = size;
    if(HAL_UART_Transmit_DMA(&huart6, dmaBuffer, size)!=HAL_OK){
    	DEBUG_PRINT("DMA transfer failed\n");
    }

    xSemaphoreGive(uartBusy);
  }
}






void uartSendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  for(i = 0; i < size; i++)
  {
    while (!(USART6->SR & UART_FLAG_TXE));
    USART6->DR = (data[i] & 0x00FF);
  }
}


int uartPutchar(int ch)
{
    uartSendData(1, (uint8_t *)&ch);
    return (unsigned char)ch;
}


void uartGetchar(char * ch)
{
  xQueueReceive(uartqueue, ch, portMAX_DELAY);
}


uint32_t uartbytesAvailable()
{
  return uxQueueMessagesWaiting(uartqueue);
}

uint32_t uartQueueMaxLength()
{
  return QUEUE_LENGTH;
}

bool uartDidOverrun()
{
  bool result = hasOverrun;
  hasOverrun = false;

  return result;
}


