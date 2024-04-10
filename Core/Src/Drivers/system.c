/*
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
 * system.c - Top level module implementation
 */
#define DEBUG_MODULE "SYS"

#include <stdbool.h>
#include <stdio.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "debug.h"
#include "version.h"
#include "config.h"
#include "led.h"
//#include "param.h"
//#include "log.h"
//#include "ledseq.h"
//#include "pm.h"

#include "system.h"
#include "platform.h"
//#include "storage.h"
//#include "configblock.h"
#include "worker.h"
#include "syslink.h"
//#include "freeRTOSdebug.h"
//#include "uart_syslink.h"
//#include "uart1.h"
//#include "uart2.h"
//#include "comm.h"
//#include "stabilizer.h"
//#include "commander.h"
#include "console.h"
//#include "usblink.h"
//#include "mem.h"
#include "crtp_mem.h"
//#include "proximity.h"
#include "watchdog.h"
#include "queuemonitor.h"
//#include "buzzer.h"
//#include "sound.h"
#include "sysload.h"
#include "estimator_kalman.h"
#include "estimator_ukf.h"
#include "estimator.h"
//#include "deck.h"
//#include "extrx.h"
//#include "app.h"
#include "static_mem.h"
//#include "peer_localization.h"
#include "cfassert.h"
//#include "i2cdev.h"
#include "autoconf.h"
//#include "vcp_esc_passthrough.h"
#if CONFIG_ENABLE_CPX
//  #include "cpxlink.h"
#endif



/* Private variable */
static bool selftestPassed;
static uint8_t dumpAssertInfo = 0;
static bool isInit;

static char nrf_version[16];
static uint8_t testLogParam;
static uint8_t doAssert;


unsigned long  time1=0;
unsigned long  time2=0;


#define BUF_SIZE 30
uint8_t usb_buf[BUF_SIZE];
uint32_t count=0;


STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

/* System wide synchronisation */
xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;

/* Private functions */
static void systemTask(void *arg);

/* Public functions */
void systemLaunch(void)
{
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
  DEBUG_PRINT("[TASK] systemTask is running!\n");

}



/* Private functions implementation */

void systemTask(void *arg)
{
  bool pass = true;

  //  InitTSysTick(96000000, 1000000U);

  uint32_t ld = SysTick->LOAD;
  time1 = DWT->CYCCNT;
//  usDelay(1000);	// 1002us
  vTaskDelay(1);	// 1ms
  time2 = DWT->CYCCNT;
  DEBUG_PRINT("delay = %d(us)\n",(uint32_t)(time2-time1)/CLOCK_PER_USEC);

#ifdef CONFIG_DEBUG_QUEUE_MONITOR
  queueMonitorInit();
#endif

#ifdef CONFIG_DEBUG_QUEUE_MONITOR

  //  uartSendDataDmaBlocking(36, (uint8_t *)" Testing UART1 DMA and it is working\n");
  //  uartSendDataDmaBlocking(36, (uint8_t *)" Testing UART1 DMA and it is working\n");
#endif

//  ICM20602_Initialization();	// 여기서 일정 시간 지연이 있어야 STM32 VCP 포트가 활성화됨 원인파악중

  passthroughInit();	// Create passthrough task

  systemInit();
  DEBUG_PRINT("System drivers are Initialized!\n");

  commInit();
  DEBUG_PRINT("System communications are Initialized!\n");

  commanderInit();
  DEBUG_PRINT("System commander are Initialized!\n");


  StateEstimatorType estimator = StateEstimatorTypeAutoSelect;

  #ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
  estimatorKalmanTaskInit();
  #endif

  #ifdef CONFIG_ESTIMATOR_UKF_ENABLE
  errorEstimatorUkfTaskInit();
  #endif

  uartslkEnableIncoming();

  memInit();
  deckInit();

  estimator = deckGetRequiredEstimator();
  stabilizerInit(estimator);

  if (deckGetRequiredLowInterferenceRadioMode() && platformConfigPhysicalLayoutAntennasAreClose())
  {
//    platformSetLowInterferenceRadioMode();
  }
//  soundInit();
  crtpMemInit();

#ifdef PROXIMITY_ENABLED
  proximityInit();
#endif

  systemRequestNRFVersion();

  //Test the modules
  DEBUG_PRINT("About to run tests in system.c.\n");
  if (systemTest() == false) {
    pass = false;
    DEBUG_PRINT("system [FAIL]\n");
  }
/*  if (configblockTest() == false) {
    pass = false;
    DEBUG_PRINT("configblock [FAIL]\n");
  }
  if (storageTest() == false) {
    pass = false;
    DEBUG_PRINT("storage [FAIL]\n");
  }
  if (commTest() == false) {
    pass = false;
    DEBUG_PRINT("comm [FAIL]\n");
  }
  if (commanderTest() == false) {
    pass = false;
    DEBUG_PRINT("commander [FAIL]\n");
  }
  if (stabilizerTest() == false) {
    pass = false;
    DEBUG_PRINT("stabilizer [FAIL]\n");
  }

  #ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
  if (estimatorKalmanTaskTest() == false) {
    pass = false;
    DEBUG_PRINT("estimatorKalmanTask [FAIL]\n");
  }
  #endif

  #ifdef CONFIG_ESTIMATOR_UKF_ENABLE
  if (errorEstimatorUkfTaskTest() == false) {
    pass = false;
    DEBUG_PRINT("estimatorUKFTask [FAIL]\n");
  }
  #endif

  if (deckTest() == false) {
    pass = false;
    DEBUG_PRINT("deck [FAIL]\n");
  }
  if (soundTest() == false) {
    pass = false;
    DEBUG_PRINT("sound [FAIL]\n");
  }
  if (memTest() == false) {
    pass = false;
    DEBUG_PRINT("mem [FAIL]\n");
  }
  if (crtpMemTest() == false) {
    pass = false;
    DEBUG_PRINT("CRTP mem [FAIL]\n");
  }
  if (watchdogNormalStartTest() == false) {
    pass = false;
    DEBUG_PRINT("watchdogNormalStart [FAIL]\n");
  }
  if (cfAssertNormalStartTest() == false) {
    pass = false;
    DEBUG_PRINT("cfAssertNormalStart [FAIL]\n");
  }
  if (peerLocalizationTest() == false) {
    pass = false;
    DEBUG_PRINT("peerLocalization [FAIL]\n");
  }

  //Start the firmware
  if(pass)
  {
    DEBUG_PRINT("Self test passed!\n");
    selftestPassed = 1;
    systemStart();
    soundSetEffect(SND_STARTUP);
    ledseqRun(&seq_alive);
    ledseqRun(&seq_testPassed);
  }
  else
  {
    selftestPassed = 0;
    if (systemTest())
    {
      while(1)
      {
        ledseqRun(&seq_testFailed);
        vTaskDelay(M2T(2000));
        // System can be forced to start by setting the param to 1 from the cfclient
        if (selftestPassed)
        {
	        DEBUG_PRINT("Start forced.\n");
          systemStart();
          break;
        }
      }
    }
    else
    {
      ledInit();
      ledSet(SYS_LED, true);
    }
  }
  DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());

  workerLoop();

  //Should never reach this point!
  while(1)
    vTaskDelay(portMAX_DELAY);


    */

}




bool systemTest()
{
  bool pass=isInit;

  pass &= ledseqTest();
  pass &= pmTest();
  pass &= workerTest();
  pass &= buzzerTest();
  return pass;
}


void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
    vTaskDelay(2);

  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}



// This must be the first module to be initialized!
void systemInit(void)
{
  if(isInit)
    return;

  canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

  usblinkInit();
  DEBUG_PRINT("[TASK] usblinkTask is running!\n");

  sysLoadInit();
  DEBUG_PRINT("sysLoadMonitorTimer is Initialized\n");

#if CONFIG_ENABLE_CPX
//  cpxlinkInit();
#endif

  /* Initialized here so that DEBUG_PRINT (buffered) can be used early */
//  debugInit();
  crtpInit();
  DEBUG_PRINT("[TASK] crtpTxTask is running!\n");
  DEBUG_PRINT("[TASK] crtpRxTask is running!\n");

  consoleInit();

  DEBUG_PRINT("%s is up and running!\n", platformConfigGetDeviceTypeName());

  if (V_PRODUCTION_RELEASE) {
    DEBUG_PRINT("Production release %s\n", V_STAG);
  } else {
    DEBUG_PRINT("Build %s:%s (%s) %s\n", V_SLOCAL_REVISION,
                V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
  }
  DEBUG_PRINT("I am 0x%08X%08X%08X and I have %dKB of flash!\n",
              *((int*)(MCU_ID_ADDRESS+8)), *((int*)(MCU_ID_ADDRESS+4)),
              *((int*)(MCU_ID_ADDRESS+0)), *((short*)(MCU_FLASH_SIZE_ADDRESS)));

//  configblockInit();			// 현재 디바이스에 EEPROM이 존재하지 않음
//  storageInit();
  workerInit();

//  ledseqInit();
  DEBUG_PRINT("[TASK] lesdeqCmdTask is running!\n");


  pmInit();
  DEBUG_PRINT("[TASK] pmTask is running!\n");

  buzzerInit();
/*  buzzerOn(1000);
  HAL_Delay(2);
  buzzerOn(2000);
  HAL_Delay(2);
  buzzerOn(3000);
  HAL_Delay(2);
  buzzerOn(2000);
  HAL_Delay(2);
  */
  buzzerOff();

  peerLocalizationInit();
  DEBUG_PRINT("peerLocalization is Initialized!\n");


#ifdef CONFIG_APP_ENABLE
  DEBUG_PRINT("-------------- Start User application -----------------\n");
  appInit();
  DEBUG_PRINT("[TASK] appTask is running!\n");
#endif

  isInit = true;
}






void systemRequestShutdown()
{
  SyslinkPacket slp;

  slp.type = SYSLINK_PM_ONOFF_SWITCHOFF;
  slp.length = 0;
  syslinkSendPacket(&slp);
}




void systemSyslinkReceive(SyslinkPacket *slp)
{
  if (slp->type == SYSLINK_SYS_NRF_VERSION)
  {
    size_t len = slp->length - 1;

    if (sizeof(nrf_version) - 1 <=  len) {
      len = sizeof(nrf_version) - 1;
    }
    memcpy(&nrf_version, &slp->data[0], len );
    DEBUG_PRINT("NRF51 version: %s\n", nrf_version);
  }
}

void systemRequestNRFVersion()
{
  SyslinkPacket slp;

  slp.type = SYSLINK_SYS_NRF_VERSION;
  slp.length = 0;
  syslinkSendPacket(&slp);
}



void vApplicationIdleHook( void )
{
  static uint32_t tickOfLatestWatchdogReset = M2T(0);

  portTickType tickCount = xTaskGetTickCount();

  if (tickCount - tickOfLatestWatchdogReset > M2T(WATCHDOG_RESET_PERIOD_MS))
  {
    tickOfLatestWatchdogReset = tickCount;
    watchdogReset();
  }

  if (dumpAssertInfo != 0) {
    printAssertSnapshotData();
    dumpAssertInfo = 0;
  }

  // Enter sleep mode. Does not work when debugging chip with SWD.
  // Currently saves about 20mA STM32F405 current consumption (~30%).
#ifndef DEBUG
  { __asm volatile ("wfi"); }
#endif
}

static void doAssertCallback(void) {
  if (doAssert) {
    ASSERT_FAILED();
  }
}

