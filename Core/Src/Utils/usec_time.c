
#include <stdbool.h>
#include "usec_time.h"
#include "cfassert.h"
#include "param.h"

#include "stm32f4xx.h"

static bool isInit = false;
static uint8_t reset = 0;
static uint32_t usecTimerHighCount;



void usecTimerReset(void)
{
  IF_DEBUG_ASSERT(isInit);

  const uint32_t zero = 0;
  __atomic_store(&usecTimerHighCount, &zero, __ATOMIC_SEQ_CST);

  TIM1->CNT = 0;
}

uint64_t usecTimestamp(void)
{
  IF_DEBUG_ASSERT(isInit);

  uint32_t high0;
  __atomic_load(&usecTimerHighCount, &high0, __ATOMIC_SEQ_CST);
  uint32_t low = TIM1->CNT;
  uint32_t high;
  __atomic_load(&usecTimerHighCount, &high, __ATOMIC_SEQ_CST);

  // There was no increment in between
  if (high == high0)
  {
    return (((uint64_t)high) << 16) + low;
  }
  // There was an increment, but we don't expect another one soon
  return (((uint64_t)high) << 16) + TIM1->CNT;
}


/**
 * Parameters for the usec timer
 * */
static void resetParamCallback(void)
{
  if (reset) {
    usecTimerReset();
    reset = 0;
  }
}

PARAM_GROUP_START(usec)

/**
 * @brief Reset the time to zero.
 * 
 * Useful for time synchronization between UAVs, if reset is send as a broadcast.
 */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, reset, &reset, &resetParamCallback)

PARAM_GROUP_STOP(usec)
