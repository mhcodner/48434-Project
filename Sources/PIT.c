/*! @file PIT.c
 *
 *  @brief routines for periodic interrupts
 *
 *
 *  @author 11989668
 *  @date 2019-06-24
 */
/*!
**  @addtogroup pit_module PIT module documentation
**  @{
*/

#include "PIT.h"
#include "MK70F12.h"

static uint32_t ModuleClock;
static uint32_t PIT0_Period;
static uint32_t PIT1_Period;

bool PIT_Init(const uint32_t moduleClk)
{
  PIT0Semaphore = OS_SemaphoreCreate(0); // Create PIT Semaphore for Channel 0
  PIT1Semaphore = OS_SemaphoreCreate(0); // Create PIT Semaphore for Channel 1

  ModuleClock = moduleClk;

  SIM_SCGC6 |= SIM_SCGC6_PIT_MASK; //Enable PIT clock

  PIT_MCR |= PIT_MCR_MDIS_MASK; // Disable PIT while enabling interrupts
  PIT_MCR |= PIT_MCR_FRZ_MASK;  // Timers are stopped in Debug Mode

  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK; // Enable PIT interrupts

  NVICICPR2 = (1 << (68 % 32)); // Clear any pending interrupts on PIT Channel 0
  NVICISER2 = (1 << (68 % 32)); // Enable PIT Channel 0 interrupts

  NVICICPR2 = (1 << (69 % 32)); // Clear any pending interrupts on PIT Channel 1
  NVICISER2 = (1 << (69 % 32)); // Enable PIT Channel 1 interrupts

  PIT_MCR &= ~PIT_MCR_MDIS_MASK; // Enable PIT timer (0 to enable)

  return true;
}

void PIT_Set(uint8_t channelNb, const uint32_t period, const bool restart)
{
  uint32_t freqHz = 1e9 / period;
  uint32_t cycleCount = ModuleClock / freqHz;
  uint32_t triggerVal = cycleCount - 1;

  if (restart)
    PIT_Enable(channelNb, false); //Disable the timer

  switch (channelNb)
  {
  case 0:
    PIT0_Period = period;
    PIT_LDVAL0 = PIT_LDVAL_TSV(triggerVal);
    break;
  case 1:
    PIT1_Period = period;
    PIT_LDVAL1 = PIT_LDVAL_TSV(triggerVal);
    break;
  }

  if (restart)
    PIT_Enable(channelNb, true); // Re-Enable the timer

  PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK; // Enable PIT interrupts
}

void PIT_Enable(uint8_t channelNb, const bool enable)
{
  switch (channelNb)
  {
  case 0:
    if (enable)
      PIT_TCTRL0 |= PIT_TCTRL_TEN_MASK; // Enable the timer
    else
      PIT_TCTRL0 &= ~PIT_TCTRL_TEN_MASK; // Disable the timer
    break;
  case 1:
    if (enable)
      PIT_TCTRL1 |= PIT_TCTRL_TEN_MASK; // Enable the timer
    else
      PIT_TCTRL1 &= ~PIT_TCTRL_TEN_MASK; // Disable the timer
    break;
  }
}

void __attribute__((interrupt)) PIT_ISR(void)
{
  OS_ISREnter();

  if (PIT_TFLG0 & PIT_TFLG_TIF_MASK)
  {
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;    // Acknowledge interrupt
    OS_SemaphoreSignal(PIT0Semaphore); // Signal PIT0 Semaphore
  }
  if (PIT_TFLG1 & PIT_TFLG_TIF_MASK)
  {
    PIT_TFLG1 |= PIT_TFLG_TIF_MASK;    // Acknowledge interrupt
    OS_SemaphoreSignal(PIT1Semaphore); // Signal PIT1 Semaphore
  }

  OS_ISRExit();
}

/*!
** @}
*/
