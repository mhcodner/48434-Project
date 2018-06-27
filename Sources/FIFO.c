/*!
**  @file FIFO.c
**
**  @brief Contains functions for initializing and manipulating data in FIFO arrays
**         Initializes a FIFO by resetting values to 0 and also allows for the input and output
**         of data to the UART module using a 256 bit shifting array.
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Michael Codner
**  @{
*/
/* MODULE FIFO */

#include "FIFO.h"
#include "Cpu.h"
#include "OS.h"

static OS_ECB* FIFOAccess;

void FIFO_Init(TFIFO * const FIFO)
{
  FIFOAccess = OS_SemaphoreCreate(1);

  FIFO->Start     = 0;
  FIFO->End       = 0;
  FIFO->UsedBytes = OS_SemaphoreCreate(0);
  FIFO->FreeBytes = OS_SemaphoreCreate(FIFO_SIZE);
}

bool FIFO_Put(TFIFO * const FIFO, const uint8_t data)
{
  OS_SemaphoreWait(FIFOAccess, 0); // wait to get exclusive access

  OS_SemaphoreWait(FIFO->FreeBytes, 0);
  OS_SemaphoreSignal(FIFO->UsedBytes);

  FIFO->Buffer[FIFO->End] = data;
  FIFO->End++;

  if (FIFO->End >= FIFO_SIZE)
    FIFO->End = 0;

  OS_SemaphoreSignal(FIFOAccess);

  return true;
}

bool FIFO_Get(TFIFO * const FIFO, uint8_t * const dataPtr)
{
  OS_SemaphoreWait(FIFOAccess, 0); // wait to get exclusive access

  OS_SemaphoreWait(FIFO->UsedBytes, 0);
  OS_SemaphoreSignal(FIFO->FreeBytes);

  *dataPtr = FIFO->Buffer[FIFO->Start];
  FIFO->Start++;

  if (FIFO->Start >= FIFO_SIZE)
    FIFO->Start = 0;

  OS_SemaphoreSignal(FIFOAccess);

  return true;
}

/* END FIFO */
/*!
** @}
*/
