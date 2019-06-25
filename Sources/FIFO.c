/*! @file FIFO.c
 *
 *  @brief Routines for initialising the FIFO and using it
 *
 *
 *  @author 11989668, 98114388
 *  @date 2019-03-21
 */
/*!
**  @addtogroup fifo_module FIFO module documentation
**  @{
*/

/*! @brief Initialize the FIFO before first use.
 *
 *  @param fifo A pointer to the FIFO that needs initializing.
 *  @return bool - TRUE if the FIFO was successfully initialised
 */

#include "FIFO.h"
#include "Cpu.h"

bool FIFO_Init(TFIFO *const fifo)
{
  fifo->Start = 0;
  fifo->End = 0;
  fifo->NbBytes = 0;
  fifo->BufferAccess = OS_SemaphoreCreate(1);           //Create Semaphore (1 to indicate it is not being used)
  fifo->SpaceAvailable = OS_SemaphoreCreate(FIFO_SIZE); //Create Semaphore which maintains space available in FIFO
  fifo->ItemsAvailable = OS_SemaphoreCreate(0);         //Create Semaphore to wait on space available
}

/*! @brief Put one character into the FIFO.
 *
 *  @param fifo A pointer to a FIFO struct where data is to be stored.
 *  @param data A byte of data to store in the FIFO buffer.
 *  @return bool - TRUE if data is successfully stored in the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
bool FIFO_Put(TFIFO *const fifo, const uint8_t data)
{
  OS_SemaphoreWait(fifo->SpaceAvailable, 0); // Wait on space available
  OS_SemaphoreWait(fifo->BufferAccess, 0);   // Get exclusive access

  if (fifo->NbBytes >= FIFO_SIZE) // Can't add more bytes than the FIFO size
  {
    return false;
  }

  fifo->NbBytes++;
  fifo->Buffer[fifo->End] = data;
  fifo->End = (fifo->End + 1) % FIFO_SIZE; // Cycle to the beginning if we reached the end

  OS_SemaphoreSignal(fifo->BufferAccess);   // Signal exclusive access
  OS_SemaphoreSignal(fifo->ItemsAvailable); // Signal space available
  return true;
}

/*! @brief Get one character from the FIFO.
 *
 *  @param fifo A pointer to a FIFO struct with data to be retrieved.
 *  @param dataPtr A pointer to a memory location to place the retrieved byte.
 *  @return bool - TRUE if data is successfully retrieved from the FIFO.
 *  @note Assumes that FIFO_Init has been called.
 */
bool FIFO_Get(TFIFO *const fifo, uint8_t *const dataPtr)
{
  OS_SemaphoreWait(fifo->ItemsAvailable, 0); //Wait on items available
  OS_SemaphoreWait(fifo->BufferAccess, 0);   //Wait on exclusive access

  if (fifo->NbBytes == 0) // Can't get bytes from empty FIFO
  {
    return false;
  }

  *dataPtr = fifo->Buffer[fifo->Start];
  fifo->Start = (fifo->Start + 1) % FIFO_SIZE; // Cycle to the end if we reached the beginning
  fifo->NbBytes--;

  OS_SemaphoreSignal(fifo->BufferAccess);   //Signal no exclusive access taking place
  OS_SemaphoreSignal(fifo->SpaceAvailable); //Signal space available in FIFO
  return true;
}

/*!
** @}
*/
