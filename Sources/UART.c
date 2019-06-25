/*! @file UART.c
 *
 *  @brief Routines for initialising the UART and using it
 *
 *
 *  @author 11989668, 98114388
 *  @date 2019-03-21
 */
/*!
**  @addtogroup uart_module UART module documentation
**  @{
*/

#include "UART.h"
#include "FIFO.h"
#include "MK70F12.h"
#include "Cpu.h"
#include "OS.h"
#include "types.h"

TFIFO TxFIFO, RxFIFO;
OS_ECB *RxSem, *TxSem;

bool UART_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  TxSem = OS_SemaphoreCreate(0);
  RxSem = OS_SemaphoreCreate(0);

  SIM_SCGC4 |= SIM_SCGC4_UART2_MASK; //Enabling UART2
  SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK; //Enabling PortE for pin routing
  PORTE_PCR16 = PORT_PCR_MUX(3);     //Modifying the PortE register for the alt3 multiplex option (UART2_Tx) //only = because of the w1c
  PORTE_PCR17 = PORT_PCR_MUX(3);     //Modifying the PortE register for the alt3 multiplex option (UART2_Rx)

  UART2_C2 &= ~UART_C2_RE_MASK; //Enabling the Receiver Enable bit
  UART2_C2 &= ~UART_C2_TE_MASK; //Enabling the Transmitter Enable bit

  // Setting the baud rate fine adjust
  uint8_t fine_adjust = (uint8_t)(moduleClk * 2) / (baudRate) % 32;
  UART2_C4 = (fine_adjust & 0x1F);

  // Requested baud rate setup
  uint16union_t setting;                               // Setting the unions to efficiently access high(Hi) and low(Lo) parts of integers and words
  setting.l = (uint16_t)(moduleClk / (baudRate * 16)); // Setting the baud rate which is synchronized with the module clock
  UART2_BDH |= (uint8_t)(setting.s.Hi & 0x1F);         // Buffers the high half of the new value
  UART2_BDL = (uint8_t)setting.s.Lo;                   // Reset to a nonzero value (fraction of 4/32)

  UART2_C2 |= UART_C2_RE_MASK; //Enabling the Receiver Enable bit
  UART2_C2 |= UART_C2_TE_MASK; //Enabling the Transmitter Enable bit

  //interrupts
  UART2_C2 |= UART_C2_RIE_MASK; // Enable receive interrupt
  UART2_C2 |= UART_C2_TIE_MASK; // Enable transmit interrupts

  NVICICPR1 = (1 << (49 % 32)); // Clear any pending error status sources interrupts on UART2
  NVICISER1 = (1 << (49 % 32)); // Enable error status sources interrupts from UART2

  FIFO_Init(&RxFIFO); // Initializing the RxFIFO
  FIFO_Init(&TxFIFO); // Initializing the TxFIFO

  return (setting.l != 0);
}

bool UART_InChar(uint8_t *const dataPtr)
{
  return FIFO_Get(&RxFIFO, dataPtr); //Gets a value from the Receive Buffer
}

bool UART_OutChar(const uint8_t data)
{
  OS_DisableInterrupts();

  UART2_C2 &= ~UART_C2_TIE_MASK;         //Disable the transmit interrupt
  bool status = FIFO_Put(&TxFIFO, data); //Puts a value into the Transmit Buffer
  UART2_C2 |= UART_C2_TIE_MASK;          //Enable the transmit interrupt

  OS_EnableInterrupts();
  return status;
}

void RxThread()
{
  for (;;)
  {
    OS_SemaphoreWait(RxSem, 0);
    FIFO_Put(&RxFIFO, UART2_D);

    UART2_C2 |= UART_C2_RIE_MASK;
  }
}

void TxThread()
{
  for (;;)
  {
    OS_SemaphoreWait(TxSem, 0);
    FIFO_Get(&TxFIFO, &UART2_D);

    UART2_C2 |= UART_C2_TIE_MASK;
  }
}

void __attribute__((interrupt)) UART_ISR(void)
{
  OS_ISREnter();

  if (UART2_C2 & UART_C2_RIE_MASK)
  {
    if (UART2_S1 & UART_S1_RDRF_MASK)
    {
      OS_SemaphoreSignal(RxSem);
      UART2_C2 &= ~UART_C2_RIE_MASK;
    }
  }

  if (UART2_C2 & UART_C2_TIE_MASK)
  {
    if (UART2_S1 & UART_S1_TDRE_MASK)
    {
      OS_SemaphoreSignal(TxSem);
      UART2_C2 &= ~UART_C2_TIE_MASK;
    }
  }

  OS_ISRExit();
}

/*!
** @}
*/
