/*! @file packet.c
 *
 *  @brief Routines to implement packet encoding and decoding for the serial port.
 *
 *  This contains the implementation of functions for implementing the "Tower to PC Protocol" 5-byte packets.
 *
 *  @date 21 Mar 2019
 *  @author 98114388, 11989668
 */
/*!
**  @addtogroup packet_module Packet module documentation
**  @{
*/

#include "packet.h"
#include "UART.h"
#include "Cpu.h"
#include "OS.h"

static uint8_t PacketPosition = 0;
static uint8_t PacketChecksum;

TPacket Packet;
const uint8_t PACKET_ACK_MASK = 0x80;

/*! @brief Generates a checksum based on the parameters and the command
 *
 *  @param command The packet command
 *  @param parameter1 The first parameter
 *  @param parameter2 The second parameter
 *  @param parameter3 The third parameter
 *  @return uint8_t - The calculated checksum
 */
uint8_t CalcChecksum(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  return command ^ parameter1 ^ parameter2 ^ parameter3;
}

bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  return UART_Init(baudRate, moduleClk);
}

bool Packet_Get(void)
{
  OS_DisableInterrupts();

  static uint8_t Position = 0;
  uint8_t uartData;
  if (!UART_InChar(&uartData))
  {
    OS_EnableInterrupts();
    return false;
  }
  switch (Position)
  {
  case 0:
    Packet_Command = uartData;
    Position++;
    OS_EnableInterrupts();
    return false;
  case 1:
    Packet_Parameter1 = uartData;
    Position++;
    OS_EnableInterrupts();
    return false;
  case 2:
    Packet_Parameter2 = uartData;
    Position++;
    OS_EnableInterrupts();
    return false;
  case 3:
    Packet_Parameter3 = uartData;
    Position++;
    OS_EnableInterrupts();
    return false;
  case 4:
    // test the checksum
    if (uartData == CalcChecksum(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3))
    {
      Position = 0;
      OS_EnableInterrupts();
      return true;
    }
    Packet_Command = Packet_Parameter1;
    Packet_Parameter1 = Packet_Parameter2;
    Packet_Parameter2 = Packet_Parameter3;
    Packet_Parameter3 = uartData;
    OS_EnableInterrupts();
    return false;
  default:
    //reset the counter
    Position = 0;
    OS_EnableInterrupts();
    return false;
  }

  OS_EnableInterrupts();
  return false;
}

bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  //If there is an error (false) in UART_OutChar, return a false

  if (!UART_OutChar(command))
    return false;
  if (!UART_OutChar(parameter1))
    return false;
  if (!UART_OutChar(parameter2))
    return false;
  if (!UART_OutChar(parameter3))
    return false;

  if (!UART_OutChar(CalcChecksum(command, parameter1, parameter2, parameter3))) // Sending the checksum
    return false;

  return true; // All sent fine
}

/*!
** @}
*/
