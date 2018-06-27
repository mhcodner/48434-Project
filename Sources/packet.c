/*!
**  @file Packet.c
**
**  @brief Contains functions for handling packets in accordance with the Tower Communication Protocol document
**         Packet parameters are sent to and taken out of the UART module via these functions as well
**         having parameters such as checksum verified to ensure valid packets are being read
*/
/*!
**  @addtogroup main_module main module documentation
**
**  @author Michael Codner
**  @{
*/
/* MODULE packet */

#include "packet.h"
#include "UART.h"
#include "OS.h"

TPacket Packet;                       // Declaration of new packet structure as of lab 2
const uint8_t PACKET_ACK_MASK = 0x80; // Acknowledgment Bit Mask in Hex

bool Packet_Init(const uint32_t baudRate, const uint32_t moduleClk)
{
  return UART_Init(baudRate, moduleClk); // Simply send parameters along to UART_Init
}

bool Packet_Get(void)
{
  static uint8_t packet[5] = {0}; // Array to temporarily hold packet parameters before they form a full packet
  static uint8_t packetIndex = 0; // Index to the packet array

  // As long as there are parameters in the FIFO, keep taking them out
  while (UART_InChar(&packet[packetIndex]))
  {
    // Once we have all 5 elements of a packet we can begin testing for validity and using it
    if (packetIndex == 4)
    {
      uint8_t checksum = packet[0] ^packet[1] ^(packet[2] ^ packet[3]);

      if (packet[4] == checksum) // If the checksum is valid copy the packet across
      {
        Packet_Command = packet[0];
        Packet_Parameter1 = packet[1];
        Packet_Parameter2 = packet[2];
        Packet_Parameter3 = packet[3];
        Packet_Checksum = packet[4];

        // Concatenated parameters
        Packet_Parameter12 = (((uint16_t)(Packet_Parameter1 << 8)) || ((uint16_t)(Packet_Parameter2)));
        Packet_Parameter23 = (((uint16_t)(Packet_Parameter2 << 8)) || ((uint16_t)(Packet_Parameter3)));

        packetIndex = 0; // Reset packetIndex to allow a new packet to be built

        return true;
      }
      else // If the checksum fails to validate shift the packet along so it can eventually re-sync
      {
        for (uint8_t i = 0; i < 4; i++)
          packet[i] = packet[i + 1];
      }
    }
    else // Increment the index so that we can get the next byte in the packet from the FIFO
      packetIndex++;
  }

  return false; // If there is nothing in the FIFO, return false
}

bool Packet_Put(const uint8_t command, const uint8_t parameter1, const uint8_t parameter2, const uint8_t parameter3)
{
  // Creating the checksum, which is the XOR of all previous parameters
  uint8_t checksum = command ^ parameter1 ^ (parameter2 ^ parameter3);

  // Return the entire packet, one parameter at a time
  return (UART_OutChar(command) &&
          UART_OutChar(parameter1) &&
          UART_OutChar(parameter2) &&
          UART_OutChar(parameter3) &&
          UART_OutChar(checksum));
}

/* END packet */
/*!
** @}
*/
