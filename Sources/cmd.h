#ifndef CMD_H
#define CMD_H

#include "types.h"
#include "packet.h"
#include "PMcL_Flash.h"

static enum Command {
  Startup = 0x04,
  Flash_Program = 0x07,
  Flash_Read = 0x08,
  Version = 0x09,
  Number = 0x0B,
  DOR = 0x70,
  DORCurrent = 0x71
} Command;

/*! @brief initialises the flash values
 *
 *  @return bool - TRUE if the initialisation was successfully
 */
bool CMD_SetFlashValues();

/*! @brief sends the startup packet to the PC
 *
 *  @return bool - TRUE if the packet was successfully sent
 */
bool CMD_SendStartupPacket();

/*! @brief sends the tower version to the PC
 *
 *  @return bool - TRUE if the packet was successfully sent
 */
bool CMD_SendVersionPacket();

/*! @brief sends the tower number to the PC
 *
 *  @return bool - TRUE if the packet was successfully sent
 */
bool CMD_SendNumberPacket();

/*! @brief sends the DOR current to the PC
 *
 *  @return bool - TRUE if the packet was successfully sent
 */
bool CMD_SendDORCurrentPacket();

/*! @brief checks the parameters and then sends the startup values
 *
 *  @return bool - TRUE if the packet was successfully handled and parameters were correct
 */
bool CMD_HandleStartupPacket();

/*! @brief checks the parameters and then sends the version
 *
 *  @return bool - TRUE if the packet was successfully handled and parameters were correct
 */
bool CMD_HandleVersionPacket();

/*! @brief checks the parameters and then may set the tower number based on received parameters
 *
 *  @return bool - TRUE if the packet was successfully handled and parameters were correct
 */
bool CMD_HandleNumberPacket();

/*! @brief polls for new packets and calls the appropriate function to handle it
 *
 *  @return bool - TRUE if the packet was successfully handled
 */
bool CMD_PacketHandle();

#endif
