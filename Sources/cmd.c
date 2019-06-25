/*! @file cmd.c
 *
 *  @brief packet handling routines for sending and receiving commands
 *
 *
 *  @author 11989668
 *  @date 2019-06-20
 */
/*!
**  @addtogroup cmd_module command module documentation
**  @{
*/

#include "cmd.h"

static const uint16_t TowerNb = 0x25C4; //Last 4 digits of student number as hex, 9668 in hex is 0x25C4

const uint8_t TOWER_VERSION_HI = 6;
const uint8_t TOWER_VERSION_LO = 0;

static volatile uint16union_t *NvTowerNb;
static volatile uint16union_t *NvTowerMode;

// Characteristic in use - 0:inverse, 1:very inverse, 2:extremely inverse
uint8_t *RelayCharacteristic;

// Number of times the DOR has sent out a trip signal
uint16union_t *NumberOfTrips;

TDORThreadData DORThreadData[3];

float Frequency;
FAULT LastFault;

static uint8_t PacketCommand,
    PacketParameter1,
    PacketParameter2,
    PacketParameter3;

bool CMD_SendStartupPacket()
{
  return Packet_Put(Startup, 0, 0, 0);
}

bool CMD_SendVersionPacket()
{
  return Packet_Put(Version, 'v', TOWER_VERSION_HI, TOWER_VERSION_LO);
}

bool CMD_SendNumberPacket()
{
  return Packet_Put(Number, 1, NvTowerNb->s.Lo, NvTowerNb->s.Hi);
}

bool CMD_SendDORCurrentPacket()
{
  bool status = false;
  for (uint8_t phase = 0; phase < 3; phase++)
  {
    if (!(status = Packet_Put(DORCurrent, phase, (uint8_t)DORThreadData[phase].iRMS, (DORThreadData[phase].iRMS - (uint8_t)DORThreadData[phase].iRMS) * 100)))
      break;
  }
  return status;
}

bool CMD_SetFlashValues()
{
  if (!PMcL_Flash_AllocateVar((void *)&RelayCharacteristic, sizeof(*RelayCharacteristic))) //Allocate the flash space for characteristic type
    return false;
  if (!PMcL_Flash_AllocateVar((void *)&NumberOfTrips, sizeof(*NumberOfTrips))) //Allocate the flash space for number of times tripped
    return false;
  if (*RelayCharacteristic == 0xFFFF)          //If flash is empty, use default value
    PMcL_Flash_Write8(RelayCharacteristic, 0); // Inverse characteristic

  if (!PMcL_Flash_AllocateVar((volatile void **)&NvTowerNb, sizeof(*NvTowerNb))) //Allocate the flash space for tower number
    return false;
  if (NvTowerNb->l == 0xFFFF)
    if (!PMcL_Flash_Write16((uint16_t *)NvTowerNb, TowerNb)) //If flash is empty, use default value
      return false;

  if (!PMcL_Flash_AllocateVar((volatile void **)&NvTowerMode, sizeof(*NvTowerMode))) //Allocate the flash space for tower mode
    return false;
  if (NvTowerMode->l == 0xFFFF)
    if (!PMcL_Flash_Write16((uint16_t *)NvTowerMode, 0x0001)) //If flash is empty, use default value
      return false;

  return true;
}

bool CMD_HandleStartupPacket()
{
  if (Packet_Parameter1 != 0 || Packet_Parameter2 != 0 || Packet_Parameter3 != 0) //Check that the values are correct
    return false;

  return CMD_SendStartupPacket() && CMD_SendVersionPacket() && CMD_SendNumberPacket();
}

bool CMD_HandleVersionPacket()
{
  if (Packet_Parameter1 != 'v' || Packet_Parameter2 != 'x' || Packet_Parameter3 != 0xD) //Check that the values are correct
    return false;

  return CMD_SendVersionPacket();
}

bool CMD_HandleNumberPacket()
{
  if (Packet_Parameter1 == 1 && Packet_Parameter2 == 0 && Packet_Parameter3 == 0) //Check that the values are correct
    return CMD_SendNumberPacket();
  else if (Packet_Parameter1 == 2 && !PMcL_Flash_Write16((uint16_t *)NvTowerNb, Packet_Parameter23))
    return false;

  return true;
}

bool CMD_HandleFlashProgramPacket()
{
  if (Packet_Parameter1 == 8 && Packet_Parameter2 == 0)
    return PMcL_Flash_Erase();
  else if (Packet_Parameter1 >= 0 && Packet_Parameter1 < FLASH_SIZE && Packet_Parameter2 == 0)
    return PMcL_Flash_Write8((uint8_t *)(FLASH_DATA_START + Packet_Parameter1), Packet_Parameter3);

  return false;
}

bool CMD_HandleFlashReadPacket()
{
  if (Packet_Parameter1 >= 0 && Packet_Parameter1 < FLASH_SIZE && Packet_Parameter2 == 0 && Packet_Parameter3 == 0)
    return Packet_Put(Flash_Read, Packet_Parameter1, 0, _FB(FLASH_DATA_START + Packet_Parameter1));

  return false;
}

bool CMD_HandleDORPacket()
{
  switch (Packet_Parameter1)
  {
  case 0:
    // 010 return chartype
    if (Packet_Parameter23 == 0x10)
    {
      return Packet_Put(DOR, 0, 1, *RelayCharacteristic);
    }
    // 02[0-2] set chartype
    else if (Packet_Parameter2 == 2 && (Packet_Parameter3 >= 0 && Packet_Parameter3 <= 2))
    {
      return PMcL_Flash_Write8(RelayCharacteristic, Packet_Parameter3);
    }
    else
      return false;
  case 1:
    // 100 get currents
    if (Packet_Parameter23 == 0x00)
      return CMD_SendDORCurrentPacket();
    else
      return false;
  case 2:
    // 200 get frequency
    if (Packet_Parameter23 == 0x00)
      return Packet_Put(DOR, 2, (uint8_t)Frequency, (Frequency - (uint8_t)Frequency) * 100);
    else
      return false;
  case 3:
    // 300 get TimesTripped
    if (Packet_Parameter23 == 0x00)
      return Packet_Put(DOR, 3, NumberOfTrips->s.Lo, NumberOfTrips->s.Hi);
    else
      return false;
  case 4:
    // 400 get type of the last fault detected on the system (3-phase, 2-phase or 1-phase fault)
    if (Packet_Parameter23 == 0x00)
      return Packet_Put(DOR, 4, LastFault, 0);
    else
      return false;
  default:
    return false;
  }
}

bool CMD_PacketHandle()
{
  bool success = false;

  switch (Packet_Command & ~PACKET_ACK_MASK)
  {
  case Startup:
    success = CMD_HandleStartupPacket();
    break;
  case Version:
    success = CMD_HandleVersionPacket();
    break;
  case Number:
    success = CMD_HandleNumberPacket();
    break;
  case Flash_Program:
    success = CMD_HandleFlashProgramPacket();
    break;
  case Flash_Read:
    success = CMD_HandleFlashReadPacket();
    break;
  case DOR:
    success = CMD_HandleDORPacket();
    break;
  default:
    break;
  }
  if (Packet_Command & PACKET_ACK_MASK)
  {
    if (success)
      Packet_Put(Packet_Command, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
    else
      Packet_Put(Packet_Command & ~PACKET_ACK_MASK, Packet_Parameter1, Packet_Parameter2, Packet_Parameter3);
  }

  return success;
}

/*!
** @}
*/
