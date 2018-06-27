/*! @file
 *
 *  @brief Routines for setting up and reading from the ADC.
 *
 *  This contains the functions for reading analog values from the LTC1859 ADC on the TWR-ADCDAC-LTC board.
 *  The ADC is 16-bit, and configured with a bipolar voltage range of +/- 10V.
 *  This contains the functions for writing analog values to the LTC2704 DAC on the TWR-ADCDAC-LTC board.
 *  The DAC is 16-bit, and configured with a bipolar voltage range of +/- 10V.
 *
 *  @author PMcL
 *  @date 2016-09-23
 */

#ifndef ANALOG_H
#define ANALOG_H

// new types
#include "types.h"

// Maximum number of channels
#define ANALOG_NB_INPUTS  4
#define ANALOG_NB_OUTPUTS 4

#define ANALOG_WINDOW_SIZE 16

#pragma pack(push)
#pragma pack(2)

typedef struct
{
  float value;                         /*!< The current "processed" analog value (the user updates this value). */
  float oldValue;                      /*!< The previous "processed" analog value (the user updates this value). */
  int16_t values[ANALOG_WINDOW_SIZE];  /*!< An array of sample values to create a "sliding window". */
  uint32_t time[ANALOG_WINDOW_SIZE];   /*!< Time stamp of the samples taken using OS_TimeGet(). */
  int16_t* putPtr;                     /*!< A pointer into the array of the last sample taken. */
} TAnalogInput;

#pragma pack(pop)

extern TAnalogInput Analog_Inputs[ANALOG_NB_INPUTS]; /*!< Analog input channels. */

/*! @brief Sets up the ADC before first use.
 *
 *  @param moduleClock The module clock rate in Hz.
 *  @return bool - true if the UART was successfully initialized.
 */
bool Analog_Init(const uint32_t moduleClock);

/*! @brief Gets a value from an analog input channel.
 *
 *  @param channelNb is the number of the analog input channel to get a value from.
 *  @param valuePtr A pointer to a memory location to place the analog value.
 *  @return bool - true if the analog value was acquired successfully.
 */
bool Analog_Get(const uint8_t channelNb, int16_t* const valuePtr);

/*! @brief Sends a value to an analog input channel.
 *
 *  @param channelNb is the number of the analog output channel to send the value to.
 *  @param value is the value to write to the analog channel.
 *  @return bool - true if the analog value was output successfully.
 */
bool Analog_Put(uint8_t const channelNb, int16_t const value);

#endif
