/* ###################################################################
**     Filename    : main.c
**     Project     : Project
**     Processor   : MK70FN1M0VMJ12
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2015-07-20, 13:27, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 6.0
** @brief
**         Main module.
**         This module contains user's application code.
*/
/*!
**  @addtogroup main_module main module documentation
**  @{
*/
/* MODULE main */

// CPU module - contains low level hardware initialization routines
#include "Cpu.h"
#include "Events.h"
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "OS.h"

#include "packet.h"
#include "types.h"
#include "LEDs.h"
#include "cmd.h"
#include "PIT.h"
#include "UART.h"
#include "analog.h"

#define THREAD_STACK_SIZE 100
#define NB_ANALOG_CHANNELS 3

const uint32_t BAUD_RATE = 115200;
static uint64_t PIT_PERIOD = 1250000; // 1.25ms = 50Hz

extern float Frequency;
extern uint8_t *RelayCharacteristic;
extern uint16_t *NumberOfTrips;

// Output signals (16000 corresponds to 5V which will trip the breaker at iRMS >= 1.03)
static const int16_t TIMING_SIGNAL_LOW = 0;
static const int16_t TIMING_SIGNAL_HIGH = 16000;
static const int16_t TRIP_SIGNAL_LOW = 0;
static const int16_t TRIP_SIGNAL_HIGH = 16000;

// DOR constants
const static double iRMSThreshold = 1.03;
const static double inverseTimingThreshold = 1.00;

// Private global for toggling fault-sensitive mode
static bool SensitiveMode = false;

OS_ECB *OutputSemaphore;

// Thread declarations
static void InitThread(void *pData);
static void PacketCheckerThread(void *pData);
static void Pit0Thread(void *pData);
static void Pit1Thread(void *pData);
static void InputThread(void *pData);
static void OutputThread(void *pData);

// Helper functions
static void frequencyTracking(TDORThreadData *channelData, uint8_t count);
static void handleTrip(TDORThreadData *channelData);
static float fsqrt(float n);
static float calculateTimeOffset(float sample1, float sample2);
static float rawToVoltage(int16_t raw);
static uint16_t voltageToRaw(float voltage);
static float calculateRMS(float values[]);
static double calculateTiming(double iRMS);
static void resetDOR();
static void resetChannel(uint8_t channel);

// Stacks
OS_THREAD_STACK(InitThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(PacketCheckerThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(RxThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(TxThreadStack, THREAD_STACK_SIZE);
OS_THREAD_STACK(InputThreadStacks[NB_ANALOG_CHANNELS], THREAD_STACK_SIZE * 2);
OS_THREAD_STACK(OutputThreadStack, THREAD_STACK_SIZE);

static OUTPUT_SIGNAL TimingOutputSignal = OUTPUT_LOW;
static OUTPUT_SIGNAL TripOutputSignal = OUTPUT_LOW;

static RELAY_CHARACTERISTIC relayCharacteristic = Inverse;

/*
 * Inverse Timing values from https://www.jcalc.net/idmt-relay-trip-time-calculator
 */
static float k[3] = {
    0.14, // Inverse
    13.5, // VeryInverse
    80    // ExtremelyInverse
};

static float a[3] = {
    0.02, // Inverse
    1,    // VeryInverse
    2     // ExtremelyInverse
};

extern FAULT LastFault;
extern TDORThreadData DORThreadData[NB_ANALOG_CHANNELS];

/* @brief Thread for initialising the tower
 *
 */
static void InitThread(void *pData)
{
  for (;;)
  {
    OS_DisableInterrupts();

    Frequency = 0;
    LastFault = NoFault;

    TDORThreadData DORThreadData[NB_ANALOG_CHANNELS] =
        {
            {
                .sampleSemaphore = NULL,
                .channelNb = 0,
                .samples[0] = 0,
                .iRMS = 0.0,
                .tripTime = 0.0,
                .intervalCounter = 0,
                .numberOfIntervals = 0,
                .timerStatus = TIMER_INACTIVE,
                .tripped = false,

                .crossingNb = 1,
                .offset1 = 0,
                .offset2 = 0,
                .sampleOffset = 0,
            },
            {
                .sampleSemaphore = NULL,
                .channelNb = 1,
                .samples[0] = 0,
                .iRMS = 0.0,
                .tripTime = 0.0,
                .intervalCounter = 0,
                .numberOfIntervals = 0,
                .timerStatus = TIMER_INACTIVE,
                .tripped = false,

                .crossingNb = 1,
                .offset1 = 0,
                .offset2 = 0,
                .sampleOffset = 0,
            },
            {
                .sampleSemaphore = NULL,
                .channelNb = 2,
                .samples[0] = 0,
                .iRMS = 0.0,
                .tripTime = 0.0,
                .intervalCounter = 0,
                .numberOfIntervals = 0,
                .timerStatus = TIMER_INACTIVE,
                .tripped = false,

                .crossingNb = 1,
                .offset1 = 0,
                .offset2 = 0,
                .sampleOffset = 0,
            },
        };

    bool analogStatus = Analog_Init(CPU_BUS_CLK_HZ);
    bool packetStatus = Packet_Init(BAUD_RATE, CPU_BUS_CLK_HZ);
    bool flashStatus = PMcL_Flash_Init();
    bool ledStatus = LEDs_Init();
    bool pitStatus = PIT_Init(CPU_BUS_CLK_HZ);

    if (packetStatus && flashStatus && ledStatus && pitStatus)
      LEDs_On(LED_ORANGE);

    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
      DORThreadData[analogNb].sampleSemaphore = OS_SemaphoreCreate(0);

    PIT_Set(0, PIT_PERIOD, true); // Set Pit Channel 0 1.25ms = 50Hz every clock cycle and Enable
    PIT_Set(1, 1000000, false);   // Set Pit Channel 1 to run every 1 ms
    CMD_SetFlashValues();

    OS_EnableInterrupts();

    CMD_SendStartupPacket();
    CMD_SendVersionPacket();
    CMD_SendNumberPacket();

    // Start outputs at 0
    Analog_Put(1, TIMING_SIGNAL_LOW);
    Analog_Put(2, TRIP_SIGNAL_LOW);

    OS_ThreadDelete(OS_PRIORITY_SELF); // Thread not accessed again
  }
}

/* @brief Thread for the checking and handling packets
 *
 */
static void PacketCheckerThread(void *pData)
{
  for (;;)
  {
    if (Packet_Get())
    {
      CMD_PacketHandle();
    }
  }
}

/* @brief Thread for signalling each channel to take a sample
 *
 */
static void Pit0Thread(void *pData)
{
  for (;;)
  {
    //Wait on PIT0 Semaphore
    OS_SemaphoreWait(PIT0Semaphore, 0);

    // Signal the analog channels to take a sample
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
      OS_SemaphoreSignal(DORThreadData[analogNb].sampleSemaphore);

    OS_SemaphoreSignal(OutputSemaphore);
  }
}

/* @brief Thread that checks for any trips
 *
 */
static void Pit1Thread(void *pData)
{
  for (;;)
  {
    //Wait on PIT1 Semaphore
    OS_SemaphoreWait(PIT1Semaphore, 0);

    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    {
      // Check that a trip timer has been activated
      if (DORThreadData[analogNb].timerStatus == TIMER_ACTIVE)
      {
        if (DORThreadData[analogNb].intervalCounter >= DORThreadData[analogNb].numberOfIntervals)
        {
          DORThreadData[analogNb].tripped = true;
          DORThreadData[analogNb].intervalCounter = 0;          // Reset interval counter
          DORThreadData[analogNb].timerStatus = TIMER_INACTIVE; // 'deactivate' timer
        }
        DORThreadData[analogNb].intervalCounter++;
      }
    }
  }
}

/* @brief Thread that reads in data from the ADC
 *
 */
static void InputThread(void *pData)
{
  TDORThreadData *data = (TDORThreadData *)pData;
  uint8_t count = 0;

  for (;;)
  {
    int16_t analogInputValue;
    OS_SemaphoreWait(data->sampleSemaphore, 0);

    Analog_Get(data->channelNb, &analogInputValue);

    // Store analog sample in samples array
    data->samples[count] = rawToVoltage(analogInputValue);

    // Frequency Tracking
    frequencyTracking(data, count);

    // Filter Harmonics
    if (SensitiveMode)
    {
      // Not implemented but would ideally use FFT and then an LPF
    }

    count++;

    if (count == 16)
    {
      // Calculate iRMS
      data->iRMS = calculateRMS(data->samples);

      if (data->iRMS >= iRMSThreshold)
        handleTrip(data);
      else if (data->timerStatus == TIMER_ACTIVE)
        data->timerStatus = TIMER_INACTIVE; // Deactivate the channel

      // Reset counter position
      count = 0;
    }
  }
}

/* @brief Thread that outputs to the DAC
 *
 */
static void OutputThread(void *pData)
{
  for (;;)
  {
    OS_SemaphoreWait(OutputSemaphore, 0);

    uint8_t timingChannels = 0; // Counts the number of channels over the iRMS threshold
    uint8_t tripChannels = 0;

    // For each channel..
    for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
    {
      // If any of the channels have breached the threshold...
      if (DORThreadData[analogNb].iRMS >= iRMSThreshold)
      {
        timingChannels++;
      }

      // If any of the channels have tripped...
      if (DORThreadData[analogNb].tripped)
      {
        tripChannels++;

        if (TripOutputSignal == OUTPUT_LOW)
        {
          Analog_Put(1, voltageToRaw(5.00)); // Activate Trip Signal
          PMcL_Flash_Write16((uint16_t volatile *)NumberOfTrips, (uint16_t)*NumberOfTrips + 1);
          TripOutputSignal = OUTPUT_HIGH;
        }
      }
    }

    // If there channels with thresholds greater than 1.03
    // And the output isn't high already..
    if (timingChannels > 0 && TimingOutputSignal == OUTPUT_LOW)
    {
      Analog_Put(0, voltageToRaw(5.00)); // Set Timing output to 5V
      TimingOutputSignal = OUTPUT_HIGH;
    }
    else if (timingChannels == 0) // No channels above threshold
    {
      // If not already low..
      if (TimingOutputSignal == OUTPUT_HIGH)
      {
        Analog_Put(0, 0); // Reset Timing output
        TimingOutputSignal = OUTPUT_LOW;
      }

      // If not already low..
      if (TripOutputSignal == OUTPUT_HIGH)
      {
        Analog_Put(1, 0); // Reset Trip output
        resetDOR();       // Reset DOR
        TripOutputSignal = OUTPUT_LOW;
      }
    }

    if (tripChannels > 0)
    {
      LastFault = tripChannels;
    }
  }
}

/*!
 * @brief Calculates a new PIT period by using zero crossing and linear interpolation to find
 * the time between two zero crossings.
 *
 * @param pointer to channel target data
 * @param uint8_t - integer representing number of samples taken
 */
static void frequencyTracking(TDORThreadData *channelData, uint8_t count)
{
  // Frequency Tracking (only on channel 0)
  if (channelData->channelNb == 0)
  {
    if (count != 0)
    {
      if (channelData->samples[count] > 0 && channelData->samples[count - 1] < 0)
      {
        switch (channelData->crossingNb)
        {
        case 1:
          channelData->offset1 = calculateTimeOffset(channelData->samples[count - 1], channelData->samples[count]);
          channelData->sampleOffset = 0; // Reset sample offset
          channelData->crossingNb = 2;   // We've found the first zero crossing, find the next..
          break;
        case 2:
          channelData->offset2 = calculateTimeOffset(channelData->samples[count - 1], channelData->samples[count]);
          double new_period = (channelData->sampleOffset - channelData->offset1 + channelData->offset2) * ((float)PIT_PERIOD / 1e9); // Period of wave in s
          double frequency = (1 / (new_period));                                                                                     // Calculate frequency

          // Filter 'bad' frequencies
          if (frequency >= 47.5 && frequency <= 52.5)
          {
            Frequency = frequency;                     // Set global frequency
            PIT_PERIOD = ((1 / frequency) / 16) * 1e9; // Period in nanoseconds
            PIT_Set(0, PIT_PERIOD, true);              // Redefine PIT period and restart
          }
          channelData->crossingNb = 1;
          break;
        }
      }
    }
    // Increment sample offset
    channelData->sampleOffset++;
  }
}

/*!
 * @brief Handles the tripping of a signal when iRMS >= 1.03
 *
 * @param channelData - pointer to channel target data
 */
static void handleTrip(TDORThreadData *channelData)
{
  double time;
  // Check that the channel hasn't already 'tripped'
  if (channelData->tripped == false)
  {
    time = calculateTiming(channelData->iRMS);
    switch (channelData->timerStatus)
    {
    // Timer is currently inactive
    case TIMER_INACTIVE:
      channelData->numberOfIntervals = (time * 1000); // Determine number of intervals [ (triptime (ms) / PIT1 interval (=1ms) ]
      PIT_Enable(1, true);                            // Start PIT
      channelData->tripTime = time;                   // Update threadData trip time
      channelData->timerStatus = TIMER_ACTIVE;        // Update Timer Status
      break;

    // Timer is currently active, might need to modify the numberOfIntervals based on inverse timing mechanism
    case TIMER_ACTIVE:
      // If the new time is within a 1 second threshold, ignore it
      if (abs(time - channelData->tripTime) >= inverseTimingThreshold)
      {
        PIT_Enable(1, false);                                                                                                   // Disable PIT while modifying values Should I be disabling PIT, what about other channels?, However the PIT thread could override while performing these adjustments
        channelData->numberOfIntervals = (1 - (channelData->intervalCounter / channelData->numberOfIntervals)) * (time * 1000); //Determine number of intervals [ (triptime (ms) / PIT1 interval (=1ms) ]
        channelData->intervalCounter = 0;                                                                                       // Reset interval counter
        channelData->tripTime = time;                                                                                           // Store latest trip time in thread data
        PIT_Enable(1, true);                                                                                                    // Re-enable PIT
      }
      break;
    }
  }
}

/*!
 * @brief Uses linear interpolation to calculate time offset
 *
 * @param sample1 - the first sample
 * @param sample2 - the second sample
 * @return float - time offset in ms
 */
static float calculateTimeOffset(float sample1, float sample2)
{
  float gradient = sample2 - sample1;
  float timeOffset = ((-sample1) / gradient);
  return timeOffset;
}

/*
 * @brief srt using fast inverse square root function from https://codegolf.stackexchange.com/a/85556
 *
 * @param n - the float value to sqrt
 * @return y - the sqrt of n
 */
static float fsqrt(float n)
{
  n = 1.0f / n; // Invert so we get the sqrt of the number we actually want
  long i;
  float x, y;

  x = n * 0.5f;
  y = n;
  i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (x * y * y));

  return y;
}

/*!
 * @brief Converts analogue input into voltage in Volts
 *
 * @param raw - the analogue value to convert
 * @return float - voltage value
 */
static float rawToVoltage(int16_t raw)
{
  return ((float)raw * 20) / pow(2, 16);
}

/*!
 * @brief Converts analogue input into voltage in Volts
 *
 * @param voltage - the voltage to convert
 * @return float - voltage value
 */
static uint16_t voltageToRaw(float voltage)
{
  return (uint16_t)((voltage * pow(2, 16)) / 20);
}

/*!
 * @brief Calculates i RMS based on voltage samples
 *
 * @param values[] the array of voltage samples
 * @return float iRMS
 */
static float calculateRMS(float values[])
{
  float sumOfVoltages = 0;

  int i;
  for (i = 0; i < ANALOG_WINDOW_SIZE; i++)
  {
    sumOfVoltages += (values[i] * values[i]);
  }

  return (fsqrt(((float)sumOfVoltages) / ANALOG_WINDOW_SIZE) / 0.35);
}

/*!
 * @brief Calculates trip time based on iRMS value, formula and values from https://www.jcalc.net/idmt-relay-trip-time-calculator
 *
 * @param iRMS - The RMS value
 * @return double - trip time
 */
static double calculateTiming(double iRMS)
{
  return (k[relayCharacteristic] / (pow(iRMS, a[relayCharacteristic]) - 1));
}

/*!
 * @brief Resets DOR channels
 */
static void resetDOR()
{
  // Disable PITs while resetting
  PIT_Enable(0, false);
  PIT_Enable(1, false);

  // Restore each DOR channel data to initial state
  for (uint8_t analogNb = 0; analogNb < NB_ANALOG_CHANNELS; analogNb++)
  {
    resetChannel(analogNb);
  }

  // Re-enable PITs
  PIT_Enable(0, true);
  PIT_Enable(1, true);
}

/*!
 * @brief Reset single DOR channel
 */
static void resetChannel(uint8_t channel)
{
  DORThreadData[channel].intervalCounter = 0;
  DORThreadData[channel].iRMS = 0.0;
  DORThreadData[channel].tripTime = 0.0;
  DORThreadData[channel].intervalCounter = 0;
  DORThreadData[channel].numberOfIntervals = 0;
  DORThreadData[channel].timerStatus = TIMER_INACTIVE;
  DORThreadData[channel].tripped = false;
}

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  OS_Init(CPU_CORE_CLK_HZ, false);

  OutputSemaphore = OS_SemaphoreCreate(0);

  OS_ERROR error;

  error = OS_ThreadCreate(InitThread, NULL, &InitThreadStack[THREAD_STACK_SIZE - 1], 0);
  error = OS_ThreadCreate(RxThread, NULL, &RxThreadStack[THREAD_STACK_SIZE - 1], 1);
  error = OS_ThreadCreate(TxThread, NULL, &TxThreadStack[THREAD_STACK_SIZE - 1], 2);

  const uint8_t ANALOG_THREAD_PRIORITIES[NB_ANALOG_CHANNELS] = {3, 4, 5};

  for (uint8_t threadNb = 0; threadNb < NB_ANALOG_CHANNELS; threadNb++)
  {
    error = OS_ThreadCreate(InputThread, &DORThreadData[threadNb], &InputThreadStacks[threadNb][THREAD_STACK_SIZE * 2 - 1], ANALOG_THREAD_PRIORITIES[threadNb]);
  }

  error = OS_ThreadCreate(PacketCheckerThread, NULL, &PacketCheckerThreadStack[THREAD_STACK_SIZE - 1], 6);
  error = OS_ThreadCreate(OutputThread, NULL, &OutputThreadStack[THREAD_STACK_SIZE - 1], 7);

  OS_Start();

/*** Don't write any code pass this line, or it will be deleted during code generation. ***/
/*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
#ifdef PEX_RTOS_START
  PEX_RTOS_START(); /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
#endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for (;;)
  {
  }
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.5 [05.21]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
