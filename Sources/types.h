/*! @file
 *
 *  @brief Declares new types.
 *
 *  This contains types that are especially useful for the Tower to PC Protocol.
 *
 *  @author PMcL
 *  @date 2015-07-23
 */

#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "analog.h"
#include "OS.h"

// Relay Characteristic
typedef enum
{
  Inverse = 0,
  VeryInverse = 1,
  ExtremelyInverse = 2,
} RELAY_CHARACTERISTIC;

// Fault Characteristic
typedef enum
{
  NoFault = 0,
  Phase1 = 1,
  Phase2 = 2,
  Phase3 = 3,
} FAULT;

enum TIMER_STATUS
{
  TIMER_INACTIVE,
  TIMER_ACTIVE
};

// Output Signals
typedef enum
{
  OUTPUT_LOW = 0,
  OUTPUT_HIGH = 1,
} OUTPUT_SIGNAL;

/*! @brief Data structure used to pass channel data to thread
 *
 */
typedef struct DORThreadData
{
  OS_ECB *sampleSemaphore;
  uint8_t channelNb;
  float samples[ANALOG_WINDOW_SIZE];
  double iRMS;
  double tripTime;
  uint32_t intervalCounter;
  uint32_t numberOfIntervals;
  enum TIMER_STATUS timerStatus;
  bool tripped;

  uint8_t crossingNb;
  double offset1;
  double offset2;
  uint8_t sampleOffset;

} TDORThreadData;

// Unions to efficiently access hi and lo parts of integers and words
typedef union
{
  int16_t l;
  struct
  {
    int8_t Lo;
    int8_t Hi;
  } s;
} int16union_t;

typedef union
{
  uint16_t l;
  struct
  {
    uint8_t Lo;
    uint8_t Hi;
  } s;
} uint16union_t;

// Union to efficiently access hi and lo parts of a long integer
typedef union
{
  uint32_t l;
  struct
  {
    uint16_t Lo;
    uint16_t Hi;
  } s;
} uint32union_t;

// Union to efficiently access each byte in a Big Endian Long
typedef union
{
  uint32_t l;
  struct
  {
    uint8_t Byte4;
    uint8_t Byte3;
    uint8_t Byte2;
    uint8_t Byte1;
  } s;
} uint32_8union_t;

// Union to efficiently access hi and lo parts of a "phrase" (8 bytes)
typedef union
{
  uint64_t l;
  struct
  {
    uint32_t Lo;
    uint32_t Hi;
  } s;
} uint64union_t;

// Union to efficiently access individual bytes of a float
typedef union
{
  float d;
  struct
  {
    uint16union_t dLo;
    uint16union_t dHi;
  } dParts;
} TFloat;

#endif
