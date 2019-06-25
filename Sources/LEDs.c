/*! @file LEDs.c
 *
 *  @brief routines for handling LED functionality
 *
 *
 *  @author 11989668, 98114388
 *  @date 2019-04-15
 */
/*!
**  @addtogroup led_module LED module documentation
**  @{
*/

#include "LEDs.h"
#include "MK70F12.h"

/*! @brief Sets up the LEDs before first use.
 *
 *  @return bool - TRUE if the LEDs were successfully initialized.
 */
bool LEDs_Init(void)
{
  PORTA_PCR10 |= PORT_PCR_MUX(1);
  PORTA_PCR11 |= PORT_PCR_MUX(1);
  PORTA_PCR28 |= PORT_PCR_MUX(1);
  PORTA_PCR29 |= PORT_PCR_MUX(1);

  //Turn off LEDs
  GPIOA_PSOR = LED_ORANGE_MASK;
  GPIOA_PSOR = LED_YELLOW_MASK;
  GPIOA_PSOR = LED_GREEN_MASK;
  GPIOA_PSOR = LED_BLUE_MASK;

  //Set LEDs as output
  GPIOA_PDDR |= LED_ORANGE_MASK;
  GPIOA_PDDR |= LED_YELLOW_MASK;
  GPIOA_PDDR |= LED_GREEN_MASK;
  GPIOA_PDDR |= LED_BLUE_MASK;

  //Turn on PortA
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;

  return true;
}

/*! @brief Turns an LED on.
 *
 *  @param color The color of the LED to turn on.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_On(const TLED color)
{
  GPIOA_PCOR = color;
}

/*! @brief Turns off an LED.
 *
 *  @param color THe color of the LED to turn off.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Off(const TLED color)
{
  GPIOA_PSOR = color;
}

/*! @brief Toggles an LED.
 *
 *  @param color THe color of the LED to toggle.
 *  @note Assumes that LEDs_Init has been called.
 */
void LEDs_Toggle(const TLED color)
{
  GPIOA_PTOR = color;
}

/*!
** @}
*/
