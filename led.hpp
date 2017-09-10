//          Copyright Emil Fresk 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include "stm32f411xe.h"

class ManageLED
{
public:
  /**
   * @brief   Inits the GPIO for the LED.
   */
  ManageLED()
  {
    /* Setup GPIO Port A, Pin 5 (LED) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;          // Enable GPIOA clock
    GPIOA->MODER |= (1 << GPIO_MODER_MODE5_Pos);  // Output
  }

  /**
   * @brief   Enables the LED.
   */
  void enable()
  {
    /* Enable GPIO Port A, Pin 5 (LED) */
    GPIOA->ODR |= (1 << GPIO_OTYPER_OT5_Pos);   // Enable LED
  }

  /**
   * @brief   Disables the LED.
   */
  void disable()
  {
    /* Disable GPIO Port A, Pin 5 (LED) */
    GPIOA->ODR &= ~(1 << GPIO_OTYPER_OT5_Pos);  // Disable LED
  }

  /**
   * @brief   Toggles the LED.
   */
  void toggle()
  {
    /* Toggle GPIO Port A, Pin 5 (LED) */
    GPIOA->ODR ^= (1 << GPIO_OTYPER_OT5_Pos);   // Toggle LED
  }
};

extern ManageLED led_resource;


