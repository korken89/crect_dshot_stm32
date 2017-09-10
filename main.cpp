//          Copyright Emil Fresk 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include <type_traits>

#include "crect/crect.hpp"
#include "led.hpp"
#include "tim_dma.hpp"

/* Shared LED object. */
ManageLED led_resource;

// DMA Timer
TimerDshot dshot;

/* Lower priority job */
void job1()
{
  using namespace std::chrono_literals;

  /* Access the LED resource through the claim following a monitor pattern. */
  crect::claim<Rled>([](auto &led){
    led.enable();
  });

  // Make Dshot count
  static uint16_t cnt = 0;
  dshot.set_throttle(cnt++);
  dshot.send_payload();
  cnt = cnt % (2048-1);

  // Disable led in 200ms
  crect::async<J2>(200ms);
}

/* Higher priority job */
void job2()
{
  using namespace std::chrono_literals;

  /* Access the LED resource through the claim following a monitor pattern. */
  crect::claim<Rled>([](auto &led){
    led.disable();
  });

  // Enable led in 200ms
  crect::async<J1>(200ms);
}

int main()
{
  // HW init
  dshot.init(150000);

  /* Initialization code */
  crect::initialize();

  /*
   * Convoluted way to blink a LED
   */
  crect::pend<J1>();


  while(1)
  {
    __WFI();
  }
}
