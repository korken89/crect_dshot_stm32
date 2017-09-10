//          Copyright Emil Fresk 2017.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE.md or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include <array>
#include <cstdint>
#include "stm32f411xe.h"

class TimerDshot
{
private:
  static const constexpr unsigned TIM_RATE = 100000000;
  //unsigned bit_0_;
  //unsigned bit_1_;

  std::array<uint32_t, 17> payload_;
  bool request_telemetry_ = false;

public:
  uint16_t prepareDshotPacket(const uint16_t value)
  {
    uint16_t packet = (value << 1) | (request_telemetry_ ? 1 : 0);
    request_telemetry_ = false;

    // compute checksum
    uint16_t csum = 0;
    uint16_t csum_data = packet;

    for (int i = 0; i < 3; i++)
    {
        csum ^= csum_data;
        csum_data >>= 4;
    }

    packet = (packet << 4) | (csum & 0xf);

    return packet;
  }

  void set_throttle(const uint16_t value)
  {
    constexpr const auto bit_period_ = (TIM_RATE + 1200000/2) / 1200000;
    constexpr const auto bit_0_ = (bit_period_ * 1 + 1) / 3 - 1;
    constexpr const auto bit_1_ = (bit_period_ * 3 + 2) / 4 - 1;

    uint16_t packet = prepareDshotPacket(value);

    for (auto i = 0; i < 16; i++)
    {
      // Dshot is MSB first
      payload_[i] = (packet & 0x8000) ? bit_1_ : bit_0_;
      packet <<= 1;
    }
  }

  void send_payload()
  {
    // Reset DMA payload size, flags and enable it
    DMA2_Stream5->NDTR = payload_.size();
    DMA2->HIFCR = DMA2->HISR;
    DMA2_Stream5->CR |= DMA_SxCR_EN;

    // Reset and update Timer registers
    TIM1->EGR |= TIM_EGR_UG;
  }

  void request_telemetry()
  {
    request_telemetry_ = true;
  }

  void init(unsigned bitrate)
  {
    //auto bit_period_ = (TIM_RATE + bitrate/2) / bitrate;
    //bit_0_ = (bit_period_ * 1 + 1) / 3 - 1;
    //bit_1_ = (bit_period_ * 3 + 2) / 3 - 1;

    // Set the payload end
    payload_[16] = 0;

    //
    // Setup GPIO Port A, Pin 8, AF01 (Pulse output)
    //
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;                // Enable GPIOA clock
    GPIOA->AFR[1] |= (1 << GPIO_AFRH_AFSEL8_Pos);       // AF01
    GPIOA->MODER |= (2 << GPIO_MODER_MODE8_Pos);        // Alternate function
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEED8_Pos);  // Fast mode


    //
    // Setup DMA
    //

    // DMA2 clock enable
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Configure DMA2 Stream5 CR register
    // Set CHSEL bits according to DMA Channel 6
    // Set DIR bits according to Memory to peripheral direction
    // Set PINC bit according to DMA Peripheral Increment Disable
    // Set MINC bit according to DMA Memory Increment Enable
    // Set PSIZE bits according to Peripheral DataSize Word
    // Set MSIZE bits according to Memory DataSize Word
    // Set CIRC bit according to disable circular mode
    // Set PL bits according to very high priority
    // Set MBURST bits according to single memory burst
    // Set PBURST bits according to single peripheral burst
    DMA2_Stream5->CR = (6 << DMA_SxCR_CHSEL_Pos) |
                       (1 << DMA_SxCR_DIR_Pos) |
                       (1 << DMA_SxCR_MINC_Pos) |
                       (0 << DMA_SxCR_PINC_Pos) |
                       (2 << DMA_SxCR_MSIZE_Pos) |
                       (2 << DMA_SxCR_PSIZE_Pos) |
                       (3 << DMA_SxCR_PL_Pos);

    // Write Timer DMAR address
    DMA2_Stream5->PAR = reinterpret_cast<uintptr_t>(&TIM1->DMAR);

    // Set the address to the memory buffer
    DMA2_Stream5->M0AR = reinterpret_cast<uintptr_t>(payload_.data());

    //
    // Setup timer PWM mode
    //

    // Enable clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Reset settings
    TIM1->CR1 = 0;
    TIM1->CR2 = 0;

    // Configure the period
    constexpr const auto bit_period_ = (TIM_RATE + 1200000/2) / 1200000;
    TIM1->ARR = bit_period_ - 1;

    // Configure the Timer prescaler
    const auto div = 1200000 / bitrate - 1;
    TIM1->PSC = div;

    // Configure pulse width
    TIM1->CCR1 = 0;

    // Enable auto-reload Preload
    TIM1->CR1 |= TIM_CR1_ARPE;

    //
    // Set PWM mode
    //

    // Select the output compare mode 1
    // Enable output compare 1 Preload
    TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) |
                  (1 << TIM_CCMR1_OC1PE_Pos);

    // Enable the TIM1 Main Output
    TIM1->BDTR = TIM_BDTR_MOE;

    // Enable CC1 output
    TIM1->CCER = TIM_CCER_CC1E;

    //
    // Setup Timer DMA settings
    //

    // Configure of the DMA Base register to CCR1 and the DMA Burst Length to 1
    TIM1->DCR = (0 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);

    // TIM1 DMA Update enable
    TIM1->DIER |= TIM_DIER_UDE;

    // Enable the TIM Counter
    TIM1->CR1 |= TIM_CR1_CEN;
  }
};
