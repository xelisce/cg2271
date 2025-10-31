/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file    MCXC444_Buzzer_PWM_Fixed.c
 * @brief   Fixed PWM buzzer test for PTA12
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* Buzzer pin - PTA12 uses TPM1_CH0 */
#define BUZZER_PIN 12 // PTA12

/**
 * @brief   Initialize buzzer PWM on PTA12 using TPM1 Channel 0
 */

// Configure the MCG Internal Reference Clock
void setMCGIRClk()
{

  MCG->C1 &= ~MCG_C1_CLKS_MASK;
  // Choose MCG clock source of 01 for LIRC
  // and set IRCLKEN to 1 to enable LIRC
  MCG->C1 |= ((MCG_C1_CLKS(0b01) | MCG_C1_IRCLKEN_MASK));

  // Set IRCS to 1 to choose 8 MHz clock
  MCG->C2 |= MCG_C2_IRCS_MASK;

  // Choose FCRDIV of 0 for divisor of 1
  MCG->SC &= ~MCG_SC_FCRDIV_MASK;
  MCG->SC |= MCG_SC_FCRDIV(0b0);

  // Choose LIRC_DIV2 of 0 for divisor of 1
  MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
  MCG->MC |= MCG_MC_LIRC_DIV2(0b0);
}

void setTPMClock()
{

  // Set MCGIRCLK
  setMCGIRClk();

  // Choose MCGIRCLK (8 MHz)
  SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
  SIM->SOPT2 |= SIM_SOPT2_TPMSRC(0b11);
}

void initBuzzer()
{
  NVIC_DisableIRQ(TPM1_IRQn);

  // Turn on clock gating to TPM1
  SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

  // Turn on clock gating to PORTA
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

  // Set the pin multiplexor for PWM (TPM1_CH0 is Alt 3 on PTA12)
  PORTA->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTA->PCR[BUZZER_PIN] |= PORT_PCR_MUX(0b011); // Alt 3 for TPM1_CH0

  // Set pin to output
  GPIOA->PDDR |= (1 << BUZZER_PIN);

  // Set up TPM1
  // Turn off TPM1 and clear the prescaler field
  TPM1->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

  // Set prescaler to 128 (62500Hz)
  TPM1->SC |= TPM_SC_PS(0x7);

  // Select edge-aligned PWM mode (not center-aligned)
  TPM1->SC &= ~TPM_SC_CPWMS_MASK;

  // Initialize count
  TPM1->CNT = 0;

  // Set MOD for 262Hz (middle C / NOTE_C4)
  // 62500Hz / 262Hz = 238
  TPM1->MOD = 238;

  // Configure channel 0 for edge-aligned PWM
  // MSnB = 1, MSnA = 0
  // Configure channel 0 for high-true pulses
  // ELSnB = 1, ELSnA = 0
  TPM1->CONTROLS[0].CnSC &= ~(TPM_CnSC_MSA(1) | TPM_CnSC_MSB(1) |
                              TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK);
  TPM1->CONTROLS[0].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSB_MASK);

  // Set 50% duty cycle
  TPM1->CONTROLS[0].CnV = 119; // 50% of 238

  PRINTF("Buzzer PWM initialized:\r\n");
  PRINTF("  - Pin: PTA12\r\n");
  PRINTF("  - TPM: TPM1 Channel 0\r\n");
  PRINTF("  - Frequency: 262Hz (middle C)\r\n");
  PRINTF("  - TPM1->MOD: %lu\r\n", TPM1->MOD);
  PRINTF("  - TPM1->CONTROLS[0].CnV: %lu\r\n", TPM1->CONTROLS[0].CnV);

  // Set priority to high
  NVIC_SetPriority(TPM1_IRQn, 0);

  NVIC_EnableIRQ(TPM1_IRQn);
}

/**
 * @brief   Start buzzer
 */
void startBuzzer()
{
  // Initialize count
  TPM1->CNT = 0;
  // Enable buzzer
  TPM1->SC |= TPM_SC_CMOD(0b01);
  PRINTF("Buzzer started (TPM1->SC = 0x%08X)\r\n", TPM1->SC);
}

/**
 * @brief   Stop buzzer
 */
void stopBuzzer()
{
  TPM1->SC &= ~TPM_SC_CMOD_MASK;
  PRINTF("Buzzer stopped\r\n");
}

/**
 * @brief   Set buzzer frequency
 * @param   frequency - Frequency in Hz
 */
void setBuzzerFrequency(uint16_t frequency)
{
  stopBuzzer();
  if (frequency == 0)
  {
    return;
  }

  // Calculate MOD value: Clock / Frequency
  // Period of timer update is 1 / 62500
  // Wanted period is 1 / frequency
  // mod value will be wanted period / period of timer
  uint32_t mod_value = 62500 / frequency;

  // Initialize count
  TPM1->CNT = 0;

  // Set new MOD value
  TPM1->MOD = mod_value;

  // Set 50% duty cycle
  TPM1->CONTROLS[0].CnV = mod_value / 2;
}

/**
 * @brief   Play a tone for a duration
 * @param   frequency - Frequency in Hz
 * @param   duration_ms - Duration in milliseconds
 */
void playTone(uint16_t frequency, uint32_t duration_ms)
{
  setBuzzerFrequency(frequency);

  startBuzzer();

  // Simple delay (adjust multiplier based on actual timing)
  for (volatile uint32_t i = 0; i < (duration_ms * 80); i++)
    ;

  stopBuzzer();
}

/*
 * @brief   Application entry point.
 */
int main(void)
{
  /* Init board hardware. */
  BOARD_InitBootPins();
  BOARD_InitBootClocks();
  BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
  /* Init FSL debug console. */
  BOARD_InitDebugConsole();
#endif

  PRINTF("\r\n=== BUZZER PWM TEST (FIXED) ===\r\n");
  PRINTF("Testing buzzer on PTA12 using TPM1_CH0\r\n\r\n");

  setTPMClock();
  // Initialize buzzer
  initBuzzer();

  // Wait 1 second
  PRINTF("Waiting 1 second...\r\n");
  for (volatile uint32_t i = 0; i < 800000; i++)
    ;

  // Test different frequencies
  PRINTF("\r\nTest 1: 262Hz (C4) for 1 second\r\n");
  playTone(262, 1000);
  for (volatile uint32_t i = 0; i < 800000; i++)
    ; // 0.5s pause

  PRINTF("Test 2: 294Hz (D4) for 1 second\r\n");
  playTone(294, 1000);
  for (volatile uint32_t i = 0; i < 800000; i++)
    ;

  PRINTF("Test 3: 330Hz (E4) for 1 second\r\n");
  playTone(330, 1000);
  for (volatile uint32_t i = 0; i < 800000; i++)
    ;

  PRINTF("Test 4: 440Hz (A4) for 1 second\r\n");
  playTone(440, 1000);

  PRINTF("\r\n=== TEST COMPLETE ===\r\n");
  PRINTF("If you heard the tones, PWM is working!\r\n");

  /* Enter an infinite loop */
  volatile static int i = 0;
  while (1)
  {
    i++;
    __asm volatile("nop");
  }
  return 0;
}
