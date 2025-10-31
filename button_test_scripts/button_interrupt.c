/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    MCXC444_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* LED pin numbers */
#define RED_PIN 31  // PTE31
#define GREEN_PIN 5 // PTD5
#define BLUE_PIN 29 // PTE29

/* Button pin number */
#define BUTTON_PIN 13 // PTA13

typedef enum tl
{
  RED,
  GREEN,
  BLUE
} TLED;

/* Volatile flag to track LED state */
volatile bool ledOn = false;

/**
 * @brief   Initialize GPIO for LEDs
 */
void initGPIO()
{
  // Set up the clock gating for PORTD and PORTE
  SIM->SCGC5 |= (SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK);

  // Configure GREEN LED (PTD5)
  PORTD->PCR[GREEN_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[GREEN_PIN] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << GREEN_PIN);

  // Configure RED LED (PTE31)
  PORTE->PCR[RED_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[RED_PIN] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << RED_PIN);

  // Configure BLUE LED (PTE29)
  PORTE->PCR[BLUE_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[BLUE_PIN] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << BLUE_PIN);
}

/**
 * @brief   Turn on LED
 */
void ledOn_func(TLED led)
{
  switch (led)
  {
  case RED:
    GPIOE->PCOR |= (1 << RED_PIN);
    break;

  case GREEN:
    GPIOD->PCOR |= (1 << GREEN_PIN);
    break;

  case BLUE:
    GPIOE->PCOR |= (1 << BLUE_PIN);
    break;
  }
}

/**
 * @brief   Turn off LED
 */
void ledOff(TLED led)
{
  switch (led)
  {
  case RED:
    GPIOE->PSOR |= (1 << RED_PIN);
    break;

  case GREEN:
    GPIOD->PSOR |= (1 << GREEN_PIN);
    break;

  case BLUE:
    GPIOE->PSOR |= (1 << BLUE_PIN);
    break;
  }
}

/**
 * @brief   Initialize button interrupt on PTA5
 */
void initButtonInterrupt()
{
  // Disable interrupts
  NVIC_DisableIRQ(PORTA_IRQn);

  // Enable clock gating for PORTA
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;

  // Configure PTA5 as GPIO
  PORTA->PCR[BUTTON_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTA->PCR[BUTTON_PIN] |= PORT_PCR_MUX(1);

  // Set pull-up resistor
  PORTA->PCR[BUTTON_PIN] &= ~PORT_PCR_PS_MASK;
  PORTA->PCR[BUTTON_PIN] |= PORT_PCR_PS(1);
  PORTA->PCR[BUTTON_PIN] &= ~PORT_PCR_PE_MASK;
  PORTA->PCR[BUTTON_PIN] |= PORT_PCR_PE(1);

  // Set as input
  GPIOA->PDDR &= ~(1 << BUTTON_PIN);

  // Configure the interrupt for falling edge (button press)
  PORTA->PCR[BUTTON_PIN] &= ~PORT_PCR_IRQC_MASK;
  PORTA->PCR[BUTTON_PIN] |= PORT_PCR_IRQC(0b1010); // Falling edge

  // Set NVIC priority (0 = highest, 192 = lowest)
  NVIC_SetPriority(PORTA_IRQn, 128); // Medium priority

  // Clear pending interrupts and enable interrupts
  NVIC_ClearPendingIRQ(PORTA_IRQn);
  NVIC_EnableIRQ(PORTA_IRQn);
}

/**
 * @brief   PORTA interrupt handler
 */
void PORTA_IRQHandler()
{
  // Clear pending IRQ
  NVIC_ClearPendingIRQ(PORTA_IRQn);

  // Check that button (PTA5) was triggered
  if (PORTA->ISFR & (1 << BUTTON_PIN))
  {
    // Toggle LED state
    ledOn = !ledOn;

    if (ledOn)
    {
      ledOn_func(RED); // Turn RED LED ON
      PRINTF("Button pressed - LED ON\r\n");
    }
    else
    {
      ledOff(RED); // Turn RED LED OFF
      PRINTF("Button pressed - LED OFF\r\n");
    }

    // Clear ISFR (Interrupt Status Flag Register)
    PORTA->ISFR |= (1 << BUTTON_PIN);
  }
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

  PRINTF("Button ISR Demo Started\r\n");
  PRINTF("Press button on PTA5 to toggle RED LED\r\n\r\n");

  // Initialize GPIO and button interrupt
  initGPIO();
  initButtonInterrupt();

  // Turn off all LEDs initially
  ledOff(RED);
  ledOff(GREEN);
  ledOff(BLUE);

  /* Force the counter to be placed into memory. */
  volatile static int i = 0;
  /* Enter an infinite loop, just incrementing a counter. */
  while (1)
  {
    i++;
    /* 'Dummy' NOP to allow source level single stepping of
        tight while() loop */
    __asm volatile("nop");
  }
  return 0;
}
