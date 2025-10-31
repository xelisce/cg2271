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
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* LED pin numbers */
#define RED_PIN 31  // PTE31
#define GREEN_PIN 5 // PTD5
#define BLUE_PIN 29 // PTE29

/* Button pin number */
#define BUTTON_PIN 13 // PTA13

/* Buzzer pin number */
#define BUZZER_PIN 2 // PTD2 (D9)

typedef enum tl
{
  RED,
  GREEN,
  BLUE
} TLED;

/* Electricity allocation states */
typedef enum
{
  NOTHING,
  UV_LIGHTING,
  BUZZER_PLAYING
} ElectricityState;

/* Mutex and shared variable */
SemaphoreHandle_t xMutex = NULL;
volatile ElectricityState electricity_allocation = NOTHING; // Shared variable protected by mutex

/* Task notification handle for buzzer task */
TaskHandle_t xBuzzerTaskHandle = NULL;

/**
 * @brief   Initialize GPIO for LEDs
 */
void initLEDs()
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
void onLED(TLED led)
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
void offLED(TLED led)
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
 * @brief   Toggle LED
 */
void toggleLED(TLED led)
{
  switch (led)
  {
  case RED:
    GPIOE->PTOR |= (1 << RED_PIN);
    break;

  case GREEN:
    GPIOD->PTOR |= (1 << GREEN_PIN);
    break;

  case BLUE:
    GPIOE->PTOR |= (1 << BLUE_PIN);
    break;
  }
}

/**
 * @brief   Initialize buzzer PWM on PTD2 using TPM0 Channel 2
 */
void initBuzzer()
{
  // Turn on clock gating to TPM0
  SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;

  // Turn on clock gating to PORTD
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;

  // Set the pin multiplexor for PWM (TPM0_CH2 is Alt 4 on PTD2)
  PORTD->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTD->PCR[BUZZER_PIN] |= PORT_PCR_MUX(0b100);

  // Set pin to output
  GPIOD->PDDR |= (1 << BUZZER_PIN);

  // Set up TPM0
  // Turn off TPM0 and clear the prescaler field
  TPM0->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

  // Set prescaler to 1 (no division) for higher resolution
  TPM0->SC |= TPM_SC_PS(0x0);

  // Select edge-aligned PWM mode (NOT centre-aligned for buzzer)
  TPM0->SC &= ~TPM_SC_CPWMS_MASK;

  // Initialize count
  TPM0->CNT = 0;

  // Set MOD for middle C (262Hz) initially
  // Assuming 48MHz bus clock: 48MHz / 262Hz = 183206
  TPM0->MOD = 183206;

  // Configure the MSB:MSA and ELSB:ELSA bits for channel 2
  // Edge-aligned PWM, high-true pulses
  TPM0->CONTROLS[2].CnSC &= ~(TPM_CnSC_MSA_MASK | TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK);
  TPM0->CONTROLS[2].CnSC |= (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK);

  // Set 50% duty cycle
  TPM0->CONTROLS[2].CnV = 91603;
}

/**
 * @brief   Start PWM for buzzer
 */
void startBuzzerPWM()
{
  TPM0->SC |= TPM_SC_CMOD(0b1);
}

/**
 * @brief   Stop PWM for buzzer
 */
void stopBuzzerPWM()
{
  TPM0->SC &= ~TPM_SC_CMOD_MASK;
}

/**
 * @brief   Play a tone at specified frequency
 * @param   frequency - Frequency in Hz (e.g., 262 for C4, 440 for A4)
 */
void playTone(uint16_t frequency)
{
  if (frequency == 0)
  {
    stopBuzzerPWM();
    return;
  }

  // Calculate MOD value for desired frequency
  // Assuming 48MHz bus clock with prescaler of 1
  uint32_t mod_value = 48000000UL / frequency;

  // Ensure MOD is within valid range
  if (mod_value > 65535)
  {
    // Frequency too low, use minimum MOD
    mod_value = 65535;
  }

  TPM0->MOD = mod_value;
  TPM0->CONTROLS[2].CnV = mod_value / 2; // 50% duty cycle

  startBuzzerPWM();
}

/**
 * @brief   Stop buzzer
 */
void stopBuzzer()
{
  stopBuzzerPWM();
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
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Clear pending IRQ
  NVIC_ClearPendingIRQ(PORTA_IRQn);

  // Check that button (PTA13) was triggered
  if (PORTA->ISFR & (1 << BUTTON_PIN))
  {
    PRINTF("Button pressed - Notifying buzzer task\r\n");

    // Notify the buzzer task to wake up and play song
    vTaskNotifyGiveFromISR(xBuzzerTaskHandle, &xHigherPriorityTaskWoken);

    // Toggle RED LED for visual feedback
    toggleLED(RED);

    // Clear ISFR (Interrupt Status Flag Register)
    PORTA->ISFR |= (1 << BUTTON_PIN);
  }

  // Request context switch if needed
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief   Buzzer task - plays a song when button is pressed
 */
static void buzzerTask(void *pvParameters)
{
  while (1)
  {
    // Block and wait for notification from button ISR
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    PRINTF("[Buzzer Task] Woke up! Attempting to get electricity allocation...\r\n");

    // Take mutex to check/modify electricity allocation
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      // Check if electricity is available (not being used by UV light)
      if (electricity_allocation == NOTHING)
      {
        // Claim electricity for buzzer
        electricity_allocation = BUZZER_PLAYING;
        xSemaphoreGive(xMutex); // Release mutex while playing

        PRINTF("[Buzzer Task] Got electricity! Playing song...\r\n");
        onLED(GREEN); // Visual indicator

        // Try multiple frequencies to test buzzer
        PRINTF("Testing 500Hz...\r\n");
        playTone(500);
        vTaskDelay(pdMS_TO_TICKS(500));

        PRINTF("Testing 1000Hz...\r\n");
        playTone(1000);
        vTaskDelay(pdMS_TO_TICKS(500));

        PRINTF("Testing 2000Hz...\r\n");
        playTone(2000);
        vTaskDelay(pdMS_TO_TICKS(500));

        PRINTF("Testing 3000Hz...\r\n");
        playTone(3000);
        vTaskDelay(pdMS_TO_TICKS(500));

        PRINTF("Testing 4000Hz...\r\n");
        playTone(4000);
        vTaskDelay(pdMS_TO_TICKS(500));

        stopBuzzer();

        PRINTF("[Buzzer Task] Song finished!\r\n");
        offLED(GREEN);

        // Take mutex again to release electricity
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
        {
          electricity_allocation = NOTHING;
          xSemaphoreGive(xMutex);
        }
      }
      else
      {
        // Electricity being used by UV light
        PRINTF("[Buzzer Task] Electricity in use by UV light, cannot play\r\n");
        xSemaphoreGive(xMutex);
      }
    }
  }
}

/**
 * @brief   Monitor task - displays current electricity allocation
 */
static void monitorTask(void *pvParameters)
{
  ElectricityState localState;

  while (1)
  {
    // Wait 2 seconds
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Take mutex
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      // Read shared variable safely
      localState = electricity_allocation;

      // Release mutex
      xSemaphoreGive(xMutex);

      // Print the state
      const char *stateStr;
      switch (localState)
      {
      case NOTHING:
        stateStr = "NOTHING";
        break;
      case UV_LIGHTING:
        stateStr = "UV_LIGHTING";
        break;
      case BUZZER_PLAYING:
        stateStr = "BUZZER_PLAYING";
        break;
      default:
        stateStr = "UNKNOWN";
      }
      PRINTF("[Monitor Task] Electricity allocation: %s\r\n", stateStr);
    }
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

  PRINTF("Button ISR and Buzzer Task Demo\r\n");
  PRINTF("Press button on PTA13 to play buzzer song\r\n\r\n");

  // Initialize GPIO and button interrupt
  initLEDs();
  initBuzzer();
  initButtonInterrupt();

  // Turn off all LEDs initially
  offLED(RED);
  offLED(GREEN);
  offLED(BLUE);

  // Create mutex
  xMutex = xSemaphoreCreateMutex();

  if (xMutex == NULL)
  {
    PRINTF("ERROR: Failed to create mutex!\r\n");
    while (1)
      ; // Halt on error
  }
  else
  {
    PRINTF("Mutex created successfully\r\n");
  }

  // Create buzzer task (higher priority)
  if (xTaskCreate(buzzerTask, "Buzzer", configMINIMAL_STACK_SIZE + 100, NULL, 3, &xBuzzerTaskHandle) != pdPASS)
  {
    PRINTF("ERROR: Failed to create buzzer task!\r\n");
    while (1)
      ; // Halt on error
  }
  else
  {
    PRINTF("Buzzer task created successfully\r\n");
  }

  // Create monitor task
  if (xTaskCreate(monitorTask, "Monitor", configMINIMAL_STACK_SIZE + 100, NULL, 2, NULL) != pdPASS)
  {
    PRINTF("ERROR: Failed to create monitor task!\r\n");
    while (1)
      ; // Halt on error
  }
  else
  {
    PRINTF("Monitor task created successfully\r\n");
  }

  PRINTF("Starting FreeRTOS scheduler...\r\n\r\n");

  // Start the scheduler
  vTaskStartScheduler();

  // Should never reach here
  PRINTF("ERROR: Scheduler failed to start!\r\n");

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