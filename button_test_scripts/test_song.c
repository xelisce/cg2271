/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file    RTOS_Buzzer_Project.c
 * @brief   RTOS project with button interrupt, mutex, and buzzer task
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

/* Buzzer pin - PTA12 uses TPM1_CH0 */
#define BUZZER_PIN 12
#define BUZZER_PORT PORTA
#define BUZZER_TIMER_IRQ TPM1_IRQn
#define BUZZER_TIMER TPM1
#define BUZZER_TIMER_CHANNEL 0
#define BUZZER_PORT_CLOCK_GATING_MASK SIM_SCGC5_PORTA_MASK
#define BUZZER_TIMER_CLOCK_GATING_MASK SIM_SCGC6_TPM1_MASK

/* Button pin - PTA13 */
#define BUTTON_PIN 13
#define BUTTON_PORT PORTA
#define BUTTON_GPIO GPIOA
#define BUTTON_PORT_IRQ PORTA_IRQn
#define BUTTON_PORT_CLOCK_GATING_MASK SIM_SCGC5_PORTA_MASK

/* Note definitions (frequencies in Hz) */
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_REST 0

/* Note duration definitions (in milliseconds) */
#define TEMPO 200 // in bpm
#define QUARTER 60000 / TEMPO
#define WHOLE QUARTER * 4
#define HALF QUARTER * 2
#define EIGHTH QUARTER / 2
#define SIXTEENTH QUARTER / 4
#define DEFAULT_NOTE_DELAY_MS 10 // delay between each note

/* "Twinkle Twinkle Little Star" melody */
const uint16_t melody[] = {
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4,
    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
    NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
    NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
    NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4};

const uint16_t durations[] = {
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF,
    QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, QUARTER, HALF};

const uint8_t melody_length = sizeof(melody) / sizeof(melody[0]);

/* RTOS Handles */
SemaphoreHandle_t buzzerSemaphore;
SemaphoreHandle_t resourceMutex; // Shared mutex for resource allocation

/* Resource allocation states */
typedef enum
{
  NOTHING,
  UV_LIGHTING,
  BUZZER_PLAYING
} ResourceState;

/* Shared variable protected by mutex - represents current resource allocation */
volatile ResourceState electricity_allocation = NOTHING;

/* ============================================================================
 * HARDWARE INITIALIZATION FUNCTIONS
 * ============================================================================ */

// Configure the MCG Internal Reference Clock
void setMCGIRClk()
{
  // Choose MCG clock source of 01 for LIRC
  // and set IRCLKEN to 1 to enable LIRC
  MCG->C1 &= ~MCG_C1_CLKS_MASK;
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
  NVIC_DisableIRQ(BUZZER_TIMER_IRQ);
  // Turn on clock gating to TPM1
  SIM->SCGC6 |= BUZZER_TIMER_CLOCK_GATING_MASK;
  // Turn on clock gating to PORTA
  SIM->SCGC5 |= BUZZER_PORT_CLOCK_GATING_MASK;
  // Set the pin multiplexor for PWM (TPM1_CH0 is Alt 3 on PTA12)
  BUZZER_PORT->PCR[BUZZER_PIN] &= ~PORT_PCR_MUX_MASK;
  BUZZER_PORT->PCR[BUZZER_PIN] |= PORT_PCR_MUX(0b011); // Alt 3 for TPM1_CH0
  // Set pin to output
  GPIOA->PDDR |= (1 << BUZZER_PIN);
  // Set up TPM1
  // Turn off TPM1 and clear the prescaler field
  BUZZER_TIMER->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
  // Set prescaler to 128 (62500Hz)
  BUZZER_TIMER->SC |= TPM_SC_PS(0x7);
  // Select edge-aligned PWM mode (not center-aligned)
  BUZZER_TIMER->SC &= ~TPM_SC_CPWMS_MASK;
  // Initialize count
  BUZZER_TIMER->CNT = 0;
  // Set MOD for 262Hz (middle C / NOTE_C4)
  // 62500Hz / 262Hz = 238
  BUZZER_TIMER->MOD = 238;
  // Configure channel 0 for edge-aligned PWM
  // MSnB = 1, MSnA = 0
  // Configure channel 0 for high-true pulses
  // ELSnB = 1, ELSnA = 0
  BUZZER_TIMER->CONTROLS[BUZZER_TIMER_CHANNEL].CnSC &= ~(TPM_CnSC_MSA(1) | TPM_CnSC_MSB(1) |
                              TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK);
  BUZZER_TIMER->CONTROLS[BUZZER_TIMER_CHANNEL].CnSC |= (TPM_CnSC_MSB(1) | TPM_CnSC_ELSB_MASK);
  // Set 50% duty cycle
  BUZZER_TIMER->CONTROLS[BUZZER_TIMER_CHANNEL].CnV = 119; // 50% of 238
  PRINTF("Buzzer PWM initialized:\r\n");
  PRINTF("  - Pin: PTA12\r\n");
  PRINTF("  - TPM: TPM1 Channel 0\r\n");
  PRINTF("  - Frequency: 262Hz (middle C)\r\n");
  PRINTF("  - TPM1->MOD: %lu\r\n", BUZZER_TIMER->MOD);
  PRINTF("  - TPM1->CONTROLS[0].CnV: %lu\r\n", BUZZER_TIMER->CONTROLS[BUZZER_TIMER_CHANNEL].CnV);
  // Enable timer interrupt
  NVIC_SetPriority(BUZZER_TIMER_IRQ, 1); // High priority (0 = highest, 192 = lowest)
  NVIC_EnableIRQ(BUZZER_TIMER_IRQ);
}

void initButton()
{
  // Disable interrupts during setup
  NVIC_DisableIRQ(BUTTON_PORT_IRQ);
  // Enable clock to PORTA
  SIM->SCGC5 |= BUTTON_PORT_CLOCK_GATING_MASK;
  // Configure PTA13 as GPIO
  BUTTON_PORT->PCR[BUTTON_PIN] &= ~PORT_PCR_MUX_MASK;
  BUTTON_PORT->PCR[BUTTON_PIN] |= PORT_PCR_MUX(0b001);
  // Set pull-up resistor
  BUTTON_PORT->PCR[BUTTON_PIN] &= ~PORT_PCR_PS_MASK;
  BUTTON_PORT->PCR[BUTTON_PIN] |= PORT_PCR_PS_MASK; // Pull-up
  BUTTON_PORT->PCR[BUTTON_PIN] &= ~PORT_PCR_PE_MASK;
  BUTTON_PORT->PCR[BUTTON_PIN] |= PORT_PCR_PE_MASK; // Pull enable
  // Set as input
  BUTTON_GPIO->PDDR &= ~(1 << BUTTON_PIN);
  // Configure interrupt on falling edge (button press)
  BUTTON_PORT->PCR[BUTTON_PIN] &= ~PORT_PCR_IRQC_MASK;
  BUTTON_PORT->PCR[BUTTON_PIN] |= PORT_PCR_IRQC(0b1010);
  // Enable PORTA interrupt
  NVIC_SetPriority(BUTTON_PORT_IRQ, 128); // Medium priority (0 = highest, 192 = lowest)
  NVIC_ClearPendingIRQ(BUTTON_PORT_IRQ);
  NVIC_EnableIRQ(BUTTON_PORT_IRQ);
  // Feedback
  PRINTF("Button initialized on PTA13\r\n");
}

/* ============================================================================
 * BUZZER CONTROL FUNCTIONS
 * ============================================================================ */

/**
 * @brief   Start buzzer by enabling the timer
 */
void startBuzzer()
{
  BUZZER_TIMER->CNT = 0;
  BUZZER_TIMER->SC |= TPM_SC_CMOD(0b01);
}

/**
 * @brief   Stop buzzer by disabling the timer
 */
void stopBuzzer()
{
  BUZZER_TIMER->SC &= ~TPM_SC_CMOD_MASK;
}

/**
 * @brief   Set buzzer frequency
 * @param   frequency - Frequency in Hz
 */
void setBuzzerFrequency(uint16_t frequency)
{
  // Turn off timer before doing anything
  stopBuzzer();
  // Handle 0 division error
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
  BUZZER_TIMER->CNT = 0;
  // Set new MOD value
  BUZZER_TIMER->MOD = mod_value;
  // Set 50% duty cycle
  BUZZER_TIMER->CONTROLS[BUZZER_TIMER_CHANNEL].CnV = mod_value / 2;
}

/**
 * @brief   Play a tone for a duration
 * @param   frequency - Frequency in Hz
 * @param   duration_ms - Duration in milliseconds
 */
void playTone(uint16_t frequency, uint32_t duration_ms)
{
  if (frequency == NOTE_REST)
  {
    stopBuzzer();
  }
  else
  {
    setBuzzerFrequency(frequency);
    startBuzzer();
  }

  vTaskDelay(pdMS_TO_TICKS(duration_ms));
  stopBuzzer();
  vTaskDelay(pdMS_TO_TICKS(DEFAULT_NOTE_DELAY_MS)); // Small pause between notes
}

void playSong()
{
  PRINTF("\r\nPlaying 'Twinkle Twinkle Little Star\r\n");

  for (uint8_t i = 0; i < melody_length; i++)
  {
    // Check if we still have the resource before each note
    // (In case another task needs to interrupt)
    if (electricity_allocation == BUZZER_PLAYING)
    {
      playTone(melody[i], durations[i]);
    }
    else
    {
      PRINTF("Song interrupted - resource released\r\n"); // but this shouldnt happen, the uv light should also check.
      break;
    }
  }

  PRINTF("Song complete!\r\n\r\n");
}

/* ============================================================================
 * INTERRUPT SERVICE ROUTINE
 * ============================================================================ */

void PORTA_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Clear pending IRQ
  NVIC_ClearPendingIRQ(PORTA_IRQn);
  // Check if our button (PTA13) triggered the interrupt
  if (PORTA->ISFR & (1 << BUTTON_PIN))
  {
    // Clear interrupt flag
    BUTTON_PORT->ISFR |= (1 << BUTTON_PIN);
    // Give semaphore to wake up buzzer task
    xSemaphoreGiveFromISR(buzzerSemaphore, &xHigherPriorityTaskWoken);
    // Perform context switch if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  // Clear ISFR (Interrupt Status Flag Register)
  PORTA->ISFR |= (1 << BUTTON_PIN);
}

/* ============================================================================
 * RTOS TASKS
 * ============================================================================ */

/**
 * @brief   Buzzer task - waits for semaphore, then plays song
 */
void buzzerTask(void *pvParameters)
{
  PRINTF("Buzzer task started - waiting for button press...\r\n");

  while (1)
  {
    // Wait for button press (blocks here until semaphore given)
    xSemaphoreTake(buzzerSemaphore, portMAX_DELAY);

    PRINTF("\r\nButton pressed! Requesting buzzer resource...\r\n");

    // Take mutex to check and modify electricity allocation
    if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE)
    {
      // Check if resource is available
      if (electricity_allocation == NOTHING)
      {
        // Allocate resource to buzzer
        electricity_allocation = BUZZER_PLAYING;
        PRINTF("Resource allocated: BUZZER_PLAYING\r\n");

        // Release mutex while playing song
        xSemaphoreGive(resourceMutex);

        // Play the song
        playSong();

        // Take mutex again to release resource
        xSemaphoreTake(resourceMutex, portMAX_DELAY);
        electricity_allocation = NOTHING;
        PRINTF("Resource released: NOTHING\r\n");
        xSemaphoreGive(resourceMutex);
      }
      else if (electricity_allocation == UV_LIGHTING)
      {
        PRINTF("Resource unavailable: UV_LIGHTING is active\r\n");
        xSemaphoreGive(resourceMutex); // if we want to have it queued up then we don't give up the mutex here
      }
      else
      {
        PRINTF("Resource unavailable: BUZZER already playing\r\n");
        xSemaphoreGive(resourceMutex);
      }
    }

    PRINTF("Ready for next button press...\r\n");
    // Task will block again at the top of the loop
  }
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

void createBuzzerSemaphore() {
  // Create semaphore (binary semaphore for signaling)
  buzzerSemaphore = xSemaphoreCreateBinary();
  if (buzzerSemaphore == NULL)
  {
    PRINTF("ERROR: Failed to create buzzer semaphore!\r\n");
    while (1)
      ;
  }
  PRINTF("Buzzer semaphore created\r\n");
}

void createResourceMutex() {
  // Create mutex for resource allocation (shared with UV light task)
  resourceMutex = xSemaphoreCreateMutex();
  if (resourceMutex == NULL)
  {
    PRINTF("ERROR: Failed to create resource mutex!\r\n");
    while (1)
    ;
  }
  PRINTF("Resource mutex created\r\n");
}


void createBuzzerTask() {
  if (xTaskCreate(buzzerTask,
                  "BuzzerTask",
                  configMINIMAL_STACK_SIZE + 200,
                  NULL,
                  tskIDLE_PRIORITY + 2,
                  NULL) != pdPASS)
  {
    PRINTF("ERROR: Failed to create buzzer task!\r\n");
    while (1)
      ;
  }
  PRINTF("Buzzer task created\r\n");
}

int main(void)
{
  /* Init board hardware */
  BOARD_InitBootPins();
  BOARD_InitBootClocks();
  BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
  BOARD_InitDebugConsole();
#endif

  PRINTF("\r\n========================================\r\n");
  PRINTF("  RTOS BUZZER PROJECT\r\n");
  PRINTF("========================================\r\n");
  PRINTF("Song: Twinkle Twinkle Little Star\r\n");
  PRINTF("\r\n");
  PRINTF("Resource States:\r\n");
  PRINTF("  - NOTHING: No resource in use\r\n");
  PRINTF("  - UV_LIGHTING: UV light active\r\n");
  PRINTF("  - BUZZER_PLAYING: Buzzer active\r\n");
  PRINTF("========================================\r\n\r\n");

  // Initialize hardware
  setTPMClock();
  initBuzzer();
  initButton();

  // Initialise sempaphores and mutexes
  createBuzzerSemaphore();
  createResourceMutex();

  // Initialise tasks
  createBuzzerTask();

  // Start the scheduler
  PRINTF("\r\nStarting FreeRTOS scheduler...\r\n");
  vTaskStartScheduler();

  // Print instructions
  PRINTF("Press button on PTA13 to play the song!\r\n\r\n");

  // Should never reach here
  PRINTF("ERROR: Scheduler failed to start!\r\n");
  while (1)
  {
    __asm volatile("nop");
  }

  return 0;
}
