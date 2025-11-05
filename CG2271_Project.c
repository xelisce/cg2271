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

/* Settings */
#define MONITOR_ON 1 // comment this out if dont want print the resource allocation

/* Motor Pins*/
#define MOTOR_IN1_PIN 1      // PTA1 - Direction/Speed control 1
#define MOTOR_IN2_PIN 2      // PTA2 - Direction/Speed control 2
#define MOTOR_PORT PORTA
#define MOTOR_GPIO GPIOA
#define MOTOR_PORT_CLOCK_GATING_MASK SIM_SCGC5_PORTA_MASK

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

/* Flame pin - PTB18 */
#define FLAME_PIN 18
#define FLAME_PORT PORTB
#define FLAME_GPIO GPIOB
#define FLAME_PORT_IRQ PORTB_IRQn
#define FLAME_PORT_CLOCK_GATING_MASK SIM_SCGC5_PORTB_MASK

// Soil Moisture pins
#define ADC_PIN 20 // PTE20
#define UART_TX_PTE22 22
#define UART_RX_PTE23 23
#define UART2_INT_PRIO 128
#define SOIL_BAUD 9600
#define ADC_CHANNEL 0
#define MAX_MSG_LEN 2 // send only 1 char + null terminator

#define BAD_MOISTURE 3000    // moisture > 3000 means too dry
#define MEDIUM_MOISTURE 2000 // 2000 < Moisture < 3000 means decent moisture

/* Monitor delay */
#define MONITOR_DELAY_MS 3000 // It will print every 3 seconds

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
#define TEMPO 1000 // in bpm
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

// global varibales
char send_buffer[MAX_MSG_LEN];

/* RTOS Handles */
SemaphoreHandle_t buzzerSemaphore; // Released when the button is pressed
SemaphoreHandle_t resourceMutex; // Shared mutex for resource allocation, prevent conflicting access to electricity allocation
SemaphoreHandle_t flameSemaphore; // Released when there is a fire

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

void initFlame()
{
  // Disable interrupts during setup
  NVIC_DisableIRQ(FLAME_PORT_IRQ);
  // Enable clock to PORTB
  SIM->SCGC5 |= FLAME_PORT_CLOCK_GATING_MASK;
  // Configure PTB18 as GPIO
  FLAME_PORT->PCR[FLAME_PIN] &= ~PORT_PCR_MUX_MASK;
  FLAME_PORT->PCR[FLAME_PIN] |= PORT_PCR_MUX(0b001);
  // Set pull-up resistor
  FLAME_PORT->PCR[FLAME_PIN] &= ~PORT_PCR_PS_MASK;
  FLAME_PORT->PCR[FLAME_PIN] |= PORT_PCR_PS_MASK; // Pull-up
  FLAME_PORT->PCR[FLAME_PIN] &= ~PORT_PCR_PE_MASK;
  FLAME_PORT->PCR[FLAME_PIN] |= PORT_PCR_PE_MASK; // Pull enable
  // Set as input
  FLAME_GPIO->PDDR &= ~(1 << FLAME_PIN);
  // Configure interrupt on either edge (flame change, whether present to not or other way around)
  FLAME_PORT->PCR[FLAME_PIN] &= ~PORT_PCR_IRQC_MASK;
  FLAME_PORT->PCR[FLAME_PIN] |= PORT_PCR_IRQC(0b1011);
  // Enable PORTB interrupt
  NVIC_SetPriority(FLAME_PORT_IRQ, 128); // Medium priority (0 = highest, 192 = lowest)
  NVIC_ClearPendingIRQ(FLAME_PORT_IRQ);
  NVIC_EnableIRQ(FLAME_PORT_IRQ);
  // Feedback
  PRINTF("Flame sensor initialized on PTB18\r\n");
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

void startADC(int channel)
{
  PRINTF("Started ADC on channel %d. Waiting\r\n", channel);

  ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
  ADC0->SC1[0] |= ADC_SC1_ADCH(channel);
}

// INitialize ADC and UART for Soil Moisture Sensor
void initSoilADC()
{
  NVIC_DisableIRQ(ADC0_IRQn);
  NVIC_ClearPendingIRQ(ADC0_IRQn);

  SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

  // Set pin for ADC input (e.g., PTE20 → ADC0_SE0)
  PORTE->PCR[ADC_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[ADC_PIN] |= PORT_PCR_MUX(0); // ALT0

  // Disable ADC interrupt since polling
  ADC0->SC1[0] &= ~ADC_SC1_AIEN_MASK;

  // Select Single ended mode ADC for PTE20 and disable diff mode
  ADC0->SC1[0] &= ~ADC_SC1_DIFF_MASK;
  ADC0->SC1[0] |= ADC_SC1_DIFF(0b0);

  // Set 12 bit conversion
  ADC0->CFG1 &= ~ADC_CFG1_MODE_MASK;
  ADC0->CFG1 |= ADC_CFG1_MODE(0b01); // can try 0b00 for 8 bit conversion

  // Use software trigger
  ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;

  // Use VALTH and VALTL
  ADC0->SC2 &= ~ADC_SC2_REFSEL_MASK;
  ADC0->SC2 |= ADC_SC2_REFSEL(0b01);

  // Don't use averaging
  ADC0->SC3 &= ~ADC_SC3_AVGE_MASK;
  ADC0->SC3 |= ADC_SC3_AVGE(0);

  // Use continuous conversion
  ADC0->SC3 &= ~ADC_SC3_ADCO_MASK;
  ADC0->SC3 |= ADC_SC3_ADCO(0); //                       /* TX only */

  PRINTF("ADC polling mode initialized on PTE20, Channel 0\r\n");
}

void initUART2(uint32_t baud_rate)
{
  NVIC_DisableIRQ(UART2_FLEXIO_IRQn);

  SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

  UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

  PORTE->PCR[UART_TX_PTE22] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[UART_TX_PTE22] |= PORT_PCR_MUX(4);
  PORTE->PCR[UART_RX_PTE23] &= ~PORT_PCR_MUX_MASK;
  PORTE->PCR[UART_RX_PTE23] |= PORT_PCR_MUX(4);

  uint32_t bus_clk = CLOCK_GetBusClkFreq();
  uint32_t sbr = (bus_clk + (baud_rate * 8)) / (baud_rate * 16);

  UART2->BDH &= ~UART_BDH_SBR_MASK;
  UART2->BDH |= ((sbr >> 8) & UART_BDH_SBR_MASK);
  UART2->BDL = (uint8_t)(sbr & 0xFF);

  UART2->C1 = 0x00;             // 8-bit, no parity
  UART2->C2 |= UART_C2_RE_MASK; // enable receiver (for safety)

  NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
  NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
  NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
}

void initMotor()
{
  // Turn on clock gating to PORTA
  SIM->SCGC5 |= MOTOR_PORT_CLOCK_GATING_MASK;
  
  // Configure IN1 and IN2 as GPIO outputs
  MOTOR_PORT->PCR[MOTOR_IN1_PIN] &= ~PORT_PCR_MUX_MASK;
  MOTOR_PORT->PCR[MOTOR_IN1_PIN] |= PORT_PCR_MUX(0b001); // GPIO
  MOTOR_PORT->PCR[MOTOR_IN2_PIN] &= ~PORT_PCR_MUX_MASK;
  MOTOR_PORT->PCR[MOTOR_IN2_PIN] |= PORT_PCR_MUX(0b001); // GPIO
  
  // Set IN1 and IN2 as outputs
  MOTOR_GPIO->PDDR |= (1 << MOTOR_IN1_PIN);
  MOTOR_GPIO->PDDR |= (1 << MOTOR_IN2_PIN);
  
  // Initialize both to LOW (motor stopped)
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN1_PIN);
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN2_PIN);
  
  PRINTF("Motor driver (DRV8833) initialized:\r\n");
  PRINTF("  - IN1: PTA%d\r\n", MOTOR_IN1_PIN);
  PRINTF("  - IN2: PTA%d\r\n", MOTOR_IN2_PIN);
  PRINTF("  - Control: Direct H-bridge (no EN pin)\r\n");
}

/* ============================================================================
 * UART CONTROL FUNCTIONS
 * ============================================================================ */

uint16_t read_adc_channel_poll(int channel)
{
  ADC0->SC1[0] = ADC_SC1_ADCH(channel);
  while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
  {
    ; // wait for conversion (polling)
  }
  return (int)ADC0->R[0];
}

void sendMessage(char *message)
{
  strncpy(send_buffer, message, MAX_MSG_LEN);
  UART2->C2 |= UART_C2_TIE_MASK;
  UART2->C2 |= UART_C2_TE_MASK;
}

static uint8_t adc_to_level(uint16_t raw)
{
  if (raw > BAD_MOISTURE)
    return 1;
  else if (raw > MEDIUM_MOISTURE)
    return 2;
  else
    return 3;
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

// SOIL MOISTURE SENSOR FUNCTIONS AND UART HANDLING
void UART2_FLEXIO_IRQHandler(void)
{
  static int send_ptr = 0;

  if (UART2->S1 & UART_S1_TDRE_MASK)
  {
    if (send_buffer[send_ptr] == '\0')
    {
      send_ptr = 0;
      UART2->C2 &= ~UART_C2_TIE_MASK;
      UART2->C2 &= ~UART_C2_TE_MASK;
    }
    else
    {
      UART2->D = send_buffer[send_ptr++];
    }
  }
}

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

void PORTB_IRQHandler(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Clear pending IRQ
  NVIC_ClearPendingIRQ(PORTB_IRQn);
  // Check if our flame sensor (PTB18) triggered the interrupt
  if (PORTB->ISFR & (1 << FLAME_PIN))
  {
    // Read the current pin state
    uint32_t pinState = (GPIOB->PDIR >> FLAME_PIN) & 1;
    // Clear interrupt flag
    FLAME_PORT->ISFR |= (1 << FLAME_PIN);
    // Give semaphore to wake up motor task
    if (pinState)
    {
      xSemaphoreGiveFromISR(flamePresentSemaphore, &xHigherPriorityTaskWoken);
    }
    else
    {
      xSemaphoreGiveFromISR(flameAbsentSemaphore, &xHigherPriorityTaskWoken);
    }
    // Perform context switch if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  // Clear ISFR (Interrupt Status Flag Register)
  PORTB->ISFR |= (1 << FLAME_PIN);
}
/* ============================================================================
 * MOTOR FUNCTIONS
 * ============================================================================ */

void rollShutter()
{
  PRINTF("Rolling shutter (moving forward for 5 seconds)...\r\n");

  // Set direction: IN1=HIGH, IN2=LOW (forward at full speed)
  MOTOR_GPIO->PSOR |= (1 << MOTOR_IN1_PIN); // Set IN1 HIGH
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN2_PIN); // Set IN2 LOW

  // Motor runs for 5 seconds
  vTaskDelay(pdMS_TO_TICKS(5000));

  // Stop motor: IN1=LOW, IN2=LOW (coast stop)
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN1_PIN);
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN2_PIN);

  PRINTF("Shutter rolled. Motor stopped.\r\n");
}

void unrollShutter()
{
  PRINTF("Unrolling shutter (reversing for 5 seconds)...\r\n");

  // Set direction: IN1=LOW, IN2=HIGH (reverse at full speed)
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN1_PIN); // Set IN1 LOW
  MOTOR_GPIO->PSOR |= (1 << MOTOR_IN2_PIN); // Set IN2 HIGH

  // Motor runs for 5 seconds
  vTaskDelay(pdMS_TO_TICKS(5000));

  // Stop motor: IN1=LOW, IN2=LOW (coast stop)
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN1_PIN);
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN2_PIN);

  PRINTF("Shutter unrolled. Motor stopped.\r\n");
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
    if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == true)
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
    else
    {
      PRINTF("[Buzzer] Unable to read resource status (mutex timeout)\r\n");
    }

    PRINTF("Ready for next button press...\r\n");
    // Task will block again at the top of the loop
  }
}

/**
 * @brief   Flame task - waits for semaphore, then unrolls shutter
 */
void motorUnrollTask(void *pvParameters)
{
  PRINTF("Motor unroll task started - waiting for flame on\r\n");

  while (1)
  {
    // Wait for flame on (blocks here until semaphore given)
    xSemaphoreTake(flamePresentSemaphore, portMAX_DELAY);

    PRINTF("\r\nFlame present! Requesting motor unroll...\r\n");
    unrollShutter();
    PRINTF("\r\nFinished unrolling");
  }
}

/**
 * @brief   Flame task - waits for semaphore, then rolls shutter
 */
void motorRollTask(void *pvParameters)
{
  PRINTF("Motor roll task started - waiting for flame off\r\n");

  while (1)
  {
    // Wait for flame off (blocks here until semaphore given)
    xSemaphoreTake(flameAbsentSemaphore, portMAX_DELAY);

    PRINTF("\r\nFlame absent! Requesting motor roll...\r\n");
    rollShutter();
    PRINTF("\r\nFinished rolling");
  }
}

/**
 * @brief   Monitor task - periodically displays electricity allocation status
 */
void monitorTask(void *pvParameters)
{
  PRINTF("Monitor task started\r\n\r\n");

  const char *stateNames[] = {"NOTHING", "UV_LIGHTING", "BUZZER_PLAYING"};

  while (1)
  {
    // Wait 2 seconds between status updates
    vTaskDelay(pdMS_TO_TICKS(MONITOR_DELAY_MS));

    // Read shared variable (protected by mutex)
    if (xSemaphoreTake(resourceMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      ResourceState currentState = electricity_allocation;
      xSemaphoreGive(resourceMutex);

      PRINTF("[Monitor] Electricity Allocation: %s\r\n", stateNames[currentState]);
    }
    else
    {
      PRINTF("[Monitor] Unable to read resource status (mutex timeout)\r\n");
    }
  }
}

// Soil Moisture Task
static void soilTask(void *p)
{
  const TickType_t delay = pdMS_TO_TICKS(100);
  while (1)
  {
    int raw = read_adc_channel_poll(ADC_CHANNEL);
    uint8_t level = adc_to_level(raw);

    char msg[2];
    msg[0] = '0' + level;
    msg[1] = '\0';

    // Print for debugging
    PRINTF("Water level is at %c\n", msg[0]);
    PRINTF("Raw ADC: %d, Level: %d\r\n", raw, level);

    sendMessage(msg);

    vTaskDelay(delay);
  }
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

void createBuzzerSemaphore()
{
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

void createResourceMutex()
{
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

void createMonitorTask()
{
  if (xTaskCreate(monitorTask,
                  "MonitorTask",
                  configMINIMAL_STACK_SIZE + 100,
                  NULL,
                  tskIDLE_PRIORITY + 1, // higher priority is higher
                  NULL) != pdPASS)
  {
    PRINTF("ERROR: Failed to create monitor task!\r\n");
    while (1)
      ;
  }
  PRINTF("Monitor task created\r\n");
}

void createBuzzerTask()
{
  if (xTaskCreate(buzzerTask,
                  "BuzzerTask",
                  configMINIMAL_STACK_SIZE + 100,
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

void createMotorRollTask()
{
  if (xTaskCreate(motorRollTask,
                  "MotorRollTask",
                  configMINIMAL_STACK_SIZE + 100,
                  NULL,
                  tskIDLE_PRIORITY + 2,
                  NULL) != pdPASS)
  {
    PRINTF("ERROR: Failed to create motor roll task!\r\n");
    while (1)
      ;
  }
  PRINTF("Motor roll task created\r\n");
}

void createMotorUnrollTask()
{
  if (xTaskCreate(motorUnrollTask,
                  "MotorUnrollTask",
                  configMINIMAL_STACK_SIZE + 100,
                  NULL,
                  tskIDLE_PRIORITY + 2,
                  NULL) != pdPASS)
  {
    PRINTF("ERROR: Failed to create motor unroll task!\r\n");
    while (1)
      ;
  }
  PRINTF("Motor unroll task created\r\n");
}

void createSoilTask()
{
  if (xTaskCreate(
          soilTask,
          "SoilTask",
          configMINIMAL_STACK_SIZE + 100,
          NULL,
          tskIDLE_PRIORITY + 3,
          NULL) != pdPASS)
  {
    PRINTF("ERROR: Failed to create soil task!\r\n");
    while (1)
      ;
  }
  PRINTF("Soil task created\r\n");
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
  initMotor();

  // Initialize Soil Moisture Sensor ADC and UART
  initSoilADC();
  initUART2(SOIL_BAUD);
  startADC(ADC_CHANNEL);

  PRINTF("Soil Moisture TX Started\r\n");

  // Initialise sempaphores and mutexes
  createBuzzerSemaphore();
  createResourceMutex();

// Initialise tasks
#if MONITOR_ON == 1
  createMonitorTask();
#endif
  createBuzzerTask();
  createSoilTask();
  createMotorRollTask();
  createMotorUnrollTask();

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
