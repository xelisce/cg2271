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

/* UV LED Configuration */
#define UV_LED_PIN 4
#define UV_LED_PORT PORTA
#define UV_LED_GPIO GPIOA
#define UV_LED_PORT_CLOCK_GATING_MASK SIM_SCGC5_PORTA_MASK

#define LED_ON_DURATION 500 // ms

/* UART Configuration */
#define BAUD_RATE 9600
#define UART_TX 22 // PTE22
#define UART_RX 23 // PTE23
#define UART2_INT_PRIO 128

/* Motor Pins*/
#define MOTOR_IN1_PIN 4 // PTD6 -
#define MOTOR_IN2_PIN 6 // PTD4 -
#define MOTOR_PORT PORTD
#define MOTOR_GPIO GPIOD
#define MOTOR_PORT_CLOCK_GATING_MASK SIM_SCGC5_PORTD_MASK

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

// Soil Moisture pins
#define ADC_PIN 20 // PTE20
#define UART_TX_PTE22 22
#define UART_RX_PTE23 23
#define UART2_INT_PRIO 128
#define SOIL_BAUD 9600
#define ADC_CHANNEL 0
#define MAX_MSG_LEN 2 // send only 1 char + null terminator
// UART RX queue length for incoming control chars
#define QLEN 5

#define SOIL_DELAY_MS 100

#define BAD_MOISTURE 3400    // moisture > 3000 means too dry
#define MEDIUM_MOISTURE 2700 // 2000 < Moisture < 3000 means decent moisture

/* Monitor delay */
#define MONITOR_DELAY_MS 5000

#define PRINT

#define TEMPO_BPM 800 // Beats per minute (adjust for speed)
#define WHOLE_MS (240000UL / TEMPO_BPM)
#define HALF_MS (120000UL / TEMPO_BPM)
#define QUARTER_MS (60000UL / TEMPO_BPM)
#define EIGHTH_MS (30000UL / TEMPO_BPM)
#define SIXTEENTH_MS (15000UL / TEMPO_BPM)
#define DOTTED_QUARTER_MS (QUARTER_MS + (QUARTER_MS / 2))
#define DOTTED_EIGHTH_MS (EIGHTH_MS + (EIGHTH_MS / 2))
#define DEFAULT_NOTE_DELAY_MS 10

/* ============================================
   NOTE FREQUENCIES
   ============================================ */

/*  RickRollCode

    AUTHOR: Rowan Packard
    rowanpackard@gmail.com

    DISCLAIMER: The song "Never Gonna Give You Up" by Rick Astley
    is not  the creative property of the author. This code simply
    plays a Piezo buzzer  rendition of the song.
*/

#define a3f 208 // 208 Hz
#define b3f 233 // 233 Hz
#define b3 247  // 247 Hz
#define c4 261  // 261 Hz MIDDLE C
#define c4s 277 // 277 Hz
#define e4f 311 // 311 Hz
#define f4 349  // 349 Hz
#define a4f 415 // 415 Hz
#define b4f 466 // 466 Hz
#define b4 493  //  493 Hz
#define c5 523  // 523 Hz
#define c5s 554 // 554  Hz
#define e5f 622 // 622 Hz
#define f5 698  // 698 Hz
#define f5s 740 // 740 Hz
#define a5f 831 // 831 Hz

#define rest -1

//  Parts 4 or 6 (Chorus)

const uint16_t melody[] =
    {b4f, b4f, a4f, a4f,
     f5, f5, e5f, b4f, b4f, a4f, a4f, e5f, e5f, c5s, c5, b4f,
     c5s, c5s, c5s, c5s,
     c5s, e5f, c5, b4f, a4f, a4f, a4f, e5f, c5s,
     b4f, b4f, a4f, a4f,
     f5, f5, e5f, b4f, b4f, a4f, a4f, a5f, c5, c5s, c5, b4f,
     c5s, c5s, c5s, c5s,
     c5s, e5f, c5, b4f, a4f, rest, a4f, e5f, c5s, NOTE_REST};

const uint16_t durations[] =
    {SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS,
     DOTTED_EIGHTH_MS, DOTTED_EIGHTH_MS, DOTTED_QUARTER_MS, SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS, DOTTED_EIGHTH_MS, DOTTED_EIGHTH_MS, DOTTED_EIGHTH_MS, SIXTEENTH_MS, EIGHTH_MS,
     SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS,
     DOTTED_EIGHTH_MS, DOTTED_EIGHTH_MS, DOTTED_EIGHTH_MS, SIXTEENTH_MS, EIGHTH_MS, EIGHTH_MS, EIGHTH_MS, QUARTER_MS, HALF_MS,
     SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS,
     DOTTED_EIGHTH_MS, DOTTED_EIGHTH_MS, DOTTED_QUARTER_MS, SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS, DOTTED_EIGHTH_MS, DOTTED_EIGHTH_MS, DOTTED_EIGHTH_MS, SIXTEENTH_MS, EIGHTH_MS,
     SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS, SIXTEENTH_MS,
     DOTTED_EIGHTH_MS, DOTTED_EIGHTH_MS, DOTTED_EIGHTH_MS, SIXTEENTH_MS, EIGHTH_MS, EIGHTH_MS, EIGHTH_MS, QUARTER_MS, HALF_MS, QUARTER_MS};

const uint8_t melody_length = sizeof(melody) / sizeof(melody[0]);

bool flameOn = false;
bool prevFlameOn = false;

// global varibales
char send_buffer[MAX_MSG_LEN];
QueueHandle_t uartQueue; // Queue for UART RX characters ('y'/'n')

/* RTOS Handles */
SemaphoreHandle_t buzzerSemaphore;      // Released when the button is pressed
SemaphoreHandle_t resourceMutex = NULL; // Shared mutex for resource allocation, prevent conflicting access to electricity allocation
SemaphoreHandle_t ledSemaphore;         // Semaphore for LED control

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

#ifdef PRINT
  PRINTF("Buzzer PWM initialized:\r\n");
  PRINTF("  - Pin: PTA12\r\n");
  PRINTF("  - TPM: TPM1 Channel 0\r\n");
  PRINTF("  - Frequency: 262Hz (middle C)\r\n");
  PRINTF("  - TPM1->MOD: %lu\r\n", BUZZER_TIMER->MOD);
  PRINTF("  - TPM1->CONTROLS[0].CnV: %lu\r\n", BUZZER_TIMER->CONTROLS[BUZZER_TIMER_CHANNEL].CnV);
#endif
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

#ifdef PRINT // Feedback
  PRINTF("Button initialized on PTA13\r\n");
#endif
}

void startADC(int channel)
{
#ifdef PRINT
  PRINTF("Started ADC on channel %d. Waiting\r\n", channel);
#endif

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

#ifdef PRINT
  PRINTF("ADC polling mode initialized on PTE20, Channel 0\r\n");
#endif
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

  // Disable parity
  UART2->C1 &= ~UART_C1_PE_MASK;
  // 8-bit mode
  UART2->C1 &= ~UART_C1_M_MASK;
  // Enable RX interrupt
  UART2->C2 |= UART_C2_RIE_MASK;
  // Enable the receiver
  UART2->C2 |= UART_C2_RE_MASK;

#ifdef PRINT
  PRINTF("UART2 initialized on RX=PTE%d, TX=PTE%d\r\n", UART_RX, UART_TX);
#endif
}

void initLED(void)
{
  // Set up the clock gating
  SIM->SCGC5 |= UV_LED_PORT_CLOCK_GATING_MASK;
  PORTA->PCR[UV_LED_PIN] &= ~PORT_PCR_MUX_MASK;
  PORTA->PCR[UV_LED_PIN] = PORT_PCR_MUX(1);
  GPIOA->PDDR |= (1 << UV_LED_PIN);
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

#ifdef PRINT
  PRINTF("Motor driver (DRV8833) initialized:\r\n");
  PRINTF("  - IN1: PTA%d\r\n", MOTOR_IN1_PIN);
  PRINTF("  - IN2: PTA%d\r\n", MOTOR_IN2_PIN);
  PRINTF("  - Control: Direct H-bridge (no EN pin)\r\n");
#endif
}

void initAllInterrupts()
{
  // Enable timer interrupt for buzzer
  NVIC_SetPriority(BUZZER_TIMER_IRQ, 1); // High priority (0 = highest, 192 = lowest)
  NVIC_EnableIRQ(BUZZER_TIMER_IRQ);
  // Enable PORTA interrupt for button
  NVIC_SetPriority(BUTTON_PORT_IRQ, 128); // Medium priority (0 = highest, 192 = lowest)
  NVIC_ClearPendingIRQ(BUTTON_PORT_IRQ);
  NVIC_EnableIRQ(BUTTON_PORT_IRQ);
  // Enable UART interrupts for comms
  NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
  NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
  NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
}

/* ============================================================================
 * LED CONTROL FUNCTIONS
 * ============================================================================ */

void turnLEDOn(void)
{
  UV_LED_GPIO->PCOR |= (1 << UV_LED_PIN);
}

void turnLEDOff(void)
{
  UV_LED_GPIO->PSOR |= (1 << UV_LED_PIN);
}

/* ============================================================================
 * ADC CONTROL FUNCTIONS
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
 * UART FUNCTIONS
 * ============================================================================ */

void sendMessage(char *message)
{
  strncpy(send_buffer, message, MAX_MSG_LEN);
  UART2->C2 |= UART_C2_TIE_MASK;
  UART2->C2 |= UART_C2_TE_MASK;
}

bool uartDataAvailable(void)
{
  return (UART2->S1 & UART_S1_RDRF_MASK);
}

int uartReadChar(void)
{
  if (uartDataAvailable())
  {
    int receivedChar = (UART2->D);
    if ((receivedChar == 'y') || (receivedChar == 'n'))
    {
      return receivedChar;
    }
  }
  return -1;
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
#ifdef PRINT
  PRINTF("\r\nPlaying 'Twinkle Twinkle Little Star\r\n");
#endif

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
#ifdef PRINT
      PRINTF("Song interrupted - resource released\r\n"); // but this shouldnt happen, the uv light should also check.
#endif
      break;
    }
  }

#ifdef PRINT
  PRINTF("Song complete!\r\n\r\n");
#endif
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

  static char rxBuffer[2];
  static uint8_t rxIndex = 0;

  if (UART2->S1 & UART_S1_RDRF_MASK)
  {
    char c = UART2->D; // read clears flag
    rxBuffer[rxIndex++] = c;

    if (rxIndex == 2)
    {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      if (uartQueue)
      {
        xQueueSendFromISR(uartQueue, &rxBuffer, &xHigherPriorityTaskWoken);
      }
      rxIndex = 0; // reset for next pair
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
    // Give semaphore to wake up buzzer task
    xSemaphoreGiveFromISR(buzzerSemaphore, &xHigherPriorityTaskWoken);
    // Perform context switch if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    // Clear ISFR (Interrupt Status Flag Register)
    PORTA->ISFR |= (1 << BUTTON_PIN);
  }
}

// void PORTC_PORTD_IRQHandler(void)
// {
//   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//   // Clear pending IRQ
//   NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
//   // Check if our flame sensor (PTB18) triggered the interrupt
//   if (PORTC->ISFR & (1 << FLAME_PIN))
//   {
//     // Read the current pin state
//     uint32_t pinState = (GPIOC->PDIR >> FLAME_PIN) & 1;
//     // Give semaphore to wake up motor task
//     if (pinState)
//     {
//       xSemaphoreGiveFromISR(flamePresentSemaphore, &xHigherPriorityTaskWoken);
//     }
//     else
//     {
//       xSemaphoreGiveFromISR(flameAbsentSemaphore, &xHigherPriorityTaskWoken);
//     }
//     // Perform context switch if needed
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//     PORTC->ISFR |= (1 << FLAME_PIN);
//   }
// }

/* ============================================================================
 * MOTOR FUNCTIONS
 * ============================================================================ */

void rollShutter()
{
#ifdef PRINT
  PRINTF("Rolling shutter (moving forward for 5 seconds)...\r\n");
#endif

  // Set direction: IN1=HIGH, IN2=LOW (forward at full speed)
  MOTOR_GPIO->PSOR |= (1 << MOTOR_IN1_PIN); // Set IN1 HIGH
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN2_PIN); // Set IN2 LOW

  // Motor runs for 5 seconds
  vTaskDelay(pdMS_TO_TICKS(300));

  // Stop motor: IN1=LOW, IN2=LOW (coast stop)
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN1_PIN);
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN2_PIN);

#ifdef PRINT
  PRINTF("Shutter rolled. Motor stopped.\r\n");
#endif
}

void unrollShutter()
{
#ifdef PRINT
  PRINTF("Unrolling shutter (reversing for 5 seconds)...\r\n");
#endif

  // Set direction: IN1=LOW, IN2=HIGH (reverse at full speed)
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN1_PIN); // Set IN1 LOW
  MOTOR_GPIO->PSOR |= (1 << MOTOR_IN2_PIN); // Set IN2 HIGH

  // Motor runs for 5 seconds
  vTaskDelay(pdMS_TO_TICKS(300));

  // Stop motor: IN1=LOW, IN2=LOW (coast stop)
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN1_PIN);
  MOTOR_GPIO->PCOR |= (1 << MOTOR_IN2_PIN);

  vTaskDelay(pdMS_TO_TICKS(1000));

#ifdef PRINT
  PRINTF("Shutter unrolled. Motor stopped.\r\n");
#endif
}

/* ============================================================================
 * RTOS TASKS
 * ============================================================================ */

/**
 * @brief   LED task - controls red LED based on semaphore
 */
void ledTask(void *pvParameters)
{
#ifdef PRINT
  PRINTF("LED task started\r\n");
#endif
  turnLEDOff();
  while (1)
  {
    xSemaphoreTake(ledSemaphore, portMAX_DELAY);

#ifdef PRINT
// PRINTF("Turning LED On");
#endif
    turnLEDOn();
    vTaskDelay(pdMS_TO_TICKS(LED_ON_DURATION));

#ifdef PRINT
// PRINTF("Turning LED Off");
#endif
    turnLEDOff();
    // Wait for uart to be 1 (blocks here until semaphore given)

    if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE)
    {
      electricity_allocation = NOTHING;
      xSemaphoreGive(resourceMutex);
    }
    else
    {
#ifdef PRINT
      PRINTF("Error writing to resource allocation.");
#endif
    }
  }
}

/**
 * @brief   UART task - receives knob values and controls UV LED
 */
void readUARTTask(void *pvParameters)
{
#ifdef PRINT
  PRINTF("UART task started - waiting for 2-char messages ('y/n' + 'x/o')...\r\n");
#endif

  char rxMsg[2];

  while (1)
  {
    if (xQueueReceive(uartQueue, &rxMsg, portMAX_DELAY) == pdTRUE)
    {
      char firstChar = rxMsg[0];
      char secondChar = rxMsg[1]; // 'x' = flame on, 'o' = flame off

#ifdef PRINT
      PRINTF("UART: Received [%c, %c]\r\n", firstChar, secondChar);
#endif

      // --- Handle UV LED as before (first char) ---
      if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE)
      {
        if (firstChar == 'y')
        {
          if (electricity_allocation == NOTHING)
          {
            electricity_allocation = UV_LIGHTING;
            xSemaphoreGive(ledSemaphore);
#ifdef PRINT
            PRINTF("UART: UV LED activated\r\n");
#endif
          }
        }
        xSemaphoreGive(resourceMutex);
      }

      // --- Handle Flame Logic (second char) ---
      prevFlameOn = flameOn;
      flameOn = (secondChar == 'x'); // x = on, o = off

      if (!prevFlameOn && flameOn)
      {
#ifdef PRINT
        PRINTF("🔥 Transition: Flame OFF → ON, unrolling shutter...\r\n");
#endif
        unrollShutter();
      }
      else if (prevFlameOn && !flameOn)
      {
#ifdef PRINT
        PRINTF("🔥 Transition: Flame ON → OFF, rolling shutter...\r\n");
#endif
        rollShutter();
      }
    }
  }
}

void buzzerTask(void *pvParameters)
{
#ifdef PRINT
  PRINTF("Buzzer task started - waiting for button press...\r\n");
#endif

  while (1)
  {
    // Wait for button press (blocks here until semaphore given)
    xSemaphoreTake(buzzerSemaphore, portMAX_DELAY);

#ifdef PRINT
    PRINTF("\r\nButton pressed! Requesting buzzer resource...\r\n");
#endif

    // Take mutex to check and modify electricity allocation
    if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == true)
    {

      // Check if resource is available
      if (electricity_allocation == NOTHING)
      {
        // Allocate resource to buzzer

        electricity_allocation = BUZZER_PLAYING;
#ifdef PRINT
        PRINTF("Resource allocated: BUZZER_PLAYING\r\n");
#endif

        // Release mutex while playing song
        xSemaphoreGive(resourceMutex);

        // Play the song
        playSong();

        // Take mutex again to release resource
        xSemaphoreTake(resourceMutex, portMAX_DELAY);

        electricity_allocation = NOTHING;
#ifdef PRINT
        PRINTF("Resource released: NOTHING\r\n");
#endif
        xSemaphoreGive(resourceMutex);
      }
      else if (electricity_allocation == UV_LIGHTING)
      {
#ifdef PRINT
        PRINTF("Resource unavailable: UV_LIGHTING is active\r\n");
#endif
        xSemaphoreGive(resourceMutex); // if we want to have it queued up then we don't give up the mutex here
      }
      else
      {
#ifdef PRINT
        PRINTF("Resource unavailable: BUZZER already playing\r\n");
#endif
        xSemaphoreGive(resourceMutex);
      }
    }
    else
    {
#ifdef PRINT
      PRINTF("[Buzzer] Unable to read resource status (mutex timeout)\r\n");
#endif
    }

#ifdef PRINT
    PRINTF("Ready for next button press...\r\n");
#endif
    // Task will block again at the top of the loop
  }
}

void monitorTask(void *pvParameters)
{
#ifdef PRINT
  PRINTF("Monitor task started\r\n\r\n");

  const char *stateNames[] = {"NOTHING", "UV_LIGHTING", "BUZZER_PLAYING"};

  while (1)
  {
    // Wait 2 seconds between status updates
    vTaskDelay(pdMS_TO_TICKS(MONITOR_DELAY_MS));

    // Read shared variable (protected by mutex)
    if (xSemaphoreTake(resourceMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      xSemaphoreGive(resourceMutex);

      ResourceState currentState = electricity_allocation;
      PRINTF("[Monitor] State: %s | Free heap: %u | Min heap: %u\r\n",
             stateNames[currentState],
             xPortGetFreeHeapSize(),
             xPortGetMinimumEverFreeHeapSize());
    }
    else
    {
      PRINTF("[Monitor] Unable to read resource status (mutex timeout)\r\n");
    }
  }
#endif
}

// Soil Moisture Task
static void soilTask(void *p)
{
  const TickType_t delay = pdMS_TO_TICKS(SOIL_DELAY_MS);
  while (1)
  {
    int raw = read_adc_channel_poll(ADC_CHANNEL);
    uint8_t level = adc_to_level(raw);

    char msg[2];
    msg[0] = '0' + level;
    msg[1] = '\0';

#ifdef PRINT
    PRINTF("Water level is at %c\n", msg[0]);
    PRINTF("Raw ADC: %d, Level: %d\r\n", raw, level);
#endif

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
#ifdef PRINT
    PRINTF("ERROR: Failed to create buzzer semaphore!\r\n");
#endif
    while (1)
      ;
  }

#ifdef PRINT
  PRINTF("Buzzer semaphore created\r\n");
#endif
}

void createResourceMutex()
{
  // Create mutex for resource allocation (shared with UV light task)
  resourceMutex = xSemaphoreCreateMutex();
  if (resourceMutex == NULL)
  {
#ifdef PRINT
    PRINTF("ERROR: Failed to create resource mutex!\r\n");
#endif
    while (1)
      ;
  }

#ifdef PRINT
  PRINTF("Resource mutex created\r\n");
#endif
}

void createLEDSemaphore()
{
  // Create LED semaphore (binary semaphore for signaling)
  ledSemaphore = xSemaphoreCreateBinary();
  if (ledSemaphore == NULL)
  {
#ifdef PRINT
    PRINTF("ERROR: Failed to create LED semaphore!\r\n");
#endif
    while (1)
      ;
  }
#ifdef PRINT
  PRINTF("LED semaphore created\r\n");
#endif
}

void createMonitorTask()
{
  if (xTaskCreate(monitorTask,
                  "MonitorTask",
                  configMINIMAL_STACK_SIZE + 300,
                  NULL,
                  tskIDLE_PRIORITY + 1, // higher priority is higher
                  NULL) != pdPASS)
  {
#ifdef PRINT
    PRINTF("ERROR: Failed to create monitor task!\r\n");
#endif
    while (1)
      ;
  }
#ifdef PRINT
  PRINTF("Monitor task created\r\n");
#endif
}

void createLEDTask()
{
  if (xTaskCreate(ledTask,
                  "LEDTask",
                  configMINIMAL_STACK_SIZE + 100,
                  NULL,
                  tskIDLE_PRIORITY + 2,
                  NULL) != pdPASS)
  {
#ifdef PRINT
    PRINTF("ERROR: Failed to create LED task!\r\n");
#endif
    while (1)
      ;
  }
#ifdef PRINT
  PRINTF("LED task created\r\n");
#endif
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
#ifdef PRINT
    PRINTF("ERROR: Failed to create buzzer task!\r\n");
#endif
    while (1)
      ;
  }
#ifdef PRINT
  PRINTF("Buzzer task created\r\n");
#endif
}

void createSoilTask()
{
  if (xTaskCreate(
          soilTask,
          "SoilTask",
          configMINIMAL_STACK_SIZE + 100,
          NULL,
          tskIDLE_PRIORITY + 1,
          NULL) != pdPASS)
  {
#ifdef PRINT
    PRINTF("ERROR: Failed to create soil task!\r\n");
#endif
    while (1)
      ;
  }
#ifdef PRINT
  PRINTF("Soil task created\r\n");
#endif
}

void createUARTTask()
{
  // Create UART task
  if (xTaskCreate(readUARTTask,
                  "UARTTask",
                  configMINIMAL_STACK_SIZE + 100,
                  NULL,
                  tskIDLE_PRIORITY + 1, // Higher priority for UART
                  NULL) != pdPASS)
  {
#ifdef PRINT
    PRINTF("ERROR: Failed to create UART task!\r\n");
#endif
    while (1)
      ;
  }
#ifdef PRINT
  PRINTF("UART task created\r\n");
#endif
}

void createUARTQueue()
{
  uartQueue = xQueueCreate(QLEN, sizeof(char[2]));
  if (uartQueue == NULL)
  {
#ifdef PRINT
    PRINTF("ERROR: Failed to create UART queue!\r\n");
#endif
    while (1)
      ;
  }
#ifdef PRINT
  PRINTF("UART queue created\r\n");
#endif
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

#ifdef PRINT
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
#endif

  // Initialize hardware
  setTPMClock();
  initBuzzer();
  initButton();
  initMotor();
  initLED();

  // Initialize Soil Moisture Sensor ADC and UART
  initSoilADC();

  initUART2(SOIL_BAUD);
#ifdef PRINT
  PRINTF("Soil Moisture TX Started\r\n");
#endif

  // Initialise sempaphores and mutexes
  createResourceMutex();

  createBuzzerSemaphore();
  createLEDSemaphore();
  createUARTQueue();
  // createFlamePresentSemaphore();
  // createFlameAbsentSemaphore();

  // Begin interrupts and stuff
  initAllInterrupts();
  startADC(ADC_CHANNEL);

  // Initialise tasks
#if MONITOR_ON == 1
  createMonitorTask();
#endif
  createBuzzerTask();
  createSoilTask();
  // createMotorRollTask();
  // createMotorUnrollTask();
  createUARTTask();
  createLEDTask();

#ifdef PRINT
  PRINTF("\r\nStarting FreeRTOS scheduler...\r\n");
  PRINTF("About to start scheduler. Heap free: %d bytes\r\n",
         xPortGetFreeHeapSize());
#endif

  vTaskStartScheduler();

#ifdef PRINT
  // Print instructions
  PRINTF("Press button on PTA13 to play the song!\r\n\r\n");
  // Should never reach here
  PRINTF("ERROR: Scheduler failed to start!\r\n");
#endif

  while (1)
  {
    __asm volatile("nop");
  }

  return 0;
}
