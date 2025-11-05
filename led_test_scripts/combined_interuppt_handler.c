/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file    combined_uv_led_control.c
 * @brief   Combined UV LED control with UART and LED timer tasks
 */
#include <stdio.h>
#include <string.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

/* ============================================================================
 * CONFIGURATION
 * ============================================================================ */

/* UV LED Configuration */
#define UV_LED_PIN 0
#define UV_LED_PORT PORTB
#define UV_LED_GPIO GPIOB
#define UV_LED_PORT_CLOCK_GATING_MASK SIM_SCGC5_PORTB_MASK

/* UART Configuration */
#define BAUD_RATE 9600
#define UART_TX 22 // PTE22
#define UART_RX 23 // PTE23
#define UART2_INT_PRIO 128

/* LED Configuration */
#define RED_LED_PIN 4 // PTA4

/* Timing Configuration */
#define MONITOR_ON 1
#define LED_ON_DURATION 5000      // ms

#define QLEN 5

/* Soil Moisture Configuration (ADD THESE) */
#define ADC_PIN 20
#define UART_TX_PTE22 22  
#define UART_RX_PTE23 23
#define ADC_CHANNEL 0
#define MAX_MSG_LEN 2

/* Global Variables (ADD THESE) */
char send_buffer[MAX_MSG_LEN];    // For soil moisture TX

/* Semaphores and Mutex */
SemaphoreHandle_t resourceMutex;  // Shared mutex for resource allocation
SemaphoreHandle_t ledSemaphore;   // Semaphore for LED control
QueueHandle_t uartQueue;          // Queue for UART messages



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
 * HARDWARE INITIALIZATION
 * ============================================================================ */

/**
 * @brief   Initialize UV LED GPIO
 */
void initUVLED(void)
{
    // Enable clock gating to PORTB
    SIM->SCGC5 |= UV_LED_PORT_CLOCK_GATING_MASK;

    // Configure LED pin as GPIO
    UV_LED_PORT->PCR[UV_LED_PIN] &= ~PORT_PCR_MUX_MASK;
    UV_LED_PORT->PCR[UV_LED_PIN] |= PORT_PCR_MUX(1); // GPIO mode

    // Set as output
    UV_LED_GPIO->PDDR |= (1 << UV_LED_PIN);

    // Turn off initially (PSOR = set to 1 = off for active low)
    UV_LED_GPIO->PSOR |= (1 << UV_LED_PIN);

    PRINTF("UV LED initialized on PTB%d\r\n", UV_LED_PIN);
}

/**
 * @brief   Initialize UART2 for communication
 */
void initUART2(uint32_t baud_rate)
{
    NVIC_DisableIRQ(UART2_FLEXIO_IRQn);

    // enable clock to UART2 and PORTE
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Ensure Tx and Rx are disabled before configuration
    UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

    // connect UART pins for PTE22, PTE23
    PORTE->PCR[UART_TX] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_TX] |= PORT_PCR_MUX(4);

    PORTE->PCR[UART_RX] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_RX] |= PORT_PCR_MUX(4);

    // Set the baud rate
    uint32_t bus_clk = CLOCK_GetBusClkFreq();

    // This version of sbr does integer rounding.
    uint32_t sbr = (bus_clk + (baud_rate * 8)) / (baud_rate * 16);

    // Set SBR. Bits 8 to 12 in BDH, 0-7 in BDL.
    // MUST SET BDH FIRST!
    UART2->BDH &= ~UART_BDH_SBR_MASK;
    UART2->BDH |= ((sbr >> 8) & UART_BDH_SBR_MASK);
    UART2->BDL = (uint8_t)(sbr & 0xFF);

    // Disable loop mode
    UART2->C1 &= ~UART_C1_LOOPS_MASK;
    UART2->C1 &= ~UART_C1_RSRC_MASK;

    // Disable parity
    UART2->C1 &= ~UART_C1_PE_MASK;

    // 8-bit mode
    UART2->C1 &= ~UART_C1_M_MASK;

    // Enable RX interrupt
    UART2->C2 |= UART_C2_RIE_MASK;

    // Enable the receiver
    UART2->C2 |= UART_C2_RE_MASK;

    NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
    NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
    NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
    PRINTF("UART2 initialized on RX=PTE%d, TX=PTE%d\r\n", UART_RX, UART_TX);
}

/**
 * @brief   Initialize Red LED GPIO
 */
void initLED(void)
{
     // Set up the clock gating
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    PORTA->PCR[RED_LED_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[RED_LED_PIN] = PORT_PCR_MUX(1);
    GPIOA->PDDR |= (1 << RED_LED_PIN);
}

/**
 * @brief   Turn Red LED on
 */
void turnLEDOn(void)
{
    GPIOA->PCOR |= (1 << RED_LED_PIN);
}

/**
 * @brief   Turn Red LED off
 */
void turnLEDOff(void)
{
    GPIOA->PSOR |= (1 << RED_LED_PIN);
}

/* ============================================================================
 * UART FUNCTIONS
 * ============================================================================ */
/**
 * @brief   Send message via UART (for soil moisture)
 */
void sendMessage(char *message)
{
    strncpy(send_buffer, message, MAX_MSG_LEN);
    UART2->C2 |= UART_C2_TIE_MASK;  // Enable TX interrupt
    UART2->C2 |= UART_C2_TE_MASK;   // Ensure transmitter is enabled
}

/* ============================================================================
 * UART INTERRUPT HANDLER (Receive Only)
 * ============================================================================ */

/* ============================================================================
 * COMBINED UART INTERRUPT HANDLER (TX + RX)
 * ============================================================================ */

void UART2_FLEXIO_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static int send_ptr = 0;  // For soil moisture TX

    // Handle TRANSMIT (Soil Moisture - Friend's code)
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

    // Handle RECEIVE (UV LED - Your code)
    if (UART2->S1 & UART_S1_RDRF_MASK)
    {
        char receivedChar = UART2->D;  // Read character (clears interrupt)
        
        // Only process 'y' or 'n' characters for UV LED
        if (receivedChar == 'y' || receivedChar == 'n') {
            // Send character to your queue
            xQueueSendFromISR(uartQueue, &receivedChar, &xHigherPriorityTaskWoken);
        }
    }

    // Yield if a message was queued
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief   Check if UART data is available
 * @return  true if data available, false otherwise
 */
bool uartDataAvailable(void)
{
    return (UART2->S1 & UART_S1_RDRF_MASK);
}

/**
 * @brief   Read a character from UART
 * @return  Received character, or -1 if no data
 */
int uartReadChar(void)
{
    if (uartDataAvailable())
    {
        int receivedChar = (UART2->D);
        if ((receivedChar == 'y') || (receivedChar == 'n')) {
            return receivedChar;
        }
    }
    return -1;
}

/* ============================================================================
 * TASK FUNCTIONS
 * ============================================================================ */

/**
 * @brief   LED task - controls red LED based on semaphore
 */
void ledTask(void *pvParameters)
{
    PRINTF("LED task started\r\n");
    while (1)
    {
        PRINTF("Turning LED Off");
        turnLEDOff();
        // Wait for uart to be 1 (blocks here until semaphore given)
        xSemaphoreTake(ledSemaphore, portMAX_DELAY); 
        PRINTF("Turning LED On");
        turnLEDOn();
        vTaskDelay(pdMS_TO_TICKS(LED_ON_DURATION));

        if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE) {
            electricity_allocation = NOTHING;
            xSemaphoreGive(resourceMutex);
        } else {
            PRINTF("Error writing to resource allocation.");
        }
    }
}


/**
 * @brief   UART task - receives knob values and controls UV LED
 */
void uartReceiveTask(void *pvParameters)
{
    PRINTF("UART Task: Waiting for 'y' or 'n' from ESP32...\r\n");

    while (1)
    {
        char receivedChar;
        
        // Wait for character from UART interrupt (blocks efficiently)
        if (xQueueReceive(uartQueue, &receivedChar, portMAX_DELAY) == pdTRUE) {
            PRINTF("UART: Received '%c'\r\n", receivedChar);

            // Use shared mutex to access resource allocation
            if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE) {
                if (receivedChar == 'y') {
                    if (electricity_allocation == NOTHING) {
                        // Turn on LED and allocate resource
                        electricity_allocation = UV_LIGHTING;
                        xSemaphoreGive(ledSemaphore);
                        PRINTF("UART: UV LED activated - Resource allocated\r\n");
                    } else {
                        PRINTF("UART: Resource busy - %s active\r\n",
                            (electricity_allocation == BUZZER_PLAYING) ? "BUZZER" : "UV_LIGHT");
                    }
                }
                else if (receivedChar == 'n') {
                    PRINTF("UART: Received 'n' (no action)\r\n");
                }
                xSemaphoreGive(resourceMutex);
            }
        }
    }
}

/* ============================================================================
 * TASK CREATION AND INITIALIZATION
 * ============================================================================ */

/**
 * @brief   Create all semaphoresandqueue
 */
void createSemaphoresAndQueue(void)
{
    // Create resource mutex
    resourceMutex = xSemaphoreCreateMutex();
    if (resourceMutex == NULL) {
        PRINTF("ERROR: Failed to create resource mutex!\r\n");
        while (1);
    }

    // Create LED semaphore
    ledSemaphore = xSemaphoreCreateBinary();
    if (ledSemaphore == NULL) {
        PRINTF("ERROR: Failed to create LED semaphore!\r\n");
        while (1);
    }

    // Create UART character queue (single characters)
    uartQueue = xQueueCreate(QLEN, sizeof(char));
    if (uartQueue == NULL) {
        PRINTF("ERROR: Failed to create UART queue!\r\n");
        while (1);
    }

    PRINTF("Semaphores and queue created\r\n");
}

/**
 * @brief   Create all tasks
 */
void createTasks(void)
{
    // Create LED task
    if (xTaskCreate(ledTask,
                    "LEDTask",
                    configMINIMAL_STACK_SIZE + 100,
                    NULL,
                    tskIDLE_PRIORITY + 1,
                    NULL) != pdPASS)
    {
        PRINTF("ERROR: Failed to create LED task!\r\n");
        while (1);
    }
    PRINTF("LED task created\r\n");

    // Create UART task
    if (xTaskCreate(uartReceiveTask,
                    "UARTTask",
                    configMINIMAL_STACK_SIZE + 200,
                    NULL,
                    tskIDLE_PRIORITY + 2, // Higher priority for UART
                    NULL) != pdPASS)
    {
        PRINTF("ERROR: Failed to create UART task!\r\n");
        while (1);
    }
    PRINTF("UART task created\r\n");
}

/* ============================================================================
 * MAIN FUNCTION
 * ============================================================================ */

int main(void)
{
    /* Init board hardware */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    PRINTF("\r\n=== Combined UV LED and Timer LED System ===\r\n");

    // Initialize hardware
    initUVLED();
    initUART2(BAUD_RATE);
    initLED();

    // Create semaphores and tasks
    createSemaphoresAndQueue();
    createTasks();

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    PRINTF("All tasks created successfully - System running!\r\n");

    // Should never reach here
    while (1)
    {
        __asm volatile("nop");
    }

    return 0;
}