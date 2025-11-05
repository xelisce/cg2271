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

/* Semaphores and Mutex */
SemaphoreHandle_t resourceMutex;  // Shared mutex for resource allocation
SemaphoreHandle_t ledSemaphore;   // Semaphore for LED control

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

    NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);

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
void readUARTTask(void *pvParameters)
{
    PRINTF("UART task started - waiting for knob values...\r\n");
    PRINTF("Expecting 'y' or 'n' characters from ESP32 at 9600 baud\r\n");

    while (1)
    {
        int receivedChar = uartReadChar();

        if ((receivedChar == 'y') || (receivedChar == 'n')) {
            PRINTF("Character received: '%c' (0x%02X)\r\n", receivedChar, receivedChar);

            // Use shared mutex to access resource allocation
            if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE) {
                if (receivedChar == 'y') {
                    if (electricity_allocation == NOTHING) {
                        xSemaphoreGive(ledSemaphore);
                        electricity_allocation = UV_LIGHTING;
                        PRINTF("UV LED ON - Resource allocated: UV_LIGHTING\r\n");
                    } else {
                        PRINTF("Resource unavailable: %s active\r\n",
                            (electricity_allocation == BUZZER_PLAYING) ? "BUZZER" : "UV_LIGHT");
                    }
                }
                xSemaphoreGive(resourceMutex);
            } else {
                PRINTF("Error accessing resource mutex: readUARTTask");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent overwhelming the CPU
    }
}

/* ============================================================================
 * TASK CREATION AND INITIALIZATION
 * ============================================================================ */

/**
 * @brief   Create all semaphores
 */
void createSemaphores(void)
{
    // Create resource mutex
    resourceMutex = xSemaphoreCreateMutex();
    if (resourceMutex == NULL)
    {
        PRINTF("ERROR: Failed to create resource mutex!\r\n");
        while (1);
    }
    PRINTF("Resource mutex created\r\n");

    // Create LED semaphore (binary semaphore for signaling)
    ledSemaphore = xSemaphoreCreateBinary();
    if (ledSemaphore == NULL)
    {
        PRINTF("ERROR: Failed to create LED semaphore!\r\n");
        while (1);
    }
    PRINTF("LED semaphore created\r\n");
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
    if (xTaskCreate(readUARTTask,
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
    createSemaphores();
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