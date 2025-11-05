/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/**
 * @file    swetha_uv_control.c
 * @brief   Swetha's complete UV LED control with UART knob reading
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
 * SWETHA'S CONFIGURATION
 * ============================================================================ */

/* UV LED Configuration */
#define UV_LED_PIN 0
#define UV_LED_PORT PORTB
#define UV_LED_GPIO GPIOB
#define UV_LED_PORT_CLOCK_GATING_MASK SIM_SCGC5_PORTB_MASK

#define BAUD_RATE 9600
#define UART_TX_PTE22 	22
#define UART_RX_PTE23 	23
#define UART2_INT_PRIO	128

/* UV Light Duration */
#define UV_LIGHT_DURATION_MS 5000  // 5 seconds


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
 * UV LED CONTROL FUNCTIONS
 * ============================================================================ */

/**
 * @brief   Initialize UV LED GPIO
 */
void initUVLED(void)
{
    // Enable clock gating to PORTA
    SIM->SCGC5 |= UV_LED_PORT_CLOCK_GATING_MASK;
    
    // Configure LED pin as GPIO
    UV_LED_PORT->PCR[UV_LED_PIN] &= ~PORT_PCR_MUX_MASK;
    UV_LED_PORT->PCR[UV_LED_PIN] |= PORT_PCR_MUX(1); // GPIO mode
    
    // Set as output
    UV_LED_GPIO->PDDR |= (1 << UV_LED_PIN);
    
    // Turn off initially (PSOR = set to 1 = off for active low)
    UV_LED_GPIO->PSOR |= (1 << UV_LED_PIN);
    
    PRINTF("UV LED initialized on PTA%d\r\n", UV_LED_PIN);
}

/**
 * @brief   Set UV LED state
 * @param   state - true = ON, false = OFF
 */
void setUVLED(bool state)
{
    if (state) {
        UV_LED_GPIO->PCOR |= (1 << UV_LED_PIN);  // Clear = ON (active low)
    } else {
        UV_LED_GPIO->PSOR |= (1 << UV_LED_PIN);  // Set = OFF
    }
}

void initUART2(uint32_t baud_rate)
{
	NVIC_DisableIRQ(UART2_FLEXIO_IRQn);

	//enable clock to UART2 and PORTE
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	//Ensure Tx and Rx are disabled before configuration
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	//connect UART pins for PTE22, PTE23
	PORTE->PCR[UART_TX_PTE22] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_TX_PTE22] |= PORT_PCR_MUX(4);

	PORTE->PCR[UART_RX_PTE23] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX_PTE23] |= PORT_PCR_MUX(4);

	// Set the baud rate
	uint32_t bus_clk = CLOCK_GetBusClkFreq();

	// This version of sbr does integer rounding.
	uint32_t sbr = (bus_clk + (baud_rate * 8)) / (baud_rate * 16);

	// Set SBR. Bits 8 to 12 in BDH, 0-7 in BDL.
	// MUST SET BDH FIRST!
	UART2->BDH &= ~UART_BDH_SBR_MASK;
	UART2->BDH |= ((sbr >> 8) & UART_BDH_SBR_MASK);
	UART2->BDL = (uint8_t) (sbr &0xFF);

	// Disable loop mode
	UART2->C1 &= ~UART_C1_LOOPS_MASK;
	UART2->C1 &= ~UART_C1_RSRC_MASK;

	// Disable parity
	UART2->C1 &= ~UART_C1_PE_MASK;

	// 8-bit mode
	UART2->C1 &= ~UART_C1_M_MASK;

	//Enable RX interrupt
	UART2->C2 |= UART_C2_RIE_MASK;

	// Enable the receiver
	UART2->C2 |= UART_C2_RE_MASK;

	NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);

    PRINTF("UART2 initialized on RX=PTE%d, TX=PTE%d\r\n", UART_RX_PTE23, UART_TX_PTE22);
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
    if (uartDataAvailable()) {
        return UART2->D;
    }
    return -1;
}


/* ============================================================================
 * TIMER CALLBACK
 * ============================================================================ */

/**
 * @brief   Timer callback for auto-turn-off
 */
void uvTimerCallback(TimerHandle_t xTimer)
{
    if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE) {
        setUVLED(false);
        if (electricity_allocation == UV_LIGHTING) {
            electricity_allocation = NOTHING;
            PRINTF("UV LED auto-off: Resource released\r\n");
        }
        xSemaphoreGive(resourceMutex);
    }
}

void swethaInit(){
    initUVLED();
    initUART();

    TimerHandle_t uvTimer = NULL;
    uvTimer = xTimerCreate(
        "UVTimer",
        pdMS_TO_TICKS(UV_LIGHT_DURATION_MS),
        pdFALSE,  // One-shot timer
        NULL,
        uvTimerCallback
    );
    if (uvTimer == NULL) {
        PRINTF("ERROR: Failed to create UV timer!\r\n");
        return;
    }


}



/* ============================================================================
 * MAIN UART TASK
 * ============================================================================ */

/**
 * @brief   Task to receive knob values via UART and control UV LED
 */
void swethaUARTTask(void *pvParameters)
{
    PRINTF("Swetha's UART task started - waiting for knob values...\r\n");
    PRINTF("Expecting '1' or '0' characters from ESP32 at 9600 baud\r\n");
    
    TimerHandle_t uvTimer = NULL;
    
    // Create timer for auto-turn-off
    uvTimer = xTimerCreate(
        "UVTimer",
        pdMS_TO_TICKS(UV_LIGHT_DURATION_MS),
        pdFALSE,  // One-shot timer
        NULL,
        uvTimerCallback
    );
    
    if (uvTimer == NULL) {
        PRINTF("ERROR: Failed to create UV timer!\r\n");
        vTaskDelete(NULL);
    }
    
    while (1) {
        // CHANGED: Read single characters (Saheer sends '1' or '0')
        int receivedChar = uartReadChar();
        
        if (receivedChar != -1) {
            // CHANGED: Handle single characters instead of strings
            if (receivedChar == '1' || receivedChar == '0') {
                int knobValue = (receivedChar == '1') ? 1 : 0;
                
                PRINTF("Received from ESP32: '%c' -> knob value: %d\r\n", receivedChar, knobValue);
                
                // Use shared mutex to access resource allocation
                if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE) {
                    
                    if (knobValue == 1) {
                        // Request UV lighting resource
                        if (electricity_allocation == NOTHING) {
                            electricity_allocation = UV_LIGHTING;
                            setUVLED(true);
                            xTimerStart(uvTimer, 0);
                            PRINTF("Resource allocated: UV_LIGHTING\r\n");
                        } else {
                            PRINTF("Resource unavailable: %s active\r\n", 
                                   (electricity_allocation == BUZZER_PLAYING) ? "BUZZER" : "UV_LIGHT");
                        }
                    } 
                    else if (knobValue == 0) {
                        // Turn off UV light and release resource
                        setUVLED(false);
                        xTimerStop(uvTimer, 0);
                        if (electricity_allocation == UV_LIGHTING) {
                            electricity_allocation = NOTHING;
                            PRINTF("Resource released: NOTHING\r\n");
                        }
                    }
                    
                    xSemaphoreGive(resourceMutex);
                }
            }
            else {
                PRINTF("Unknown character received: '%c' (0x%02X)\r\n", receivedChar, receivedChar);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent overwhelming the CPU
    }
}

/* ============================================================================
 * TASK CREATION
 * ============================================================================ */

/**
 * @brief   Create all of Swetha's tasks
 */
void main(void)
{
    // Initialize hardware
    initUVLED();
    initUART2(9600);
    
    // Create UART task
    if (xTaskCreate(swethaUARTTask,
                    "SwethaUARTTask",
                    configMINIMAL_STACK_SIZE + 300,
                    NULL,
                    tskIDLE_PRIORITY + 1,
                    NULL) != pdPASS) {
        PRINTF("ERROR: Failed to create Swetha's UART task!\r\n");
        while (1);
    }
    
    PRINTF("Swetha's tasks created successfully: Ready to receive from ESP32!\r\n");
}