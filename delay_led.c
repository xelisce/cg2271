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

#define MONITOR_ON 1

// LED pin numbers
#define RED_LED_PIN 13 // PTA13

void initLED()
{
    // Set up the clock gating
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
    PORTA->PCR[RED_LED_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTA->PCR[RED_LED_PIN] = PORT_PCR_MUX(1);
    GPIOA->PDDR |= (1 << RED_LED_PIN);
}

void turnLEDOn()
{

    GPIOA->PCOR |= (1 << RED_LED_PIN);
}

void turnLEDOff()
{
    GPIOA->PSOR |= (1 << RED_LED_PIN);
}

/**
 * @brief   Monitor task - periodically displays electricity allocation status
 */
void ledTask(void *pvParameters)
{
    PRINTF("LED task started\r\n\r\n");
    while (1)
    {
        // Wait 2 seconds between status updates
        PRINTF("Turning LED On");
        turnLEDOn();
        vTaskDelay(pdMS_TO_TICKS(2000));
        PRINTF("Turning LED Off");
        turnLEDOff();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void createLEDTask()
{
    if (xTaskCreate(ledTask,
                    "LEDTask",
                    configMINIMAL_STACK_SIZE + 100,
                    NULL,
                    tskIDLE_PRIORITY + 1, // higher priority is higher
                    NULL) != pdPASS)
    {
        PRINTF("ERROR: Failed to create LED task!\r\n");
        while (1)
            ;
    }
    PRINTF("LED task created\r\n");
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

    initLED();
    createLEDTask();

    vTaskStartScheduler();

    // Should never reach here
    PRINTF("ERROR: Scheduler failed to start!\r\n");
    while (1)
    {
        __asm volatile("nop");
    }

    return 0;
}
