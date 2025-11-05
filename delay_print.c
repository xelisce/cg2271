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


/**
 * @brief   Monitor task - periodically displays electricity allocation status
 */
void monitorTask(void *pvParameters)
{
  PRINTF("Monitor task started\r\n\r\n");
  while (1)
  {
    // Wait 2 seconds between status updates
    vTaskDelay(pdMS_TO_TICKS(2000));
    PRINTF("Hello world");
  }
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

#if MONITOR_ON == 1
  createMonitorTask();
#endif
 
  vTaskStartScheduler();

  // Should never reach here
  PRINTF("ERROR: Scheduler failed to start!\r\n");
  while (1)
  {
    __asm volatile("nop");
  }

  return 0;
}