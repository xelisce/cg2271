#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

/* ==============================
 * Definitions
 * ============================== */
#define BAUD_RATE        9600
#define ADC_PIN          20
#define UART_TX_PTE22    22
#define UART_RX_PTE23    23
#define UART2_INT_PRIO   128
#define ADC_CHANNEL      3
#define MAX_MSG_LEN      16

#define GOOD_MOISTURE 3000  // Very wet
#define MEDIUM_MOISTURE  1500  // Medium wet

/* ==============================
 * Globals
 * ============================== */
char send_buffer[MAX_MSG_LEN];


// // UART + ADC Function Prototypes
// void initUART2(uint32_t baud_rate);
// void sendMessage(char *message);
// void initADC_poll(void);
// uint16_t read_adc_channel_poll(int channel);
// static uint8_t adc_to_level(uint16_t raw);


// ADC Setup
void initADC_poll(void)
{
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // Set pin for ADC input (e.g., PTE22 → ADC0_SE3)
    PORTE->PCR[ADC_PIN] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[ADC_PIN] |= PORT_PCR_MUX(0);

    // ADC0 configuration
    ADC0->CFG1 &= ~ADC_CFG1_MODE_MASK;
    ADC0->CFG1 |= ADC_CFG1_MODE(0b01);   // 12-bit
    ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;    // software trigger
    ADC0->SC3 &= ~ADC_SC3_AVGE_MASK;     // disable averaging
}

uint16_t read_adc_channel_poll(int channel)
{
    ADC0->SC1[0] = ADC_SC1_ADCH(channel);
    while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)) {
        ; // wait for conversion
    }
    return (uint16_t)ADC0->R[0];
}

// scales from 0 to 4096
static uint8_t adc_to_level(uint16_t raw)
{
    if (raw > GOOD_MOISTURE) return 3;
    else if (raw > MEDIUM_MOISTURE) return 2;
    else return 1;
}


// UART Initialization + IRQ
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

    UART2->C1 = 0x00; // 8-bit, no parity
    UART2->C2 |= UART_C2_RIE_MASK; // enable RX interrupt (not strictly needed here)
    UART2->C2 |= UART_C2_RE_MASK;  // enable receiver (for safety)

    NVIC_SetPriority(UART2_FLEXIO_IRQn, UART2_INT_PRIO);
    NVIC_ClearPendingIRQ(UART2_FLEXIO_IRQn);
    NVIC_EnableIRQ(UART2_FLEXIO_IRQn);
}


// UART IRQ
void UART2_FLEXIO_IRQHandler(void)
{
    static int send_ptr = 0;

    if (UART2->S1 & UART_S1_TDRE_MASK)
    {
        if (send_buffer[send_ptr] == '\0') {
            send_ptr = 0;
            UART2->C2 &= ~UART_C2_TIE_MASK;
            UART2->C2 &= ~UART_C2_TE_MASK;
        } else {
            UART2->D = send_buffer[send_ptr++];
        }
    }
}


void sendMessage(char *message)
{
    strncpy(send_buffer, message, MAX_MSG_LEN);
    UART2->C2 |= UART_C2_TIE_MASK;
    UART2->C2 |= UART_C2_TE_MASK;
}


// Soil Moisture Task
static void soilTask(void *p)
{
    const TickType_t delay = pdMS_TO_TICKS(1000);
    while (1)
    {
        uint16_t raw = read_adc_channel_poll(ADC_CHANNEL);
        uint8_t level = adc_to_level(raw);

        char msg[4];
        sprintf(msg, "%c\n", '0' + level);

        PRINTF("Raw ADC: %d, Level: %d\r\n", raw, level);

        sendMessage(msg);

        vTaskDelay(delay);
    }
}


int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    BOARD_InitDebugConsole();
#endif

    initADC_poll();
    initUART2(BAUD_RATE);

    PRINTF("Soil Moisture TX Started\r\n");

    xTaskCreate(soilTask, "SoilTask", configMINIMAL_STACK_SIZE + 200, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1) {
        __asm volatile("nop");
    }
}
