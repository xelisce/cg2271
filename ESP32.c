#include <Arduino.h>
#include "driver/uart.h"  

// ---------- CONFIG ----------
const int RX2_PIN = 16;
const int TX2_PIN = 17;
const int UART_NUM = UART_NUM_2;  //for custom rx/tx pins

// RGB LED pins
const int PIN_R = 21;
const int PIN_G = 22;
const int PIN_B = 23;

// PWM configuration
const int CH_R = 0;
const int CH_G = 1;
const int CH_B = 2;
const int PWM_FREQ = 5000;
const int PWM_RES = 8;

// Knob pin
const int KNOB_PIN = 14;
const unsigned long SEND_INTERVAL_MS = 60000UL;

// ---------- GLOBALS ----------
volatile int knobReading = 0;
unsigned long lastSendMillis = 0;

// Moisture status shared between ISR and loop, read by Loop, written by ISR
volatile char moistureLevel = '0';  
portMUX_TYPE moistureMux = portMUX_INITIALIZER_UNLOCKED;

// ---------- LED HELPERS ----------
void setColor(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(CH_R, r);
  ledcWrite(CH_G, g);
  ledcWrite(CH_B, b);
}

void setRed() { 
  setColor(255, 0, 0); 
  }

void setYellow() {
  setColor(255, 255, 0); 
  }

void setGreen() {
  setColor(0, 255, 0); 
  }

void setOff() {
  setColor(0, 0, 0); 
  }

// ---------- UART ISR ----------
static void IRAM_ATTR uartISR(void *arg) {   
  uint16_t rx_fifo_len;
  uint16_t i;
  uint8_t data[128];

  // Get number of bytes in UART buffer
  rx_fifo_len = UART2.status.rxfifo_cnt;
  for (i = 0; i < rx_fifo_len; i++) {
    uint8_t c = UART2.fifo.rw_byte;
    if (c == '1' || c == '2' || c == '3') {
      portENTER_CRITICAL_ISR(&moistureMux);
      moistureLevel = c;   // Update moisture level atomically
      portEXIT_CRITICAL_ISR(&moistureMux);
    }
  }
  // Clear UART interrupt flags
  uart_clear_intr_status(UART_NUM_2, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("ESP32 UART moisture interrupt example...");

  const int uart_buffer_size = 256;
  uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };

  uart_param_config(UART_NUM, &uart_config);
  uart_set_pin(UART_NUM, TX2_PIN, RX2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(UART_NUM, uart_buffer_size, 0, 0, NULL, 0);
  uart_isr_register(UART_NUM, uartISR, NULL, ESP_INTR_FLAG_IRAM, NULL);
  uart_enable_rx_intr(UART_NUM);   // Enable RX interrupt

  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(CH_G, PWM_FREQ, PWM_RES);
  ledcSetup(CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_R, CH_R);
  ledcAttachPin(PIN_G, CH_G);
  ledcAttachPin(PIN_B, CH_B);
  setOff();

  pinMode(KNOB_PIN, INPUT_PULLUP);
  lastSendMillis = millis();
}

void loop() {
  char level;
  portENTER_CRITICAL(&moistureMux);
  level = moistureLevel;
  portEXIT_CRITICAL(&moistureMux);

  // Update LED color based on moisture level
  switch (level) {
    case '1':
      setRed();
      break;
    case '2':
      setYellow();
      break;
    case '3':
      setGreen();
      break;
    default:
      setOff();
      break;
  }

  // Read knob value
  int raw = digitalRead(KNOB_PIN);
  knobReading = (raw == HIGH) ? 1 : 0;

  unsigned long now = millis();
  if (now - lastSendMillis >= SEND_INTERVAL_MS) {
    lastSendMillis = now;
    char outChar = (knobReading == 1) ? '1' : '0';
    Serial2.write(outChar);
    Serial.print("Sent knobReading: ");
    Serial.println(outChar);
  }

  delay(10);
}
