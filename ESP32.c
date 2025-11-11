#include <Arduino.h>
#include "driver/uart.h"

// ---------- CONFIG ----------
const int RX2_PIN = 18;
const int TX2_PIN = 17;
const uart_port_t UART_NUM = UART_NUM_1;

// RGB LED pins
const int PIN_R = 20;
const int PIN_G = 19;
const int PIN_B = 21;

// Ultrasonic sensor pins
const int TRIG_PIN = 14;
const int ECHO_PIN = 15;
const float DISTANCE_THRESHOLD_CM = 15.0;
const unsigned long SEND_INTERVAL_MS = 1000UL; // send via uart every 60 seconds

// Flame sensor (KY-026)
#define FLAME_ANALOG_PIN A0  // KY-026 analog pin
#define FLAME_THRESHOLD 7200 // lower value = stronger flame

// ---------- GLOBALS ----------
volatile bool pressOccurred = false;
volatile bool senseFlame = false;
unsigned long lastSendMillis = 0;

volatile char moistureLevel = '3';
portMUX_TYPE moistureMux = portMUX_INITIALIZER_UNLOCKED;

#define FLAME_ANALOG_PIN A0 // KY-026 A0 → Arduino A0

// UART event queue
QueueHandle_t uart_queue;

// ---------- LED HELPERS ----------
void setRed()
{
  digitalWrite(PIN_R, HIGH);
  digitalWrite(PIN_G, LOW);
  digitalWrite(PIN_B, LOW);
}

void setYellow()
{
  digitalWrite(PIN_R, HIGH);
  digitalWrite(PIN_G, HIGH);
  digitalWrite(PIN_B, LOW);
}

void setGreen()
{
  digitalWrite(PIN_R, LOW);
  digitalWrite(PIN_G, HIGH);
  digitalWrite(PIN_B, LOW);
}

void setOff()
{
  digitalWrite(PIN_R, LOW);
  digitalWrite(PIN_G, LOW);
  digitalWrite(PIN_B, LOW);
}

// ---------- ULTRASONIC SENSOR ----------
float getDistance()
{
  // Send 10us pulse to trigger
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo pulse duration
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout

  // Calculate distance in cm (speed of sound = 343 m/s)
  // distance = (duration / 2) / 29.1
  if (duration == 0)
  {
    return -1; // No echo received
  }

  float distance = duration * 0.0343 / 2.0;
  return distance;
}

// ---------- FLAME SENSOR ----------
bool isFlameDetected()
{
  int flameValue = analogRead(FLAME_ANALOG_PIN);
  //  Serial.print("Flame analog value: ");
  //  Serial.print(flameValue);
  //  Serial.print(" | Threshold: ");
  //  Serial.println(FLAME_THRESHOLD);
  return (flameValue < FLAME_THRESHOLD); // true if flame detected
}

// ---------- UART EVENT TASK ----------
void uartEventTask(void *pvParameters)
{
  uart_event_t event;
  uint8_t data[128];

  while (true)
  {
    // Wait for UART event
    if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
    {
      if (event.type == UART_DATA)
      {
        // Read available data
        int len = uart_read_bytes(UART_NUM, data, event.size, 100 / portTICK_PERIOD_MS);

        for (int i = 0; i < len; i++)
        {
          if (data[i] == '1' || data[i] == '2' || data[i] == '3')
          {
            portENTER_CRITICAL(&moistureMux);
            moistureLevel = data[i];
            portEXIT_CRITICAL(&moistureMux);
            //            Serial.print("Received moisture level: ");
            //            Serial.println((char)data[i]);
          }
        }
      }
    }
  }
}

// ---------- SETUP ----------
void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("ESP32-S2 UART moisture example...");

  const int uart_buffer_size = 256;
  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT,
  };

  // Configure UART parameters
  uart_param_config(UART_NUM, &uart_config);

  // Set UART pins
  uart_set_pin(UART_NUM, TX2_PIN, RX2_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Install UART driver with event queue
  uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 20, &uart_queue, 0);

  // Create task to handle UART events
  xTaskCreate(uartEventTask, "uart_event_task", 2048, NULL, 12, NULL);

  // Initialize LED pins as OUTPUT
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  setOff();

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  lastSendMillis = millis();

  Serial.println("Setup complete!");
}

void loop()
{
  char level;
  portENTER_CRITICAL(&moistureMux);
  level = moistureLevel;
  portEXIT_CRITICAL(&moistureMux);

  // Serial.print("Current level: ");
  // Serial.println(level);

  // Update LED color based on moisture level
  switch (level)
  {
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

  // Read ultrasonic sensor
  float distance = getDistance();

  if (distance > 0 && distance <= DISTANCE_THRESHOLD_CM)
  {
    if (!pressOccurred)
    { // Only set once per detection
      pressOccurred = true;
      //      Serial.print("Object detected at ");
      //      Serial.print(distance);
      //      Serial.println(" cm");
    }
  }
  else if (distance > DISTANCE_THRESHOLD_CM)
  {
    // Optionally log distance when no object detected
    // Serial.print("Distance: ");
    // Serial.print(distance);
    // Serial.println(" cm");
  }

  bool flameDetected = isFlameDetected();

  if (flameDetected)
  {
    // Serial.println("🔥 Flame detected!");
    senseFlame = true;
  }
  else
  {
    // Serial.println("No flame.");
  }

  unsigned long now = millis();
  if (now - lastSendMillis >= SEND_INTERVAL_MS)
  {
    lastSendMillis = now;
    char firstChar = pressOccurred ? 'y' : 'n'; // object detection
    char secondChar = senseFlame ? 'x' : 'o';   // flame detection

    char outMsg[3] = {firstChar, secondChar, '\0'};
    uart_write_bytes(UART_NUM, outMsg, 2);

    Serial.print("Sent: ");
    Serial.print(outMsg[0]);
    Serial.print(outMsg[1]);
    Serial.println();
    if (senseFlame)
    {
      Serial.println("FIREEEEEEEEEEEE");
    }

    pressOccurred = false; // reset after send
    senseFlame = false;
  }

  delay(10);
}