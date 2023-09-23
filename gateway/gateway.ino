// dont know why but MUST use this include

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define RXD2 16
#define TXD2 17

#define LORA_M0_PIN 18
#define LORA_M1_PIN 19

typedef struct {
  char command; // 'g' for get, 's' for set, 'r' for response
  unsigned int nodeAddress;
  int value1; // First value (e.g., temperature or relay state)
  int value2; // Second value (e.g., humidity or error code)
} LoRaMessage;

#define BAUDRATE 9600
// define two tasks for LoraSend & LoraReceive
void TaskLoraSend(void *pvParameters);
void TaskLoraReceive(void *pvParameters);
LoRaMessage parseLoRaMessage(const char *message);

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(BAUDRATE);
  Serial2.begin(BAUDRATE, SERIAL_8N1, RXD2, TXD2);

  pinMode(LORA_M0_PIN, OUTPUT);
  pinMode(LORA_M1_PIN, OUTPUT);
  digitalWrite(LORA_M0_PIN, LOW);
  digitalWrite(LORA_M1_PIN, LOW);

  Serial.println("GATEWAY ONLINE");

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO,
      // MICRO, YUN, and other 32u4 based boards.
  }

  // Now set up two tasks to run independently.
  xTaskCreate(TaskLoraSend, "LoraSend" // A name just for humans
              ,
              1000 // This stack size can be checked & adjusted by reading the
                   // Stack Highwater
              ,
              NULL, 1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the
                      // highest, and 0 being the lowest.
              ,
              NULL);

  xTaskCreate(TaskLoraReceive, "LoraReceive", 2000 // Stack size
              ,
              NULL, 1 // Priority
              ,
              NULL);

  // Now the task scheduler, which takes over control of scheduling individual
  // tasks, is automatically started.
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

/*
st{1122:0:0} turn off LED
st{1122:1:0} turn on LED
gt{1122:0:0} get environment value
*/

void TaskLoraSend(void *pvParameters) // This is a task.
{
  (void)pvParameters;
  int randomValue;
  String bufferRx;
  bool isOpen = false;
  for (;;) // A Task shall never return or exit.
  {
    if (Serial.available()) {
      bufferRx = Serial.readString();
      Serial.print("Received from computer: ");
      Serial.println(bufferRx);
      Serial2.print(bufferRx);
    }
    // randomValue = rand() % 20;
    // if (randomValue % 2 == 0) {
    //   // snprintf(requestFrame, sizeof(requestFrame), "gt{1122:0:0}");
    //   Serial2.print("gt{1122:0:0}");
    //   Serial.print("Send: ");
    //   Serial.println("gt{1122:0:0}");
    // } else {
    //   if (isOpen) {
    //     // snprintf(requestFrame, sizeof(requestFrame), "st{1122:0:0}");
    //     Serial2.print("st{1122:0:0}");
    //     Serial.print("Send: ");
    //     Serial.println("st{1122:0:0}");
    //     isOpen = !isOpen;
    //   } else {
    //     // snprintf(requestFrame, sizeof(requestFrame), "st{1122:0:1}");
    //     Serial2.print("st{1122:0:1}");
    //     Serial.print("Send: ");
    //     Serial.println("st{1122:0:1}");
    //   }
    // }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void TaskLoraReceive(void *pvParameters) // This is a task.
{
  (void)pvParameters;
  String jsonBufferRx;
  float temperature = 0;
  float humidity = 0;
  for (;;) {
    if (Serial2.available()) {
      jsonBufferRx = Serial2.readString();
      Serial.print("Received from node: ");
      Serial.println(jsonBufferRx);
      LoRaMessage receivedMessage = parseLoRaMessage(jsonBufferRx.c_str());

      if (receivedMessage.value1 == 0) {
        if (receivedMessage.value2 == 0) {
          Serial.println("Set command success");
        } else {
          Serial.println("Set command false");
        }
      } else {
        temperature = receivedMessage.value1 / 100.0;
        humidity = receivedMessage.value2 / 100.0;

        Serial.print("Temperature ");
        Serial.println(temperature);
        Serial.print("Humidity ");
        Serial.println(humidity);
      }
      // clear this mf variable to ensure everything null for the next request
      jsonBufferRx = "";
    }
    // one tick delay (100ms) in between reads for stability
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

LoRaMessage parseLoRaMessage(const char *message) {
  LoRaMessage result;

  if (strlen(message) < 8 || message[2] != '{' ||
      message[strlen(message) - 1] != '}') {
    // Invalid message format
    result.command = 'r'; // Response with error code
    result.nodeAddress = 0;
    result.value1 = 0;
    result.value2 = 1; // 1 for invalid message
    return result;
  }

  result.command = message[0];
  result.nodeAddress = atoi(&message[3]);

  if (result.command == 'g') {
    result.value1 = 0; // Initialize to default values for get command
    result.value2 = 0;
  } else if (result.command == 's' || result.command == 'r') {
    char *endptr;

    // Parse value1 and value2
    result.value1 = strtol(&message[8], &endptr, 10);
    if (*endptr != ':') {
      // Invalid format
      result.command = 'r'; // Response with error code
      result.value1 = 0;
      result.value2 = 2; // 2 for invalid format
      return result;
    }

    result.value2 = strtol(endptr + 1, NULL, 10);
  } else {
    // Invalid command
    result.command = 'r'; // Response with error code
    result.value1 = 0;
    result.value2 = 3; // 3 for invalid command
  }

  return result;
}
