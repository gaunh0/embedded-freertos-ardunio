

#include <Adafruit_SHT31.h>
#include <Arduino_FreeRTOS.h>
#include <SoftwareSerial.h>

#define NODE 1100

typedef struct {
  char command; // 'g' for get, 's' for set, 'r' for response
  unsigned int nodeAddress;
  int value1; // First value (e.g., temperature or relay state)
  int value2; // Second value (e.g., humidity or error code)
} LoRaMessage;

#define BAUDRATE 9600

#define LED_PIN 13
float humidity;
float temperature;

Adafruit_SHT31 sht31 = Adafruit_SHT31();
SoftwareSerial loraSerial(11, 10); // RX, TX

// define two tasks for Sensor & LoraProcess
void TaskSensor(void *pvParameters);
void TaskLoraProcess(void *pvParameters);
LoRaMessage parseLoRaMessage(const char *message);

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(BAUDRATE);
  loraSerial.begin(BAUDRATE);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO,
      // MICRO, YUN, and other 32u4 based boards.
  }

  Serial.println("SHT31 inital");
  if (!sht31.begin(0x44)) { // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1)
      delay(1);
  }

  // Just need this piece of code for LORA module
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Now set up two tasks to run independently.
  xTaskCreate(TaskSensor, "Sensor" // A name just for humans
              ,
              128 // This stack size can be checked & adjusted by reading the
                  // Stack Highwater
              ,
              NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the
                      // highest, and 0 being the lowest.
              ,
              NULL);

  xTaskCreate(TaskLoraProcess, "LoraProcess", 128 // Stack size
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

void TaskSensor(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  // Task get sensor value every one second

  for (;;) // A Task shall never return or exit.
  {
    float t = sht31.readTemperature();
    float h = sht31.readHumidity();

    if (!isnan(t)) { // check if 'is not a number'
      Serial.print("Temp *C = ");
      Serial.print(t);
      Serial.print("\t\t");
      temperature = t * 100;
    } else {
      Serial.println("Failed to read temperature");
      temperature = -1;
    }

    if (!isnan(h)) { // check if 'is not a number'
      Serial.print("Hum. % = ");
      Serial.println(h);
      humidity = h * 100;
    } else {
      humidity = -1;
      Serial.println("Failed to read humidity");
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // wait for five second
  }
}

void TaskLoraProcess(void *pvParameters) // This is a task.
{
  (void)pvParameters;
  String jsonBufferRx;

  for (;;) {
    if (loraSerial.available()) {
      jsonBufferRx = loraSerial.readString();
      Serial.print("Received: ");
      Serial.println(jsonBufferRx);
      LoRaMessage receivedMessage = parseLoRaMessage(jsonBufferRx.c_str());

      // only response if the address match
      if (NODE == receivedMessage.nodeAddress) {
        if (receivedMessage.command == 'g') {
          // Handle "get" command
          // Send sensor data as a response
          char response[50];
          snprintf(response, sizeof(response), "rp{%d:%d:%d}",
                   receivedMessage.nodeAddress, (int)temperature,
                   (int)humidity);
          loraSerial.write((uint8_t *)response, strlen(response));

        } else if (receivedMessage.command == 's') {
          // Handle "set" command
          // You can implement the relay control logic here
          // For now, we'll send a response indicating success
          char successResponse[30];
          if (receivedMessage.value1 == 1) {
            digitalWrite(LED_PIN, HIGH);
            snprintf(successResponse, sizeof(successResponse), "rp{%d:0:0}",
                     receivedMessage.nodeAddress);
          } else if (receivedMessage.value1 == 0) {
            digitalWrite(LED_PIN, LOW);
            snprintf(successResponse, sizeof(successResponse), "rp{%d:0:0}",
                     receivedMessage.nodeAddress);
          } else {
            snprintf(successResponse, sizeof(successResponse), "rp{%d:0:3}",
                     receivedMessage.nodeAddress);
          }

          loraSerial.write((uint8_t *)successResponse, strlen(successResponse));

        } else {
          // Invalid command, send an error response
          char invalidCommandResponse[30];
          snprintf(invalidCommandResponse, sizeof(invalidCommandResponse),
                   "rp{%d:0:2}", receivedMessage.nodeAddress);
          loraSerial.write((uint8_t *)invalidCommandResponse,
                           strlen(invalidCommandResponse));
        }
      }
      // one tick delay (100ms) in between reads for stability
      vTaskDelay(pdMS_TO_TICKS(10));
    }
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
