
// header file that includes main libraries used, configuration variables for sampling and queue handling
#include "sampling_conf.h"

#define COMMUNICATION_METHOD 1  // 1 for WiFi-MQTT 2 for LoraWan-TTN
#define LATENCY_TEST 1

// !! WiFi configuraiton !!

#if COMMUNICATION_METHOD == 1

#include <WiFi.h>
#include <PubSubClient.h>

//Function definition
void send_over_wifi_mqtt(double val);

// WiFi connection variables
const char* ssid = "Angelo";
const char* password = "Pompeo00";
const char* mqttServer = "192.168.56.34";  //ip address of smartphone
int port = 1883;
const char* topic = "angelo/signal";
char clientId[50];

WiFiClient espClient;
PubSubClient client(espClient);

#if LATENCY_TEST == 1

void callback(char* topic, byte* payload, unsigned int length);
unsigned long lastSentTime = 0;
#endif

#elif COMMUNICATION_METHOD == 2

#include "LoRaWan_APP.h"
#include "LoRaWAN_TTN_conf.h"  // file containing all lorawan configuration variables and functions
volatile double lora_average = 0.0;
#endif




// Use one task to sample , another to compute the avg with a sliding window  and make them communicate through a queue
void setup() {

  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("System Ready");
  delay(1000);

  analogReadResolution(12);

#if COMMUNICATION_METHOD == 1

  //Connection to MQTT broker
  Serial.println("WiFi connection setup");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println(".");
  }
  Serial.println("WiFi connection established");
  client.setServer(mqttServer, port);

  #if LATENCY_TEST == 1

  client.setCallback(callback);

  #endif

#elif COMMUNICATION_METHOD == 2

  xTaskCreate(send_over_lora_ttn, "send_over_lora_ttn", 4096, NULL, 1, NULL);


#endif

  //Create the queues used to exchange messages between tasks

  samples_queue = xQueueCreate(1, sizeof(double));
  if (samples_queue == 0) {
    printf("Failed to create queue= %p\n", samples_queue);
  }

  transmission_queue = xQueueCreate(1, sizeof(double));
  if (transmission_queue == 0) {
    printf("Failed to create queue= %p\n", transmission_queue);
  }

  // Create 3 tasks
  xTaskCreate(generate_signal_task, "generate_signal_task", 4096, NULL, 1, NULL); // This task internally generates the signal 
  xTaskCreate(sampling_signal_task, "sampling_signal_task", 4096, NULL, 1, NULL); // This task samples the signal using a sliding window
  xTaskCreate(avg_transmission_task, "avg_transmission_task", 4096, NULL, 1, NULL); // This task is responsible to handle the transmission of the aggregate value over either wifi or lora

  

}

void loop() {



#if COMMUNICATION_METHOD == 2
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
#if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
#endif
        LoRaWAN.init(loraWanClass, loraWanRegion);

        // Default data rate DR3 for EU868
        LoRaWAN.setDefaultDR(3);

        deviceState = DEVICE_STATE_JOIN;
        break;
      }

    case DEVICE_STATE_JOIN:
      {
        // OTAA in our case -> overTheAirActivation = true;
        LoRaWAN.join();
        break;
      }

    case DEVICE_STATE_SEND:
      {
        prepareTxFrame(appPort);
        LoRaWAN.send();

        deviceState = DEVICE_STATE_CYCLE;
        break;
      }

    case DEVICE_STATE_CYCLE:
      {
        // Next uplink in appTxDutyCycle plus random offset
        txDutyCycleTime =
          appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);  //defaulted by Heltec library to 1000ms

        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;
        break;
      }

    case DEVICE_STATE_SLEEP:
      {
        // Sleep radio until next cycle
        LoRaWAN.sleep(loraWanClass);
        break;
      }

    default:
      {
        deviceState = DEVICE_STATE_INIT;  // Reset to init state in case of error or disconnection
        break;
      }
  }
#endif
}

void avg_transmission_task(void* pvParameters) {

  double avg_freq;
  char msg[100];

  while (1) {
    if (xQueueReceive(transmission_queue, &avg_freq, pdMS_TO_TICKS(2000))) {


#if COMMUNICATION_METHOD == 1

      send_over_wifi_mqtt(avg_freq);

#elif COMMUNICATION_METHOD == 2

      lora_average = avg_freq;

#endif
    }
  }
}

void sampling_signal_task(void* pvParameters) {

  float sum = 0;
  double sample;
  float received_samples[WINDOW_SIZE] = { 0 };
  int pos = 0;
  int count = 0;


  while (1) {
    if (xQueueReceive(samples_queue, &sample, (TickType_t)0)) {

      //find_best_sampling_freq();
      received_samples[pos] = sample;
      pos = (pos + 1) % WINDOW_SIZE;     // Circular buffer index
      if (count < WINDOW_SIZE) count++;  // Ensure we don't exceed the array size
      // Compute the moving average
      float sum = 0;
      for (int i = 0; i < count; i++) {
        sum += received_samples[i];
      }
      double avgFrequency = sum / count;
      //printf("Average Frequence: %.2f Hz\n", avgFrequency);
      
      Serial.print(avgFrequency);
      Serial.print(" ");
      Serial.println(sample);

      xQueueOverwrite(transmission_queue, &avgFrequency); // used to create a circular buffer queue else we risk to block the queue or go in overflow
      vTaskDelay(pdMS_TO_TICKS(sampling_period));
    }
  }
}


void generate_signal_task(void* pvParameters) {
  
  double angle1 = 0.0;
  double angle2 = 0.0;
 
 while(1){

   double sample = (amplitude1 * sin(angle1) / 2.0) + (amplitude2 * sin(angle2) / 2.0) + 3.0;
   
   angle1 += ratio1;
   angle2 += ratio2;
   if (angle1 >= 2.0 * PI) {
    angle1 -= 2.0 * PI;
  }
  if (angle2 >= 2.0 * PI) {
    angle2 -= 2.0 * PI;
  }
   xQueueOverwrite(samples_queue, &sample);
   vTaskDelay(pdMS_TO_TICKS(1));
  }
}




#if COMMUNICATION_METHOD == 1

void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    //long r = random(10);
    sprintf(clientId, "clientId-%ld", 10);

    if (client.connect(clientId)) {
      Serial.println(" connected");
      client.subscribe("angelo/ack");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds \n");
      delay(5000);
    }
  }
}
void send_over_wifi_mqtt(double val) {

  const int MSG_BUFFER_SIZE = 100;
  char msg[MSG_BUFFER_SIZE];

  if (!client.connected()) {
    mqttReconnect();
  }
  client.loop();

  

  snprintf(msg, MSG_BUFFER_SIZE, "%.2f",val);

  #if LATENCY_TEST == 1
  lastSentTime = millis();

  sprintf(msg, "%lu-%.2f", lastSentTime , val);  // Send UNIX timestamp-value

  #endif

  Serial.printf("Publishing message: %.2f to topic %s \n", val, topic);
  
  client.publish(topic, msg);
}

#if LATENCY_TEST == 1
 
void callback(char* topic, byte* payload, unsigned int length) {
  // Convert the payload to string and extract the timestamp
  String payloadStr = "";
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  // Parse the timestamp from the payload (format: "timestamp:<timestamp_value>")
  unsigned long receivedTimestamp = payloadStr.substring(0,payloadStr.indexOf("-") + 1).toInt();

  // Calculate the round-trip latency (current time - sent timestamp)
  unsigned long latency = millis() - receivedTimestamp;
  Serial.print("Received message with timestamp: ");
  Serial.println(receivedTimestamp);
  Serial.print("Round-trip latency: ");
  Serial.print(latency);
  Serial.println(" ms");
}
#endif

#elif COMMUNICATION_METHOD == 2

static void prepareTxFrame(uint8_t port) {
  // Send rolling average as a 4-byte float
  float val = (float)lora_average;

  // The Heltec library gives us global appData[] & appDataSize to be set!
  memcpy(appData, &val, sizeof(float));
  appDataSize = sizeof(float);
}
#endif
