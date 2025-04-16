
// header file that includes main libraries used, configuration variables for sampling and queue handling
#include "sampling_conf.h"

#define COMMUNICATION_METHOD 1  // 1 for WiFi-MQTT 2 for LoraWan-TTN
#define LATENCY_TEST 0
#define POWER_MEASURE 1  // Enable power measurement feature

// !! WiFi configuraiton !!

#if COMMUNICATION_METHOD == 1

#include <WiFi.h>
#include <PubSubClient.h>

//Function definition
void send_over_wifi_mqtt(double val);

// WiFi connection variables
const char* ssid = "Angelo";
const char* password = "Pompeo00";
const char* mqttServer = "192.168.150.60";  //ip address of smartphone
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

#endif




// Use one task to sample , another to compute the avg with a sliding window  and make them communicate through a queue
void setup() {

  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("System Ready");
  delay(1000);

  analogReadResolution(12);

  // Create the queues used to exchange messages between tasks
  samples_queue = xQueueCreate(QUEUE_SIZE, sizeof(double));
  if (samples_queue == 0) {
    printf("Failed to create samples_queue = %p\n", samples_queue);
  }

  transmission_queue = xQueueCreate(QUEUE_SIZE, sizeof(double));
  if (transmission_queue == 0) {
    printf("Failed to create transmission_queue = %p\n", transmission_queue);
  }


  // Create the tasks
  xTaskCreate(generate_signal_task, "generate_signal_task", 4096, NULL, 1, NULL); // This task internally generates the signal 
  xTaskCreate(sampling_signal_task, "sampling_signal_task", 4096, NULL, 1, NULL); // This task samples the signal using a sliding window

  #if COMMUNICATION_METHOD == 1
  xTaskCreate(wifi_transmission_task, "wifi_transmission_task", 4096, NULL, 2, NULL); // This task is responsible to handle the transmission of the aggregate value over either wifi or lora
  
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
  xTaskCreate(lorawan_transmission_task, "lorawan_transmission_task", 4096, NULL, 2, NULL);
  #endif
  

}

void loop() {}
// TASKS

//Computes the rolling average of the generated signal, samples are taken from a queue
void sampling_signal_task(void* pvParameters) {

  static bool adaptive_sampling_done = false;

  if (!adaptive_sampling_done) {
    findOptimalSamplingFrequency();
    adaptive_sampling_done = true;
  }

  float sum = 0;
  double sample;
  float received_samples[WINDOW_SIZE] = { 0 };
  int pos = 0;
  int count = 0;


  while (1) {
    if (xQueueReceive(samples_queue, &sample,  pdMS_TO_TICKS(sampling_period) )) {

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
      
      if (xQueueSend(transmission_queue, &avgFrequency, 0) != pdPASS) {
        // Serial.println("transmission_queue full — avg frequency dropped");
      }

      
    }
  }
}

//Generates the signal to analyse
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
    if (xQueueSend(samples_queue, &sample, 0) != pdPASS) {
      Serial.println("samples_queue full — signal dropped");
    }
  
   vTaskDelay(pdMS_TO_TICKS(1000/generationSignalRate));
  }
}


#if COMMUNICATION_METHOD == 1

//Transmits the rolling average through the communication method specified in COMMUNITCATION_METHOD
void wifi_transmission_task(void* pvParameters) {

  double avg_freq;

  while (1) {


    if (xQueueReceive(transmission_queue, &avg_freq, pdMS_TO_TICKS(1) )) {

      send_over_wifi_mqtt(avg_freq);

      #if POWER_MEASURE == 1

      const int SLEEP_DURATION_US = 5e6;         // 5 seconds (in microseconds)

        Serial.println("Entering deep sleep for power measurement...");

        // Configure wake-up timer
        esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);

        Serial.flush();  // Ensure all logs are printed before sleeping
        esp_deep_sleep_start();
      
      #endif

    }
  }
}



void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    long r = random(10);
    sprintf(clientId, "clientId-%ld", r);

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
  
  if (client.publish(topic, msg)) {
    Serial.println("Publish successful.");
  } else {
    Serial.println("Publish failed!");
  }
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
  Serial.println(receivedTimestamp/2);
  Serial.print("Round-trip latency: ");
  Serial.print(latency);
  Serial.println(" ms");
}
#endif

#elif COMMUNICATION_METHOD == 2

static void prepareTxFrame(uint8_t port, float val) {
  memcpy(appData, &val, sizeof(float));
  appDataSize = sizeof(float);
}

void lorawan_transmission_task(void* pvParameters) {
  float last_avg_value = 0.0;

  while (1) {
    // Try to receive a new average from queue if available (non-blocking)
    xQueueReceive(transmission_queue, &last_avg_value, 0);

    switch (deviceState) {
      case DEVICE_STATE_INIT:
      {
  #if (LORAWAN_DEVEUI_AUTO)
        LoRaWAN.generateDeveuiByChipID();
  #endif
        LoRaWAN.init(loraWanClass, loraWanRegion);
        LoRaWAN.setDefaultDR(3);
        deviceState = DEVICE_STATE_JOIN;
        break;
      }

      case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }

      case DEVICE_STATE_SEND:
      {
        prepareTxFrame(appPort, last_avg_value);
        LoRaWAN.send();
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }

      case DEVICE_STATE_CYCLE:
      {
        txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
        LoRaWAN.cycle(txDutyCycleTime);
        deviceState = DEVICE_STATE_SLEEP;

        #if POWER_MEASURE == 1

        const int SLEEP_DURATION_US = 5e6;         // 5 seconds (in microseconds)

        Serial.println("Entering deep sleep for power measurement...");

        // Configure wake-up timer
        esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);

        Serial.flush();  // Ensure all logs are printed before sleeping
        esp_deep_sleep_start();
      
      #endif
        break;
      }

      case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep(loraWanClass);
        break;
      }

      default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent tight loop
  }
}

#endif

// Auxiliary functions

// Finds optimal sampling frequeuncy

void findOptimalSamplingFrequency() {
  double sample;
  int collected = 0;

  // Collect samples from generated signal queue
  while (collected < samples) {
    if (xQueueReceive(samples_queue, &sample, pdMS_TO_TICKS(100))) {
      vReal[collected] = sample;
      vImag[collected] = 0;
      collected++;
    }
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Find peak frequency
  double peak = 0.0;
  int index = 0;
  for (int i = 1; i < samples / 2; i++) {
    if (vReal[i] > peak) {
      peak = vReal[i];
      index = i;
    }
  }

  double peakFrequency = (index * sampling_frequency) / samples;

  // Update frequency and period
  sampling_frequency = (2 * peakFrequency) + 5;
  sampling_period = (1000.0 / sampling_frequency + 0.5);
  Serial.printf("Peak frequency found is : %lf \n", peakFrequency);
  Serial.print("Adaptive sampling frequency set to: ");
  Serial.print(sampling_frequency);
  Serial.print(" Hz, period = ");
  Serial.print(sampling_period);
  Serial.println(" ms");
}
