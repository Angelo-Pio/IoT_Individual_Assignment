

#include <arduinoFFT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define COMMUNICATION_METHOD 2  // 1 for WiFi-MQTT 2 for LoraWan-TTN

#if COMMUNICATION_METHOD == 1

#include <WiFi.h>
#include <PubSubClient.h>
#include <NetworkClientSecure.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "secrets.h"

// WiFi connection variables
const char* ssid = "Angelo";
const char* password = "Pompeo00";
const char* mqttServer = AWS_IOT_ENDPOINT;  //ip address of smartphone
int port = 8883;
const char* topic = "angelo/signal";
char clientId[50];

WiFiClientSecure espClient;
PubSubClient client(espClient);

#elif COMMUNICATION_METHOD == 2

##include <LoRa.h>

#include "ttn.h"


#define LORA_RST 14        // LoRa Reset pin
#define LORA_MOSI 27       // LoRa MOSI pin
#define LORA_MISO 19       // LoRa MISO pin
#define LORA_SCK 5        // LoRa SCK pin
#define LORA_CS 18         // LoRa CS pin

//#define LORA_IRQ 26        // LoRa IRQ pin

// TTN credentials
const char * appEui = JOINEUI;   // Replace with your AppEUI
const char * appKey = APPKEY;   // Replace with your AppKey
const char * devEui = DEVEUI;   // Replace with your DevEUI

#endif


// Signal ggeneration variables
const uint16_t samples = 64;                 // This value MUST ALWAYS be a power of 2
const double old_samplingFrequency = 11;  // Sampling frequency in Hz
const double best_samplingFrequency = NULL;  // Sampling frequency in Hz
const double signalFrequency1 = 3;           // Frequency of first sine wave (3 Hz)
const double signalFrequency2 = 5;           // Frequency of second sine wave (5 Hz)
const double amplitude1 = 2;                 // Amplitude of first sine wave
const double amplitude2 = 4;                 // Amplitude of second sine wave

// FFT variables
double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, old_samplingFrequency);


// Queue used to continous sampling and rolling average and another to send info to edge server
#define WINDOW_SIZE 5
#define QUEUE_SIZE 4
QueueHandle_t samples_queue;
QueueHandle_t transmission_queue;


// Variables to find max frequency
#define ADC_PIN 38
#define SAMPLE_RATE 1000000
unsigned long sampleCount = 0;
unsigned long startTime = 0;





void find_max_freq();
void find_best_sampling_freq();
void mqttReconnect();
void send_over_wifi_mqtt(double val);
void generate_signal_task(void* pvParameters);
void sampling_signal_task(void* pvParameters);
void avg_transmission_task(void* pvParameters);



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

  Serial.println(AWS_IOT_ENDPOINT);

  espClient.setCACert(AWS_CERT_CA);
  espClient.setCertificate(AWS_CERT_CRT);
  espClient.setPrivateKey(AWS_CERT_PRIVATE);

  client.setServer(mqttServer, port);

#elif COMMUNICATION_METHOD == 2
  
 

#endif

  //Find max sampling frequency v1
  //startTime = millis();

  //Generate and sample the signal

  samples_queue = xQueueCreate(WINDOW_SIZE, sizeof(double));
  if (samples_queue == 0) {
    printf("Failed to create queue= %p\n", samples_queue);
  }

  transmission_queue = xQueueCreate(QUEUE_SIZE, sizeof(double));
  if (transmission_queue == 0) {
    printf("Failed to create queue= %p\n", transmission_queue);
  }

  xTaskCreate(generate_signal_task, "generate_signal_task", 4096, NULL, 1, NULL);
  xTaskCreate(sampling_signal_task, "sampling_signal_task", 4096, NULL, 1, NULL);
  xTaskCreate(avg_transmission_task, "avg_transmission_task", 4096, NULL, 1, NULL);
}

void loop() {

}

void avg_transmission_task(void* pvParameters) {

  double avg_freq;
  char msg[100];

  while (1) {
    if (xQueueReceive(transmission_queue, &avg_freq, pdMS_TO_TICKS(2000))) {

      Serial.printf("Received message to publish on queue trasmission_queue: %.2f \n", avg_freq);
#if COMMUNICATION_METHOD == 1
      send_over_wifi_mqtt(avg_freq);
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
    if (xQueueReceive(samples_queue, &sample, pdMS_TO_TICKS(2000))) {

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
      printf("Average Frequence: %.2f Hz\n", avgFrequency);
      xQueueSend(transmission_queue, &avgFrequency, (TickType_t)0);
    }
  }
}


void generate_signal_task(void* pvParameters) {
  
  /*
  double ratio1 = twoPi * signalFrequency1 / old_samplingFrequency;  // Fraction of cycle for first sine wave
  double ratio2 = twoPi * signalFrequency2 / old_samplingFrequency;  // Fraction of cycle for second sine wave
  for (uint16_t i = 0; i < samples; i++) {
    // Generate the composite signal: 2*sin(2*pi*3*t) + 4*sin(2*pi*5*t)
    vReal[i] = (amplitude1 * sin(i * ratio1) / 2.0) + (amplitude2 * sin(i * ratio2) / 2.0);
    vImag[i] = 0.0;


    Serial.printf("Sample number %d : %lf \n", i, vReal[i]);
  }

  // Apply windowing (optional, can reduce spectral leakage)
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);

  // Compute the FFT (forward FFT)
  FFT.compute(FFT_FORWARD);

  

  // Convert complex FFT result to magnitude (for frequency analysis)
  FFT.complexToMagnitude();

  // Output the frequency spectrum (peak frequencies)
  double peakFrequency1 = FFT.majorPeak();  // Find the dominant frequency in the FFT result
  Serial.print("Dominant Frequency: ");
  Serial.println(peakFrequency1, 6);  // Print the peak frequency detected
  */
  
  double val = 6.0;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {

    for (int i = 0; i < 50; i++) {

      val = (double)i;


      xQueueSend(samples_queue, &val, (TickType_t)0);
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));  // We use vTaskDelayUntil to get more precise delay rate
    }
  }

  while(1){}
  delay(2000);
}


void find_max_freq() {

  int adc_value = analogRead(ADC_PIN);  // Read ADC value from specified pin
  sampleCount++;                        // Increment the sample count

  // Every second, calculate the sampling rate and print it
  if (millis() - startTime >= 1000) {
    unsigned long elapsedTime = millis() - startTime;
    float frequency = sampleCount / (elapsedTime / 1000.0);
    Serial.print("Sample Rate: ");
    Serial.print(frequency);
    Serial.println(" Hz");

    startTime = millis();
    sampleCount = 0;
  }
}

void find_best_sampling_freq() {

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */
  FFT.compute(FFTDirection::Forward);                       /* Compute FFT */
  FFT.complexToMagnitude();
  double x = FFT.majorPeak(); /* Compute magnitudes */

  Serial.printf("Peak Frequency: %.2f Hz\n", x);
}

#if COMMUNICATION_METHOD == 1
void mqttReconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    //long r = random(10);
    sprintf(clientId, "clientId-%ld", 10);

    if (client.connect(clientId)) {
      Serial.println(" connected");
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

  snprintf(msg, MSG_BUFFER_SIZE, "%.2f", val);
  Serial.printf("Publishing message: %.2f to topic %s \n", val, topic);
  client.publish(topic, msg);
}
#endif
