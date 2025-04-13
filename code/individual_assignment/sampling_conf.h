
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <arduinoFFT.h>
#include "esp_sleep.h"

//Function declatration
void mqttReconnect();
void generate_signal_task(void* pvParameters);
void sampling_signal_task(void* pvParameters);
void avg_transmission_task(void* pvParameters);
void find_max_freq();

// Signal ggeneration variables
const uint16_t samples = 64;                 // This value MUST ALWAYS be a power of 2
double sampling_frequency = 22000;           // Sampling frequency in Hz
const double signalFrequency1 = 150;           // Frequency of first sine wave (3 Hz)
const double signalFrequency2 = 400;           // Frequency of second sine wave (5 Hz)
const double amplitude1 = 2;                 // Amplitude of first sine wave
const double amplitude2 = 4;                 // Amplitude of second sine wave
const double generationSignalRate = 1000;

double sampling_period = (int)(1000.0 / sampling_frequency + 0.5); 

double ratio1 = twoPi * signalFrequency1 / 5000;  // Fraction of cycle for first sine wave
double ratio2 = twoPi * signalFrequency2 / 5000;  // Fraction of cycle for second sine wave

// FFT variables
double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, sampling_frequency);
// Variables to find max frequency
#define ADC_PIN 38
#define SAMPLE_RATE 1000000
unsigned long sampleCount = 0;
unsigned long startTime = 0;

// Queue used to continous sampling and rolling average and another to send info to edge server
#define WINDOW_SIZE 5
#define QUEUE_SIZE 10
QueueHandle_t samples_queue;
QueueHandle_t transmission_queue;



