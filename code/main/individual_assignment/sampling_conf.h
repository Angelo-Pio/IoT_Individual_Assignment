#include <arduinoFFT.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

void find_max_freq();
void find_best_sampling_freq();
void mqttReconnect();
void generate_signal_task(void* pvParameters);
void sampling_signal_task(void* pvParameters);
void avg_transmission_task(void* pvParameters);

// Signal ggeneration variables
const uint16_t samples = 64;                 // This value MUST ALWAYS be a power of 2
const double sampling_frequency = 22000;  // Sampling frequency in Hz
const double best_samplingFrequency = NULL;  // Sampling frequency in Hz
const double signalFrequency1 = 3;           // Frequency of first sine wave (3 Hz)
const double signalFrequency2 = 5;           // Frequency of second sine wave (5 Hz)
const double amplitude1 = 2;                 // Amplitude of first sine wave
const double amplitude2 = 4;                 // Amplitude of second sine wave
static const int sampling_period = (int)(1000.0 / sampling_frequency + 0.5); 

// FFT variables
double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, sampling_frequency);


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
