// Signal generation variables
#include <arduinoFFT.h>

const uint16_t samples = 64;                 // This value MUST ALWAYS be a power of 2
const double sampling_frequency = 100;  // Sampling frequency in Hz
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
// Variables to find max frequency
#define ADC_PIN 38
#define SAMPLE_RATE 1000000
unsigned long sampleCount = 0;
unsigned long startTime = 0;


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

void signal_generation() {
  
  double ratio1 = twoPi * signalFrequency1 / sampling_frequency;  // Fraction of cycle for first sine wave
  double ratio2 = twoPi * signalFrequency2 / sampling_frequency;  // Fraction of cycle for second sine wave
  
  for (uint16_t i = 0; i < samples; i++) {
    // Generate the composite signal: 2*sin(2*pi*3*t) + 4*sin(2*pi*5*t)
    vReal[i] = (amplitude1 * sin(i * ratio1) / 2.0) + (amplitude2 * sin(i * ratio2) / 2.0);
    vImag[i] = 0.0;


    Serial.printf("Sample number %d : %lf \n", i, vReal[i]);
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */
  FFT.compute(FFTDirection::Forward);                       /* Compute FFT */
  FFT.complexToMagnitude();
  double x = FFT.majorPeak(); /* Compute magnitudes */

  Serial.printf("Peak Frequency: %.2f Hz\n", x);

  

}

void setup() {
  Serial.begin(115200);

  signal_generation();

}

void loop() {
  find_max_freq();
}
