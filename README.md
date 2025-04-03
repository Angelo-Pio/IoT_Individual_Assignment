# Introduction

The goal of this assignment is to study and prove to build an IoT infrastructurev that is able to:
* Find the maximum sampling frequency of the esp32 board used 
* Generate a signal
* Sample the signal using the maximum frequency found and then adjust the sampling frequency through the help of the FFT
* Compute the average of the signal samples over a window
* Trasmit such results to an edge computer using wireless trasmission mediums 
* Measure a series of relevant metrics.

We will follow a step by step approach to show our work and our results.

# Setting up the environment

Our study requires a signal source to analyze, without it we cannot procede, so here we will show the technologies used and how the signal is locally generated, the signal we will generate ahs the following form: __2\*sin(2\*pi*3\*t)+4\*sin(2\*pi*5\*t)__.

The file `individual_assignment.ino` contains all the code of the project, in order to improve compilation efficiency and make the code cleaner and easier to use to someone who wants to test it, we have added some preprocessor directives.

These directive let you for example decide which kind of communication stack to use (either WiFi-MQTT or LoraWAN-TTN) by simply editing the MACRO `COMMUNICATION_METHOD`, set it to 1 for WiFi or 2 for LoraWAN.

There is another file called `sampling_frequency.ino` in which we do some tests to find the maximum sampling frequency the board is capable of and the best sampling frequency for the signal we generate.

# Finding the maximum sampling frequency


We want to find out which is the real maximum sampling frequency we can achieve using the ADC pin of our esp32 board, the sampling frequency is measured in Hz. 
The function _find\_max\_freq()_ is called in the _loop()_ so it is in a infinite loop where it reads a value from an analog pin enabled for ADC and increases a counter, after 1 second the counter is printed on the serial output and its value is reset to 0, giving us the number of samples taken in one seconds, hence the sampling frequency.
The output of this function is the following:

```
Sample Rate: 22032.00 Hz
```

We found out that the maximum sampling frequency is 22 kHz, this value seems low but not unexpected, from the datasheet the ADC pin of this board should be able to sample at maximum 100 kHz but here we are measuring this frequency in a very simple way, also there is a certain degree of overhead brought by the instructions, memory management, serial print etc. that slows down our function. 
This value however is very high for our purposes and we will show in the following section that is also too high.


# Identify optimal sampling frequency

We now know the maximum sampling frequency our board can achieve over the ADC pin (23 KHz), we will use this value to oversample the signal we generated, then thanks to the FFT function we will find the maximum frequency of the signal and adapt the sampling frequency to the double of such value, this will optimize the sampling by saving precious resources (energy,time,computational power etc.).

The function `signal_generation` generates our signal wave and using the FFT function provided by the arduinoFFT library finds the peak frequency of our signal.
We initially use as sampling frequency 22 kHz, this gives us as result a peak frequency of 8323.88Hz, this result is very strange, we know that the peak frequency of our signal is 5 Hz as we have generated it by ourselves.
Let's try to sample at 8300Hz, now the output is 
```
Peak Frequency: 3162.11 Hz
```
Still too high, we can try again and find as result 1200 Hz, so it lowers progessively but it is still too high compared to our expectations, let's try to sample at 100Hz.
In this case we get :

```
Peak Frequency: 5.01 Hz
```

Which is exactly what we expected, knowing this we can set the sampling frequency at 10Hz following the sampling theorem, to be sure we can set it to slightly larger value like 15 Hz.

The innaccuracies we found using the FFT function are probably related to how the library works and values of sampling frequency and singal frequecy too distant are not well handled by it.



# Generating the Signal Samples
We can now move to the `individual_assignment.ino` file to continue our study process.
Here we will change how we generate and sample the signal to better adapt this process to our study.
 
The process of generating signal samples is handled asynchronously within the `generate_signal_task`. This task simulates the creation of signal data by producing values and placing them into a queue. In this case, the task generates a simple signal pattern and sends each sample into the `samples_queue`. By using a dedicated task for signal generation, the system can continuously produce samples without blocking other operations, ensuring smooth real-time processing. The task communicates with the rest of the system through the queue, allowing for efficient data transfer between different processing steps.

# Sample the Signal Over a Sliding Window
To manage the incoming signal samples, the code employs a sliding window approach. The `sampling_signal_task` is responsible for receiving the generated signal samples from the `samples_queue`. As the signal samples arrive, they are stored in a circular buffer, and the task computes the average of the most recent samples to smooth out fluctuations. This task ensures that the system processes the data in manageable chunks and maintains an updated view of the signal’s characteristics. The sliding window technique ensures that only the most recent data is considered, keeping the computation efficient and relevant. The processed average is then passed through a separate queue, `transmission_queue`, for further handling.

# Sending the Aggregated Value to the MQTT Broker
We will send the aggregate value to our smartphone running mosquitto. The explanation on how to set up the mosquitto server on a Android device can be found above in the section **How to test**.

Once the aggregated signal value (such as the average frequency) is computed by the `sampling_signal_task`, it is passed through the `transmission_queue` to the `avg_transmission_task`. This task is responsible for taking the averaged result and sending it to an MQTT broker via the `send_over_wifi_mqtt` function. By separating the transmission logic into a dedicated task, the system ensures that the data is transmitted without interference from the data processing pipeline. The use of queues guarantees that values are sent in the correct order and without loss, providing reliable communication to external systems or clients. The asynchronous nature of the tasks and the queues allows for non-blocking, continuous operation even under high data loads.


