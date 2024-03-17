#include "AD9833.h"
#include "arduinoFFT.h"
#include "arm_math.h"

arduinoFFT FFT;
AD9833 AD(PA3, PA7, PA5);  //  SW SPI over the HW SPI pins (UNO);
//  AD(10);      //  HW SPI
#define PIN PA0
double freq = 42000.0;
double peak;

#define SAMPLES             512    // Must be a power of 2  // 128 - 1024
#define SAMPLING_FREQUENCY  12000  // Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;


/*Measurement duration D.
D = BL / fs.
At fs = 48 kHz and BL = 1024, this yields 1024/48000 Hz = 21.33 ms

Frequency resolution df.
df = fs / BL
At fs = 48 kHz and BL = 1024, this gives a df of 48000 Hz / 1024 = 46.88 Hz.
*/
double vReal[SAMPLES];
double vImag[SAMPLES];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  AD.begin();
  pinMode(PIN, INPUT);
  AD.setWave(AD9833_SINE);
  Serial.println(AD.getWave());

  AD.setFrequency(freq, 0);
  Serial.println(AD.getFrequency(0));
  AD.setFrequencyChannel(0);
}


void loop() {
  // data = analogRead(PIN)-512;             // wait for 100mS
  // Serial.println(data);
  // microseconds = micros();
  // for (int i = 0; i < SAMPLES; i++) {
  //   vReal[i] = analogRead(PIN) - 512;
  //   vImag[i] = 0;
  //   while (micros() - microseconds < sampling_period_us) {
  //     //empty loop
  //   }
  //   microseconds += sampling_period_us;
  // }
  // FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
  /* Print the results of the sampling according to time */
  // Serial.println("Data:");
  // PrintVector(vReal, samples, SCL_TIME);
  // FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
  // Serial.println("Weighed data:");
  // PrintVector(vReal, samples, SCL_TIME);
  // FFT.Compute(FFT_FORWARD); /* Compute FFT */
  // Serial.println("Computed Real values:");
  // PrintVector(vReal, samples, SCL_INDEX);
  // Serial.println("Computed Imaginary values:");
  // PrintVector(vImag, samples, SCL_INDEX);
  // FFT.ComplexToMagnitude(); /* Compute magnitudes */
  // Serial.println("Computed magnitudes:");
  // PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  fft_analysis();
  Serial.println(peak);  //Print out what frequency is the most dominant.

  // delay(500);
}
//  -- END OF FILE --

void fft_analysis() {
  /*SAMPLING*/
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();  //Overflows after around 70 minutes!

    vReal[i] = analogRead(PIN);
    vImag[i] = 0;

    while (micros() < (microseconds + sampling_period_us)) {
    }
  }

  /*FFT*/
  FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  peak = FFT.MajorPeak();
}