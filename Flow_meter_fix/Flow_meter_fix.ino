#include "arduinoFFT.h"
#include <math.h>
#include <LiquidCrystal_I2C.h>
#include "AD9833.h"

arduinoFFT FFT;
//define I2C address......
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define PIN PA0
#define PI 3.14159265358979323846
#define DIAMETER 0.046  // Diameter pipa dalam meter (46 mm)
#define degree 45
#define vSoundF 1478.0  // water with 20C

long trans = 42000.0;

AD9833 AD(PA3, PA7, PA5);  //  SW SPI over the HW SPI pins (UNO);

const uint16_t samples = 4096;            //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 112500;  //Hz, must be less than 10000 due to ADC 
// const double samplingFrequency = 112000;  //Hz, must be less than 10000 due to ADC 
//the sampling frequency must be at least double the highest frequency of the signal
unsigned int sampling_period_us;
unsigned long microseconds;

/*Measurement duration D.
D = BL / fs.
At fs = 48 kHz and BL = 1024, this yields 1024/48000 Hz = 21.33 ms

Frequency resolution df.
df = fs / BL
At fs = 48 kHz and BL = 1024, this gives a df of 48000 Hz / 1024 = 46.88 Hz.
*/

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

double calculateFlowRate(double receivedFrequency, double transmissionFrequency, double relativeAngle);

void setup() {
  Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
  pinMode(PIN, INPUT);
  lcd.init();
  AD.begin();
  // lcd.clear();
  lcd.backlight();
  AD.setWave(AD9833_SINE);
  Serial.println(AD.getWave());
  AD.setFrequency(trans, 0);
  AD.getFrequency(0);
  Serial.println(AD.getFrequency(0));
}
void loop() {
  // data = analogRead(PIN)-512;             // wait for 100mS
  // Serial.println(data);

  double x = 0, Debit = 0;
  microseconds = micros();
  for (int i = 0; i < samples; i++) {
    vReal[i] = analogRead(PIN);
    vImag[i] = 0;
    while (micros() - microseconds < sampling_period_us) {
      //empty loop
    }
    microseconds += sampling_period_us;
  }
  FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
  /* Print the results of the sampling according to time */
  Serial.println("Data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
  Serial.println("Weighed data:");
  PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(FFT_FORWARD); /* Compute FFT */
  Serial.println("Computed Real values:");
  PrintVector(vReal, samples, SCL_INDEX);
  Serial.println("Computed Imaginary values:");
  PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(); /* Compute magnitudes */
  Serial.println("Computed magnitudes:");
  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  x = FFT.MajorPeak();
  // double x = FFT.MajorPeak();
  // x /= 1000;
  // 41263.766
  // if (x < 10000) {
  //   x = 0;
  // }
  // x /= 1000;
  Serial.println(x, 3);                           //Print out what frequency is the most dominant.
  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("Freq : " + String((x/1000),3) + "KHz");
  // Debit = calculateFlowRate(x, 42900.0, degree);  // niltembakan dari function generator ke transmit
  // Serial.println("Debit : " + String(Debit, 5));
  // lcd.setCursor(0,1);
  // lcd.print("Debit: " + String(Debit, 2) + "m^3/s");
  // lcd.print("Debit: " + String(Debit,5));
  // while(1); /* Run Once */
  delay(250);
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType) {
  for (uint16_t i = 0; i < bufferSize; i++) {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType) {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
        break;
    }
    Serial.print(abscissa, 6);
    if (scaleType == SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}

double calculateFlowRate(double receivedFrequency, double transmissionFrequency, double relativeAngle) {
  // Hitung kecepatan aliran menggunakan rumus efek Doppler
  double velocity = (vSoundF * (receivedFrequency - transmissionFrequency)) / (2 * transmissionFrequency * cos(relativeAngle));
  // Hitung luas penampang pipa
  double crossSectionArea = PI * pow((DIAMETER / 2.0), 2);
  // Hitung debit air
  double flowRate = crossSectionArea * velocity;
  if (receivedFrequency == 0 || receivedFrequency == transmissionFrequency) {
    flowRate = 0.0;
  }
  return fabs(flowRate);
}
