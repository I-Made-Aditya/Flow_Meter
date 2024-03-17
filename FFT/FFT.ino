#include "ArduinoFFT.h"


#define TRIGGER_PIN PA1  // Pin trigger sensor ultrasonik terhubung ke pin 2
#define ECHO_PIN PA0      // Pin echo sensor ultrasonik terhubung ke pin 3
#define SAMPLES 128
#define SAMPLING_FREQUENCY 1000

ArduinoFFT FFT = ArduinoFFT();

void setup() {
  // initialize digital pin PB2 as an output.
  Serial.begin(115200);
  
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  FFT.begin(SAMPLES, SAMPLING_FREQUENCY);

  while(!Serial);
  Serial.println("Ready");
}
void loop() {
  // Serial.println("Ultrasonic Sensor Initialized");
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH);
  // Serial.println(duration);
  float distance = duration * 0.034 / 2;
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" cm");

  delay(80);  // wait for 100mS

}