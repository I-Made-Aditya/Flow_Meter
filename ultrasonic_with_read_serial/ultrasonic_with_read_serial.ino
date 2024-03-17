#define TRIGGER_PIN 2 // Pin trigger sensor ultrasonik terhubung ke pin 2
#define ECHO_PIN 3    // Pin echo sensor ultrasonik terhubung ke pin 3

long incomeData = 0;

void setup() {
  Serial.begin(9600);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  long duration, distance = 0;

  // Kirim sinyal ultrasonik
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Baca waktu pantulan gelombang ultrasonik
  duration = pulseIn(ECHO_PIN, HIGH);
  // Serial.write(duration)
  // Hitung jarak dalam centimeter
  distance = duration * 0.034 / 2;
  Serial.write(distance);

  if(Serial.available() > 0){
    incomeData = Serial.read();
    Serial.write(incomeData);
  }

  delay(80); .
}

HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET); // Turn on LEDs
HAL_Delay(10)
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET); // Turn on LEDs
HAL_Delay(10)
	
