
int i = 23; //IDE mengirim/upload value (23) ke mikrokontroler

int incomingByte = 0;
// The setup routine runs once when you press reset:
void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // delay(1000);
  Serial.write(i);
  // Serial.println(i);
  // Serial.print("Kirim Sukses");
}

// The loop routine runs over and over again forever:
void loop() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    // say what you got:
    // Serial.println(incomingByte, DEC);
    Serial.write(incomingByte);
  }
  // Serial.println("Gagal");
  delay(100);
  Serial.println(incomingByte);
} 

//TESTING KOMUNIKASI MIKRO DAN MATLAB KEMUDIA CHANGE VALUE LEWAT MATLAB
//sukses mengirim arduino,... 
//arduino ada data 23
