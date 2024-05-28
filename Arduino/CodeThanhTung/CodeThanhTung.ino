#include "BluetoothSerial.h"
BluetoothSerial serialBT;
char cmd;

void setup() {
  // put your setup code here, to run once:
  serialBT.begin("My-ESP32");
  pinMode(2,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(serialBT.available()){
    cmd = serialBT.read();
  }
  if(cmd == '1'){
    digitalWrite(2, HIGH);
  }
  if(cmd == '0'){
    digitalWrite(2, LOW);
  }
  delay(1000);
}
