//author:Milind Devnani
#include <SoftwareSerial.h>
#define rxPin 10
#define txPin 11

const int status_led = 13;
String outByte = "";
SoftwareSerial mySerial =  SoftwareSerial(rxPin, txPin);

void setup() {
  // put your setup code here, to run once:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  pinMode(status_led, OUTPUT);
//  Serial.begin(9600);
  mySerial.begin(9600);
  analogWrite(9, 127);
      digitalWrite(status_led,HIGH);
    delay(1000);
    digitalWrite(status_led,LOW);

}

void loop() {


  
  if (mySerial.available()) {

      outByte = mySerial.readStringUntil('?');
      outByte.trim();
//      mySerial.print("This is the command: ");
    mySerial.print(outByte);
//    mySerial.write(outByte);
    digitalWrite(status_led,HIGH);
    delay(1000);
    digitalWrite(status_led,LOW);
    outByte="";
  }

}
