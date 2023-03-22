#include <SoftwareSerial.h>
#define txPin 11
#define rxPin 10
#define PWM1 3
#define PWM2 5
#define PWM3 6
#define PWM4 9
SoftwareSerial hSerial =  SoftwareSerial(rxPin, txPin);

byte buffer[32];
int ptr = 0;

void parseCommand() {
  Serial.write("Buffer Contents\n");
  Serial.println(buffer[0], DEC);
  Serial.println(buffer[1], DEC);
  if (buffer[0] == 0x00) {
    analogWrite(PWM1, buffer[1]);
  } else if (buffer[0] == 0x01) {
    analogWrite(PWM2, buffer[1]);
  } else if (buffer[0] == 0x02) {
    analogWrite(PWM3, buffer[1]);
  } else if (buffer[0] == 0x03) {
    analogWrite(PWM4, buffer[1]);
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  hSerial.begin(9600);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (hSerial.available() > 0) {
    buffer[ptr++] = hSerial.read();
    Serial.println(buffer[ptr - 1], DEC);
    Serial.write("Y\n");
  }
  if (buffer[ptr - 1] == '\n') {
    parseCommand();
    ptr = 0;
  }
}
