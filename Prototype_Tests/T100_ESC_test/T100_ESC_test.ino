#include <Servo.h>

byte motor1Pin = 9;
byte motor2Pin = 10;

Servo motor1;
Servo motor2;

void setup() {
  Serial.begin(115200);
  
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);

  motor1.writeMicroseconds(1500); // send "stop" signal to ESC.
  motor2.writeMicroseconds(1500); // send "stop" signal to ESC.
  
  delay(1000); // delay to allow the ESC to recognize the stopped signal
}

void loop() {
  // Upper bound 2000. 
  // Lower bound 1000

  while (Serial.available() > 0) {
    int motor1Value = Serial.parseInt();
    int motor2Value = Serial.parseInt();

    Serial.print("motor1: ");
    Serial.print(motor1Value);
    Serial.print(". motor2: ");
    Serial.println(motor2Value);

    if (Serial.read() == '\n') {
      motor1.writeMicroseconds(motor1Value);
      motor2.writeMicroseconds(motor2Value);
    }
  }
}
