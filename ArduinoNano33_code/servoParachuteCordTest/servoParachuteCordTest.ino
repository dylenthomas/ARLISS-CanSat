#include "Servo.h"

Servo myservo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);

  myservo.attach(3);

  Serial.println("Servo has been attached, enter a value between 0 and 180 to move servo.");
}

char cmd[3];
int pos = -1;
int i = 0;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n'){
      pos = atoi(cmd);
      i = 0;
    }
    else {
      if (i < 3) {
        cmd[i] = c;
        i++;
      }
      else {
        Serial.print("Input is too long, must be no more than 3 characters");
        i = 0;
      }
    }
  }

  if (pos > -1 && pos <= 180) {
    Serial.print("Read the position: ");
    Serial.println(cmd);
150

    myservo.write(pos);
  } 
  else if (pos > -1) {
    Serial.print("The angle: ");
    Serial.print(cmd);
    Serial.println(" is too high.");
  }

  if (pos != -1) {
    i = 0;
    pos = -1;
    memset(&cmd[0], 0, sizeof(cmd));
  }
}
