#include <Servo.h>
#define HWSERIAL Serial1 //UART from pins 1 and 0 on Teensy 3.2

Servo arm;
int servo_pin = 9;

int values[4];
char buf[50];
String incoming; 

int STOP[4] = {0, 0, 0, 0};
int FORWARD[4] = {235, 0, 235, 0};
int BACKWARD[4] = {0, 255, 0, 255};
int TURN[4] = {0, 255, 255, 0};

int motorLeftPWMA = 21;
int motorLeftPWMB = 20;
int motorRightPWMA = 9;
int motorRightPWMB = 10;

int rightMotorOffset = 0;
 
void setup() {
  Serial.begin(9600);
  HWSERIAL.begin(9600);

  arm.attach(servo_pin);
  
  pinMode(motorLeftPWMA, OUTPUT);
  pinMode(motorLeftPWMB, OUTPUT);
  pinMode(motorRightPWMA, OUTPUT);
  pinMode(motorRightPWMB, OUTPUT);

  driveTest(); // Test motors 

}

void loop() {
  while (HWSERIAL.available())  {
    incoming = HWSERIAL.readString();
    HWSERIAL.flush();
    Serial.println(incoming);
    incoming.toCharArray(buf, 50);
    int_extractor(buf, values, ',');
    drive(values);
  }

  drive(STOP); // STOP if no serial input detected
}

void drive(int* command) {
   analogWrite(motorLeftPWMA, constrain(command[0], 0, 255));
   analogWrite(motorLeftPWMB, constrain(command[1], 0, 255));
   analogWrite(motorRightPWMA, constrain(command[2] - rightMotorOffset, 0, 255));
   analogWrite(motorRightPWMB, constrain(command[3] - rightMotorOffset, 0, 255));
}

void driveTest(){
  drive(FORWARD);
  delay(10000);
//  drive(BACKWARD);
//  delay(5000);
//  drive(TURN);
//  delay(5000);
//  drive(STOP);
//  delay(5000);
}

void int_extractor(char* data_array, int* output_values, char delimiter) {
    char* ptr;
    char* delPointer = &delimiter;
    ptr = strtok(data_array, delPointer);
    int i = 0;
    while (ptr != NULL){
    output_values[i] = atoi(ptr);
    ptr = strtok(NULL, delPointer);
    i++;
    }
}
