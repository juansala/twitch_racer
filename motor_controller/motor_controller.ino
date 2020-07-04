#include <Servo.h>
#define HWSERIAL Serial1 //UART from pins 1 and 0 on Teensy 3.2

Servo arm;
int servo_pin = 9;

char values[6];
String incoming; 

int motorLeftPWMA = 23;
int motorLeftPWMB = 22;
int motorRightPWMA = 19;
int motorRightPWMB = 18;
 
void setup() {
  Serial.begin(9600);
  HWSERIAL.begin(9600);

  arm.attach(servo_pin);
  
  pinMode(motorLeftPWMA, OUTPUT);
  pinMode(motorLeftPWMB, OUTPUT);
  pinMode(motorRightPWMA, OUTPUT);
  pinMode(motorRightPWMB, OUTPUT);

}

void loop() {
  while (HWSERIAL.available())  {
    incoming = HWSERIAL.readString();
    HWSERIAL.flush();
    Serial.println(incoming);
  }
}

void int_extractor(char* data_array, int* output_values, char delimiter){
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
