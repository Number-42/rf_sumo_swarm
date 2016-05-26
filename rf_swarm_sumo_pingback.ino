#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"

unsigned long maxdis = 30;
unsigned long ping;
unsigned long time_now;
uint8_t master = '1';
uint8_t slave = '2';
byte switchCommand = 0;

int CEPin = 9;
int CSNPin = 10;

RF24 radio(CEPin,CSNPin);

int radio_name = 1;
byte radio_ping = 1;
byte slave_ping = radio_ping+1;

#define HERD_SIZE 5
byte addresses[][6] = {"0Node","1Node","2Node","3Node","4Node"};

bool sync = 1;
bool listening = 1;
bool pinged = 0;

int tmp; //temporary var for speed control

//Movement triggers
const char forward = '8';
const char backward = '2';
const char leftTurn = '4';
const char rightTurn = '6';
const char frontLeftCurve = '7';
const char frontRightCurve = '9';
const char backLeftCurve = '1';
const char backRightCurve = '3';
const char brake = '5';
const char toggleSpeed = '0';
const char toggleMode = 'a';

//Motor control pins
const int LMB = 52; //left motor backward, In2
const int LMF = 50; //left motor forward, In1
const int RMB = 48; //right motor backward, In3
const int RMF = 46; //right motor forward, In4
const int LMS = 2; //left motor Speed control, EnB
const int RMS = 3; //right motor Speed control, EnA

//Speed, i.e. PWM frequency
int Speed = 255;
int altSpeed = 128;

void command(char cmd) { //Condense all motor control into one function
  switch(cmd) {
    case forward:
      digitalWrite(LMF, HIGH);
      digitalWrite(LMB, LOW);
      digitalWrite(RMF, HIGH);
      digitalWrite(RMB, LOW);
      break;
    case backward:
      digitalWrite(LMF, LOW);
      digitalWrite(LMB, HIGH);
      digitalWrite(RMF, LOW);
      digitalWrite(RMB, HIGH);
      break;
    case leftTurn:
      digitalWrite(LMF, LOW);
      digitalWrite(LMB, HIGH);
      digitalWrite(RMF, HIGH);
      digitalWrite(RMB, LOW);
      break;
    case rightTurn:
      digitalWrite(LMF, HIGH);
      digitalWrite(LMB, LOW);
      digitalWrite(RMF, LOW);
      digitalWrite(RMB, HIGH);
      break;
    case frontLeftCurve:
      digitalWrite(LMF, LOW);
      digitalWrite(LMB, LOW);
      digitalWrite(RMF, HIGH);
      digitalWrite(RMB, LOW);
      break;
    case frontRightCurve:
      digitalWrite(LMF, HIGH);
      digitalWrite(LMB, LOW);
      digitalWrite(RMF, LOW);
      digitalWrite(RMB, LOW);
      break;
    case backLeftCurve:
      digitalWrite(LMF, LOW);
      digitalWrite(LMB, LOW);
      digitalWrite(RMF, LOW);
      digitalWrite(RMB, HIGH);
      break;
    case backRightCurve:
      digitalWrite(LMF, LOW);
      digitalWrite(LMB, HIGH);
      digitalWrite(RMF, LOW);
      digitalWrite(RMB, LOW);
      break;
    case brake:
      digitalWrite(LMF, HIGH);
      digitalWrite(LMB, HIGH);
      digitalWrite(RMF, HIGH);
      digitalWrite(RMB, HIGH);
      break;
    case toggleSpeed:
      tmp = Speed;
      Speed = altSpeed;
      altSpeed = tmp;
      analogWrite(LMS, Speed);
      analogWrite(RMS, Speed);
      break;
    case toggleMode:
      sync = !sync;
      break;
    default:
      digitalWrite(LMF, LOW);
      digitalWrite(LMB, LOW);
      digitalWrite(RMF, LOW);
      digitalWrite(RMB, LOW);
      break;
  }
}

void setup() {
  pinMode(LMB, OUTPUT);
  pinMode(LMF, OUTPUT);
  pinMode(RMB, OUTPUT);
  pinMode(RMF, OUTPUT);
  analogWrite(RMS, Speed);
  analogWrite(LMS, Speed);
  radio.begin();
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.setPayloadSize(1);
  radio.setPALevel(RF24_PA_LOW);
  radio.openWritingPipe(addresses[radio_name]);
  radio.openReadingPipe(1,addresses[radio_name-1]);
  if(radio_name != HERD_SIZE-1) radio.openReadingPipe(2,addresses[radio_name+1]);
  radio.startListening();
  if(radio_name != HERD_SIZE-1) radio.writeAckPayload(2,&slave_ping,1);
  listening = 1;
}

void loop() {
  if(sync) {
    char order = 'z';
    if(!listening) {
      radio.startListening();
      listening = 1;
    }
    if(radio.available(&master)) radio.read(&order, sizeof(char));
    command(order);
    radio.stopListening();
    listening = 0;
    radio.write(&order, sizeof(char));
  }
  else {
    command('z');
    byte gotPing;
    if(radio_name == 1) {
      char order;
      if(!listening) {
       radio.startListening();
       listening = 1;
      }
      if(radio.available(&master)) {
        radio.read(&order, sizeof(char));
        command(order);
        if(order == toggleMode) radio.writeAckPayload(2,&switchCommand,1);
      }
      if(radio.available(&slave)) {
        radio.read(&gotPing,1);
        if(order != toggleMode) radio.writeAckPayload(2,&gotPing,1);
      }
    }
    else {
      unsigned long pingOld = ping;
      byte pingReceived;
      if(!listening && pinged) {
        radio.startListening();
        listening = 1;
      }
      if(!pinged) {
        if(listening) {
          radio.stopListening();
          listening = 0;
        }
        radio.write(&radio_ping,1);
        time_now = micros();
        pinged = 1;
        radio.startListening();
        listening = 1;
      }
      if(radio.available()) {
        radio.read(&pingReceived,1);
        if(pingReceived == switchCommand) {
          command(toggleMode);
          if(radio_name != HERD_SIZE-1) radio.writeAckPayload(2,&switchCommand,1);
         }
        }
        else if(pingReceived == radio_ping) {
          ping = micros() - time_now;
          pinged = 0;
          if(ping > maxdis) {
            if(ping > pingOld) command(frontLeftCurve);
            else command(forward);
          }
        }
        if(pingReceived != switchCommand && radio_name != HERD_SIZE-1) radio.writeAckPayload(2,&slave_ping,1);
      }
    }
  }
}
