#include <EEPROM.h>

const byte TURNED_LEFT = 0;
const byte TURNED_RIGHT = 1;
byte turn_state; // 0 - turned left ; 1 - turned right
int stateAddr = 0; // EEPROM memory address for storing turn state

// for reading from serial
String inData = "";

// forward and backward motors
int move_speed = 200; 
int move_in1 = 11;
int move_in2 = 10;

// turn left and right motor
int turn_delay = 500;
int turn_speed = 180;
int turn_in1 = 9;
int turn_in2 = 6;


void init_motors () {
  
  // moving motors
  pinMode(move_in1, OUTPUT);
  digitalWrite(move_in1, LOW);
  
  pinMode(move_in2, OUTPUT);
  digitalWrite(move_in2, LOW);

  // turning motor
  pinMode(turn_in1, OUTPUT);
  digitalWrite(turn_in1, LOW);
  
  pinMode(turn_in2, OUTPUT);
  digitalWrite(turn_in2, LOW);
}

/* Forward and backward moving control*/

void stop_move(){
  digitalWrite(move_in1, LOW);
  digitalWrite(move_in2, LOW);
}


void move_forward(){
  stop_move();
  analogWrite(move_in1, move_speed);
}


void move_backward(){
  stop_move();
  analogWrite(move_in2, move_speed);
}

/* Left and right turning control*/

void stop_turn(){
    digitalWrite(turn_in1, LOW);
    digitalWrite(turn_in2, LOW);
}


void turn_left(){
    stop_turn();
    analogWrite(turn_in1, turn_speed);
    digitalWrite(turn_in2, LOW);
    delay(turn_delay);
    stop_turn();
}


void turn_right(){
    stop_turn();
    digitalWrite(turn_in1, LOW);
    analogWrite(turn_in2, turn_speed);
    delay(turn_delay);
    stop_turn();
}

/* Command handling */

boolean handle_command(String code) {


  if (code == "f") {
    move_forward();
    return true;
  }
  
  if (code == "b") {
    move_backward();
    return true;
  }
  
  if (code == "s") {
    stop_move();
    return true;
  }

  if (code == "l") {
    // getting state first
    turn_state = EEPROM.read(stateAddr);

    // only if the car is turned right we can turn left.
    if(turn_state == TURNED_RIGHT){
      turn_left();
      EEPROM.write(stateAddr,0);
      return true;
    }
    
    return false;
  }

  if (code == "r") {
    // getting state first
    turn_state = EEPROM.read(stateAddr);

    // only if the car is turned left we can turn right.
    if(turn_state == TURNED_LEFT){
      turn_right();
      EEPROM.write(stateAddr,1);
      return true;
    }

    return false;
  }

  if (code == "c") {
    turn_state = EEPROM.read(stateAddr);
    Serial.println("Current state is : ");
    Serial.println(turn_state);
    return true;
  }

  return false;
}


void setup() {
  
  // setting up the builtin led to indicate the state
  pinMode(LED_BUILTIN, OUTPUT);

  // setting up the motors pins
  init_motors();

  // 'indicating' serial loading
  digitalWrite(LED_BUILTIN, HIGH);   
  
  // starting serial
  Serial.begin(9600);  
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  // starting to send initial message until response available
  while (Serial.available() <= 0) {
    Serial.println("Hi");   // send an initial string
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  } 
  
}


void loop() {

  while (Serial.available()) {
    inData += (char) Serial.read();
    delay(10);
  }

  if (inData.length() > 0) {
  
    if (handle_command(inData)) {
      
      // indicatng that the command was proceeded
      digitalWrite(LED_BUILTIN, HIGH); 
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      
    }else{
      Serial.println(inData);
    }
    
    // cleaning up
    inData = "";
  }

  
  
}
