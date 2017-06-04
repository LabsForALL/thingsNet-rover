#include <Stepper.h>

//---( Number of steps per revolution of INTERNAL motor in 4-step mode )---
#define STEPS_PER_MOTOR_REVOLUTION 32   

//---( Steps per OUTPUT SHAFT of gear reduction )---
#define STEPS_PER_OUTPUT_REVOLUTION 32 * 64  //2048  

//The pin connections need to be 4 pins connected
// to Motor Driver In1, In2, In3, In4  and then the pins entered
// here in the sequence 1-3-2-4 for proper sequencing
Stepper small_stepper(STEPS_PER_MOTOR_REVOLUTION, 8, 10, 9, 11);


/*-----( Declare Variables )-----*/
int  Steps2Take;

// forward and backward motors
int move_speed = 200; 
#define move_in1 9
#define move_in2 6

// turn left and right motor
int turn_delay = 400;
int turn_speed = 200;
#define turn_in1 11
#define turn_in2 10

// rotary encoder
byte encoder_state = 0;
#define TURNED_LEFT 13
#define TURNED_RIGHT 7
#define TURNED_CENTER 14
#define re_in1 8
#define re_in2 7
#define re_in3 5
#define re_in4 4


void init_rotary_encoder(){
  pinMode(re_in1, INPUT);
  digitalWrite(re_in1, HIGH);
  
  pinMode(re_in2, INPUT);
  digitalWrite(re_in2, HIGH);
  
  pinMode(re_in3, INPUT);
  digitalWrite(re_in3, HIGH);
  
  pinMode(re_in4, INPUT);
  digitalWrite(re_in4, HIGH);
}


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


void turn_left() {
    stop_turn();
    bool isTurnedLeft = false;
    unsigned long long end_time = millis() + turn_delay; 
    analogWrite(turn_in1, turn_speed);
    digitalWrite(turn_in2, LOW);
    while(TURNED_LEFT != read_encoder_state() && end_time > millis()){
      isTurnedLeft = true;
    }
    stop_turn();

    //some cheats
    if(isTurnedLeft) {
      analogWrite(turn_in1, turn_speed);
      digitalWrite(turn_in2, LOW);
      delay(150);
      stop_turn();
    }
}


void turn_right(){
    stop_turn();
    unsigned long long end_time = millis() + turn_delay; 
    analogWrite(turn_in1, LOW);
    digitalWrite(turn_in2, turn_speed);
    while(TURNED_RIGHT != read_encoder_state() && end_time > millis());
    stop_turn();
}


void turn_center() {
  stop_turn();
  unsigned long long end_time = millis() + turn_delay; 
  byte current_state = read_encoder_state();

  if(current_state == TURNED_LEFT){
    analogWrite(turn_in1, LOW);
    digitalWrite(turn_in2, turn_speed);
    while(TURNED_CENTER != read_encoder_state() && end_time > millis());

  }else if (current_state == TURNED_RIGHT){
    analogWrite(turn_in1, turn_speed);
    digitalWrite(turn_in2, LOW);
    while(TURNED_CENTER != read_encoder_state() && end_time > millis());
  }
  stop_turn();
}


byte read_encoder_state() {
  byte state = 0;
  state = digitalRead(re_in1);
  state |= digitalRead(re_in2) << 1;
  state |= digitalRead(re_in3) << 2;
  state |= digitalRead(re_in4) << 3;
  return state;
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
    turn_left();
    return true;
  }

  if (code == "r") {
    turn_right();
    return true;
  }

  if (code == "c") {
    turn_center();
    return true;
  }

  if (code == "rl") {
    Steps2Take  = 256;
    small_stepper.setSpeed(700);   
    
    while(true) {
      small_stepper.step(Steps2Take);
      if (get_serial_data() == "sr") break;
    }
    
    return true;
  }

  if (code == "rr") {
    Steps2Take  = - 256;  // Rotate CW 1/2 turn
    small_stepper.setSpeed(700);   
    
    while(true) {
      small_stepper.step(Steps2Take);
      if (get_serial_data() == "sr") break;
    }
    
    return true;
  }

  return false;
}

class Commander
{
  protected:
    // all variables represent desired state, they are to be compared with current state
    uint8_t direction_ = 128; // 128 - center, 0 - left, 255 - right
    uint8_t speed_ = 128; // 0 - full reverse, 128 - stop, 255 - full forward
    uint8_t stepper_position = 128; // 64 - look left, 128 - forward, 192 - right
    uint8_t sonar_position = 128; // -------------------||-----------------------
    uint8_t sonar_altitude = 0; // 0 - look down, 255 - look up
    
    bool update_needed[5] = {true, true, true, true, true};

    uint8_t input_buffer[50];
    
  public:
    enum error_codes
    {
      no_error = 0,
      crc_error,
      command_not_complete,
      error_codes_count
    };
    
    Commander(){}
    send_msg(uint8_t *data, uint8_t len){}
    process_input_msg()
    {
      // exaust any junk data before start of msg
      while(Serial.available() && Serial.read() != 2); 
      #ifdef DEBUG_COMMANDER
        Serial.println("exausted junk data");
      #endif
      
      uint8_t in_byte = 0;
      uint8_t prev_in_byte = 0;
      uint8_t crc = 0;
      uint16_t i = 0;
      for(;Serial.available() && 3 != (in_byte = Serial.read()); i++)
      {
        crc += prev_in_byte; // do not add crc byte
        input_buffer[i] = in_byte;
        prev_in_byte = in_byte;
      }

      #ifdef DEBUG_COMMANDER
      // print array
      for(int j=0; j <= i; j++)
      {
        Serial.print(input_buffer[j]);
        Serial.print(", ");
      }
      Serial.println();
      #endif

      if(crc != input_buffer[i])
      {// crc error
        uint8_t data[] = {crc_error};
        send_msg(data, 1);
        #ifdef DEBUG_COMMANDER
          Serial.println("crc error");
        #endif
      }

      //process message
      for(int j=0; j <= i;)
      {
        switch(input_buffer[j])
        {
          case 'd':
          {
            direction_ = input_buffer[j + 1];
            update_needed[0] = true;
            j += 2;
            break;
          }
          
          default:
          {
            j = i + 1; // exit out of loop 
            #ifdef DEBUG_COMMANDER
              Serial.println("default:");
            #endif
          }
          
        }
      }
      
    }

    implement_commands()
    {
      /*for(int i = 0; i < 5; i++)
      {
        if(update_needed[i])
      }*/
      if(update_needed[0])
      {
        if(direction_ == 128)
          turn_center();
        else if(direction_ > 128)
          turn_right();
        else if(direction_ < 128)
          turn_left();

        update_needed[0] = false;
      }
    }
    
};


/* Setup and serial communication */

String get_serial_data(){
  String data = "";
  while (Serial.available()) {
    data += (char) Serial.read();
    delay(10);
  }
  return data;
}


void setup() {
  
  // setting up the builtin led to indicate the state
  pinMode(LED_BUILTIN, OUTPUT);

  // setting up the motors pins
  init_motors();

  // setting up rotary encoder
  init_rotary_encoder();

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

Commander cmd;

void loop() {

  /*String data = get_serial_data();
  
  if (data.length() > 0) {
    if (handle_command(data)) {
      // indicatng that the command was proceeded
      digitalWrite(LED_BUILTIN, HIGH); 
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
    }else{
      Serial.println(data);
    }
  }*/
  cmd.process_input_msg();
  cmd.implement_commands();
}
