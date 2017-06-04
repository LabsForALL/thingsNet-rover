#include <Stepper.h>

//---( Number of steps per revolution of INTERNAL motor in 4-step mode )---
#define STEPS_PER_MOTOR_REVOLUTION 32   

//---( Steps per OUTPUT SHAFT of gear reduction )---
#define STEPS_PER_OUTPUT_REVOLUTION 32 * 64  //2048  

//The pin connections need to be 4 pins connected
// to Motor Driver In1, In2, In3, In4  and then the pins entered
// here in the sequence 1-3-2-4 for proper sequencing
Stepper small_stepper(STEPS_PER_MOTOR_REVOLUTION, 2, 12, 3, 13);

//#define DEBUG_COMMANDER


/*-----( Declare Variables )-----*/
int  Steps2Take = 64;

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

  small_stepper.setSpeed(700);
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
class Commander
{
  protected:
    enum value_index
    {
      INDEX_DIRECTION = 0,
      INDEX_SPEED = 1,
      INDEX_STEPPER_MOTOR = 2,
    };
    // all variables represent desired state, they are to be compared with current state
    uint8_t direction_ = 128; // 128 - center, 0 - left, 255 - right
    uint8_t speed_ = 128; // 0 - full reverse, 128 - stop, 255 - full forward
    uint8_t stepper_position = 128; // 64 - turn left, 128 - stop, 192 - turn right
    // TODO: not implemented yet
    uint8_t sonar_position = 128; // -------------------||-----------------------
    uint8_t sonar_altitude = 0; // 0 - look down, 255 - look up

    // if the displayed state needs to be adjusted to be closer to the desired state
    // for example stepper needs periodic adjustment in order not to block the threead 
    bool update_needed[5] = {true, true, true, true, true};
    uint8_t input_buffer[50];
    int32_t current_stepper_position = 0; // home position
    
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
      uint8_t i = 0;
      for(;Serial.available() && 3 != (in_byte = Serial.read()) && i < 50; i++)
      {
        crc += prev_in_byte; // do not add crc byte
        input_buffer[i] = in_byte;
        prev_in_byte = in_byte;
      }

      #ifdef DEBUG_COMMANDER
        Serial.print("i = ");
        Serial.println(i);
      #endif

      #ifdef DEBUG_COMMANDER
      // print msg array
      for(int j=0; j <= i - 1; j++)
      {
        Serial.print(input_buffer[j]);
        Serial.print(", ");
      }
      Serial.println();
      #endif

      if(crc != input_buffer[i - 1])
      {// crc error
        uint8_t data[] = {crc_error};
        send_msg(data, 1);
        #ifdef DEBUG_COMMANDER
          Serial.print("crc error = ");
          Serial.println(crc);
        #endif
        // TODO
      }

      //process message
      for(int j=0; j <= i;)
      {
        switch(input_buffer[j])
        {
          case 'd':
          {
            direction_ = input_buffer[j + 1];
            #ifdef DEBUG_COMMANDER
              Serial.print("direction_ = ");
              Serial.println(direction_);
            #endif
            update_needed[INDEX_DIRECTION] = true;
            j += 2;
            break;
          }

          case 's':
          {
            speed_ = input_buffer[j + 1];
            #ifdef DEBUG_COMMANDER
              Serial.print("speed_ = ");
              Serial.println(speed_);
            #endif
            update_needed[INDEX_SPEED] = true;
            j += 2;
            break;
          }

          case 'l':
          {
            speed_ = input_buffer[j + 1];
            #ifdef DEBUG_COMMANDER
              Serial.print("stepper = ");
              Serial.println(stepper_position);
            #endif
            update_needed[INDEX_STEPPER_MOTOR] = true;
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
      if(update_needed[INDEX_DIRECTION])
      {
        #ifdef DEBUG_COMMANDER
          Serial.print("turning: ");
        #endif
        if(direction_ == 128)
        {
          turn_center();
          #ifdef DEBUG_COMMANDER
            Serial.println("center");
          #endif
        }
        else if(direction_ > 128)
        {
          turn_right();
          #ifdef DEBUG_COMMANDER
            Serial.println("right");
          #endif
        }
        else if(direction_ < 128)
        {
          turn_left();
          #ifdef DEBUG_COMMANDER
            Serial.println("left");
          #endif
        }

        update_needed[INDEX_DIRECTION] = false;
      }

      if(update_needed[INDEX_SPEED])
      {
        #ifdef DEBUG_COMMANDER
          Serial.print("moving: ");
        #endif
        if(speed_ == 128)
        {
          #ifdef DEBUG_COMMANDER
            Serial.println("stop");
          #endif
          stop_move();
        }
        else if(speed_ > 128)
        {
          #ifdef DEBUG_COMMANDER
            Serial.println("forward");
          #endif
          move_forward();
        }
        else if(speed_ < 128)
        {
          #ifdef DEBUG_COMMANDER
            Serial.println("back");
          #endif
          move_backward();
        }

        update_needed[INDEX_SPEED] = false;
      }

      if(update_needed[INDEX_STEPPER_MOTOR])
      {
        if(speed_ == 128)
        {
          #ifdef DEBUG_COMMANDER
            Serial.println("stop step");
          #endif
          update_needed[INDEX_STEPPER_MOTOR] = false;
        }
        else if(speed_ > 128)
        {
          #ifdef DEBUG_COMMANDER
            Serial.println("step to right");
          #endif
          small_stepper.step(-Steps2Take);
          current_stepper_position += -Steps2Take;
        }
        else if(speed_ < 128)
        {
          #ifdef DEBUG_COMMANDER
            Serial.println("step to left");
          #endif
          small_stepper.step(Steps2Take);
          current_stepper_position += Steps2Take;
        }
      }
    }
};

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
  if(Serial.available())
  {
    cmd.process_input_msg();
  }
  cmd.implement_commands();
  delay(20);
}
