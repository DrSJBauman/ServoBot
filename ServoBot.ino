/*
================================================================================

    File........... NES Controller Basic Use with Servo motors
    Purpose........ To demonstrate how to interface a 2-servo robot to an NES controller
    Author......... Dr Stephen J Bauman
    E-mail......... sjbauman0@gmail.com
    Started........ 10/04/2019
    Finished....... 10/08/2019
    Updated........ --/--/----
 
================================================================================
   Notes
================================================================================  
- Adapted and modified from Arduino Robot Bonanza's TeachbotServo_NES sketch and
  Joseph Corletto's NES_Controller_Test_Code on allaboutcircuits.com

- The NES controller contains one 8-bit shift register inside. 
- This register takes parallel inputs and converts them into a serial output.

- This code first latches the data and then shifts in the first bit on the data line. 
  Then it clocks and shifts in on the data line until all bits are received.
- What is debugged are the button states of the NES controller.
- A logical "1" means the button is not pressed. A logical "0" means the button is
  pressed.
- This code shifts the first bit of data into the LSB (0000000X).

- The order of shifting for the buttons is shown in the table below:
    Button  | Position | Order as shifted onto Data pin
    -------------------------
 *  A       | 11111110 | bit 0
 *  B       | 11111101 | bit 1
 *  SELECT  | 11111011 | bit 2
 *  START   | 11110111 | bit 3
 *  UP      | 11101111 | bit 4
 *  DOWN    | 11011111 | bit 5
 *  LEFT    | 10111111 | bit 6
 *  RIGHT   | 01111111 | bit 7

- The NES controller pinout is shown below (looking into controller's
  connector end):
    __________
   /          |
  /       O 1 | 1 - Ground
  |           | 2 - Clock
  | 7 O   O 2 | 3 - Latch
  |           | 4 - Data Out
  | 6 O   O 3 | 5 - No Connection
  |           | 6 - No Connection
  | 5 O   O 4 | 7 - +5V
  |___________|

================================================================================
  Updates
================================================================================
*/

//===============================================================================
//  Header Files
//===============================================================================
#include <Servo.h>

//===============================================================================
//  Constants
//===============================================================================
// Here we have a bunch of constants that will become clearer when we look at the
// readNesController() function. Basically, we will use these contents to clear
// a bit. These are chosen according to the table above.
const short A_BUTTON         = 0;
const short B_BUTTON         = 1;
const short SELECT_BUTTON    = 2;
const short START_BUTTON     = 3;
const short UP_BUTTON        = 4;
const short DOWN_BUTTON      = 5;
const short LEFT_BUTTON      = 6;
const short RIGHT_BUTTON     = 7;

// Create objects of the Servo class
Servo servoLeft;
Servo servoRight;

//===============================================================================
//  Variables
//===============================================================================
byte nesRegister  = 0;    // We will use this to hold current button states

//===============================================================================
//  Pin Declarations
//===============================================================================
//Inputs:
#define nesData   4     // The data pin for the NES controller

//Outputs:
#define nesClock  2     // The clock pin for the NES controller
#define nesLatch  3     // The latch pin for the NES controller
#define LServoPin 9     // The pin for the left servo motor
#define RServoPin 10    // The pin for the right servo motor

//===============================================================================
//  Initialization
//===============================================================================
void setup() {
  // Initialize serial port speed for the serial terminal
  Serial.begin(9600);

  // Set appropriate pins to inputs
  pinMode(nesData, INPUT);

  // Set appropriate pins to outputs
  pinMode(nesClock, OUTPUT);
  pinMode(nesLatch, OUTPUT);

  // Use the attach method to assign each object to a pin
  servoLeft.attach(LServoPin);
  servoRight.attach(RServoPin);

  // Set initial states
  digitalWrite(nesLatch, LOW);
  digitalWrite(nesClock, LOW);
}

//===============================================================================
//  Main
//===============================================================================
void loop() {
  // This function call will return the states of all NES controller's register
  // in a nice 8 bit variable format. Remember to refer to the table and
  // constants above for which button maps where!
  nesRegister = nesRead();
  
  if (bitRead(nesRegister, RIGHT_BUTTON) == 0) {   // If Right button press
    turnRight();                                   // Run the right turn motion routine
    Serial.println("RIGHT");
  }

  if (bitRead(nesRegister, LEFT_BUTTON) == 0) {   // If Left button press
    turnLeft();                                   // Run the left turn motion routine
    Serial.println("LEFT");
  }

  if (bitRead(nesRegister, DOWN_BUTTON) == 0) {   // If Down button press
      reverse();                                  // Run the reverse motion routine
      Serial.println("DOWN");
  }

  if (bitRead(nesRegister, UP_BUTTON) == 0) {   // If Up button press
      forward();                                // Run the forward motion routine
      Serial.println("UP");
  }

  if (bitRead(nesRegister, START_BUTTON) == 0) {   // If Start button press
      Serial.println("START");
  }

  if (bitRead(nesRegister, SELECT_BUTTON) == 0) {   // If Select button press
      Serial.println("SELECT");
  }

  if (bitRead(nesRegister, B_BUTTON) == 0) {   // If B button press
      Serial.println("B");
  }

  if (bitRead(nesRegister, A_BUTTON) == 0) {   // If A button press
      Serial.println("A");
  }

  if(nesRegister == 255) {    // If no button press
      stopRobot();            // Run the stop motion routine
  }

  // Slight delay before we debug what was pressed so we don't spam the
  // serial monitor.
  delay(100);
}

//===============================================================================
//  Functions
//===============================================================================
/////////////
// nesRead //
/////////////
byte nesRead() {
  byte value = 0;                 // Reset the value read from the previous loop cycle
  digitalWrite(nesLatch, HIGH);   // Begin initiating Latch pin to activate shift register
  delayMicroseconds(5);         
  digitalWrite(nesLatch, LOW);    // Cause shift register to "latch" current button presses
  for (int i=0; i<8; i++) {       // Cycle through all 8 bits
    digitalWrite(nesClock, LOW);  // Each clock cycle, one of the bits appears at Data pin
    value |= digitalRead(nesData) << (i); // Read Data pin and store current bit in the
                                           // appropriate position of "dataIn"
    digitalWrite(nesClock, HIGH); // Finish the clock cycle
  }
  return(value);                  // Return the full byte where 0 = button pressed
}

/////////////////////
// Motion Routines //
/////////////////////
// Motion routines for forward, reverse, turns, and stop
void forward() {
  reAttach();
  servoLeft.write(180);
  servoRight.write(0);
}

void reverse() {
  reAttach();
  servoLeft.write(0);
  servoRight.write(180);
}

void turnRight() {
  reAttach();
  servoLeft.write(180);
  servoRight.write(180);
}

void turnLeft() {
  reAttach();
  servoLeft.write(0);
  servoRight.write(0);
}

void stopRobot() {
  servoLeft.detach();
  servoRight.detach();
}

//////////////
// reAttach //
//////////////
// Reattaches servos to chosen pins
void reAttach() {
  if(!servoLeft.attached())
    servoLeft.attach(LServoPin);
  if(!servoRight.attached())
    servoRight.attach(RServoPin);
}
