// *******************************************************************
//  Arduino Nano 3.3V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Left Sensor cable (long wired cable)
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// • Option 2: Serial on Right Sensor cable (short wired cable) - recommended, so the ADCs on the other cable are still available
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// *******************************************************************

//https://github.com/rogerclarkmelbourne/Arduino_STM32   in arduino/hardware
//Board: Generic STM32F103C series
//Upload method: serial
//20k RAM 64k Flash

// RX ist A10, TX ist A9   (3v3 level)
//to flash set boot0 (the one further away from reset button) to 1 and press reset, flash, program executes immediately
//set boot0 back to 0 to run program on powerup

// ########################## DEFINES ##########################
#define SERIAL_CONTROL_BAUD   38400       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xAAAA       // [-] Start frme definition for reliable serial communication

#define A2BIT_CONV 50

//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#define MAXADCVALUE 4095

#define PIN_LED PC13
#define SENDPERIOD 100 //ms. delay for sending speed and steer data to motor controller via serial
#define PIN_POTI PA0

long last_send = 0;

long last_validReceive=0;


// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  speedLeft;
   int16_t  speedRight;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedL;
   int16_t  speedR;
   int16_t  speedL_meas;
   int16_t  speedR_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   int16_t  curL_DC;
   int16_t  curR_DC;
   uint16_t  checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup() 
{

  Serial.begin(115200); //Debug and Program. A9=TX1,  A10=RX1  (3v3 level)

  Serial2.begin(SERIAL_CONTROL_BAUD); //control.  B10=TX3, B11=RX3 (Serial2 is Usart 3)
  Serial1.begin(SERIAL_CONTROL_BAUD); //control.  A2=TX2, A3=RX2 (Serial1 is Usart 2)
    
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Initialized"); 
}

// ########################## SEND ##########################
void SendSerial1(int16_t uSpeedLeft, int16_t uSpeedRight)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.speedLeft    = (int16_t)uSpeedLeft;
  Command.speedRight    = (int16_t)uSpeedRight;
  Command.checksum = (uint16_t)(Command.start ^ Command.speedLeft ^ Command.speedRight);

  // Write to Serial
  Serial1.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void ReceiveSerial1()
{
  // Check for new data availability in the Serial buffer
  if (Serial1.available()) {
    incomingByte    = Serial1.read();                                 // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingBytePrev) << 8) +  incomingByte;  // Construct the start frame    
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
    Serial.print(incomingByte);
    return;
  #endif      
  
  // Copy received data
  if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
    p     = (byte *)&NewFeedback;
    *p++  = incomingBytePrev;
    *p++  = incomingByte;   
    idx   = 2;    
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++  = incomingByte; 
    idx++;
  } 
  
  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {    
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedL ^ NewFeedback.speedR
          ^ NewFeedback.speedL_meas^ NewFeedback.speedR_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.curL_DC ^ NewFeedback.curR_DC);
  
    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      
      Serial.print(millis()-last_validReceive); Serial.print("ms ");
      
      // Print data to built-in Serial
      /*Serial.print("1: ");   Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");  Serial.print(Feedback.speedL);
      Serial.print(" 4: ");  Serial.print(Feedback.speedR);
      Serial.print(" 5: ");  Serial.print(Feedback.speedL_meas);
      Serial.print(" 6: ");  Serial.print(Feedback.speedR_meas);
      Serial.print(" 7: ");  Serial.print(Feedback.batVoltage);
      Serial.print(" 8: ");  Serial.print(Feedback.boardTemp);
      Serial.print(" 9: ");  Serial.print(Feedback.curL_DC); //in mA (100mA resolution), in hoverbrett negative sign for forward
      Serial.print(" 10: ");  Serial.println(Feedback.curR_DC); //in mA (100mA resolution), in hoverbrett negative sign for forward
      */

      Serial.print(" LS: ");  Serial.print(Feedback.speedL_meas);
      Serial.print(" RS: ");  Serial.print(Feedback.speedR_meas);
      Serial.print(" LC: ");  Serial.print(Feedback.curL_DC*1.0/A2BIT_CONV); //in A, in hoverbrett negative sign for forward
      Serial.print(" RC: ");  Serial.println(Feedback.curR_DC*1.0/A2BIT_CONV); //in A, in hoverbrett negative sign for forward

      last_validReceive=millis();
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }
  
  // Update previous states
  incomingBytePrev  = incomingByte;
}

// ########################## LOOP ##########################



void loop() {

  ReceiveSerial1(); // Check for new received data
    
  if (millis() - last_send > SENDPERIOD) {
    
    
    int16_t speedvalue=constrain(analogRead(PIN_POTI)*1.0/MAXADCVALUE*1000, 0, 1000);

    SendSerial1(speedvalue,speedvalue);
    //Serial.print("Send "); Serial.print(millis()); Serial.print("ms, steer=0"); Serial.print(", speed="); Serial.println(speedvalue);

    last_send = millis();

  }

}

// ########################## END ##########################
