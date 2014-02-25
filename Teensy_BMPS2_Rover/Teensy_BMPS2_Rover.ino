
/****************************************************************************
 * Kurt's Rover - Running on an Arduino...
 *     Can configure for Sabertooth or Roboclaw
 *     using either PWM or Packet serial.
 *     This supports an optional Pan/Tilt option. 
 *
 *PS2 CONTROLS:
 *	- Start				Turn on/off Rover / Arm
 *
 *	ROVER CONTROL
 *	Tank mode:
 *	- Left Stick U/D	Left Wheels forward / reverse
 *	- Right Stick U/D	Right Wheels forward / reverse
 *
 *	One Stick mode:
 *	- Right Stick U/D	forward / reverse
 *	- Right Stick L/R	steering
 *       - Left Stick            Pan/Tilt - delta from current position. 
 *
 *	universal:
 *	- D-Pad up		Shift up gear (1 to 4)
 *	- D-Pad down		Shift down gear (1 to 4)
 *	- R3			Change steering mode (Tank / One stick)
 *       - L3                    Move Pan/Tilt back to zero position. 
 *
 * On Roboclaw:
 *    Switch settings for Packet mode: 0010100000
 *                           PWM Mode: 1000e00000
 *    The Motor Controller should be configured with Mixed Mode off as this program
 *    allows you to control the rover in both Tank mode and in a logical mixed mode.
 *
 * Configured for use with BotboardDuino
 *    Pin        Usage
 *    0-1        USB
 *    
 *    5          Sound Pin
 *    6-9        PS2
 *    10-11      Motor Controller (Channel 1/2)  
 *
 ****************************************************************************/

#define DEBUG_OUTPUT
//--------------------------------------------------------------------------
// Included libraries
//--------------------------------------------------------------------------
//#include <icrmacros.h>
#include <PS2X_lib.h>
#include <RoboClaw.h>
#include <BMPS2X.h>
#include <BMSerial.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

//--------------------------------------------------------------------------
// Global Defines
//--------------------------------------------------------------------------

// Move most of the configuration gunk into header file.
#include "Rover_Config.h"

enum {
  ONE_STICK_MODE, TANK_MODE};    // define different steering modes

//--------------------------------------------------------------------------
// Global classes and variables
//--------------------------------------------------------------------------

BMPS2 ps2x(10); // create BMPS2 Controller Class Serial2
RoboClaw RClaw(7, 8, 1000, PSABC_NONE);  // Serial 3  no pressures. 

// Rover State Variables...
boolean     g_fRoverActive;        // Is the rover active?
boolean     g_fRoverActivePrev;    // Previous state...
boolean     g_fServosInit;
byte        g_bGear;               // What gear our we in?
byte        g_bSteeringMode;       //

// Some definitions for Pan and Tilt

// Normalized data coming back from PS2 function
short        LStickX;	            //Stick position to calculate one stick steering
short        LStickY;		    // Stick position to calculate one stick steering
short        RStickX;		    // Stick position to calculate one stick steering
short        RStickY;		    // Stick position to calculate one stick steering

short        sRDrivePWM;
short        sLDrivePWM;

// Scratch variables
word        w;
short     RDrivePrev;
short     LDrivePrev;

// Debug stuff
boolean g_fShowDebugPrompt;
boolean g_fDebugOutput;



//--------------------------------------------------------------------------
// External function definitions
//--------------------------------------------------------------------------
extern void MSound(uint8_t _pin, byte cNotes, ...);


//--------------------------------------------------------------------------
// SETUP: the main arduino setup function.
//--------------------------------------------------------------------------
void setup(){
  int error;

  delay(250); // Give me time to get terminal up
  Serial.begin(57600);
  delay(5000); // Give me time to get terminal up

  Serial.println("Botboardduino Rover Program Startup");
  RClaw.begin(38400);  // Not part of RClaw when simple stream sub-class

  Serial.println("After RClaw begin");

//  ps2x.config_gamepad();  // Setup gamepad (clock, command, attention, data) pins
  Serial.println("After PS2 Init");

  g_fDebugOutput = true;			// start with it off!
  g_fShowDebugPrompt = true;
  g_fRoverActive = false;
  g_fRoverActivePrev = false;
  g_fServosInit = false;
  g_bGear = 3;                                // We init in 3rd gear.
  g_bSteeringMode = ONE_STICK_MODE;

}

//--------------------------------------------------------------------------
// Loop: the main arduino main Loop function
//--------------------------------------------------------------------------
void loop(){
  // We also have a simple debug monitor that allows us to 
  // check things. call it here..
  if (TerminalMonitor())
    return;           

  CheckVoltages();    // check voltages - if we get too low shut down the servos...    

  // Lets get the PS2 data...
  CheckAndProcessPS2Data();

  // Drive the rover
  if (g_fRoverActive) {
    if (g_bSteeringMode == TANK_MODE) {
      sRDrivePWM = RStickY;
      sLDrivePWM = LStickY;
    } 
    else {    // One stick driving
      if ((RStickY >=0) && (RStickX >= 0)) {    // Quadrant 1
        sRDrivePWM = RStickY - RStickX;
        sLDrivePWM = max(RStickX, RStickY);
      } 
      else if ((RStickY<0) && (RStickX>=0))   { //Quadrant 2
        sRDrivePWM = (RStickY + RStickX); 
        sLDrivePWM = min (-RStickX, RStickY);    

      } 
      else if ((RStickY<0)  && (RStickX<0)) {    //Quadrant 3
        sRDrivePWM = min (RStickX, RStickY);
        sLDrivePWM = (RStickY - RStickX);

      } 
      else if ((RStickY>=0) && (RStickX<0)) {    // Quadrant 4
        sRDrivePWM = max(-RStickX, RStickY);
        sLDrivePWM = (RStickY + RStickX);
      } 
      else {	
        sRDrivePWM = 0;
        sLDrivePWM = 0;
      }
    }

    // Lets output the appropriate stuff to the motor controller
    // ok lets figure out our speeds to output to the two motors.  two different commands
    // depending on going forward or backward.
    // Scale the two values for the motors.
    sRDrivePWM = max(min((sRDrivePWM * g_bGear) / 4, 127), -127);    // This should keep up in the -127 to +127 range and scale it depending on what gear we are in.
    sLDrivePWM = max(min((sLDrivePWM * g_bGear) / 4, 127), -127);

#ifdef DEBUG_OUTPUT
    if (g_fDebugOutput) {
      Serial.print(RStickY, DEC);
      Serial.print(",");
      Serial.print(RStickX, DEC);
      Serial.print(" - ");
      Serial.print(sRDrivePWM, DEC);
      Serial.println(sLDrivePWM, DEC);
    }
#endif
    // Call our motors driver code which may change depending on how we talk to the motors...
    RDrive(sRDrivePWM);
    LDrive(sLDrivePWM);


    delay (10);
  } 
  else {
    if (g_fRoverActivePrev) {    
      MSound(SOUND_PIN, 3, 100, 2500, 80, 2250, 60, 2000);
      RDrive(0);
      LDrive(0);
    }
    delay (10);
  }

  g_fRoverActivePrev = g_fRoverActive; 	

}
//==============================================================================
//  RDrive
//==============================================================================
void RDrive(short sVal) {
  if (sVal != RDrivePrev) { 
    RDrivePrev = sVal;

    if (sVal >= 0) 
      RClaw.ForwardM1(PACKETS_ADDRESS, sVal);
    else
      RClaw.BackwardM1(PACKETS_ADDRESS, -sVal);
  }
}

//==============================================================================
//  LDrive
//==============================================================================
void LDrive(short sVal) {
  if (sVal != LDrivePrev) { 
    LDrivePrev = sVal;

    if (sVal >= 0) 
      RClaw.ForwardM2(PACKETS_ADDRESS, sVal);
    else
      RClaw.BackwardM2(PACKETS_ADDRESS, -sVal);
  }
}


//==============================================================================
// Simple inline function 
//==============================================================================

inline short NormalizeJoystickValue(short s)
{
  if ((s > DEADBAND) || (s < -DEADBAND ))
    return s;
  return 0;
}

//==============================================================================
// CheckAndProcessPS2Data -This is code the checks for and processes input from
//        the PS2 controller.
//==============================================================================
unsigned long CheckAndProcessPS2Data(void)
{
  boolean fPacketChanged;
  short	JoyStickValueIn;

  // Then try to receive a packet of information from the PS2.
  if (!ps2x.read_ps2(true)) {

    // Controller did not return Analog mode...
    Serial.print("PS2 - error (");
    Serial.print(ps2x.analog(1), HEX);
    Serial.print("): " );
    Serial.print(ps2x.analog(3), HEX);
    Serial.print(" ");
    Serial.print(ps2x.analog(4), HEX);
    Serial.print(" ");
    Serial.print(ps2x.analog(5), HEX);
    Serial.print(" ");
    Serial.print(ps2x.analog(6), HEX);
    Serial.print(" ");
    Serial.print(ps2x.analog(7), HEX);
    Serial.print(" ");
    Serial.println(ps2x.analog(8), HEX);
    return 100;
  }

  if(ps2x.buttonPressed(PSB_START)) {		// OK lets try "0" button for Start. 
    if (g_fRoverActive) {
      //Turn off
      MSound(SOUND_PIN, 3, 100, 2500, 80, 2250, 60, 2000);
      g_fRoverActive = false;
    } 
    else 	{
      //Turn on
      MSound(SOUND_PIN, 3, 60, 2000, 80, 2250, 100, 2500);
      g_fRoverActive = true;
    }
  }	


  if (g_fRoverActive) {
    // Check some buttons for to see if we should be changing state...
    if (ps2x.buttonPressed(PSB_PAD_UP)) {
      if (g_bGear < 4) {
        g_bGear++;
        MSound(SOUND_PIN, 1, 50, 2000);
      } 
      else {
        MSound(SOUND_PIN, 2, 50, 2000, 50, 2500);
      }
    }

    else if (ps2x.buttonPressed(PSB_PAD_DOWN)) {
      if (g_bGear > 1) {
        g_bGear--;
        MSound(SOUND_PIN, 1, 50, 2000);
      } 
      else {
        MSound(SOUND_PIN, 2, 50, 2500, 50, 2000);
      }
    }

    if (ps2x.buttonPressed(PSB_R3)) {
      MSound(SOUND_PIN, 1, 50, 2000);

      if (g_bSteeringMode == ONE_STICK_MODE)
        g_bSteeringMode = TANK_MODE;
      else
        g_bSteeringMode = ONE_STICK_MODE;
    }

    // Ok lets grab the current Stick values.
    LStickY = NormalizeJoystickValue((ps2x.analog(PSS_LY)));
    LStickX = NormalizeJoystickValue(ps2x.analog(PSS_LX));
    RStickY = NormalizeJoystickValue((ps2x.analog(PSS_RY)));
    RStickX = NormalizeJoystickValue(ps2x.analog(PSS_RX));
  }
  // Return to ok to give us a little break;

  return 20;
}

//==============================================================================
// InitializeServos - We will delay when we call the servo init stuff until a
//        command such as start is called on.  This should make it that if your
//        robot starts off running on USB power it does not overpower it...
//==============================================================================
void InitializeServos(void)
{
}


//==============================================================================
// TerminalMonitor - Simple background task checks to see if the user is asking
//    us to do anything, like update debug levels ore the like.
//==============================================================================
boolean TerminalMonitor(void)
{
  byte szCmdLine[20];
  int ich;
  int ch;
  // See if we need to output a prompt.
  if (g_fShowDebugPrompt) {
    Serial.println("Arduino Rover Monitor");
    Serial.println("D - Toggle debug on or off");
    g_fShowDebugPrompt = false;
  }

  // First check to see if there is any characters to process.
  if (ich = Serial.available()) {
    ich = 0;
    // For now assume we receive a packet of data from serial monitor, as the user has
    // to click the send button...
    for (ich=0; ich < sizeof(szCmdLine); ich++) {
      ch = Serial.read();        // get the next character
      if ((ch == -1) || ((ch >= 10) && (ch <= 15)))
        break;
      szCmdLine[ich] = ch;
    }
    szCmdLine[ich] = '\0';    // go ahead and null terminate it...
    Serial.print("Serial Cmd Line:");        
    Serial.write(szCmdLine, ich);
    Serial.println("!!!");

    // So see what are command is.
    if (ich == 0) {
      g_fShowDebugPrompt = true;
    } 
    else if ((ich == 1) && ((szCmdLine[0] == 'd') || (szCmdLine[0] == 'D'))) {
      g_fDebugOutput = !g_fDebugOutput;
      if (g_fDebugOutput) 
        Serial.println("Debug is on");
      else
        Serial.println("Debug is off");
    }

    return true;
  }
  return false;
} 
//==============================================================================
// CheckVoltages: Try to catch when the voltages drop too low and turn off the
// servos.  This can be because the battery level got too low or if we turn it off
//     and the Arduino is still being powered by USB
//==============================================================================
word g_wVSPrev;

void CheckVoltages(void) 
{
#ifdef AVS_PIN
  word wVS = analogRead(AVS_PIN);
#ifdef DEBUG_OUTPUT
  if (g_fDebugOutput && (wVS != g_wVSPrev)) {
    Serial.print("VS: ");
    Serial.println(wVS, DEC);
    g_wVSPrev = wVS;
  }
#endif
  if ((wVS < (word)AVAL_MIN) && g_fServosInit) {
    // detach any servos we may have...
    MSound(SOUND_PIN, 3, 100, 2500, 100, 2500, 100, 2500);
  }
#endif
}



//==============================================================================
//	SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
void SoundNoTimer(uint8_t _pin, unsigned long duration,  unsigned int frequency)
{
#ifndef __MK20DX256__
#ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
#else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
#endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask = digitalPinToBitMask(SOUND_PIN);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L/(frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    *pin_port ^= pin_mask;

    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }    
  *pin_port &= ~(pin_mask);  // keep pin low after stop
#else
  tone(SOUND_PIN, frequency, duration);  // Try the arduino library
  delay(duration);
#endif
}

//==============================================================================
// Simple sound function that uses no interrupts or the like...
//==============================================================================
void MSound(uint8_t _pin, byte cNotes, ...)
{
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(_pin, uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}


