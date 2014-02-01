
// Kurt's Orion Mantis test program.  Started off just doing servos connected to Orion, but added this and that...
// Added ability for some of their hexapods as well
//#define BLADE_VENOM
#define FIREANT
//#define CUSTOMPINS

// Make sure at least one is defined
#ifndef BLADE_VENOM
#ifndef FIREANT
#define MANTIS
#endif
#endif

#include <EEPROM.h>
#include <BMSerial.h>
#include <BMPS2X.h>
#include <SPI.h>
#include <Orion.h>
#include <BMServo.h>

#define DEBUG
#define CALIBRATE
#define NORMAL

#define DEGREE 176//134

boolean g_fShowDebugPrompt = true;
boolean g_fDebugOutput = true;

#define EEKEY 0x1234  //eeprom key to determine if arduino eeprom has valid data
#define KEY 0x1234
#define DEADZONE 20

#define GREENLED A1
#define REDLED A0

//############################################################################
#ifdef MANTIS
#define LFC 20
#define LFF 21
#define LFT 22

#define LRC 16
#define LRF 17
#define LRT 18

#define RFC 8
#define RFF 9
#define RFT 10

#define RRC 4
#define RRF 5
#define RRT 6

//leg servo pin assignments
const int IKPins[] = {
  RRC,RRF,RRT,
  RFC,RFF,RFT,
  LRC,LRF,LRT,
  LFC,LFF,LFT,
};

unsigned char reverse[] = {
  1,1,0,
  1,1,0,
  0,0,1,
  0,0,1
};

const int ServoAOffsets[] = {
  0,0,900,      // Note: Different than Nathan, want 0 femur to be horizontal
  0,0,900,
  0,0,900,
  0,0,900  };


const char* IKPinsNames[] = {
  "RRC","RRF","RRT",
  "RFC","RFF","RFT",
  "LRC","LRF","LRT",
  "LFC","LFF","LFT",
};

#endif
//############################################################################
#ifdef BLADE_VENOM
// Here is the definitions for Venom and Blade.  I am only showing the default pin configuration and not the 
// custom as hopefully you will have the standard configuration.
#define LFC 23
#define LFF 22 
#define LFT 21
#define LMC 20
#define LMF 19
#define LMT 18
#define LRC 17
#define LRF 16
#define LRT 0
#define RFC 15
#define RFF 14
#define RFT 13
#define RMC 12
#define RMF 11
#define RMT 10
#define RRC 9
#define RRF 8
#define RRT 1

//leg servo pin assignments
const int IKPins[] = {
  RRC,RRF,RRT,
  RMC,RMF,RMT,
  RFC,RFF,RFT,
  LRC,LRF,LRT,
  LMC,LMF,LMT,
  LFC,LFF,LFT,
};

unsigned char reverse[  ] = {
  0,0,1,
  0,0,1,
  0,0,1,
  1,1,0,
  1,1,0,
  1,1,0};

const int ServoAOffsets[] =   {
  0,0,900,
  0,0,900,
  0,0,900,
  0,0,900,
  0,0,900,
  0,0,900};

const char* IKPinsNames[] = {
  "RRC","RRF","RRT",
  "RMC","RMF","RMT",
  "RFC","RFF","RFT",
  "LRC","LRF","LRT",
  "LMC","LMF","LMT",
  "LFC","LFF","LFT",
};

#endif
//############################################################################
#ifdef FIREANT
#define LFC 23
#define LFF 22 
#define LFT 21
#define LMC 20
#define LMF 19
#define LMT 18
#define LRC 17
#define LRF 16
#define LRT 0
#define RFC 15
#define RFF 14
#define RFT 13
#define RMC 12
#define RMF 11
#define RMT 10
#define RRC 9
#define RRF 8
#define RRT 1
#define HeadRollPin 7
#define HeadYawPin 6
#define HeadPitchPin 5
#define PincerLPin 4
#define PincerRPin 3
#define TailYawPin  -9
#define TailPitchPin -10


//leg dimensions(1/10 mm units)
const int IKPins[] = {
  RRC,RRF,RRT,
  RMC,RMF,RMT,
  RFC,RFF,RFT,
  LRC,LRF,LRT,
  LMC,LMF,LMT,
  LFC,LFF,LFT,
  HeadRollPin, HeadYawPin, HeadPitchPin, PincerLPin, PincerRPin,
  TailYawPin, TailPitchPin

};

const unsigned char reverse[] = {
  0,1,0,
  0,1,0,
  0,1,0,
  1,0,1,
  1,0,1,
  1,0,1,
  0,0,0,0,1,
  0, 0};


const int ServoAOffsets[] = {
  0,0,900,
  0,0,900,
  0,0,900,
  0,0,900,
  0,0,900,
  0,0,900,
  0,0,0,0,0,
  0,0};

const char* IKPinsNames[] = {
  "RRC","RRF","RRT",
  "RMC","RMF","RMT",
  "RFC","RFF","RFT",
  "LRC","LRF","LRT",
  "LMC","LMF","LMT",
  "LFC","LFF","LFT",
  "Head Pitch(UD)","Head Roll (ROT)","Head Yaw (LR)","L Pincer", "R Pincer", 
  "Tail Yaw(LR)", "Tail Pitch(UD"};
// Define the Tail servos which are run by the Arduino
BMServo TailYawServo(9);
BMServo TailPitchServo(10);

#endif



#define CNT_PINS (sizeof(IKPins)/sizeof(IKPins[0]))                   

boolean onoff = false;
byte iNextPin = 0xff;
int PinVal = 0;
int PincerPos = 0;

BMPS2 ps2x(6,50);


void setup() {
  pinMode(GREENLED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  digitalWrite(GREENLED, HIGH); 
  // Lets enable buttons, 
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  // IF both buttons are high neither is pushed at startup so init the Serial port
  if (!digitalRead(2) || !digitalRead(4)) {
    for (;;) {
      digitalWrite(REDLED, !digitalRead(REDLED));
      delay(250);
    }
  }  

  Serial.begin(38400);

  // initialize Orion
  Orion.begin();
  InitArduinoServos();
  StopServos();

  // Make sure directions and Degrees is set
  for(int i = 0;i<CNT_PINS;i++){
    Orion.setServoDegree(IKPins[i],DEGREE);
    Orion.setServoDir(IKPins[i],reverse[i]);
  }


  //Clears pressed/released states
  ps2x.explicitReadsOnly();
  ps2x.read_ps2(true);
  delay(100);
  ps2x.read_ps2(true);

  delay(500);
  Serial.println("*** Mantis Test Program***");
}

void WiggleServo(int iServo) {
  int iPin = IKPins[iServo];
  Serial.print("Servo: ");
  Serial.print(iPin, DEC);
  Serial.print(" : ");
  Serial.println(IKPinsNames[iServo]);

  // Now lets wiggle that pin...
  Orion.setTime(250);
  Orion.setPulse(iPin, DEGREE*10);
  Orion.execute();
  delay(250);
  Orion.setTime(500);
  Orion.setPulse(iPin, -DEGREE*10);
  Orion.execute();
  delay(500);
  Orion.setTime(250);
  Orion.setPulse(iPin, 0);
  Orion.execute();
  PinVal = 0;  // zero out the pin value...
}


void StopServos(void) {
  for(int i = 0;i<CNT_PINS;i++)
    Orion.stopPulse(IKPins[i]);
}  

void loop() {
  int rx;
  if(Orion.checkLipo()){
    StopServos();
    return; //Battery too low to do anything.
  }
  TerminalMonitor();

  ps2x.read_ps2();

#ifdef FIREANT
  TailPitchServo.update();
  TailYawServo.update();
#endif

  if (ps2x.buttonPressed(PSB_SELECT)) {
    PS2Test();
    return;
  }

  if (/*ps2x.buttonPressed(PSB_L2) && */ps2x.buttonPressed(PSB_R3)) {
    Orion.resetRegs();
    Serial.println("Reset Registers");
    Serial.flush();  // wait until the write has completed.
    asm volatile ("jmp 0x0000");  //reset
  }

  if (ps2x.buttonPressed(PSB_CROSS)) {
    PrintInfoOnServos(false);
  }


  if(ps2x.buttonPressed(PSB_START)){
    if(onoff){
      onoff=false;
      Orion.tone(NOTE_G4,100);
      StopServos();
    }
    else{
      onoff=true;

      Orion.setTime(0);
      for(int i=0;i<CNT_PINS;i++){
        Orion.setAngle(IKPins[i],Orion.queryFBAngle(IKPins[i]));
      }
      Orion.execute();
      delay(250);

      Orion.setTime(1000);
      for(int i=0;i<CNT_PINS;i++){
        Orion.setAngle(IKPins[i], 0);
      }

      Orion.execute();
      delay(1000);

      Orion.tone(NOTE_A5,100);
      Orion.tone(NOTE_B5,100);
      Orion.tone(NOTE_C6,100);
      ps2x.newButtonState();  //Clears pressed/released states

      //      Orion.EnableForce(PincerLPin, 1);
      //      Orion.EnableForce(PincerRPin, 1);
    }
  }

  if(!onoff){
    digitalWrite(GREENLED,HIGH);
    return;  
  }


  if(ps2x.buttonPressed(PSB_PAD_RIGHT)){
    if (iNextPin < CNT_PINS)
      UpdatePulseOffset(iNextPin);
    iNextPin++;
    if (iNextPin >= CNT_PINS)
      iNextPin = 0;

    WiggleServo(iNextPin);    
  }

  if(ps2x.buttonPressed(PSB_PAD_LEFT)){
    if (iNextPin < CNT_PINS) 
      UpdatePulseOffset(iNextPin);
    else
      iNextPin = CNT_PINS;  // make sure we don't go off the deep end.
    if (iNextPin)
      iNextPin--;
    else  
      iNextPin = CNT_PINS -1;
    WiggleServo(iNextPin);  
  }

  // Also use PAD Up/Down for fine tune movement...
  if (ps2x.button(PSB_PAD_UP)) {
    if (!Orion.queryMove()) {
      // Don't do while we are still moving.
      Orion.setTime(40);
      Orion.setPulse(IKPins[iNextPin], PinVal = (Orion.queryPulse(IKPins[iNextPin]) + 25));
      Orion.execute();
      Serial.print(IKPins[iNextPin], DEC);
      Serial.print(" : ");
      Serial.println(PinVal, DEC);
    }
  }

  // Also use PAD Up/Down for fine tune movement...
  if (ps2x.button(PSB_PAD_DOWN)) {
    if (!Orion.queryMove()) {
      // Don't do while we are still moving.
      Orion.setTime(40);
      Orion.setPulse(IKPins[iNextPin], PinVal = (Orion.queryPulse(IKPins[iNextPin]) - 25));
      Orion.execute();
      Serial.print(IKPins[iNextPin], DEC);
      Serial.print(" : ");
      Serial.println(PinVal, DEC);
    }
  }

  // Now lets be able to move the servos some...
  rx = ps2x.analog(PSS_RX);
  if ((abs(rx) > DEADZONE) && (!Orion.queryMove())){  // allow some dead space and only if nothing moving...
    PinVal += ((long)(DEGREE * rx)) / 64;  // Allow the servo to move
    Orion.setTime(40);
    Orion.setPulse(IKPins[iNextPin], PinVal);
    Orion.execute();
    Serial.print(IKPins[iNextPin], DEC);
    Serial.print(" : ");
    Serial.println(PinVal, DEC);
  }

}

#define AVALUE_MASK 0x3fff 
void PS2Test(){
  int aiAValues[16];
  int iAValue;
  unsigned int wButtons;
  byte i;
  boolean fChanged;
  word wMask;
  boolean fMotors = false;

  Serial.println("PS2 Test");

  Serial.println("Press Select again to exit");
  for (;;) {
    if (ps2x.read_ps2(true)) {
      if (ps2x.buttonPressed(PSB_SELECT))
        break;  // break out of here if select button is pressed.

      if (ps2x.buttonPressed(PSB_R3)) {
        if (fMotors) {
          fMotors = false;
          ps2x.motors(false, 0);
        }
        else {
          fMotors = true;
          ps2x.motors(true, 0x7f);  // turn full on
        }
      }  

      fChanged = false;
      // the raw button data is not available, so build our own...
      word wNewButtons = 0;
      for (word wMask=0x8000; wMask; wMask >>=1) {
        if (ps2x.button(wMask))
          wNewButtons |= wMask;
      };
      if (wButtons != wNewButtons) {
        wButtons = wNewButtons;
        fChanged = true;
      }
      wMask = AVALUE_MASK;
      for(i=0; wMask; i++, wMask >>=1) {
        if(wMask & 1) {
          iAValue = ps2x.analog(i);
          if (iAValue != aiAValues[i]) {
            fChanged = true;
            aiAValues[i] = iAValue;
          }
        }
      }
      if (fChanged) {
        Serial.print(wButtons, HEX);
        wMask = AVALUE_MASK;
        for(i=0; wMask; i++, wMask >>=1) {
          if(wMask & 1) {
            Serial.print(" ");
            Serial.print(aiAValues[i], DEC);          
          }
        }
        Serial.println("");
      }
    }
  }   
  ps2x.motors(false, 0);
  Serial.println("PS2 Test exited");
}

//==============================================================================
// Update pulse offset
//==============================================================================
void UpdatePulseOffset(byte iPin) {
  // We want to update the center point for the current servo center point to where it currently is.
  Orion.setPOffset(IKPins[iPin], PinVal = (Orion.queryPOffset(IKPins[iPin])+Orion.queryPulse(IKPins[iPin])));
  Orion.setPulse(IKPins[iPin], 0);  // and zero out the angle to the new offset
  Serial.print(IKPinsNames[iPin]);
  Serial.print("(");
  Serial.print(IKPins[iPin], DEC);
  Serial.print(") = ");
  Serial.println(PinVal, DEC);
}


//==============================================================================
// TerminalMonitor - Simple background task checks to see if the user is asking
//    us to do anything, like update debug levels ore the like.
//==============================================================================
boolean TerminalMonitor(void)
{
  byte szCmdLine[20];  // currently pretty simple command lines...
  byte ich;
  int ch;
  // See if we need to output a prompt.
  if (g_fShowDebugPrompt) {
    Serial.println(F("Orion Test Monitor"));
    Serial.print(F("Current Voltage: "));
    Serial.println(Orion.queryVoltage(), DEC);
    Serial.println(F("D - Toggle debug on or off"));
    Serial.println(F("I - Print Servo Info"));
    Serial.println(F("E - Dump EEPROM"));
    Serial.println(F("C - Calibrate Center position"));
    Serial.println(F("M - Min/Max Values"));
    Serial.println(F("S - Save calibration"));
    Serial.println(F("T - Track Servos"));
    Serial.println(F("L - move legs to <Coxa> <Fem> <Tib>"));
    Serial.println(F("A - <servo> <angle> [Time]"));
    Serial.println(F("F - Free servos"));
    Serial.println(F("P - Pan Servos"));
    Serial.println(F("2 - Start PS2 Test"));
    g_fShowDebugPrompt = false;
  }

  // First check to see if there is any characters to process.
  if ((ich = Serial.available())) {
    ich = 0;
    // For now assume we receive a packet of data from serial monitor, as the user has
    // to click the send button...
    for (ich=0; ich < sizeof(szCmdLine); ich++) {
      while ((ch = Serial.read()) == -1)       // get the next character 
        ;
      if ((ch >= 10) && (ch <= 15))
        break;
      szCmdLine[ich] = ch;
    }
    szCmdLine[ich] = '\0';    // go ahead and null terminate it...
    Serial.print(F("Serial Cmd Line:"));        
    Serial.write(szCmdLine, ich);
    Serial.println(F("<eol>"));

    // So see what are command is.
    if (ich == 0) {
      g_fShowDebugPrompt = true;
    } 
    else if ((ich == 1) && ((szCmdLine[0] == 'd') || (szCmdLine[0] == 'D'))) {
      g_fDebugOutput = !g_fDebugOutput;
      if (g_fDebugOutput) 
        Serial.println(F("Debug is on"));
      else
        Serial.println(F("Debug is off"));
    } 
    else if (((szCmdLine[0] == 'e') || (szCmdLine[0] == 'E'))) {
      DumpEEPROMCmd(szCmdLine);
    } 
    else if (((szCmdLine[0] == 'i') || (szCmdLine[0] == 'I'))) {
      PrintInfoOnServos(true);
    } 
    else if (((szCmdLine[0] == 'c') || (szCmdLine[0] == 'C'))) {
      CalibrateCenter();  
    } 
    else if (((szCmdLine[0] == 'm') || (szCmdLine[0] == 'M'))) {
      CalibrateRange();
    } 
    else if (((szCmdLine[0] == 'p') || (szCmdLine[0] == 'P'))) {
      PanServos(szCmdLine);
    } 
    else if (((szCmdLine[0] == 's') || (szCmdLine[0] == 'S'))) {
      SaveCalibration();
    } 
    else if (((szCmdLine[0] == 't') || (szCmdLine[0] == 'T'))) {
      TrackServos();
    } 
    else if (((szCmdLine[0] == 'l') || (szCmdLine[0] == 'L'))) {
      MoveLegsTo(szCmdLine);
    } 
    else if (((szCmdLine[0] == 'a') || (szCmdLine[0] == 'A'))) {
      SetServoAngle(szCmdLine);
    } 
    else if (((szCmdLine[0] == 'f') || (szCmdLine[0] == 'F'))) {
      onoff=false;
      StopServos();
    } 
    else if (szCmdLine[0] == '2') {
      PS2Test();
    } 
    return true;
  }
  return false;
}

//--------------------------------------------------------------------
// DumpEEPROM
//--------------------------------------------------------------------
byte g_bEEPromDumpMode = 0;  // assume mode 0 - hex dump
word g_wEEPromDumpStart = 0;  // where to start dumps from
byte g_bEEPromDumpCnt = 16;  // how much to dump at a time

void DumpEEPROM() {
  byte i;
  word wDumpCnt = g_bEEPromDumpCnt;

  while (wDumpCnt) {
    Serial.print(g_wEEPromDumpStart, HEX);
    Serial.print(" - ");

    // First in Hex
    for (i = 0; (i < 16) && (i < wDumpCnt); i ++) {
      byte b;
      b = EEPROM.read(g_wEEPromDumpStart+i);
      Serial.print(b, HEX);
      Serial.print(" ");
    }
    // Next in Ascii
    Serial.print(" : ");
    for (i = 0; (i < 16) && (i < wDumpCnt); i ++) {
      byte b;
      b = EEPROM.read(g_wEEPromDumpStart+i);
      if ((b > 0x1f) && (b < 0x7f))
        Serial.write(b);
      else
        Serial.print(".");
    }
    Serial.println("");
    g_wEEPromDumpStart += i;  // how many bytes we output
    wDumpCnt -= i;            // How many more to go...
  } 

}

//--------------------------------------------------------------------
// GetCmdLineNum - passed pointer to pointer so we can update...
//--------------------------------------------------------------------
long GetCmdLineNum(byte **ppszCmdLine) {
  byte *psz = *ppszCmdLine;
  long lVal = 0;
  short sSign = 1;

  // Ignore any blanks
  while (*psz == ' ')
    psz++;

  // See if Hex value passed in
  if ((*psz == '0') && ((*(psz+1) == 'x') || (*(psz+1) == 'X'))) {
    // Hex mode
    psz += 2;  // get over 0x
    for (;;) {
      if ((*psz >= '0') && (*psz <= '9'))
        lVal = lVal * 16 + *psz++ - '0';
      else if ((*psz >= 'a') && (*psz <= 'f'))
        lVal = lVal * 16 + *psz++ - 'a' + 10;
      else if ((*psz >= 'A') && (*psz <= 'F'))
        lVal = lVal * 16 + *psz++ - 'A' + 10;
      else
        break;
    }

  }
  else {
    if (*psz == '-') {
      psz++;
      sSign = -1;
    }

    // decimal mode
    while ((*psz >= '0') && (*psz <= '9'))
      lVal = lVal * 10 + *psz++ - '0';
  }
  *ppszCmdLine = psz;    // update command line pointer
  return lVal * sSign;

}

//--------------------------------------------------------------------
// DumpEEPROMCmd
//--------------------------------------------------------------------
void DumpEEPROMCmd(byte *pszCmdLine) {
  // first byte can be H for hex or W for words...
  if (!*++pszCmdLine)  // Need to get past the command letter first...
    DumpEEPROM();
  else if ((*pszCmdLine == 'h') || (*pszCmdLine == 'H')) 
    g_bEEPromDumpMode = 0;
  else if ((*pszCmdLine == 'w') || (*pszCmdLine == 'W')) 
    g_bEEPromDumpMode = 0;

  else {
    // First argument should be the start location to dump
    g_wEEPromDumpStart = GetCmdLineNum(&pszCmdLine);

    // If the next byte is an "=" may try to do updates...
    if (*pszCmdLine == '=') {
      // make sure we don't get stuck in a loop...
      byte *psz = pszCmdLine;
      word w;
      while (*psz) {
        w = GetCmdLineNum(&psz);
        if (psz == pszCmdLine)
          break;  // not valid stuff so bail!
        pszCmdLine = psz;  // remember how far we got...

        EEPROM.write(g_wEEPromDumpStart++, w & 0xff);
      }
    }
    else {
      if (*pszCmdLine == ' ') { // A blank assume we have a count...
        g_bEEPromDumpCnt = GetCmdLineNum(&pszCmdLine);
      }
    }
    DumpEEPROM();
  }
}

//--------------------------------------------------------------------
// MoveLegsTo
//--------------------------------------------------------------------
void MoveLegsTo(byte *pszCmdLine) {
  ++pszCmdLine;  // Need to get past the command letter first...

  int sCoxa = GetCmdLineNum(&pszCmdLine);
  int sFemur = GetCmdLineNum(&pszCmdLine);
  int sTibia = GetCmdLineNum(&pszCmdLine);

  // Make sure the legs are known by system
  Orion.setTime(0);
  for(int i=0;i<CNT_PINS;i++){
    Orion.setAngle(IKPins[i],Orion.queryFBAngle(IKPins[i]));
  }
  Orion.execute();
  delay(250);

  Serial.print("Coxas+-: ");
  Serial.println(sCoxa, DEC);
  Serial.print("Femurs: ");
  Serial.println(sFemur, DEC);
  Serial.print("Tibias: ");
  Serial.println(sTibia, DEC);

  Orion.setTime(1000);
  for(int i=0;i<CNT_PINS;i+=3){
    Orion.setAngle(IKPins[i], (i&1)? -sCoxa : sCoxa);  // front and rear offset opposit directions...
    Orion.setAngle(IKPins[i+1], sFemur);
    Orion.setAngle(IKPins[i+2], sTibia);
  }

  Orion.execute();
}

//--------------------------------------------------------------------
// PanServos
//--------------------------------------------------------------------
void PanServos(byte *pszCmdLine) {
  ++pszCmdLine;  // Need to get past the command letter first...


  int sCoxa = GetCmdLineNum(&pszCmdLine);
  int sFemur = GetCmdLineNum(&pszCmdLine);
  int sTibia = GetCmdLineNum(&pszCmdLine);

  // Make sure the legs are known by system
  Orion.setTime(0);
  for(int i=0;i<CNT_PINS;i++){
    Orion.setAngle(IKPins[i],Orion.queryFBAngle(IKPins[i]));
  }
  Orion.execute();
  delay(250);

  Serial.print("Coxas+-: ");
  Serial.println(sCoxa, DEC);
  Serial.print("Femurs: ");
  Serial.println(sFemur, DEC);
  Serial.print("Tibias: ");
  Serial.println(sTibia, DEC);

  for (int j = 0; j < 5; j++) {
    Orion.setTime(1000);
    for(int i=0;i<CNT_PINS;i+=3){
      Orion.setAngle(IKPins[i], (i&1)? -sCoxa : sCoxa);  // front and rear offset opposit directions...
      Orion.setAngle(IKPins[i+1], sFemur);
      Orion.setAngle(IKPins[i+2], sTibia);
    }
    Orion.execute();
    delay (1000);
    if (!sCoxa && !sFemur && !sTibia)
      break;  // only do once if nothing moving..

    Orion.setTime(1000);
    for(int i=0;i<CNT_PINS;i+=3){
      Orion.setAngle(IKPins[i], (i&1)? sCoxa : -sCoxa);  // front and rear offset opposit directions...
      Orion.setAngle(IKPins[i+1], -sFemur);
      Orion.setAngle(IKPins[i+2], -sTibia);
    }
    Orion.execute();
    delay (1000);
  }
}


//--------------------------------------------------------------------
// MoveLegsTo
//--------------------------------------------------------------------
void SetServoAngle(byte *pszCmdLine)
{
  ++pszCmdLine;  // Need to get past the command letter first...

  int sServo = GetCmdLineNum(&pszCmdLine);


  if (sServo == 0) { // maybe name passed in.
    for(sServo == 0; sServo < CNT_PINS; sServo++) {
      if (strncmp((const char *)pszCmdLine, IKPinsNames[sServo], 3) == 0)
        break;
    }
    if (sServo == CNT_PINS)
      return;    // no name to use...
    pszCmdLine += 3;   // get past the name of the pin  
    sServo = IKPins[sServo];  // get the actual servo number
  }

  int sAngle = GetCmdLineNum(&pszCmdLine);
  int sTime = GetCmdLineNum(&pszCmdLine);
  if (sTime <= 0)
    sTime = 250; 

  // Make sure the leg has been initialized
  Orion.setTime(0);
  Orion.setAngle(sServo,Orion.queryFBAngle(sServo));
  Orion.execute();

  Serial.print("Move: ");
  Serial.print(sServo, DEC);
  Serial.print(" to ");
  Serial.print(sAngle, DEC);
  Serial.print(" time ");
  Serial.println(sTime, DEC); 

  Orion.setTime(sTime);
  Orion.setAngle(sServo,sAngle);
  Orion.execute();

  delay(sTime);
}

//--------------------------------------------------------------------
// Print Servo Informatoin
//--------------------------------------------------------------------
void PrintServoInfo(int iServo, int iTable) {
  // Will print out information for a servo
  Serial.print(iServo, DEC);
  if (iTable >= 0) {
    Serial.print("(");
    Serial.print(IKPinsNames[iTable]);
    Serial.print(")");
  }

  Serial.print(" : ");
  Serial.print(Orion.queryFBAngle(iServo), DEC);
  Serial.print("(");
  Serial.print(Orion.queryFBPulse(iServo), DEC);
  Serial.print(") : ");
  Serial.print(Orion.queryAOffset(iServo), DEC);
  Serial.print("(");
  Serial.print(Orion.queryPOffset(iServo), DEC);
  Serial.print(") : " );
  Serial.print(Orion.queryServoMin(iServo), DEC);
  Serial.print(" = ");
  Serial.println(Orion.queryServoMax(iServo), DEC);
}
//--------------------------------------------------------------------
// Print Servo Informatoin
//--------------------------------------------------------------------
void PrintInfoOnServos(boolean fTableOrder)
{
  // Will print out information about each of the servos...
  Serial.println("Pin(name) FBAngle FBPulse AOffset POffset Min Max");
  int iTable;
  if (fTableOrder) {
    for (iTable=0; iTable < CNT_PINS; iTable++) {
      if (IKPins[iTable] >= 0)
        PrintServoInfo(IKPins[iTable], iTable);
    }
  } 
  else {
    for(int i=0; i < 24; i++) {
      iTable = -1;
      for (int j=0; j < CNT_PINS; j++) {
        if (IKPins[j] == i) {
          iTable = j;
          break;
        }
      }
      PrintServoInfo(i, iTable);
    }
  }
}

//--------------------------------------------------------------------
// Calibrate Center - Copy from Nathans Mantis program, but modified
//  for my own use...
//--------------------------------------------------------------------
void CalibrateCenter()
{
  Orion.tone(NOTE_C6,100);
  Serial.println("Calibrate Center - Hit start button to Exit");
  int apoffsetsSave[CNT_PINS];
  //*************
  //clear offsets
  //*************
  for(int i = 0;i<CNT_PINS;i++){
    if (IKPins[i] >= 0) {
      Orion.setServoDegree(IKPins[i],DEGREE);
      Orion.setServoDir(IKPins[i],reverse[i]);
      apoffsetsSave[i] = Orion.queryPOffset(IKPins[i]);    // remember what it was before
      Orion.setAOffset(IKPins[i],0);
    }
  }
#ifdef FIREANT
  TailPitchServo.setAOffset(0);
  TailYawServo.setAOffset(0);
#endif

  //**************************************************
  //Wait for user to move servos to calibration points
  //**************************************************
  for(;;) {
    ps2x.read_ps2();
    if (ps2x.buttonPressed(PSB_START) || BTNA)
      break;
    if(ps2x.buttonPressed(PSB_SELECT)){
      Orion.tone(NOTE_G6,100);
      return;
    }
    Orion.checkLipo();


    delay(100);
  }
  while(BTNA);

#ifdef DEBUG
  if (g_fDebugOutput) {
    Serial.println("Updated Pin Data");
#ifdef FIREANT    
    int offset = TailPitchServo.queryFBAngle();
    Serial.print("A10");
    Serial.print(":");
    Serial.println(offset);
    offset = TailYawServo.queryFBAngle();
    Serial.print("A9");
    Serial.print(":");
    Serial.println(offset);
#endif
  }
#endif //DEBUG
  //****************
  //Save new offsets
  //****************
  for(int i = 0;i<CNT_PINS;i++){
    if (IKPins[i] >= 0) {
      Orion.setAOffset(IKPins[i],Orion.queryFBAngle(IKPins[i])+ServoAOffsets[i]);
      // Need to update the Min/Max to be in the new logical coordinates.
      int newVal = Orion.queryServoMin(IKPins[i]) + Orion.queryPOffset(IKPins[i]) - apoffsetsSave[i];  
      Orion.setServoMin(IKPins[i], newVal);
      newVal = Orion.queryServoMax(IKPins[i]) + Orion.queryPOffset(IKPins[i]) - apoffsetsSave[i];  
      Orion.setServoMax(IKPins[i], newVal);
#ifdef DEBUG
      if (g_fDebugOutput) 
        PrintServoInfo(IKPins[i],i);
#endif      
    }
  }
#ifdef FIREANT
  TailPitchServo.setAOffset(TailPitchServo.queryFBAngle());
  TailYawServo.setAOffset(TailYawServo.queryFBAngle());
#endif
  Orion.tone(NOTE_B6,100);
}

//--------------------------------------------------------------------
// Calibrate Center - Copy from Nathans Mantis program, but modified
//  for my own use...
//--------------------------------------------------------------------
void CalibrateRange()
{
  int i;
  Orion.tone(NOTE_D6,100);
// Warning: Min/Max in logical units based on center so don't clear  
//  int apoffsetsSave[CNT_PINS];
  int servomin[CNT_PINS];
  int servomax[CNT_PINS];
  for(i = 0;i<CNT_PINS;i++){
    servomin[i] = 2000;
    servomax[i] = -2000;
    Orion.setServoDegree(IKPins[i],DEGREE);
    Orion.setServoDir(IKPins[i],reverse[i]);
//    apoffsetsSave[i] = Orion.queryPOffset(IKPins[i]);    // remember what it was befor
//    Orion.setAOffset(IKPins[i],0);
  }

  Serial.println("Calibrate Min/Max Range - Hit Start to exit or Select to Abort");
  for(;;) {
    ps2x.read_ps2();
    if (ps2x.buttonPressed(PSB_START) || BTNB)
      break;
    if(ps2x.buttonPressed(PSB_SELECT)){
      Orion.tone(NOTE_G6,100);
      return;
    }
    Orion.checkLipo();
    for(int i=0;i<CNT_PINS;i++){
      if(IKPins[i]!=-1){
        int angle = Orion.queryFBPulse(IKPins[i]);
        if(angle>-30000){  //Only update min/max if valid
          if(servomin[i]>angle)
            servomin[i]=angle;
          if(servomax[i]<angle)
            servomax[i]=angle;
        }
      }
    }    
    delay(100);
  }
  while(BTNB);

#ifdef DEBUG
  if (g_fDebugOutput) {
    Serial.println("Update Pin data");
  }
#endif
  for(int i = 0;i<CNT_PINS;i++){
    Orion.setServoMin(IKPins[i],servomin[i]);
    Orion.setServoMax(IKPins[i],servomax[i]);
//    Orion.setPOffset(IKPins[i], apoffsetsSave[i]);    // restore what it was before
#ifdef DEBUG
      if (g_fDebugOutput) 
        PrintServoInfo(IKPins[i],i);
#endif      
  }

  Orion.tone(NOTE_B6,100);
}

//--------------------------------------------------------------------
// SaveCalibration - Save away the stuff to the EEPROM of the Orion.
//--------------------------------------------------------------------
void SaveCalibration()
{
  Orion.tone(NOTE_D6,100);

  Orion.enableIKMinMax(false);
  Orion.writeRegisters();  
#ifdef FIREANT
  int offset = TailPitchServo.getAOffset();
  EEPROM.write((29*2)+0,offset>>8);
  EEPROM.write((29*2)+1,offset&0xFF);
  offset = TailYawServo.getAOffset();
  EEPROM.write((30*2)+0,offset>>8);
  EEPROM.write((30*2)+1,offset&0xFF);
  EEPROM.write(510,EEKEY>>8);
  EEPROM.write(511,EEKEY);
#endif

  delay(250);

  Orion.red(false);

  asm volatile ("jmp 0x0000");  //reset

}
//--------------------------------------------------------------------
// TrackServos -
//  for my own use...
//--------------------------------------------------------------------
void TrackServos()
{
  int asPos[CNT_PINS];
  int i;
  Orion.tone(NOTE_C6,100);

  StopServos();    // Make sure the servos are free to move...
  Serial.println("Track Servos - Hit start button to Exit");

  for(i = 0;i<CNT_PINS;i++){
#ifdef FIREANT
    if (IKPins[i] == -9)
      asPos[i] = TailYawServo.queryFBAngle();
    else if (IKPins[i] == -10)
      asPos[i] = TailPitchServo.queryFBAngle();
    else
#endif
      asPos[i] = Orion.queryFBAngle(IKPins[i]);
  }

  //**************************************************
  //Wait for user to move servos to calibration points
  //**************************************************
  for(;;) {
    ps2x.read_ps2();
    if (ps2x.buttonPressed(PSB_START) || BTNA)
      break;

    Orion.checkLipo();

    for(int i = 0;i<CNT_PINS;i++){
      int offset;
#ifdef FIREANT
      if (IKPins[i] == -9)
        offset = TailYawServo.queryFBAngle();
      else if (IKPins[i] == -10)
        offset = TailPitchServo.queryFBAngle();
      else
#endif
        offset = Orion.queryFBAngle(IKPins[i]);
      if (abs(offset - asPos[i]) > 3) {
        Serial.print(IKPinsNames[i]);
        Serial.print("(");
        Serial.print(IKPins[i], DEC);
        Serial.print("):");
        Serial.print(offset);
        Serial.print("(");
        Serial.print(Orion.queryFBPulse(IKPins[i]), DEC);
        Serial.println(")");
        asPos[i] = offset;
      }
    }

    delay(100);
  }
  while(BTNA);

  Orion.tone(NOTE_B6,100);
}

void InitArduinoServos()
{
#ifdef FIREANT
  TailPitchServo.begin();
  TailYawServo.begin();
  unsigned int key = (int)((EEPROM.read(510)<<8) | EEPROM.read(511));
  if(key==EEKEY){
    int offset=(int)((EEPROM.read((29*2)+0)<<8) | (EEPROM.read((29*2)+1)&0xFF));
    TailPitchServo.setAOffset(offset);
    offset=(int)((EEPROM.read((30*2)+0)<<8) | (EEPROM.read((30*2)+1)&0xFF));
    TailYawServo.setAOffset(offset);
  }
#endif  
}












