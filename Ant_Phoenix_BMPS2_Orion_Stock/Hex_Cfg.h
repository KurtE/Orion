//====================================================================
// Configuration information for Orion Robotics servos
//
//====================================================================
#ifndef HEX_CFG_H
#define HEX_CFG_H
#define DBGSerial Serial


#define APOD_CODE

//===================================================================
// Debug Options
#ifdef DBGSerial
#define OPT_TERMINAL_MONITOR  
#endif

//#define DEBUG_IOPINS
#ifdef DEBUG_IOPINS
#define DebugToggle(pin)  {digitalWrite(pin, !digitalRead(pin));}
#define DebugWrite(pin, state) {digitalWrite(pin, state);}
#else
#define DebugToggle(pin)  {;}
#define DebugWrite(pin, state) {;}
#endif


// Also define that we are using the AX12 driver

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
//[SERIAL CONNECTIONS]

//====================================================================
// XBEE on non mega???
#define XBeeSerial Serial
#define XBEE_BAUD        38400
#define DISP_VOLTAGE    // User wants the Battery voltage to be displayed...
#define DISP_VOLTAGE_TIME  1000  // how often to check and report in millis
//--------------------------------------------------------------------
//[DaVinci Pin Numbers]
#define SOUND_PIN    3         // Sound is on The orion shield pin 3
#define PS2_PIN      6         // Their serial PS2 receiver is plugged into pin 6

// Define minimum voltage that we will allow the servos to run
#define cTurnOffVol  650     // 6.5
#define cTurnOnVol   700     // 7v - optional part to say if voltage goes back up, turn it back on...


#ifdef FIREANT
//--------------------------------------------------------------------
// Define which pins(sevo IDS go with which joint

  // Standard Fire Ant pins...  // Note only doing legs now...
#define cRRCoxaPin      9   //Rear Right leg Hip Horizontal
#define cRRFemurPin     8   //Rear Right leg Hip Vertical
#define cRRTibiaPin     7   //Rear Right leg Knee

#define cRMCoxaPin      12   //Rear Right leg Hip Horizontal
#define cRMFemurPin     11   //Rear Right leg Hip Vertical
#define cRMTibiaPin     10   //Rear Right leg Knee

#define cRFCoxaPin      15  //Front Right leg Hip Horizontal
#define cRFFemurPin     14  //Front Right leg Hip Vertical
#define cRFTibiaPin     13   //Front Right leg Knee

#define cLRCoxaPin      17   //Rear Left leg Hip Horizontal
#define cLRFemurPin     16   //Rear Left leg Hip Vertical
#define cLRTibiaPin     0    //Rear Left leg Knee

#define cLMCoxaPin      20   //Rear Left leg Hip Horizontal
#define cLMFemurPin     19   //Rear Left leg Hip Vertical
#define cLMTibiaPin     18   //Rear Left leg Knee

#define cLFCoxaPin      23   //Front Left leg Hip Horizontal
#define cLFFemurPin     22  //Front Left leg Hip Vertical
#define cLFTibiaPin     21  //Front Left leg Knee

#define cExtraServos   7     // Extra Servos
#define cHeadRotPin    7
#define cHeadPanPin     6
#define cHeadTiltPin   5
#define cAbdomPanPin  -2
#define cAbdomTiltPin -1
#define cMandRightPin    4
#define cMandLeftPin    3



//--------------------------------------------------------------------
//[MIN/MAX ANGLES] - We will let the Orion controller handle Min Max and servo inversions.
#define SERVOS_DO_MINMAX    // the servo controller will take care of this

#define cRRCoxaInv 0
#define cRMCoxaInv 0 
#define cRFCoxaInv 0 
#define cLRCoxaInv 0 
#define cLMCoxaInv 0 
#define cLFCoxaInv 0 
#define cRRFemurInv 0 
#define cRMFemurInv 0 
#define cRFFemurInv 0 
#define cLRFemurInv 0 
#define cLMFemurInv 0 
#define cLFFemurInv 0 
#define cRRTibiaInv 1 
#define cRMTibiaInv 1 
#define cRFTibiaInv 1 
#define cLRTibiaInv 1 
#define cLMTibiaInv 1 
#define cLFTibiaInv 1


//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm

#define cXXCoxaLength     53    // Mantis leg dimensions.
#define cXXFemurLength    80
#define cXXTibiaLength    128

#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength

#define cRMCoxaLength     cXXCoxaLength	    //Right Middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength

#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength

#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength

#define cLMCoxaLength     cXXCoxaLength	    //Left Middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength

#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength

//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1    0   //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0          //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    0    //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    0   //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0          //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    0    //Default Coxa setup angle, decimals = 1

#define X_COXA          85      // MM between front and back legs /2
#define Y_COXA          72      // MM between front/back legs /2

#define cRROffsetX      -X_COXA     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      Y_COXA     //Distance Z from center of the body to the Right Rear coxa

// Warning this one is wrong...
#define cRMOffsetX      -X_COXA     //Distance X from center of the body to the Right Rear coxa
#define cRMOffsetZ      0           //Distance Z from center of the body to the Right Rear coxa

#define cRFOffsetX      -X_COXA     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -Y_COXA    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      X_COXA      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      Y_COXA     //Distance Z from center of the body to the Left Rear coxa

// Again wrong...
#define cLMOffsetX      X_COXA      //Distance X from center of the body to the Left Rear coxa
#define cLMOffsetZ      0         //Distance Z from center of the body to the Left Rear coxa

#define cLFOffsetX      X_COXA      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -Y_COXA    //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ	 125
#define CHexInitXZCos60  108 //92      // COS(60) = .866
#define CHexInitXZSin60  75 // 92     // sin(60) = .5g
#define CHexInitY	 62       //30


#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRMInitPosX     cHexInitXZ          //Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLMInitPosX     cHexInitXZ          //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin60

#endif

//--------------------------------------------------------------------
#endif // HEX_CFG_H
