//#define BLADE_VENOM
#define FIREANT
//#define CUSTOMPINS

// Make sure at least one is defined
#ifndef BLADE_VENOM
#ifndef FIREANT
#define MANTIS
#endif
#endif

#include <SPI.h>
#include <Orion.h>

#include <BMSerial.h>

#include <BMPS2X.h>


//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//   Kåre Halvorsen aka Zenta - Makes everything work correctly!     
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Orion Robotics Orion sheild.
//
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================

#define DEFINE_HEX_GLOBALS
#include <Arduino.h>

#include <EEPROM.h>



#include "Hex_Cfg.h"

#include "_Phoenix.h"

#include "_Phoenix_Input_Orion_PS2.h"
#include "_Phoenix_driver_Orion.h"
#include "_Phoenix_Code.h"

