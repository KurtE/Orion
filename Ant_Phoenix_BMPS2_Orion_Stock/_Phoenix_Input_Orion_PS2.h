//====================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix, control file.
//The control input subroutine for the phoenix software is placed in this file.
//Can be used with V2.0 and above
//Configuration version: V1.0
//
//Hardware setup: Orion Serial PS2 version
// 
//NEW IN V1.0
//- First Release
//
//Walk method 1:
//- Left StickWalk/Strafe
//- Right StickRotate
//
//Walk method 2:
//- Left StickDisable
//- Right StickWalk/Rotate
//
//
//PS2 CONTROLS:
//[Common Controls]
//- StartTurn on/off the bot
//- L1Toggle Shift mode
//- L2Toggle Rotate mode
//- CircleToggle Single leg mode
//   - Square        Toggle Balance mode
//- TriangleMove body to 35 mm from the ground (walk pos) 
//and back to the ground
//- D-Pad upBody up 10 mm
//- D-Pad downBody down 10 mm
//- D-Pad leftdecrease speed with 50mS
//- D-Pad rightincrease speed with 50mS
//
//[Walk Controls]
//- selectSwitch gaits
//- Left Stick(Walk mode 1) Walk/Strafe
// (Walk mode 2) Disable
//- Right Stick(Walk mode 1) Rotate, 
//(Walk mode 2) Walk/Rotate
//- R1Toggle Double gait travel speed
//- R2Toggle Double gait travel length
//
//[Shift Controls]
//- Left StickShift body X/Z
//- Right StickShift body Y and rotate body Y
//
//[Rotate Controls]
//- Left StickRotate body X/Z
//- Right StickRotate body Y
//
//[Single leg Controls]
//- selectSwitch legs
//- Left StickMove Leg X/Z (relative)
//- Right StickMove Leg Y (absolute)
//- R2Hold/release leg position
//
//[GP Player Controls]
//- selectSwitch Sequences
//- R2Start Sequence
//
// For Apod/Fireant (Zenta's settings) - different mappings...
//PS2 CONTROLS:
//	[Common Controls]
//	- Start			Turn on/off the bot
//	- L1			A-Pod Mandibles/gripper Open (hold to move)
//	- L2			A-Pod Mandibles/gripper Close (hold to move)
//	- Circle		Toggle Single leg mode, Removed from A-Pod
//   - Square        Toggle Balance mode
//	- Triangle		Move body to 35 mm from the ground (walk pos) 
//					and back to the ground
//- L2 + D-Pad left	Decrease gripper torque 
//- L2 + D-Pad right Increase gripper torque 
//	- D-Pad up		Body up 10 mm
//	- D-Pad down	Body down 10 mm
//	- R3 			Toogle full/half head rotation range (hint push/click the right gimbal)
//	- O (Circle)	Toggle Rotate mode
//	- X (Cross)		Toggle Shift mode 
//
//	[Walk Controls]
//	- select		Switch gaits
//	- Left Stick	Walk/Strafe
//				 	
//	- Right Stick	Rotate, body X rotate 		
//	
//	- D-Pad left	decrease speed with 50mS (moved from common to walk only control)
//	- D-Pad right	increase speed with 50mS
//	- R1			Toggle Double gait travel height
//	- R2			Toggle Double gait travel length
//
//	[Shift Controls]
//	- Left Stick	Shift body X/Z
//	- Right Stick	Shift body Y and rotate body Y
//
//	[Rotate Controls]
//	- select		Switch rotate function (Head tracking, fixed head, head only)
//	- Left Stick	L/R: Y rotate, U/D: Z Translate (Shift) 
//	- Right Stick	L/R: Z rotate, U/D: X rotate
//	- R1			Moves the Center Point of Rotation to the Head (Hold button)
//	- R2			Moves the Center Point of Rotation to the Tail (Hold button)
//	- D-Pad left	Slower but more accurate indirect control
//	- D-Pad right	Faster respons on indirect control
//	- L3 			Reset body rotations, click the left gimbal for neutralizing the IndDualShock
//
//Not used on A-Pod:
//	[Single leg Controls]
//	- select		Switch legs
//	- Left Stick	Move Leg X/Z (relative)
//	- Right Stick	Move Leg Y (absolute)
//	- R2			Hold/release leg position
//
//	[GP Player Controls]
//	- select		Switch Sequences
//	- R2			Start/Stop Sequence



//
//
//====================================================================
// [Include files]
#include <Arduino.h> // Arduino 1.0
#include <BMSerial.h>
#include <BMPS2X.h>

//[CONSTANTS]
#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4


#define cTravelDeadZone 4      //The deadzone for the analog input from the remote
#define  MAXPS2ERRORCNT  5     // How many times through the loop will we go before shutting off robot?

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

//=============================================================================
// Global - Local to this file only...
//=============================================================================
#ifndef PS2_PIN
#define PS2_PIN 6
#endif
BMPS2           ps2x(PS2_PIN,100);


// Define an instance of the Input Controller...
InputController  g_InputController;       // Our Input controller 


static short      g_BodyYOffset; 
static short      g_sPS2ErrorCnt;
static short      g_BodyYShift;
static byte       ControlMode;
static bool       DoubleHeightOn;
static bool       DoubleTravelOn;
static bool       WalkMethod;
byte              GPSeq;             //Number of the sequence
short             g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet


//Apod stuff, need to figure which need to be global like thisl
#ifdef APOD_CODE
//--------------------------------------------------------------------
//[A-Pod variables]
static short HeadRotOut1;
static short HeadPanOut1;
static short HeadTiltOut1;
static short HeadRotInput1;
//static int        HeadPanInput1;
//HeadTiltInput1 		var sword
static byte       RotateFunction;
static bool       g_fFullHeadRange;	  // Set to 1 for full range on head rotation
static byte     RPOTCtrlPos;

//;A-Pod mandible variables:
//MandibleFSRInput 		var word
//MandibleFSR_Activated 	var bit
//RPOTSensePos			var byte
//RPOTCtrlPos				var byte ; Controlled indirectly by the Circle and Cross button, inc/dec variable
//TorqueLevel				var word
//TorqueSelect			bytetable 0,64,128,192,255 ;Could be made much simpler, reusing stuff from DIY control
//TorqueIndex				var nib
//MandibleOpen			var bit 
//MandibleClose			var bit

static byte  NeutralStick[4];
static int  IndDualShock[6];    // BUGBUG used in Zentas smoothing only uses 4 values...
static byte IDSdivFactor; //This factor determine if the indirect control is fast (low value) or accurate and slow (high value)


#endif

// some external or forward function references.
extern void PS2TurnRobotOff(void);

#ifdef APOD_CODE
extern void HeadControl(short HeadRotInput1, short HeadPanInput1, short HeadTiltInput1);
extern void MandibleControl(void);
extern void IndirectDualShock(void);
extern void BranchRotateFunction(void);
#endif
//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
void InputController::Init(void)
{
  int error;

  //Clears pressed/released states
  //Clears pressed/released states
  ps2x.explicitReadsOnly();
  ps2x.read_ps2(true);
  ps2x.read_ps2(true);
#ifdef DBGSerial
  DBGSerial.print("PS2 Init: ");
  DBGSerial.println(error, DEC);
#endif
  g_BodyYOffset = 0;    
  g_BodyYShift = 0;
  g_sPS2ErrorCnt = 0;  // error count

  ControlMode = WALKMODE;
  DoubleHeightOn = false;
  DoubleTravelOn = false;
  WalkMethod = false;

  g_InControlState.SpeedControl = 100;    // Sort of migrate stuff in from Devon.
#ifdef APOD_CODE
  IDSdivFactor = 7 ;// Set Default value
  RPOTCtrlPos = 50 ; //Default Mandible position
  g_fFullHeadRange = false;
#endif
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//==============================================================================
void InputController::AllowControllerInterrupts(boolean fAllow)
{
  // We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the PS2 and then
//process any commands.
//==============================================================================
#ifndef APOD_CODE
void InputController::ControlInput(void)
{
  boolean fAdjustLegPositions = false;

  // In an analog mode so should be OK...
  g_sPS2ErrorCnt = 0;    // clear out error count...

  ps2x.read_ps2();    // should check to see if this is right for us here...
  if (!ps2x.isUpdated())
    return;
    
  if (ps2x.buttonPressed(PSB_START)) {
    if (g_InControlState.fRobotOn) {
      PS2TurnRobotOff();
    } 
    else {
      //Turn on
      g_InControlState.fRobotOn = 1;
      fAdjustLegPositions = true;
    }
  }

  if (g_InControlState.fRobotOn) {
    // [SWITCH MODES]

    //Translate mode
    if (ps2x.buttonPressed(PSB_L1)) {// L1 Button Test
      MSound( 1, 50, 2000);  
      if (ControlMode != TRANSLATEMODE )
        ControlMode = TRANSLATEMODE;
      else {
#ifdef OPT_SINGLELEG
        if (g_InControlState.SelectedLeg==255) 
          ControlMode = WALKMODE;
        else
          ControlMode = SINGLELEGMODE;
#else
        ControlMode = WALKMODE;
#endif
      }
    }

    //Rotate mode
    if (ps2x.buttonPressed(PSB_L2)) {    // L2 Button Test
      MSound( 1, 50, 2000);
      if (ControlMode != ROTATEMODE)
        ControlMode = ROTATEMODE;
      else {
#ifdef OPT_SINGLELEG
        if (g_InControlState.SelectedLeg == 255) 
          ControlMode = WALKMODE;
        else
          ControlMode = SINGLELEGMODE;
#else
        ControlMode = SINGLELEGMODE;
#endif
      }
    }

#ifdef OPT_SINGLELEG
    //Single leg mode fNO
    if (ps2x.buttonPressed(PSB_CIRCLE)) {// O - Circle Button Test
      if (abs(g_InControlState.TravelLength.x)<cTravelDeadZone && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
        && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone )   {
        if (ControlMode != SINGLELEGMODE) {
          ControlMode = SINGLELEGMODE;
          if (g_InControlState.SelectedLeg == 255)  //Select leg if none is selected
            g_InControlState.SelectedLeg=cRF; //Startleg
        } 
        else {
          ControlMode = WALKMODE;
          g_InControlState.SelectedLeg=255;
        }
      }
    }      
#endif

#ifdef OPT_GPPLAYER
    // GP Player Mode X
    if (ps2x.buttonPressed(PSB_CROSS)) { // X - Cross Button Test
      MSound(1, 50, 2000);  
      if (ControlMode != GPPLAYERMODE) {
        ControlMode = GPPLAYERMODE;
        GPSeq=0;
      } 
      else
        ControlMode = WALKMODE;
    }
#endif // OPT_GPPLAYER

    //[Common functions]
    //Switch Balance mode on/off 
    if (ps2x.buttonPressed(PSB_SQUARE)) { // Square Button Test
      g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
      if (g_InControlState.BalanceMode) {
        MSound(1, 250, 1500); 
      } 
      else {
        MSound( 2, 100, 2000, 50, 4000);
      }
    }

    //Stand up, sit down  
    if (ps2x.buttonPressed(PSB_TRIANGLE)) { // Triangle - Button Test
      if (g_BodyYOffset>0) 
        g_BodyYOffset = 0;
      else
        g_BodyYOffset = 35;
      fAdjustLegPositions = true;
    }

    if (ps2x.buttonPressed(PSB_PAD_UP)) {// D-Up - Button Test
      g_BodyYOffset += 10;

      // And see if the legs should adjust...
      fAdjustLegPositions = true;
      if (g_BodyYOffset > MAX_BODY_Y)
        g_BodyYOffset = MAX_BODY_Y;
    }

    if (ps2x.buttonPressed(PSB_PAD_DOWN) && g_BodyYOffset) {// D-Down - Button Test
      if (g_BodyYOffset > 10)
        g_BodyYOffset -= 10;
      else
        g_BodyYOffset = 0;      // constrain don't go less than zero.

      // And see if the legs should adjust...
      fAdjustLegPositions = true;
    }

    if (ps2x.buttonPressed(PSB_PAD_RIGHT)) { // D-Right - Button Test
      if (g_InControlState.SpeedControl>0) {
        g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
        MSound( 1, 50, 2000);  
      }
    }

    if (ps2x.buttonPressed(PSB_PAD_LEFT)) { // D-Left - Button Test
      if (g_InControlState.SpeedControl<2000 ) {
        g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
        MSound( 1, 50, 2000); 
      }
    }

    //[Walk functions]
    if (ControlMode == WALKMODE) {
      //Switch gates
      if (ps2x.buttonPressed(PSB_SELECT)            // Select Button Test
      && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
      && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
        && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
        g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
        if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
          MSound( 1, 50, 2000); 
        } 
        else {
          MSound(2, 50, 2000, 50, 2250); 
          g_InControlState.GaitType = 0;
        }
        GaitSelect();
      }

      //Double leg lift height
      if (ps2x.buttonPressed(PSB_R1)) { // R1 Button Test
        MSound( 1, 50, 2000); 
        DoubleHeightOn = !DoubleHeightOn;
        if (DoubleHeightOn)
          g_InControlState.LegLiftHeight = 60;
        else
          g_InControlState.LegLiftHeight = 30;
      }

      //Double Travel Length
      if (ps2x.buttonPressed(PSB_R2)) {// R2 Button Test
        MSound(1, 50, 2000); 
        DoubleTravelOn = !DoubleTravelOn;
      }

      // Switch between Walk method 1 && Walk method 2
      if (ps2x.buttonPressed(PSB_R3)) { // R3 Button Test
        MSound(1, 50, 2000); 
        WalkMethod = !WalkMethod;
      }

      //Walking
      if (WalkMethod)  //(Walk Methode) 
        g_InControlState.TravelLength.z = (ps2x.analog(PSS_RY)); //Right Stick Up/Down  

      else {
        g_InControlState.TravelLength.x = -(ps2x.analog(PSS_LX));
        g_InControlState.TravelLength.z = (ps2x.analog(PSS_LY) );
      }

      if (!DoubleTravelOn) {  //(Double travel length)
        g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
        g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
      }

      g_InControlState.TravelLength.y = -(ps2x.analog(PSS_RX))/4; //Right Stick Left/Right 
    }

    //[Translate functions]
    g_BodyYShift = 0;
    if (ControlMode == TRANSLATEMODE) {
      g_InControlState.BodyPos.x = (ps2x.analog(PSS_LX))/2;
      g_InControlState.BodyPos.z = -(ps2x.analog(PSS_LY))/3;
      g_InControlState.BodyRot1.y = (ps2x.analog(PSS_RX))*2;
      g_BodyYShift = (-(ps2x.analog(PSS_RY) - 128)/2);
    }

    //[Rotate functions]
    if (ControlMode == ROTATEMODE) {
      g_InControlState.BodyRot1.x = (ps2x.analog(PSS_LY));
      g_InControlState.BodyRot1.y = (ps2x.analog(PSS_RX))*2;
      g_InControlState.BodyRot1.z = (ps2x.analog(PSS_LX));
      g_BodyYShift = (-(ps2x.analog(PSS_RY))/2);
    }

#ifdef OPT_SINGLELEG
    //[Single leg functions]
    if (ControlMode == SINGLELEGMODE) {
      //Switch leg for single leg control
      if (ps2x.buttonPressed(PSB_SELECT)) { // Select Button Test
        MSound(1, 50, 2000); 
        if (g_InControlState.SelectedLeg<(CNT_LEGS-1))
          g_InControlState.SelectedLeg = g_InControlState.SelectedLeg+1;
        else
          g_InControlState.SelectedLeg=0;
      }

      g_InControlState.SLLeg.x= (ps2x.analog(PSS_LX))/2; //Left Stick Right/Left
      g_InControlState.SLLeg.y= (ps2x.analog(PSS_RY))/10; //Right Stick Up/Down
      g_InControlState.SLLeg.z = (ps2x.analog(PSS_LY))/2; //Left Stick Up/Down

      // Hold single leg in place
      if (ps2x.buttonPressed(PSB_R2)) { // R2 Button Test
        MSound(1, 50, 2000);  
        g_InControlState.fSLHold = !g_InControlState.fSLHold;
      }
    }
#endif

#ifdef OPT_GPPLAYER
    //[GPPlayer functions]
    if (ControlMode == GPPLAYERMODE) {

      // Lets try some speed control... Map all values if we have mapped some before
      // or start mapping if we exceed some minimum delta from center
      // Have to keep reminding myself that commander library already subtracted 128...
      if (g_ServoDriver.FIsGPSeqActive() ) {
        if ((g_sGPSMController != 32767) | (abs(ps2x.analog(PSS_RY)) > 16))
        {
          // We are in speed modify mode...
          short sNewGPSM = map(ps2x.analog(PSS_RY), -127, 127, -200, 200);
          if (sNewGPSM != g_sGPSMController) {
            g_sGPSMController = sNewGPSM;
            g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
          }

        }
      }

      //Switch between sequences
      if (ps2x.buttonPressed(PSB_SELECT)) { // Select Button Test
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          if (GPSeq < 5) {  //Max sequence
            MSound(1, 50, 1500);
            GPSeq = GPSeq+1;
          } 
          else {
            MSound(2, 50, 2000, 50, 2250);
            GPSeq=0;
          }
        }
      }
      //Start Sequence
      if (ps2x.buttonPressed(PSB_R2))// R2 Button Test
        if (!g_ServoDriver.FIsGPSeqActive() ) {
          g_ServoDriver.GPStartSeq(GPSeq);
          g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative... 
        }
        else {
          g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
          MSound (2, 50, 2000, 50, 2000);
        }


    }
#endif // OPT_GPPLAYER

    //Calculate walking time delay
    g_InControlState.InputTimeDelay = 128 - max(max(abs(ps2x.analog(PSS_LX)), abs(ps2x.analog(PSS_LY))), abs(ps2x.analog(PSS_RX)));
  }

  //Calculate g_InControlState.BodyPos.y
  g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);
  if (fAdjustLegPositions)
    AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
}
#else
//==============================================================================
// APOD/Ant version - Strip out GP Player and Single Leg mode...  May split
//    off to seperate file soon.
//==============================================================================
void InputController::ControlInput(void)
{
  boolean fAdjustLegPositions = false;

  // In an analog mode so should be OK...
  g_sPS2ErrorCnt = 0;    // clear out error count...

  ps2x.read_ps2(true);
  if (ps2x.buttonPressed(PSB_START)) {
    if (g_InControlState.fRobotOn) {
      PS2TurnRobotOff();
    } 
    else {
      //Turn on
      g_InControlState.fRobotOn = 1;
      fAdjustLegPositions = true;
    }
  }

  if (g_InControlState.fRobotOn) {
    // [SWITCH MODES]

    //Translate mode
    if (ps2x.buttonPressed(PSB_CROSS)) {// Cross button was L1 Button Test
      MSound( 1, 50, 2000);  
      if (ControlMode != TRANSLATEMODE )
        ControlMode = TRANSLATEMODE;
      else {
        ControlMode = WALKMODE;
      }
    }

    //Rotate mode
    if (ps2x.buttonPressed(PSB_CIRCLE)) {    // Circle - L2 on others
      MSound( 1, 50, 2000);
      if (ControlMode != ROTATEMODE)
        ControlMode = ROTATEMODE;
      else {
        ControlMode = WALKMODE;
      }
    }


    //[Common functions]
    // BUGBUG:: Mandible, need to figure out what we will do here.
    if (ps2x.buttonPressed(PSB_L1)) { // L1 Open Mandible
      if (RPOTCtrlPos < 250)
        RPOTCtrlPos += 5;
      else
        RPOTCtrlPos = 255;
    }

    if (ps2x.buttonPressed(PSB_L2)) { // L2 Close Mandible
      if (RPOTCtrlPos > 5)
        RPOTCtrlPos -= 5;
      else
        RPOTCtrlPos = 0;
    }

    MandibleControl();
    //Switch Balance mode on/off 
    if (ps2x.buttonPressed(PSB_SQUARE)) { // Square Button Test
      g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
      if (g_InControlState.BalanceMode) {
        MSound(1, 250, 1500); 
      } 
      else {
        MSound( 2, 100, 2000, 50, 4000);
      }
    }

    //Stand up, sit down  
    if (ps2x.buttonPressed(PSB_TRIANGLE)) { // Triangle - Button Test
      if (g_BodyYOffset>0) 
        g_BodyYOffset = 0;
      else
        g_BodyYOffset = 35;
      fAdjustLegPositions = true;
    }

    if (ps2x.buttonPressed(PSB_PAD_UP)) {// D-Up - Button Test
      g_BodyYOffset += 10;

      // And see if the legs should adjust...
      fAdjustLegPositions = true;
      if (g_BodyYOffset > MAX_BODY_Y)
        g_BodyYOffset = MAX_BODY_Y;
    }

    if (ps2x.buttonPressed(PSB_PAD_DOWN) && g_BodyYOffset) {// D-Down - Button Test
      if (g_BodyYOffset > 10)
        g_BodyYOffset -= 10;
      else
        g_BodyYOffset = 0;      // constrain don't go less than zero.

      // And see if the legs should adjust...
      fAdjustLegPositions = true;
    }

    if (ps2x.buttonPressed(PSB_R3)) {// R3 - Toggle Full head move mode
      g_fFullHeadRange = !g_fFullHeadRange;
    }
    //[Walk functions]
    if (ControlMode == WALKMODE) {
      //Switch gates
      if (ps2x.buttonPressed(PSB_SELECT)            // Select Button Test
      && abs(g_InControlState.TravelLength.x)<cTravelDeadZone //No movement
      && abs(g_InControlState.TravelLength.z)<cTravelDeadZone 
        && abs(g_InControlState.TravelLength.y*2)<cTravelDeadZone  ) {
        g_InControlState.GaitType = g_InControlState.GaitType+1;                    // Go to the next gait...
        if (g_InControlState.GaitType<NUM_GAITS) {                 // Make sure we did not exceed number of gaits...
          MSound( 1, 50, 2000); 
        } 
        else {
          MSound(2, 50, 2000, 50, 2250); 
          g_InControlState.GaitType = 0;
        }
        GaitSelect();
      }

      // Speed control Moved from global command area
      if (ps2x.buttonPressed(PSB_PAD_RIGHT)) { // D-Right - Button Test
        if (g_InControlState.SpeedControl>0) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
          MSound( 1, 50, 2000);  
        }
      }

      if (ps2x.buttonPressed(PSB_PAD_LEFT)) { // D-Left - Button Test
        if (g_InControlState.SpeedControl<2000 ) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
          MSound( 1, 50, 2000); 
        }
      }

      //Double leg lift height
      if (ps2x.buttonPressed(PSB_R1)) { // R1 Button Test
        MSound( 1, 50, 2000); 
        DoubleHeightOn = !DoubleHeightOn;
        if (DoubleHeightOn)
          g_InControlState.LegLiftHeight = 60;
        else
          g_InControlState.LegLiftHeight = 30;
      }

      //Double Travel Length
      if (ps2x.buttonPressed(PSB_R2)) {// R2 Button Test
        MSound(1, 50, 2000); 
        DoubleTravelOn = !DoubleTravelOn;
      }

      // Switch between Walk method 1 && Walk method 2
      if (ps2x.buttonPressed(PSB_R3)) { // R3 Button Test
        MSound(1, 50, 2000); 
        WalkMethod = !WalkMethod;
      }

      //Walking
      g_InControlState.TravelLength.x = (-(ps2x.analog(PSS_LX))*2)/3;
      g_InControlState.TravelLength.z = ((ps2x.analog(PSS_LY))*2)/3;

      g_InControlState.BodyRot1.z = SmoothControl(-ps2x.analog(PSS_RX), g_InControlState.BodyRot1.z, 2 );
      g_InControlState.BodyRot1.x = SmoothControl(-ps2x.analog(PSS_RY), g_InControlState.BodyRot1.x, 2);

      // Head control
      HeadControl(ps2x.analog(PSS_RX)/5, 0, -ps2x.analog(PSS_RY));
      g_InControlState.ExtraServosAngle1[cHTP] =  HeadTiltOut1;
      g_InControlState.ExtraServosAngle1[cHRP] =  HeadRotOut1;

      if (!DoubleTravelOn) {  //(Double travel length)
        g_InControlState.TravelLength.x = g_InControlState.TravelLength.x/2;
        g_InControlState.TravelLength.z = g_InControlState.TravelLength.z/2;
      }

      g_InControlState.TravelLength.y = -(ps2x.analog(PSS_RX))/6; //Right Stick Left/Right 
    }

    //[Translate functions]
    g_BodyYShift = 0;
    if (ControlMode == TRANSLATEMODE) {
      g_InControlState.BodyPos.x = (ps2x.analog(PSS_LX))/2;
      g_InControlState.BodyPos.z = -(ps2x.analog(PSS_LY))/3;
      g_InControlState.BodyRot1.y = (ps2x.analog(PSS_RX))*2;
      g_BodyYShift = (-(ps2x.analog(PSS_RY) - 128)/2);
    }

    //[Rotate functions]
    if (ControlMode == ROTATEMODE) {
      // Rotation is a lot more complex with Apod...
      if (ps2x.buttonPressed(PSB_SELECT)) {            // Select Button Test
        if (RotateFunction < 2) {
          MSound(1, 50, 2000); 
          RotateFunction++;
        } 
        else {
          RotateFunction=0;
          MSound(2, 50, 2000, 50, 2250); 
        }
      }        
      if (ps2x.buttonPressed(PSB_L3)) {            // Click down on joystick
        MSound(1, 50, 2000); 
        for (int i=0; i< 4; i++)
          NeutralStick[i] = 1;
      }    
      if (ps2x.buttonPressed(PSB_PAD_RIGHT)) { // D-Right - Button Test
        if (IDSdivFactor > 4) {
          IDSdivFactor -= 2;
          MSound( 1, 50, 2000);  
        }
        else
          MSound(1, 50, 3000);
      }
      if (ps2x.buttonPressed(PSB_PAD_LEFT)) { // D-Left - Button Test
        if (IDSdivFactor < 14) {
          IDSdivFactor += 2;
          MSound( 1, 50, 2000);  
        }
        else
          MSound(1, 50, 1000);
      }
      IndirectDualShock();
      BranchRotateFunction();

      g_InControlState.BodyPos.z = IndDualShock[PSS_LY];
      if (ps2x.button(PSB_R1))
        g_InControlState.BodyRotOffset.z = -165;
      else if (ps2x.button(PSB_R2))
        g_InControlState.BodyRotOffset.z = 165;
      else
        g_InControlState.BodyRotOffset.z = 0;
    } 
    else {   // not Rotate mode
      g_InControlState.BodyRotOffset.x = 0;
      g_InControlState.BodyRotOffset.y = 0;
      g_InControlState.BodyRotOffset.z = 0;
    }
    //Calculate walking time delay
    g_InControlState.InputTimeDelay = 128 - max(max(abs(ps2x.analog(PSS_LX)), abs(ps2x.analog(PSS_LY))), abs(ps2x.analog(PSS_RX)));
  }

  //Calculate g_InControlState.BodyPos.y
  g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);
  if (fAdjustLegPositions)
    AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
}

#endif


//==============================================================================
// PS2TurnRobotOff - code used couple of places so save a little room...
//==============================================================================
void PS2TurnRobotOff(void)
{
  //Turn off
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.TravelLength.x = 0;
  g_InControlState.TravelLength.z = 0;
  g_InControlState.TravelLength.y = 0;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
#ifdef OPT_SINGLELEG
  g_InControlState.SelectedLeg = 255;
#endif
  g_InControlState.fRobotOn = 0;
  AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
}

#ifdef APOD_CODE
//[IndirectDualShock] For an indirect control of A-Pod in rotation mode, 
// necessary for obtaining a smooth and precise body control when using the PS2 controller
void IndirectDualShock(void)
{
  for (int index = PSS_RX; index <= PSS_LY; index++) {
    if (NeutralStick[index-PSS_RX]) {
      if(IndDualShock[index] >30)
        IndDualShock[index] -= 30;
      else if (IndDualShock[index] < -30)
        IndDualShock[index] += 30;
      else  
        IndDualShock[index] = 0;

      if (IndDualShock[index] == 0)
        NeutralStick[index-PSS_RX] = 0; //Reset
    } 
    else {
      if (abs(ps2x.analog(index)) > cTravelDeadZone)
        IndDualShock[index] = min(max(IndDualShock[index] + (ps2x.analog(index)/IDSdivFactor), -128), 127);
    }
  }
}


//-----------------------------------------------------------------------------------
//Branch RotateFunction,
//
void BranchRotateFunction(void) 
{
  switch (RotateFunction) {
  case 0:
    HeadControl(-IndDualShock[PSS_RX], IndDualShock[PSS_LX], -IndDualShock[PSS_RY]);
    g_InControlState.BodyPos.x = -IndDualShock[PSS_RY];
    g_InControlState.BodyPos.y = IndDualShock[PSS_LX];
    g_InControlState.BodyPos.z = -IndDualShock[PSS_RX]*2;
    g_InControlState.ExtraServosAngle1[cHRP] =  -HeadRotOut1;
    g_InControlState.ExtraServosAngle1[cHPP] = HeadPanOut1;
    g_InControlState.ExtraServosAngle1[cHTP] =  HeadTiltOut1;
    break;
  case 1:
    g_InControlState.BodyPos.x = -IndDualShock[PSS_RY];
    g_InControlState.BodyPos.y = IndDualShock[PSS_LX];
    g_InControlState.BodyPos.z = -IndDualShock[PSS_RX]*2;
    g_InControlState.ExtraServosAngle1[cHRP] = -g_InControlState.BodyPos.z + HeadRotOut1;
    g_InControlState.ExtraServosAngle1[cHPP] = g_InControlState.BodyPos.y + HeadPanOut1;
    g_InControlState.ExtraServosAngle1[cHTP] = -g_InControlState.BodyPos.z + HeadTiltOut1;
    break;
  case 2:  
    HeadControl (IndDualShock[PSS_RX], IndDualShock[PSS_LX], -IndDualShock[PSS_RY]);
    g_InControlState.ExtraServosAngle1[cHRP] = -g_InControlState.BodyPos.z + HeadRotOut1;
    g_InControlState.ExtraServosAngle1[cHPP] = g_InControlState.BodyPos.y + HeadPanOut1;
    g_InControlState.ExtraServosAngle1[cHTP] = -g_InControlState.BodyPos.x + HeadTiltOut1;
  }
}
//;--------------------------------------------------------------------
//;Head control, 
//
void HeadControl(short HeadRotInput1, short HeadPanInput1, short HeadTiltInput1)
{

  HeadRotOut1 = SmoothControl((-HeadRotInput1*7/(2-g_fFullHeadRange)),HeadRotOut1,2); //44,8 deg half range and 89,6 deg at full range

  // BUGBUG:: in Apod code PanInput1 never was changed...
  HeadPanOut1 = SmoothControl(HeadPanInput1,HeadPanOut1,2); //'38,4 deg

  HeadTiltOut1 = SmoothControl(HeadTiltInput1*c2DEC/24,HeadTiltOut1,2); //'53,3 deg
}

//;--------------------------------------------------------------------
//;
void MandibleControl() 
{
#ifdef LATER  
  g_InControlState.ExtraServosAngle1[cMRP] = cMandRightClosedPWM - RPOTCtrlPos*5/2;
  g_InControlState.ExtraServosAngle1[cMLP] = cMandLeftClosedPWM - (g_InControlState.ExtraServosAngle1[cMRP] - cMandRightClosedPWM);
#endif  
}

#endif







