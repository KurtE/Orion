//--------------------------------------------------------------------------
// Kurts Rover configuration file.
// This configuration file is used to hide a bunch of the messy stuff where
// we have a few different configurations defined and several tables depend
// on which ones we have defined.  This is especially true about what Servos
// are defined, which we can use PWM type signals to control the motors or we
// can use serial packet mode.  Likewise we can define to have Pan and Tilt servos
// or not...
//--------------------------------------------------------------------------

// Packet Serial Mode
#define PACKETS_ADDRESS 128    //  Address of the device


#define SOUND_PIN                6        // Botboarduino JR pin number
#define PS2_DAT                  2        // PS2 Data line
#define PS2_CMD                  3        // PS2 Command line
#define PS2_SELECT               4        // PS2 Select line
#define PS2_CLOCK                5        // PS2 Clock Line

#define DEADBAND    4         // How much dead band on stick values


