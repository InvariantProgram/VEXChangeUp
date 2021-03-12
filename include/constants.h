#ifndef constants_h
#define constants_h

// Broken ports on Brain 2:
// 4 5 6 9 11 12 15 20

#define FrontLeftWheelPort 1
#define FrontRightWheelPort 2
#define BackRightWheelPort 3
#define BackLeftWheelPort 7

#define LeftIntakePort 13
#define RightIntakePort 8
#define rightUptakePort 9
#define leftUptakePort 15

#define StrafeDeadzone 20
#define IntakePower 127   // out of 127
#define reversePower 127  // out of 127
#define UptakePower 600   // out of 127

#define MovementScale 0.5
#define VoltageDiffDeadzone 5

#define forceStraight 10
#define IndexMaxTime 750

#define JoystickScaleConst 1
#define AccelerationConst 1

#define LeftEncTop 'E'
#define LeftEncBot 'F'
#define RightEncTop 'G'
#define RightEncBot 'H'
#define HorEncTop 'C'
#define HorEncBot 'D'

#define botDist 21 
#define topDist 10

#define detectLimit 150

#endif
