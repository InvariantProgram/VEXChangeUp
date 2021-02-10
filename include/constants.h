#ifndef constants_h
#define constants_h

// Broken ports on Brain 2:
// 4 5 6 9 11 12 15 20

#define FrontLeftWheelPort 16
#define FrontRightWheelPort 19
#define BackRightWheelPort 13
#define BackLeftWheelPort 14

#define LeftIntakePort 2
#define RightIntakePort 7
#define rightUptakePort 3
#define leftUptakePort 8

#define StrafeDeadzone 20
#define IntakePower 127   // out of 127
#define reversePower 127  // out of 127
#define UptakePower 600   // out of 127

#define MovementScale 0.5
#define VoltageDiffDeadzone 5

#define forceStraight 10
#define IndexMaxTime 750


#define JoystickScaleConst 2
#define AccelerationConst 1

#define leftEncTop 'G'
#define leftEncBot 'H'
#define rightEncTop 'E'
#define rightEncBot 'F'
#define horEncTop 'C'
#define horEncBot 'D'

#endif
