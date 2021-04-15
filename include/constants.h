#ifndef constants_h
#define constants_h

// Broken ports on Brain 2:
// 4 5 6 9 11 12 15 20

#define FrontLeftWheelPort 3
#define FrontRightWheelPort 8
#define BackRightWheelPort 6
#define BackLeftWheelPort 5

#define LeftIntakePort 2
#define RightIntakePort 9
#define rightUptakePort 7
#define leftUptakePort 4

#define StrafeDeadzone 20
#define IntakePower 127   // out of 127
#define reversePower 127  // out of 127
#define UptakePower 600   // out of 600

#define MovementScale 0.5
#define VoltageDiffDeadzone 5

#define forceStraight 10
#define IndexMaxTime 750

#define JoystickScaleConst 1
#define AccelerationConst 1

#define LeftEncTop 'E'
#define LeftEncBot 'F'
#define RightEncTop 'A'
#define RightEncBot 'B'
#define HorEncTop 'C'
#define HorEncBot 'D'

#define botDist 20
#define topDist 21
#define shotDist 1

#define shotDetect 60
#define detectLimit 50
#define topDetect 60

#endif
