#ifndef constants_h
#define constants_h

// Broken ports on Brain 2:
// 4 5 6 9 11 12 15 20
#define noStrafes 20

#define FrontLeftWheelPort 16
#define FrontRightWheelPort 19
#define BackRightWheelPort 13
#define BackLeftWheelPort 14

#define LeftIntakePort 2
#define RightIntakePort 7
#define UptakePort 3
#define IndexerPort 8

#define TopSlotLineSensorPort 'A'
#define TOP_SLOT_LINE_SENSOR_LIMIT 2300

#define ScoreLineSensorPort 'B'
#define SCORE_LINE_SENSOR_LIMIT 2100

#define RightEncTop 'C'
#define RightEncBot 'D'
#define LeftEncTop 'E'
#define LeftEncBot 'F'
#define HorEncTop 'G'
#define HorEncBot 'H'

#define StrafeDeadzone 20
#define IntakePower 127   // out of 127
#define IndexerPower 127  // out of 127
#define UptakePower 127   // out of 127

#define MovementScale 0.5
#define VoltageDiffDeadzone 5



#define JoystickScaleConst 1
#define AccelerationConst 1

#endif
