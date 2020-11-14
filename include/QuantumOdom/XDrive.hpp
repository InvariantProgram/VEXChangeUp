#include "api.h"

struct driveVars {
	//Associated Motors, negative for reversed:
	int leftFront;
	int rightFront;
	int leftBack;
	int rightBack;
	//Motor settings
	char gearType;

	double gearRatio;
	double width;
	double odomLengthDiff;
};

class XDrive {
	private:

		driveVars chassisScales;
		double slewRate;
	public:
		//Default Constructor
		XDrive();
		//Constructor 

};
