#pragma once

#include "api.h"
#include "structDefs.hpp"
#include <array>

class InertialOdom {
	private:
		Chassis scales;
		State storedState;
};