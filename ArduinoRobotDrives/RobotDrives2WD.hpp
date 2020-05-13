/**
	Arduino robot 2WD drive class

	@author  Mateusz £yszkiewicz
	@version 1.0
	@since   2020 - 03 - 20
*/

#ifndef _ROBOTDRIVES_2WD
#define _ROBOTDRIVES_2WD

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Drive.hpp"



/**
	Class for two axis drive robot management.
*/
class RobotDrives2WD {

public:
	enum class RobotCondition {
		stop,
		rapidStop,
		left,
		right,
		forward,
		rear,
		forwardLeft,
		forwardRight,
		rearLeft,
		rearRight
	};

private:

	Drive* left;
	Drive* right;

	RobotCondition condition;

	int speed;
	int angle;
	static constexpr uint8_t speedResolution = 255;
	static constexpr uint8_t angleResolution = 255;

public:

	RobotDrives2WD();

	RobotDrives2WD(int IN1, int IN2, int PWM1, int IN3, int IN4, int PWM2);

	void pinConfigure(int IN1, int IN2, int PWM1, int IN3, int IN4, int PWM2);



	void setVoltage(float supplyVoltage);

	void setVoltage(float supplyVoltage, float nominalVoltage);

	void setVoltage(float supplyVoltage, float nominalVoltage, float  startupVoltageOffset);



	void setSupplyVoltage(float supplyVoltage);

	void setNominalVoltage(float nominalVoltage);

	void setStartupVoltageOffset(float offsetVoltage);



	void setRampUpTime(int time);

	void setRampDownTime(int time);



	inline void setSpeed(int speed) {
		this->speed = speed;
	}

	inline int getSpeed() {
		return speed;
	}



	inline void setAngle(int angle) {
		if (angle < 0)
			this->angle = 0;
		else if (angle > angleResolution)
			this->angle = angleResolution;
		else
			this->angle = angle;
	}

	inline int getAngle(int angle) {
		return angle;
	}



	void setNextCondition(RobotCondition nextCondition);

	void conditionUpdate(bool rampUpdate = true);

	Drive* getLeftDrive();

	Drive* getRightDrive();
};

#endif

