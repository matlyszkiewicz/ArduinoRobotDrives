/**
	Arduino Drive class

	@author  Mateusz £yszkiewicz
	@version 1.0
	@since   2020 - 03 - 20

	Example usage Chrono library.

	Thanks to Sofian Audry and Thomas Ouellet Fredericks for Chrono library.
*/

#ifndef _DRIVE
#define _DRIVE

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Chrono.h"

/**
	Universal drive class to DC motor.
*/
class Drive {

public:

	enum class Condition {
		stop,
		rapidStop,
		left,
		right
	};

private:

	// Arduino logic pins for H bridge
	int IN1;
	int IN2;

	// Arduino enable pin for H bridge
	int PWM;

	uint8_t maxPWM = 255;
	uint8_t minPWM = 0;
	static constexpr uint8_t PWMresolution = 255;
	static constexpr uint8_t speedResolution = 255;

	// Drive initial supply and nominal drive voltages
	float supplyVoltage = 5.;
	float nominalVoltage = 5.;
	float actualVoltage = 5.;
	float startupVoltageOffset;

	Condition condition;
	Condition nextCondition;

	// Ramp times
	Chrono chronoRampUp;
	Chrono chronoRampDown;
	int rampUpTime = 5000;
	int rampDownTime = 5000;

	int speed;
	int actualSpeed;



	void stop();

	void rapidStop();

	void left(int pulseWidt);

	void right(int pulseWidt);

	void updatePWMRange();

	void rampUpdate();

public:

	Drive() = default;

	Drive(int IN1, int IN2, int PWM);

	void pinConfigure(int IN1, int IN2, int PWM);



	void setVoltage(float supplyVoltage);

	void setVoltage(float supplyVoltage, float nominalVoltage);

	void setVoltage(float supplyVoltage, float nominalVoltage, float  startupVoltageOffset);



	void setSupplyVoltage(float supplyVoltage);

	void setNominalVoltage(float nominalVoltage);

	void setStartupVoltageOffset(float offsetVoltage);



	void setSpeed(int speed);



	inline int getSpeed() {
		return speed;
	}

	inline int getActualSpeed() {
		return actualSpeed;
	}



	inline void setRampUpTime(int time) {
		rampUpTime = time;
	}

	inline void setRampDownTime(int time) {
		rampDownTime = time;
	}



	inline void setNextCondition(Condition nextCondition) {
		this->nextCondition = nextCondition;
	}

	inline Condition getCondition() {
		return condition;
	}

	void conditionUpdate(bool rampUpdate = false);
};

#endif

