/**
	Arduino Drive class

	@author  Mateusz £yszkiewicz
	@version 1.0
	@since   2020 - 03 - 20

	Example usage Chrono library.

	Thanks to Sofian Audry and Thomas Ouellet Fredericks for Chrono library.
*/

#include "Drive.hpp"



Drive::Drive(int IN1, int IN2, int PWM) {
	pinConfigure(IN1, IN2, PWM);
}

void Drive::pinConfigure(int IN1, int IN2, int PWM) {
	this->IN1 = IN1;
	this->IN2 = IN2;
	this->PWM = PWM;
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(PWM, OUTPUT);
}

void Drive::updatePWMRange() {

	maxPWM = nominalVoltage / supplyVoltage * PWMresolution;
	minPWM = startupVoltageOffset / supplyVoltage * PWMresolution;
}



void Drive::setVoltage(float supplyVoltage) {
	this->supplyVoltage = supplyVoltage;
	nominalVoltage = supplyVoltage;
	startupVoltageOffset = 0;

	updatePWMRange();
}

void Drive::setVoltage(float supplyVoltage, float nominalVoltage) {
	this->supplyVoltage = supplyVoltage;
	this->nominalVoltage = nominalVoltage;
	startupVoltageOffset = 0;

	updatePWMRange();
}

void Drive::setVoltage(float supplyVoltage, float nominalVoltage, float  startupVoltageOffset) {
	this->supplyVoltage = supplyVoltage;
	this->nominalVoltage = nominalVoltage;
	this->startupVoltageOffset = startupVoltageOffset;

	updatePWMRange();
}



void Drive::setSupplyVoltage(float supplyVoltage) {
	this->supplyVoltage = supplyVoltage;
	updatePWMRange();
}

void Drive::setNominalVoltage(float nominalVoltage) {
	this->nominalVoltage = nominalVoltage;
	updatePWMRange();
}

void Drive::setStartupVoltageOffset(float offsetVoltage) {
	startupVoltageOffset = offsetVoltage;
	updatePWMRange();
}

void Drive::setSpeed(int speed) {
	this->speed = map(speed, 0, speedResolution, minPWM, maxPWM);
}



void Drive::conditionUpdate(bool rampUpdate) {

	if (rampUpdate)
		Drive::rampUpdate();
	else {
		condition = nextCondition;
		if (condition == Drive::Condition::stop || condition == Drive::Condition::rapidStop)
			actualSpeed = 0;
		else
			actualSpeed = speed;
	}

	switch (condition) {
	case Condition::stop:
		stop();
		break;
	case Condition::rapidStop:
		rapidStop();
		break;
	case Condition::left:
		left(actualSpeed);
		break;
	case Condition::right:
		right(actualSpeed);
		break;
	}
}

void Drive::rampUpdate() {

	if (nextCondition != condition || actualSpeed > speed) {
		if (actualSpeed == minPWM || actualSpeed == 0 || condition == Drive::Condition::stop || condition == Drive::Condition::rapidStop) {
			condition = nextCondition;
			actualSpeed = 0;
		}
		else {
			int time = chronoRampDown.elapsed();
			if (time < rampDownTime)
				actualSpeed = map(time, 0, rampDownTime, maxPWM, minPWM);
			else
				actualSpeed = minPWM;
		}

		chronoRampUp.restart();
		if (actualSpeed != 0)
			chronoRampUp.add(map(actualSpeed, minPWM, maxPWM, 0, rampUpTime));
	}
	else {
		if (actualSpeed != speed && condition != Drive::Condition::stop && condition != Drive::Condition::rapidStop) {
			int time = chronoRampUp.elapsed();
			if (time <= rampUpTime)
				actualSpeed = map(time, 0, rampUpTime, minPWM, maxPWM);
			else
				actualSpeed = speed;
		}
		chronoRampDown.restart();
		chronoRampDown.add(map(actualSpeed, maxPWM, minPWM, 0, rampDownTime));
	}
}



void Drive::stop() {
	analogWrite(PWM, 0);
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, HIGH);
}

void Drive::rapidStop() {
	analogWrite(PWM, PWMresolution);
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, HIGH);
}

void Drive::left(int pulseWidth) {
	analogWrite(PWM, pulseWidth);
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
}

void Drive::right(int pulseWidt) {
	analogWrite(PWM, pulseWidt);
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
}