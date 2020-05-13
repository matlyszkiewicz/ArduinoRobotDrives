/**
	Arduino robot 2WD drive class

	@author  Mateusz £yszkiewicz
	@version 1.0
	@since   2020 - 03 - 20
*/

#include "RobotDrives2WD.hpp"

RobotDrives2WD::RobotDrives2WD() {
	left = new Drive();
	right = new Drive();
}

RobotDrives2WD::RobotDrives2WD(int IN1, int IN2, int PWM1, int IN3, int IN4, int PWM2) {
	left = new Drive();
	right = new Drive();
	pinConfigure(IN1, IN2, PWM1, IN3, IN4, PWM2);
}

void RobotDrives2WD::pinConfigure(int IN1, int IN2, int PWM1, int IN3, int IN4, int PWM2) {
	left->pinConfigure(IN1, IN2, PWM1);
	right->pinConfigure(IN3, IN4, PWM2);
}



void RobotDrives2WD::setVoltage(float supplyVoltage) {
	left->setVoltage(supplyVoltage);
	right->setVoltage(supplyVoltage);
}

void RobotDrives2WD::setVoltage(float supplyVoltage, float nominalVoltage) {
	left->setVoltage(supplyVoltage, nominalVoltage);
	right->setVoltage(supplyVoltage, nominalVoltage);
}

void RobotDrives2WD::setVoltage(float supplyVoltage, float nominalVoltage, float  startupVoltageOffset) {
	left->setVoltage(supplyVoltage, nominalVoltage, startupVoltageOffset);
	right->setVoltage(supplyVoltage, nominalVoltage, startupVoltageOffset);
}



void RobotDrives2WD::setSupplyVoltage(float supplyVoltage) {
	left->setSupplyVoltage(supplyVoltage);
	right->setSupplyVoltage(supplyVoltage);
}

void RobotDrives2WD::setNominalVoltage(float nominalVoltage) {
	left->setNominalVoltage(nominalVoltage);
	right->setNominalVoltage(nominalVoltage);
}

void RobotDrives2WD::setStartupVoltageOffset(float offsetVoltage) {
	left->setStartupVoltageOffset(offsetVoltage);
	right->setStartupVoltageOffset(offsetVoltage);
}



void RobotDrives2WD::setRampUpTime(int time) {
	left->setRampUpTime(time);
	right->setRampUpTime(time);
}

void RobotDrives2WD::setRampDownTime(int time) {
	left->setRampDownTime(time);
	right->setRampDownTime(time);
}



void RobotDrives2WD::setNextCondition(RobotDrives2WD::RobotCondition nextCondition) {

	switch (nextCondition) {
	case RobotDrives2WD::RobotCondition::stop:
		left->setNextCondition(Drive::Condition::stop);
		right->setNextCondition(Drive::Condition::stop);
		break;
	case RobotDrives2WD::RobotCondition::rapidStop:
		left->setNextCondition(Drive::Condition::rapidStop);
		right->setNextCondition(Drive::Condition::rapidStop);
		break;
	case RobotDrives2WD::RobotCondition::left:
		left->setNextCondition(Drive::Condition::right);
		right->setNextCondition(Drive::Condition::right);
		break;
	case RobotDrives2WD::RobotCondition::right:
		left->setNextCondition(Drive::Condition::left);
		right->setNextCondition(Drive::Condition::left);
		break;
	case RobotDrives2WD::RobotCondition::forward:
	case RobotDrives2WD::RobotCondition::forwardLeft:
	case RobotDrives2WD::RobotCondition::forwardRight:
		left->setNextCondition(Drive::Condition::left);
		right->setNextCondition(Drive::Condition::right);
		break;
	case RobotDrives2WD::RobotCondition::rear:
	case RobotDrives2WD::RobotCondition::rearLeft:
	case RobotDrives2WD::RobotCondition::rearRight:
		left->setNextCondition(Drive::Condition::right);
		right->setNextCondition(Drive::Condition::left);
		break;
	}

	condition = nextCondition;
}

void RobotDrives2WD::conditionUpdate(bool rampUpdate) {

	int turnSpeed = speed - angle;
	if (turnSpeed < 0)
		turnSpeed = 0;

	switch (condition) {
	case RobotDrives2WD::RobotCondition::stop:
	case RobotDrives2WD::RobotCondition::rapidStop:
		break;
	case RobotDrives2WD::RobotCondition::left:
	case RobotDrives2WD::RobotCondition::right:
	case RobotDrives2WD::RobotCondition::forward:
	case RobotDrives2WD::RobotCondition::rear:
		left->setSpeed(speed);
		right->setSpeed(speed);
		break;
	case RobotDrives2WD::RobotCondition::forwardLeft:
	case RobotDrives2WD::RobotCondition::rearLeft:
		left->setSpeed(turnSpeed);
		right->setSpeed(speed);
		break;
	case RobotDrives2WD::RobotCondition::forwardRight:
	case RobotDrives2WD::RobotCondition::rearRight:
		left->setSpeed(speed);
		right->setSpeed(turnSpeed);
		break;
	}

	left->conditionUpdate(rampUpdate);
	right->conditionUpdate(rampUpdate);
}



Drive* RobotDrives2WD::getLeftDrive() {
	return left;
}

Drive* RobotDrives2WD::getRightDrive() {
	return right;
}