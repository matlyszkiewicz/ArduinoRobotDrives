/**
	Arduino robot drives

	@author  Mateusz £yszkiewicz
	@version 1.0
	@since   2020 - 03 - 20

	Example usage PS2X and Chrono library. 

	Thanks to Bill Porter for his PS2X library.
	Thanks to Sofian Audry and Thomas Ouellet Fredericks for Chrono library.
*/

#include "Arduino.h"
#include <Chrono.h>
#include <PS2X_lib.h>
#include "RobotDrives2WD.hpp"

// SPI pins
constexpr uint8_t data = 12;
constexpr uint8_t command = 11;
constexpr uint8_t select = 10;
constexpr uint8_t clock = 13;

// Additional ps2 gamepad functionality
constexpr bool pressures = false;
constexpr bool rumble = false;

// Arduino logic pins for H bridge
constexpr uint8_t IN1 = 4;
constexpr uint8_t IN2 = 5;
constexpr uint8_t IN3 = 6;
constexpr uint8_t IN4 = 7;

// Arduino enable pins for H bridge
constexpr uint8_t PWM1 = 3;
constexpr uint8_t PWM2 = 9;

// Arduino auto reset pin when connecting with ps2 gamepad is lost
constexpr int resetPin = 2;

// Robot initial supply and drive voltages
constexpr float supplyVoltage = 8.2;
constexpr float nominalVoltage = 6.;
constexpr float offsetVoltage = 4.;

// Ramp times
constexpr int rampUpTime = 750;
constexpr int rampDownTime = 250;

RobotDrives2WD robotDrives2WD;
PS2X gamepad;

void setup() {

	// Timers frequency set as high as possible for better current control by PWM
	TCCR1B = TCCR1B & B11111000 | B00000001;
	TCCR2B = TCCR2B & B11111000 | B00000001;

	// Gamepad configure
	gamepad.config_gamepad(clock, command, select, data, pressures, rumble);

	// Robot with 2 axis configure
	robotDrives2WD.pinConfigure(IN1, IN2, PWM1, IN3, IN4, PWM2);
	robotDrives2WD.setVoltage(supplyVoltage, nominalVoltage, offsetVoltage);

	// Arduino auto reset configure
	digitalWrite(resetPin, HIGH);
	pinMode(resetPin, OUTPUT);
}



void loop() {

	// Data update every 10ms
	if (millis() % 10 == 0) {

		// Gamepad data update
		gamepad.read_gamepad();

		// Arduino reset when gamepad has lost signal
		if (gamepad.Analog(PSS_LY) == 128 &&
			gamepad.Analog(PSS_LX) == 128 &&
			gamepad.Analog(PSS_RY) == 128 &&
			gamepad.Analog(PSS_RX) == 128 &&
			millis() > 5000)
			digitalWrite(resetPin, LOW);

		// Turbo mode (higher supply voltage and ramp time equals zero) when L1 and R1 is pressed
		if (gamepad.Button(PSB_L1) && gamepad.Button(PSB_R1)) {
			robotDrives2WD.setRampUpTime(0);
			robotDrives2WD.setRampDownTime(0);
			robotDrives2WD.setNominalVoltage(supplyVoltage);
		}
		else {
			robotDrives2WD.setRampUpTime(rampUpTime);
			robotDrives2WD.setRampDownTime(rampDownTime);
			robotDrives2WD.setNominalVoltage(nominalVoltage);
		}

		// Speed, angle and condition settings for robot
		if (gamepad.Analog(PSS_LY) == 127) {
			robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::stop);
		}
		else {
			int speed = gamepad.Analog(PSS_LY) - 127;
			if (speed < 0) {
				speed = -speed;
				speed++;
			}
			robotDrives2WD.setSpeed(speed << 1);

			int angle = gamepad.Analog(PSS_RX) - 128;
			if (angle < 0) {
				angle = -angle;
				angle++;
			}
			robotDrives2WD.setAngle(angle << 1);

			// If L2 and R2 is pressed, robot can rotate around its axis
			if (gamepad.Analog(PSS_LY) < 127) {
				if (!angle)
					robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::forward);
				else if (gamepad.Analog(PSS_RX) < 128) {
					if (gamepad.Button(PSB_L2) && gamepad.Button(PSB_R2))
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::left);
					else
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::forwardLeft);
				}
				else {
					if (gamepad.Button(PSB_L2) && gamepad.Button(PSB_R2))
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::right);
					else
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::forwardRight);
				}
			}
			else if (gamepad.Analog(PSS_LY) > 127) {
				if (!angle)
					robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::rear);
				else if (gamepad.Analog(PSS_RX) < 128) {
					if (gamepad.Button(PSB_L2) && gamepad.Button(PSB_R2))
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::left);
					else
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::rearLeft);
				}
				else {
					if (gamepad.Button(PSB_L2) && gamepad.Button(PSB_R2))
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::right);
					else
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::rearRight);
				}
			}
		}

		// Robot move update
		robotDrives2WD.conditionUpdate();
	}
}
