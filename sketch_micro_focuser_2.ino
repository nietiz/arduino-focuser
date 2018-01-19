// --------------------------------------------------------------------------------
//
// sketch_micro_focuser_2.ino
//
// Description:	Arduino sketch for an absolute micro focuser with generic stepper motor.
//				This project uses an Arduino micro connected via USB to a remote PC
//				to control a stepper motor which moves a telescope focuser.
//				
//				FEATURES:
//				- absolute focuser;
//				- two selectable speeds, slow and fast;
//				- controlled both by a remote PC through USB interface
//				  and by a local keypad;
//				- temperature compensation;
//
//				IMPLEMENTATION
//              To provide an ABSOLUTE FOCUSER we use a limit switch, 
//				in order to find the zero position (max intra-focal);
//				furthermore if we want to move the focuser "manually"
//				near the telescope (not by the PC console) without losing
//				the step count then we have to manage a keypad with IN
//				and OUT buttons; an optional SPEED button is provided
//				to toggle speed slow/fast.
//				Temperature sensing is done by a NTC connected to an
//				analog port of the Arduino board.
//
//				NOTES
//				An unipolar/bipolar stepper motor driver, capable to provide
//				more current and micro-stepping is used; voltage from 7 to 30 V
//				are allowed; current sensing and a trimmer allow to set the current
//				into the motor windings and thus the motor couple.
//				For reference see http://www.pololu.com/product/2133 and 
//				http://www.pololu.com/file/0J590/drv8825.pdf .
//				Basing on my experiments the smoothest movement is reached when
//				driving the motor @ 32 micro-steps; BUT intermediate positions
//				are not stable and require a constant draw of current; the 
//				motor heats. Thus the step count is based on full steps;
//				each full step is done performing the whole number of micro-steps.
//				When in idle state the motor controller is disabled and the
//				the controller's H bridge outputs are in high impedance state; the
//				motor doesn't heat; the motor shaft is free and you can rotate the 
//				focuser knob manually but in this case the step count (i.e. the
//				absolute position) is lost; using the keypad maintains the counting
//				and the absolute position.
//
//				Temperature sensing is done by a NTC (RS 151-237 equivalent to
//				EPCOS EC95F103W, 10KOhm @ 25°C).
//				The attached document reports the following formula to calculate
//				the temperature from the resistance of the thermistor (from 0°C to 50°C):
//				
//              1/T = a + b * ln(Rt/R25) + c * ln(Rt/R25)^2 + d * ln(Rt/R25)^3
//
//				where Rt/R25 means (resistance @ t°C) / (resistance @ 25°C)
//				and a = 3.3540154e-3, b = 2.5627725e-4, c = 2.082921e-6, d = 7.30032e-8
//
//
//
// Author:		(NT) Tiziano Niero, tiziano.niero@gmail.com, www.tizianoniero.it
//
//              This program is free software: you can redistribute it and/or modify
//              it under the terms of the GNU General Public License as published by
//              the Free Software Foundation, either version 3 of the License, or
//              (at your option) any later version.
//
//              This program is distributed in the hope that it will be useful,
//              but WITHOUT ANY WARRANTY; without even the implied warranty of
//              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//              GNU General Public License for more details.
//
//              You should have received a copy of the GNU General Public License
//              along with this program.  If not, see <http://www.gnu.org/licenses/>.
// Edit Log:
//
// Date			Who	Vers	Description
// -----------	---	-----	-------------------------------------------------------
// 05-sep-2014	NT	1.0.0	created
// --------------------------------------------------------------------------------


#include <EEPROM.h>
#include "DebounceButton.h"
#define DEBUG
#define STEPS_REV    200

#define SPEED_FAST_DELAY   1024
#define SPEED_SLOW_DELAY   16383

#define SPEED_FAST    0
#define SPEED_SLOW    1


#define BAUD_RATE    115200//57600
#define MICRO_STEPS_DEFAULT  32


const String focuserName = "GenericStepperFocuser";
const String currentVersion = "1.0.0";



enum MOVE_STATUS
{
	STOPPED,
	KEYPAD_MOVE_IN,
	KEYPAD_MOVE_OUT,
	SERIAL_MOVE_IN,
	SERIAL_MOVE_OUT,
	ZEROING,
	TEMP_COMPENSATION
};

// pins
const int pinLimitSwitch = 2;			// D2 - limit switch (in)
const int pinMotorStep = 5;				// D5 - motor step (out)
const int pinMotorDir = 6;				// D6 - motor direction (out)
const int pinMotorEna = 8;				// D8 - motor enable (out)
const int pinMotorM0 = 9;				// D9 -  microstepping mode (out)
const int pinMotorM1 = 10;				// D10 - microstepping mode (out)
const int pinMotorM2 = 11;				// D11 - microstepping mode (out)
const int pinButtonIn = 3;				// D3 - keypad button intrafocal (in)
const int pinButtonOut = 4;				// D4 - keypad button extrafocal (in)
const int pinButtonSpeed = 7;			// D7 - keypad button toggle speed slow/fast (in)
const int pinTempProbe = A5;			// A5 - analog input for temperature probe

const int pinLed = 13;					// LED connected to digital pin 13


// variables 
long currentPosition = 0;
long targetPosition = 0;
long maxPosition = 100000;

// motor settigs
int microSteps;		// micro-steps/step
int fullStepsRev;	// number of full steps per revolution
int currentSpeed;
long currentSpeedDelay;

// temperature measurement and compensation
bool temperatureAvailable = false;
double temperature = 0;				// °C
double tempCoefficient = 10;			// step/°C
double tempAtCompensationStart;		// °C: temperature when compensation started
int positionAtCompensationStart;	// position when compensation started

// zero finding
bool isZeroingDone = false;

// keypad button with debouncing
DebounceButton* buttonIn;
DebounceButton* buttonOut;
DebounceButton* buttonSpeed;

// moving status
MOVE_STATUS status = STOPPED;

// pointer to delay function; the real delay function depends on the actual delay amount
void (*delayFunction)(void);




// -----------------------------------------------------------------
//
// SETUP FUNCTION
//
void setup()
{
#ifdef DEBUG
	pinMode(pinLed, OUTPUT);
#endif

	// read form EEPROM the maxPosition value; if inconsistent 
	// then initialize it to a reasonable value
	maxPosition = readEepromLong(0);
	if(maxPosition <= 0 || maxPosition > 1e5)
	{
		maxPosition = 1e4;
		writeEepromLong(0, maxPosition);
	}

	// setup micro-switch used to do zeroing
	// set pin 12 to input and activates internal 20K pullup resistor
	pinMode(pinLimitSwitch, INPUT_PULLUP);

	// setup stepper motor    
	pinMode(pinMotorStep, OUTPUT);
	pinMode(pinMotorDir, OUTPUT);
	pinMode(pinMotorEna, OUTPUT);
	pinMode(pinMotorM0, OUTPUT);
	pinMode(pinMotorM1, OUTPUT);
	pinMode(pinMotorM2, OUTPUT);

	MotorDisable();

	SetMicroSteps(MICRO_STEPS_DEFAULT);
	SetSpeed(SPEED_FAST);

	// initialize focuser keypad
	pinMode(pinButtonIn, INPUT_PULLUP);
	pinMode(pinButtonOut, INPUT_PULLUP);
	pinMode(pinButtonSpeed, INPUT_PULLUP);

	buttonIn = new DebounceButton(pinButtonIn, HIGH);
	buttonOut = new DebounceButton(pinButtonOut, HIGH);
	buttonSpeed = new DebounceButton(pinButtonSpeed, HIGH);

	// initialize temperature probe analog input
	pinMode(pinTempProbe, INPUT);

	// start serial comm
	Serial.begin(BAUD_RATE);
	Serial.setTimeout(200);

}

// -----------------------------------------------------------------
//
// LOOP FUNCTION
//
void loop()
{
	ParseSerialPort();  
	ParseKeypad();  
	Move();
	ReadTemperature();
}

// -----------------------------------------------------------------
//
// ReadTemperature function.
//
// NOTE: using the specified NTC (see initial notes) @ -50°C the
// value Rt/R25 is ~70 and the related analog value is ~15;
// thus, since the analog pin is pulled down by a 10K resistor,
// values below 10 are possible only when the NTC is not connected.
void ReadTemperature()
{
	if(status == STOPPED || status == TEMP_COMPENSATION)
	{
		int value = analogRead(pinTempProbe);
		if(value < 10)
		{
			temperatureAvailable = false;	// i.e. probe not connected
		}
		else
		{
			temperatureAvailable = true;
			double RtR25 = 1024.0 / (double)value - 1.0;  
			double lnRtR25 = log(RtR25);
			temperature = 1.0 /  (3.3540154e-3 + lnRtR25 * (2.5627725e-4 + lnRtR25 * (2.082921e-6 + lnRtR25 * 7.30032e-8)));
			temperature -= 273.15;	// from Kelvin to Celsius
		}
		//Serial.println(value);
	}
}

// -----------------------------------------------------------------
//
// Move function.
//
void Move()
{
	switch (status)
	{
	case STOPPED:
		break;
	case KEYPAD_MOVE_IN:
	case SERIAL_MOVE_IN:
		if (digitalRead(pinLimitSwitch) == LOW)
		{
			// check always the limit switch: maybe a manual movement changed
			// inward the real position of the focuser; 
			// if pressed then reset current and target positions and stop the motor
			currentPosition = 0;
			targetPosition = 0;
			isZeroingDone = true;
			MotorDisable();
			status = STOPPED;
		}
		else
		{
			DoFullStep();
			if(isZeroingDone)
				currentPosition--;
			if(status == SERIAL_MOVE_IN && currentPosition == targetPosition)
			{
				MotorDisable();
				status = STOPPED;
			}
		}
		break;
		break;
	case KEYPAD_MOVE_OUT:
	case SERIAL_MOVE_OUT:
		DoFullStep();
		currentPosition++;
		if(currentPosition == targetPosition)
		{
			status = STOPPED;
			MotorDisable();
		}
		break;
	case ZEROING:
		if (digitalRead(pinLimitSwitch) == LOW)
		{
			isZeroingDone = true;
			MotorDisable();
			status = STOPPED;
		}
		DoFullStep();
		break;
	case TEMP_COMPENSATION:
		Compensate();
		break;
	}
}


// -----------------------------------------------------------------
//
// ParseKeypad function.
//
void ParseKeypad()
{
	buttonIn->readState();
	buttonOut->readState();
	buttonSpeed->readState();

	// NB: button pins are in pull-up, HIGH = button released, LOW = button pressed
	switch (status)
	{
	case STOPPED:
		if(buttonIn->isHighToLow())
		{
			// we leave anyway moving intra-focal because there is a limit-switch
			SetDirectionIntraFocal();
			targetPosition = 0;
			status = KEYPAD_MOVE_IN;
			MotorEnable();
			DebugToggleLed(true);
		}
		else if(buttonOut->isHighToLow())
		{
			// moving extra-focal is allowed only if zeroing was done
			if(!isZeroingDone)
				return;
			// limit moving until maxPosition is reached
			if(currentPosition == maxPosition)
				return;
			SetDirectionExtraFocal();
			targetPosition = maxPosition;
			status = KEYPAD_MOVE_OUT;
			MotorEnable();
			DebugToggleLed(true);
		}
		if(buttonSpeed->isHighToLow())
		{
			if(currentSpeed == SPEED_FAST)
				SetSpeed(SPEED_SLOW);
			else
				SetSpeed(SPEED_FAST);
		}
		break;
	case KEYPAD_MOVE_IN:
		if(buttonIn->isLowToHigh())
		{
			MotorDisable();
			targetPosition = currentPosition;
			status = STOPPED;
			DebugToggleLed(false);
		}
		break;
	case KEYPAD_MOVE_OUT:
		if(buttonOut->isLowToHigh())
		{
			MotorDisable();
			targetPosition = currentPosition;
			status = STOPPED;
			DebugToggleLed(false);
		}
		break;
	case SERIAL_MOVE_IN:
	case SERIAL_MOVE_OUT:
	case ZEROING:
		// do nothing
		break;
	}
}

// -----------------------------------------------------------------
//
// ParseCommand function.
//
//
// Each command is made by a string terminated with newline character ('\n').
// Defined commands:
// Mx:  move x steps (x can be positive or negative)
// P: send targetPosition to client 
// H: stop moving (both GoTo and zeroing)
// I: send device information
// S: send "OK"
// V: send isMoving ("1" = true, "0" = false)
// Z: start zeroing
void ParseSerialPort()
{  

	if(Serial.available() == 0)
		return;

	String command = Serial.readStringUntil('\n');

	// -----------------------
	if(command == "POSITION_GET")
	{
		SendPosition();
		return;
	}


	// -----------------------
	if(command == "HALT")
	{
		switch (status)
		{
		case STOPPED:
			Serial.print("Already stopped\n");
			break;
		case SERIAL_MOVE_IN:
		case SERIAL_MOVE_OUT:
		case ZEROING:
			SendOK();
			Halt();
			break;
		case KEYPAD_MOVE_OUT:
		case KEYPAD_MOVE_IN:
			Serial.print("Cannot stop: moving by keypad command\n");
			break;
		}
		return;
	}

	// -----------------------
	if(command.substring(0, 6) == "MOVETO")
	{
		if (status == STOPPED) 
		{
			SendOK();
			targetPosition = command.substring(6).toInt();
			if(targetPosition == currentPosition)
				return;
			if(targetPosition < 0)
				targetPosition = 0;
			else if(targetPosition > maxPosition)
				targetPosition = maxPosition;

			if(targetPosition < currentPosition)
			{
				SetDirectionIntraFocal();
				status = SERIAL_MOVE_IN ;
			}
			else
			{
				SetDirectionExtraFocal();
				status = SERIAL_MOVE_OUT;
			}
			MotorEnable();
			return;
		}
		Serial.print("Cannot move: already moving, stop before\n");
		return;
	}


	// -----------------------
	// start zeroing
	if(command == "ZERO")
	{
		if (status == STOPPED) 
		{
			SendOK();
			SetSpeed(SPEED_FAST);
			SetDirectionIntraFocal();
			MotorEnable();
			status = ZEROING;
			currentPosition = 0;
			isZeroingDone = false;
		}
		else
			Serial.print("Cannot zero: already moving, stop before\n");
		return;
	}

	// -----------------------
	// query moving status
	if(command == "MOVING")
	{		
		SendIsMoving();
		return;
	}

	// -----------------------
	// set absolute position; this also sets the "zero" as done
	if(command.substring(0, 12) == "POSITION_SET")
	{
		if (status == STOPPED) 
		{
			SendOK();
			targetPosition = command.substring(12).toInt();
			currentPosition = targetPosition;
			isZeroingDone = true;
		}
		else
			Serial.print("Cannot set absolute position: motor is moving, stop before\n");
		return;
	}

	// -----------------------
	// query temperature
	if(command == "TEMP_GET")
	{
		SendTemperature();
		return;
	}

	// -----------------------
	// query if temperature measurement is available
	if(command == "TEMPAVAILABLE_GET")
	{
		SendTemperatureAvailable();
		return;
	}

	// -----------------------
	// change speed
	if(command.substring(0, 9) == "SPEED_SET")
	{
		if (status == STOPPED) 
		{
			int value = command.substring(9).toInt();
			if(SetSpeed(value) == true)
				SendOK();
			else
				Serial.print("Wrong parameter [speed]\n");
		}
		else
			Serial.print("stop moving before setting speed\n");
		return;
	}


	// -----------------------
	// query device name
	if(command == "NAME")
	{
		SendName();
		return;
	}

	// -----------------------
	// query firmware version
	if(command == "VERSION")
	{
		SendVersion();
		return;
	}

	// -----------------------
	// query max position (steps)
	if(command == "MAXPOS_GET")
	{
		SendMaxPosition();
		return;
	}

	// -----------------------
	// set max position (steps); this value will be storedin EEPROM 
	// (only if different from current value)
	if(command.substring(0, 10) == "MAXPOS_SET")
	{
		long value = command.substring(10).toInt();
		if(maxPosition != value)
		{
			maxPosition = value;
			writeEepromLong(0, maxPosition);
		}
		SendOK();
		return;
	}

	// -----------------------
	// query micro-step setting (possible values: 1, 2, 4, 8, 16, 32)
	if(command == "MICROSTEPS_GET")
	{
		SendMicroSteps();
		return;
	}

	// -----------------------
	// set micro-step setting (possible values: 1, 2, 4, 8, 16, 32)
	if(command.substring(0, 14) == "MICROSTEPS_SET")
	{
		if (status == STOPPED) 
		{
			int value = command.substring(14).toInt();
			if(SetMicroSteps(value) == true)
			{
				SendOK();
				SetSpeed(currentSpeed);
			}
			else			
				Serial.print("Wrong parameter [micro-steps]\n");
		}
		else
			Serial.print("Cannot set micro-steps: already moving, stop before\n");
		return;
	}

	// -----------------------
	// start temperature compensation
	if(command == "TEMPCOMP_START")
	{
		if(status == STOPPED)
		{
			if(temperatureAvailable)
			{
				SendOK();		
				StartTempCompensation();
			}
			else
				Serial.print("Cannot start temperature compensation: temperature not available\n");
		}
		else
			Serial.print("Cannot start temperature compensation: already moving, stop before\n");
		return;

	}

	// -----------------------
	// start temperature compensation
	if(command == "TEMPCOMP_STOP")
	{
		if(status == TEMP_COMPENSATION)
		{
				SendOK();		
				StopTempCompensation();
		}
		else
			Serial.print("Wrong command: temperature compensation is already stopped\n");
		return;
	}

	// -----------------------
	// query temperature compensation status
	if(command == "TEMPCOMP_GET")
	{
			
		SendTempCompensation();
		return;
	}

	// -----------------------
	// command not recognized
	Serial.print(command + ": unknown command\n");



}


// --------------------------------------------------------
//
// SendOK function.
//
//
void SendOK()
{
	Serial.print("OK\n");   
}

// --------------------------------------------------------
//
// SendMicroSteps function.
//
//
void SendMicroSteps()
{
	Serial.print(microSteps);
	Serial.print("\n");   
}

// --------------------------------------------------------
//
// SendMaxPosition function.
//
//
void SendMaxPosition()
{
	Serial.print(maxPosition);
	Serial.print("\n");   
}

// --------------------------------------------------------
//
// SetMicroSteps function.
//
//
bool SetMicroSteps(int value)
{
	// initialize micro-steps
	switch(value)
	{
	case 1:  // full step
		digitalWrite(pinMotorM0, LOW);
		digitalWrite(pinMotorM1, LOW);
		digitalWrite(pinMotorM2, LOW);
		break;
	case 2:  // half step
		digitalWrite(pinMotorM0, HIGH);
		digitalWrite(pinMotorM1, LOW);
		digitalWrite(pinMotorM2, LOW);
		break;
	case 4:  // 4 micro-steps/step
		digitalWrite(pinMotorM0, LOW);
		digitalWrite(pinMotorM1, HIGH);
		digitalWrite(pinMotorM2, LOW);
		break;
	case 8:  // 8 micro-steps/step
		digitalWrite(pinMotorM0, HIGH);
		digitalWrite(pinMotorM1, HIGH);
		digitalWrite(pinMotorM2, LOW);
		break;
	case 16: // 16 micro-steps/step
		digitalWrite(pinMotorM0, LOW);
		digitalWrite(pinMotorM1, LOW);
		digitalWrite(pinMotorM2, HIGH);
		break;
	case 32: // 32 micro-steps/step
		digitalWrite(pinMotorM0, HIGH);
		digitalWrite(pinMotorM1, LOW);
		digitalWrite(pinMotorM2, HIGH);
		break;
	default:
		return false;
	}
	microSteps = value;
	return true;

}



// --------------------------------------------------------
//
// SendVersion function.
//
//
void SendVersion()
{
	Serial.print(currentVersion);
	Serial.print("\n");   
}


// --------------------------------------------------------
//
// SendPosition function.
//
//
void SendPosition()
{
	Serial.print(currentPosition);
	Serial.print("\n");   
}

// --------------------------------------------------------
//
// SendIsMoving function.
//
//
void SendIsMoving()
{
	Serial.print(status == STOPPED);
	Serial.print("\n");   
}

// --------------------------------------------------------
//
// Compensate function.
// Do a single compensation step
//
void Compensate()
{
	static int count;

	count++;
	if(count == 500)
	{
		count = 0;
		int delta = (int)((temperature - tempAtCompensationStart) * tempCoefficient);
		targetPosition = positionAtCompensationStart + delta;
		if(targetPosition < 0)
			targetPosition = 0;
		else if(targetPosition > maxPosition)
			targetPosition = maxPosition;
		if(targetPosition < currentPosition && digitalRead(pinLimitSwitch) == HIGH)
		{
			SetDirectionIntraFocal();
			MotorEnable();
			//while (currentPosition > targetPosition)
			//{
				DoFullStep();
				currentPosition--;
			//}
			MotorDisable();
		}
		else if(targetPosition > currentPosition)
		{
			SetDirectionExtraFocal();
			MotorEnable();
			//while (currentPosition < targetPosition)
			//{
				DoFullStep();
				currentPosition++;
			//}
			MotorDisable();
		}
	}
}

// --------------------------------------------------------
//
// StartTempCompensation function.
//
//
void StartTempCompensation()
{
	tempAtCompensationStart = temperature;
	positionAtCompensationStart = currentPosition;
	SetSpeed(SPEED_SLOW);
	status = TEMP_COMPENSATION;
}

// --------------------------------------------------------
//
// StopTempCompensation function.
//
//
void StopTempCompensation()
{
	status = STOPPED;

}

// --------------------------------------------------------
//
// SendTempCompensation function.
//
//
void SendTempCompensation()
{
	Serial.print(status == TEMP_COMPENSATION);
	Serial.print("\n");   
}

// --------------------------------------------------------
//
// SendTemperature function (°C * 10).
//
//
void SendTemperature()
{
	int t = (int)(temperature * 10.0);// + temperature >= 0 ? 0.5 : -0.5);
	Serial.print(t);
	Serial.print("\n");   
}

// --------------------------------------------------------
//
// SendTemperatureAvailable function.
//
//
void SendTemperatureAvailable()
{
	Serial.print(temperatureAvailable);
	Serial.print("\n");   
}

// --------------------------------------------------------
//
// SendName function.
//
//
void SendName()
{
	Serial.print(focuserName);
	Serial.print("\n");
}

// --------------------------------------------------------
//
// Halt function.
//
//
void Halt()
{
	status = STOPPED;	
	targetPosition = currentPosition;
	MotorDisable();
}



// --------------------------------------------------------
//
// DoFullStep function.
//
//
void DoFullStep()
{
	for(int j = 0; j < microSteps; j++)
	{
		digitalWrite(pinMotorStep, HIGH);
		digitalWrite(pinMotorStep, LOW);
		delayMicroseconds(currentSpeedDelay);
	}
}







// --------------------------------------------------------
//
// SetSpeed function.
//
//
bool SetSpeed(int value)
{
	switch(value)
	{
	case SPEED_FAST:
		currentSpeedDelay = SPEED_FAST_DELAY / microSteps;
		break;
	case SPEED_SLOW:
		currentSpeedDelay = SPEED_SLOW_DELAY / microSteps;            
		break;
	default:
		return false;
	}
	currentSpeed = value;
	return true;
}

// --------------------------------------------------------
//
// SetDirectionIntraFocal function.
//
//
void SetDirectionIntraFocal()
{
	digitalWrite(pinMotorDir, HIGH); 
}

// --------------------------------------------------------
//
// SetDirectionExtraFocal function.
//
//
void SetDirectionExtraFocal()
{
	digitalWrite(pinMotorDir, LOW); 
}

// --------------------------------------------------------
//
// MotorEnable function.
//
//
void MotorEnable()
{
	digitalWrite(pinMotorEna, LOW);
}

// --------------------------------------------------------
//
// MotorDisable function.
//
//
void MotorDisable()
{
	digitalWrite(pinMotorEna, HIGH);
}


void DebugToggleLed(bool active)
{
#ifdef DEBUG
	digitalWrite(pinLed, active ? HIGH : LOW);
#endif
}



long readEepromLong(int address)
{
	long b[4];
	for(int i=0; i<4; i++)
		b[i] = EEPROM.read(address+i);
	return b[0] + (b[1] << 8) + (b[2] << 16) + (b[3] << 24);
}

void writeEepromLong(int address, long value)
{
	byte b[4];
	b[0] = value & 0xFF;
	b[1] = (value >> 8) & 0xFF;
	b[2] = (value >> 16) & 0xFF;
	b[3] = (value >> 24) & 0xFF;
	for(int i=0; i<4; i++)
		EEPROM.write(address+i, b[i]);
}