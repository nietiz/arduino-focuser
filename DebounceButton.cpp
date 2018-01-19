// --------------------------------------------------------------------------------
//
// DebounceButton.cpp
//
// Description:	source file for DebounceButton class.
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
//

#include "DebounceButton.h"

DebounceButton::DebounceButton(int pin, int initialState)
{
	writePosition = 0;
	this->pin = pin;
	for (int i = 0; i < MAX_READS; i++)
		buffer[i] = initialState;
	oldState = initialState;
	currentState = initialState;
}


void DebounceButton::readState()
{
	// read pin state
	buffer[writePosition] = digitalRead(pin);
    // circulate buffer
	writePosition++;
	if(writePosition == MAX_READS)
		writePosition = 0;
	// get new state; possible values: stabilized LOW, stabilized HIGH, not stabilized
	int newState;
	int stateHighCounter = 0;
	for (int i = 0; i < MAX_READS; i++)	
		stateHighCounter += buffer[i];
	if(stateHighCounter == 0)
		newState = LOW;		// bounce stabilized to LOW
	else if(stateHighCounter == MAX_READS)
		newState = HIGH;	// bounce stabilized to HIGH
	else
		newState = currentState; // bounce not stabilized
	// set definitive values
	if(newState == currentState)
		oldState = currentState;	// no variations from the last reading: get rid of difference if any
	else
		currentState = newState;	// confirm the variation
}


