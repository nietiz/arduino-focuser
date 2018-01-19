// --------------------------------------------------------------------------------
//
// DebounceButton.h
//
// Description:	header file for DebounceButton class.
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


#ifndef _DEBOUNCEBUTTON_h
#define _DEBOUNCEBUTTON_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#define MAX_READS	4

class DebounceButton
{
 public:
	DebounceButton(int pin, int initialState);
	void readState();

	bool isLowToHigh()
	{
		return oldState == LOW && currentState == HIGH;
	}
	bool isHighToLow()
	{
		return oldState == HIGH && currentState == LOW;
	}


 private:
	 int pin;
	 int writePosition;
	 int buffer[MAX_READS];
	 int oldState;
	 int currentState;
};


#endif

