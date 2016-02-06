// -*- c++ -*-

/*
  K3/DXE-RR2X8B control interface

  Pierfrancesco Caci, ik5pvx

  2016-02-06

  This software is PUBLIC DOMAIN

*/

/* 
   Theory of operation

   We read the band data from the K3 Accessory port (4 lines, 
   each BCD value representing one band), we convert it to 
   1 of 8 signals to drive the antenna switch. 

   Two 74HC595 connected back to back provide the serial to
   parallel conversion. First '595 corresponds to the Master
   port on the switch, second '595 corresponds to the Slave.
   This way, it would be possible to use this interface also 
   with a single-radio antenna switch without changing the code. 

*/

/* CAMELCASE SUCKS */

#include <stdlib.h>

/* Hardware setup */

// First 74HC595 (Master) 
int latchpin = 10; //ST_CP
int clockpin = 12; // SH_CP
int datapin = 11; // DS

/* 
   Second 74HC595 has ST_CP and SH_CP wired in parallel to the first,
   while DS connects to Q7' of first '595. 
   Thus, for every 2 bytes sent, the most significant 
   corresponds to the slave.
*/


void setup() {
	// configure pins, zero all the registers so all relays are open.
	pinMode(latchpin, OUTPUT);
	pinMode(clockpin, OUTPUT);
	pinMode(datapin, OUTPUT);
	digitalWrite(latchpin, LOW);
	shiftOut(datapin, clockpin, MSBFIRST, 0);  
	shiftOut(datapin, clockpin, MSBFIRST, 0);  
	digitalWrite(latchpin, HIGH);
	// start a serial monitor to help debugging
	Serial.begin(9600);
	Serial.println("Type a hex digit");
}

void loop() {
	// Let's read a character from the serial and pretend it's our band data.
	if (Serial.available() > 0) {
		int band = Serial.read();
		if (isHexadecimalDigit(band)) {
			Serial.print("Selecting antenna for band ");
			// conversion from ascii hex digit to number, taking care of case
			band -= '0'; 
			if (band > 9 ) { 
				band -= 7;
				if (band > 16) {
					band -= 32;
				}
			}
			Serial.println(band);
			sendbits(band,band);
		} else { 
			Serial.println("Not a hex digit");
		}
	}
}


void sendbits (byte master, byte slave) {
	// we don't want the relays to click while we send the data
	digitalWrite(latchpin, LOW);
	// send data to the shift registers
	shiftOut(datapin, clockpin, MSBFIRST, slave);  
	shiftOut(datapin, clockpin, MSBFIRST, master);  
	// data is complete, set latch HIGH and make things change!
	digitalWrite(latchpin, HIGH);

}
