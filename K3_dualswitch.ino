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

   Band Mapping on the K3
   
   Band mapping depends on the setting of CONFIG:KIO3. When set to NOR
   the bands are as follows:

   160 m =  1
    80 m =  2
	60 m =  0
    40 m =  3
    30 m =  4
    20 m =  5
    17 m =  6
    15 m =  7
    12 m =  8
    10 m =  9
     6 m = 10
	 
   Transverter bands are all mapped to 0.
   if CONFING:KIO3 is set to TRN, the band reported on band data lines 
   corresponds to the transverter band (TRN1 = 1, TRN2 = 2, etc.).

   Since transverter bands are irrelevant for HF operations where the antenna
   switch operates, CONFIG:KIO3 must be set to NOR for all of this to work.

   NOTE: antenna ports on the switch are numbered 1 to 8. 

*/

/* CAMELCASE SUCKS */

//#include <stdlib.h>


/************** USER CONFIGURABLE PART ***********/  

// should I decide to put an lcd on this thing
// give a name to each antenna port. keep it short.
char* antennaname[] = {
	"Yagi 6m",   // ant 1
	"Vert 6m",   // ant 2
	"",          // ant 3
	"",          // ant 4
	"",          // ant 5
	"",          // ant 6
	"autotuner", // ant 7
	"dipolo"     // ant 8
};

// for each possible band, 1 means the antenna can be tuned to it, 0 it can't	
uint8_t bandtoant[11][8] = {
	// ant1, ant2, ant3, ant4, ant5, ant6, ant7, ant8
	{ 0, 0, 0, 0, 0, 0, 0, 0 }, // 160 m
	{ 0, 0, 0, 0, 0, 0, 1, 0 }, //  80 m
	{ 0, 0, 0, 0, 0, 0, 1, 0 }, //  60 m
	{ 0, 0, 0, 0, 0, 0, 1, 1 }, //  40 m
	{ 0, 0, 0, 0, 0, 0, 1, 0 }, //  30 m
	{ 0, 0, 0, 0, 0, 0, 1, 1 }, //  20 m
	{ 0, 0, 0, 0, 0, 0, 1, 0 }, //  17 m
	{ 0, 0, 0, 0, 0, 0, 1, 1 }, //  15 m
	{ 0, 0, 0, 0, 0, 0, 1, 0 }, //  12 m
	{ 0, 0, 0, 0, 0, 0, 1, 1 }, //  10 m
	{ 1, 1, 0, 0, 0, 0, 0, 0 }, //   6 m
};


/*************************************************/



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
			Serial.println(oneofeight(band));
			sendbits(band,oneofeight(band));
		} else { 
			Serial.println("Not a hex digit");
		}
	}
}

// Write the data to the shift registers
void sendbits (byte master, byte slave) {
	// we don't want the relays to click while we send the data
	digitalWrite(latchpin, LOW);
	// send data to the shift registers
	shiftOut(datapin, clockpin, MSBFIRST, slave);  
	shiftOut(datapin, clockpin, MSBFIRST, master);  
	// data is complete, set latch HIGH and make things change!
	digitalWrite(latchpin, HIGH);

}

// Number to send to the shift registers to activate the appropriate only
// output
int oneofeight (int number) {
	if ( number ) { 
		return 1 << number-1;
	} else {
		return 0;
	}
}
