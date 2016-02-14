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

   Circuit design notes

   Jack Brindle W6FB recommends protecting the circuit by using pullups
   on the Arduino digital inputs (either internal or external) and diodes
   with Cathode facing the K3. This will prevent powering the Arduino board
   from the K3 +5V. The K3 will act as drain, pulling down the voltage on
   the Arduino pins. Recommends low voltage drop diodes (schottky, 
   e.g. 1N5711 - the KPA500 uses BAT54 which are SMD only, a suitable 
   equivalent is BAT42 which is also available in though-hole package). 

*/

/************** USER CONFIGURABLE PART ***********/  

#define DEBUG

// should I decide to put an lcd on this thing
// give a name to each antenna port. keep it short.
char* antennaname[] = {
	"***NONE***",// ant 0 doesn't exist, switch counts from 1
	"Yagi 6m",   // ant 1
	"Vert 6m",   // ant 2
	"",          // ant 3
	"",          // ant 4
	"",          // ant 5
	"",          // ant 6
	"autotuner", // ant 7
	"dipolo"     // ant 8
};

char* hfbandname[] = {
	" 60 m",
	"160 m",
    " 80 m",
	" 40 m",
	" 30 m",
	" 20 m",
	" 17 m",
	" 15 m",
	" 12 m",
	" 10 m",
	"  6 m",
};

// for each possible band, 2 means it is the preferred antenna, 
// 1 means the antenna can be used, 0 means antenna not usable.
// NOTE: 60 m is band 0. 
uint8_t bandtoant[11][8] = {
	// ant1, ant2, ant3, ant4, ant5, ant6, ant7, ant8
	{ 0, 0, 0, 0, 0, 0, 0, 0 }, //  60 m
	{ 0, 0, 0, 0, 0, 0, 0, 0 }, // 160 m
	{ 0, 0, 0, 0, 0, 0, 2, 0 }, //  80 m
	{ 0, 0, 0, 0, 0, 0, 1, 2 }, //  40 m
	{ 0, 0, 0, 0, 0, 0, 2, 0 }, //  30 m
	{ 0, 0, 0, 0, 0, 0, 1, 2 }, //  20 m
	{ 0, 0, 0, 0, 0, 0, 2, 0 }, //  17 m
	{ 0, 0, 0, 0, 0, 0, 2, 1 }, //  15 m
	{ 0, 0, 0, 0, 0, 0, 2, 0 }, //  12 m
	{ 0, 0, 0, 0, 0, 0, 1, 2 }, //  10 m
	{ 2, 1, 0, 0, 0, 0, 0, 0 }, //   6 m
};


/*************************************************/

#ifdef DEBUG
// consider defining serial.print macros here
#endif


/* Hardware setup */

// First 74HC595 (Master) 
const int latchpin = 10; //ST_CP
const int clockpin = 12; // SH_CP
const int datapin = 11; // DS

/* 
   Second 74HC595 has ST_CP and SH_CP wired in parallel to the first,
   while DS connects to Q7' of first '595. 
   Thus, for every 2 bytes sent, the most significant 
   corresponds to the slave.
*/

// The band lines from the accessory port
const int band0pin = 2;
const int band1pin = 3;
const int band2pin = 4;
const int band3pin = 5;

// The 4 pushbuttons
const int masterautopin = 6;
const int masternextpin = 7;
const int slaveautopin = 8;
const int slavenextpin = 9;


void setup() {
	// configure pins, zero all the registers so all relays are open.
	pinMode(latchpin, OUTPUT);
	pinMode(clockpin, OUTPUT);
	pinMode(datapin, OUTPUT);
	digitalWrite(latchpin, LOW);
	shiftOut(datapin, clockpin, MSBFIRST, 0);  
	shiftOut(datapin, clockpin, MSBFIRST, 0);  
	digitalWrite(latchpin, HIGH);
	pinMode(band0pin, INPUT);
	pinMode(band1pin, INPUT);
	pinMode(band2pin, INPUT);
	pinMode(band3pin, INPUT);
	pinMode(masterautopin, INPUT);
	pinMode(masternextpin, INPUT);
	pinMode(slaveautopin, INPUT);
	pinMode(slavenextpin, INPUT);
	// start a serial monitor to help debugging
#ifdef DEBUG
	Serial.begin(9600);
	Serial.println("Initialization complete");
#endif
}

void loop() {
//	// Let's read a character from the serial and pretend it's our band data.
//	if (Serial.available() > 0) {
//		int band = Serial.read();
//		if (isHexadecimalDigit(band)) {
	int band = readband();
#ifdef DEBUG
	Serial.print("Selecting antenna for band ");
#endif
//			// conversion from ascii hex digit to number, taking care of case
//			band -= '0'; 
//			if (band > 9 ) { 
//				band -= 7;
//				if (band > 16) {
//					band -= 32;
//				}
//			}
	if (band > 10) { // there are currently only 11 HF bands on the K3
#ifdef DEBUG
		Serial.print(band,BIN);
		Serial.println(" UNDEFINED BAND!");
#endif
		band = 0;
	}
				
#ifdef DEBUG
	Serial.print(band,BIN);
	Serial.print(" -> ");
	Serial.println(hfbandname[band]);
	Serial.print("Pref. ant: ");
	Serial.println(preferredant(band,2));
	Serial.print("One of Eight code: ");
	Serial.println(oneofeight(preferredant(band,2)));
	Serial.print("Resulting master antenna: ");
	Serial.println(antennaname[(preferredant(band,2))]);
	Serial.print("Alt. ant: ");
	Serial.println(preferredant(band,1));
	Serial.print("One of Eight code: ");
	Serial.println(oneofeight(preferredant(band,1)));
	Serial.print("Resulting slave antenna: ");
	Serial.println(antennaname[(preferredant(band,1))]);
#endif
	// Since the relay module expects a pin to be LOW to activate the relay,
	// we have to invert the number we write to the shift registers
	sendbits(oneofeight(preferredant(band,2)) ^ B11111111,
			 oneofeight(preferredant(band,1)) ^ B11111111);
			
//} else { 
//#ifdef DEBUG
//			Serial.println("Not a hex digit");
//#endif
//		}
//	}

#ifdef DEBUG
	delay(1000);
#endif

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

// Find the first occurrence of the specified preference in the 
// band to antenna mapping table
int preferredant(int band, int preference) {
	for (int i=0;i<8;i++) {
		if ( bandtoant[band][i] == preference) {
			return i+1;
		}
	}
	return 0;
}

// Read the band lines and merge them in a binary digit.
int readband() {
	int band0;
	int band1;
	int band2;
	int band3;
	band0 = digitalRead(band0pin);
	band1 = digitalRead(band1pin);
	band2 = digitalRead(band2pin);
	band3 = digitalRead(band3pin);
	
	return (band3 << 3) + (band2 << 2) + (band1 << 1) + band0;
}

