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

   4 pushbuttons to control operations: 2 Auto/Manual selection buttons 
   (one for master, one for slave), 2 "next" buttons to cycle through 
   available antennas. Pressing next while in auto mode selects next 
   antenna among those suitable for that band. Pressing next while in manual
   just cycles through all antenna positions. 

   There should be only one antenna with priority 2 for each band, all the 
   other usable antennas are priority 1. This should ensure that the preferred
   antenna is always connected to the same antenna port on the K3, thus 
   making good use of K3 remembering ATU settings and the like. 

   Conflict resolution

   Master has precedence. If you select the same antenna for both master
   and slave, the remote switch will actually not select the slave and you'll
   transmit into an open. 
   Upon master antenna change (manual or auto), check if chosen antenna is 
   the same as the currently selected slave, and if true advance slave to 
   next available (if set for auto) or next antenna (if set to manual). If no 
   suitable slave selection is possible, do not select any slave antenna.
   Upon slave antenna change, check if chosen antenna is same as master. If 
   true, advance slave to next available choice (auto) or next antenna 
   (manual), or leave unselected if no choice available. 

   Circuit design notes

   Jack Brindle W6FB recommends protecting the circuit by using pullups
   on the Arduino digital inputs (either internal or external) and diodes
   with Cathode facing the K3. This will prevent powering the Arduino board
   from the K3 +5V. The K3 will act as drain, pulling down the voltage on
   the Arduino pins. Recommends low voltage drop diodes (schottky, 
   e.g. 1N5711 - the KPA500 uses BAT54 which are SMD only, a suitable 
   equivalent is BAT42 which is also available in though-hole package). 

   Required libraries
   
   The interrupt handling is done via EnableInterrupts. See 
   https://github.com/GreyGnome/EnableInterrupt

   Thank you

   The following people contributed guidance, suggestions, ideas, critique:
   Lorenzo IZ1YSL, Jack W6FB, Bill AE6JV, Dick K6KR, Brian K3KO, Larry K8UT,
   Josh W6XU


*/

/************** USER CONFIGURABLE PART ***********/  

// Uncomment to start a serial monitor and see some chatter on it.
#define DEBUG

// Comment if the relay board doesn't need the inverted logic
#define RelayUseInvertedLogic

// Give a name to each antenna port (keep it short!)
// should I decide to put an lcd on this thing.
// Also used for debug
// Note that this array has 9 entries, actual antennas are 1 to 8
char* antennaname[] = {
	"***NONE***",// ant 0 doesn't exist, switch counts from 1
	"Yagi 6m",   // ant 1
	"Vert 6m",   // ant 2
	"dummy3",          // ant 3
	"dummy4",          // ant 4
	"dummy5",          // ant 5
	"dummy6",          // ant 6
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

// this corresponds to how many antenna positions the switch has
#define ANTCOUNT 8
// this corresponds to the number of different bands supported by the radio
#define BANDCOUNT 11

// for each possible band, 2 means it is the preferred antenna, 
// 1 means the antenna can be used, 0 means antenna not usable.
// NOTE: 60 m is band 0. 
uint8_t bandtoant[BANDCOUNT][ANTCOUNT] = {
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


/************** END USER CONFIGURABLE PART *******/

#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>

#ifdef DEBUG
// consider defining serial.print macros here
#endif

#ifdef RelayUseInvertedLogic
#define INVERTLOGIC B11111111
#else
#define INVERTLOGIC B00000000
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

// The 4 pushbuttons. Using contiguous analog inputs leads to interference
const int masterautopin = A0;
const int masternextpin = A2;
const int slaveautopin = A4;
const int slavenextpin = 6;

volatile bool masterauto = true;
volatile bool masternext = false;
volatile bool slaveauto = true;
volatile bool slavenext = false;
// antenna indexes count from 0 to 8 (actual antennas are 1 to 8)
volatile int masterant = 0;
volatile int slaveant = 0;

const int masterautoled = 7;
const int slaveautoled = 8;

unsigned long lastinterrupttime = 0;

int band = 0;
int oldband = 0;

void setup() {
	// configure pins, zero all the registers so all relays are open.
	pinMode(latchpin, OUTPUT);
	pinMode(clockpin, OUTPUT);
	pinMode(datapin, OUTPUT);
	digitalWrite(latchpin, LOW);
	shiftOut(datapin, clockpin, MSBFIRST, 0 ^ INVERTLOGIC);  
	shiftOut(datapin, clockpin, MSBFIRST, 0 ^ INVERTLOGIC);  
	digitalWrite(latchpin, HIGH);
	pinMode(band0pin, INPUT);
	pinMode(band1pin, INPUT);
	pinMode(band2pin, INPUT);
	pinMode(band3pin, INPUT);
	pinMode(masterautopin, INPUT);
	pinMode(masternextpin, INPUT);
	pinMode(slaveautopin, INPUT);
	pinMode(slavenextpin, INPUT);
	pinMode(masterautoled, OUTPUT);
	pinMode(slaveautoled, OUTPUT);
	// enable interrupts on the button pins
	enableInterrupt(masterautopin,buttoninterrupt,RISING);
	enableInterrupt(masternextpin,buttoninterrupt,RISING);
	enableInterrupt(slaveautopin,buttoninterrupt,RISING);
	enableInterrupt(slavenextpin,buttoninterrupt,RISING);
	// start a serial monitor to help debugging
#ifdef DEBUG
	Serial.begin(9600);
	Serial.println("Initialization complete");
#endif
	noTone(9); // clear the piezo pin
}

void loop() {
	band = readband();



#ifdef DEBUG
	Serial.println("-----------------");
	Serial.print("Selecting antenna for band ");
#endif

	if (band > 10) { // there are currently only 11 HF bands on the K3
#ifdef DEBUG
		Serial.print(band,BIN);
		Serial.print(" UNDEFINED BAND! resetting to ");
#endif
		band = 0;
	}
	
#ifdef DEBUG
	Serial.print(band,BIN);
	Serial.print(" -> ");
	Serial.print(hfbandname[band]);
	Serial.print("\t old band was: ");
	Serial.print(oldband,BIN);
	Serial.print(" -> ");
	Serial.println(hfbandname[oldband]);
	Serial.print("Preferred antenna: ");
	Serial.print(preferredant(band,2,0)); // search from 0 to find the first
	Serial.print("\tOne of Eight code: ");
	Serial.print(oneofeight(preferredant(band,2,0)));
	Serial.print("\tResulting master antenna if auto mode: ");
	Serial.println(antennaname[(preferredant(band,2,0))]);
	Serial.print("Alternate antenna: ");
	Serial.print(preferredant(band,1,0));
	Serial.print("\tOne of Eight code: ");
	Serial.print(oneofeight(preferredant(band,1,0)));
	Serial.print("\tResulting slave antenna if auto mode: ");
	Serial.println(antennaname[(preferredant(band,1,0))]);
#endif

	// check if the radio has changed band
	if ( band != oldband ) {
		// if auto mode, select the first available ant
		if (masterauto) {
			// we start the search from the currently selected antenna
			masterant = preferredant(band,2,masterant); 
		} else {
			// do not change antenna if we are not in auto mode
		}
		
		// repeat for slave ant
		// if auto mode, select the first available ant
		if (slaveauto) {
			// we start the search from the currently selected antenna
			slaveant = preferredant(band,1,slaveant);
		} else {
			// do not change antenna if we are not in auto mode
		}
	
		oldband = band;


	}

	checkconflict();
	
	// Since the relay module expects a pin to be LOW to activate the relay,
	// we have to invert the number we write to the shift registers
	sendbits(oneofeight(masterant) ^ INVERTLOGIC,
			 oneofeight(slaveant) ^ INVERTLOGIC);
			
//} else { 
//#ifdef DEBUG
//			Serial.println("Not a hex digit");
//#endif
//		}
//	}

	digitalWrite(masterautoled,masterauto);
	digitalWrite(slaveautoled,slaveauto);

#ifdef DEBUG
	Serial.print("*** Master Auto: ");
	Serial.print(masterauto);
	Serial.print(" Master Ant: ");
	Serial.print(masterant);
	Serial.print(" Slave Auto: ");
	Serial.print(slaveauto);
	Serial.print(" Slave Ant: ");
	Serial.println(slaveant);
	Serial.print("Actually selected master antenna: ");
	Serial.print(masterant);
	Serial.print("\tOne of Eight code: ");
	Serial.print(oneofeight(masterant));
	Serial.print("\tResulting master antenna: ");
	Serial.println(antennaname[masterant]);
	Serial.print("Actually selected slave antenna: ");
	Serial.print(slaveant);
	Serial.print("\tOne of Eight code: ");
	Serial.print(oneofeight(slaveant));
	Serial.print("\tResulting slave antenna: ");
	Serial.println(antennaname[slaveant]);
	delay(2000);
#endif

} /******* End of main loop *******/

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
// band to antenna mapping table, searching from the current one onwards
// the bandtoant array counts antennas from 0 to 7, need to apply appropriate
// conversions here 
int preferredant(int band, int preference,int currentant) {
	if (currentant) currentant--;
	for (int i=0;i<ANTCOUNT;i++) {
		if ( bandtoant[band][(i+currentant) % ANTCOUNT] == preference) {
			return (i+currentant) % ANTCOUNT + 1;
		}
	}
	return 0;
}

// Find the next available antenna in the 
// band to antenna mapping table, searching from the current one onwards
// the bandtoant array counts antennas from 0 to 7, need to apply appropriate
// conversions here 
int nextant(int band, int currentant) {
	for (int i=0;i<ANTCOUNT;i++) {
		if ( bandtoant[band][(i+currentant) % ANTCOUNT] ) {
			return (i+currentant) % ANTCOUNT + 1;
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

// Interrupt function
void buttoninterrupt () {
	unsigned long interrupttime = millis();

	if ( interrupttime - lastinterrupttime > 200 ) {
		switch (arduinoInterruptedPin) {
		case masterautopin:
			masterauto = !masterauto;
			// if we switch back to auto, trigger the auto process again
			if (masterauto) oldband = oldband++ % BANDCOUNT; 
			break;
		case masternextpin:
			if (masterauto) {
				// in auto mode, cycle only through antennas valid for this band
				masterant = nextant(band,masterant);
				// beep();
			} else {
				// in manual mode, cycle through all antennas
				masterant++;
				masterant %= (ANTCOUNT+1); // antennas are 0 + 1 to 8
			}
			break;
		case slaveautopin:
			slaveauto = !slaveauto;
			// if we switch back to auto, trigger the auto process again
			if (slaveauto) oldband = oldband++ % BANDCOUNT; 
			break;
		case slavenextpin:
			if (slaveauto) {
				// in auto mode, cycle only through antennas valid for this band
				slaveant = nextant(band,slaveant);
			} else {
				// in manual mode, cycle through all antennas
				slaveant++;
				slaveant %= (ANTCOUNT+1); // antennas are 0 + 1 to 8
			}
			break;
		}
	}
	lastinterrupttime = interrupttime;
}

void beep() {
	tone(9,880,100);
}

void checkconflict() {
#ifdef DEBUG
	Serial.print("Checking for conflict.");
	Serial.print("\tMaster Ant: ");
	Serial.print(masterant);
	Serial.print("\tSlave Ant: ");
	Serial.print(slaveant);
#endif	

	if ( masterant == slaveant ) {
#ifdef DEBUG
		Serial.println(" ***CONFLICT!***");
#endif
		if (slaveauto) {
		} else {
			// in manual mode, cycle through all antennas
			slaveant++;
			slaveant %= (ANTCOUNT+1); // antennas are 0 + 1 to 8
			beep();
		}
	} else {
		// nothing to do
#ifdef DEBUG
		Serial.println(" No Conflict");
#endif
	}

}
