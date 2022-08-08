#include "esphome.h"

//
//
// ESPhome custom UART componet for Kamstrup 382Lx7 energy meter
// Using the IR Eye from https://wiki.hal9k.dk/projects/kamstrup
// Code heavily inspired by https://github.com/Hal9k-dk/kamstrup/tree/master/Software%20eksempler/kamstrup_powermeter
// Arduino code on above link most likely based on the work by Poul-Henning Kamp (https://github.com/bsdphk/PyKamstrup)
//
//

// Log tag
static const char *TAG = "K382Lx7";

// Set value to 1 if you deliver energy back to the grid, e.g. via Solar Panel
#define ENERGYOUT 1

// Kamstrup optical IR serial
#define KAMTIMEOUT 500  // Kamstrup timeout after transmit

// Number of attempts/retries for the same register
#define REGISTER_RETRY_COUNT 1


// Which Kamstrup register to query (together with short description). 
// If you change registers/descriptions, you will probably have to change the
// sensor defintions from line 53 - and the update code from line 182
#if ENERGYOUT
const word kregnums[] = { 0x0001,0x0002,0x000d,0x000e,0x03ff,0x0438,0x0439,0x043a,0x0400,0x0540,0x0541,0x0542,0x0434,0x0435,0x0436,0x0027,0x041e,0x041f,0x0420 };
const char* kregstrings[]   = { "TotalEnergyIn","TotalEnergyOut","EnergyInHiRes","EnergyOutHiRes","CurrentPowerIn","PowerP1In","PowerP2In","PowerP3In","CurrentPowerOut","PowerP1Out","PowerP2Out","PowerP3Out","CurrentP1","CurrentP2","CurrentP3","MaxPower","VoltageP1","VoltageP2","VoltageP3" };
#define NUMREGS 19     			// Number of registers above
#else
const word kregnums[] = { 0x0001,0x000d,0x03ff,0x0438,0x0439,0x043a,0x0434,0x0435,0x0436,0x0027,0x041e,0x041f,0x0420 };
const char* kregstrings[]   = { "TotalEnergyIn","EnergyInHiRes","CurrentPower","PowerP1In","PowerP2In","PowerP3In","CurrentP1","CurrentP2","CurrentP3","MaxPower","VoltageP1","VoltageP2","VoltageP3" };
#define NUMREGS 13     			// Number of registers above
#endif

float fResultSet[NUMREGS];		// Array to hold the results of the queries of the registers above

// Units
const char*  units[65] = {"","Wh","kWh","MWh","GWh","j","kj","Mj",
  "Gj","Cal","kCal","Mcal","Gcal","varh","kvarh","Mvarh","Gvarh",
        "VAh","kVAh","MVAh","GVAh","kW","kW","MW","GW","kvar","kvar","Mvar",
        "Gvar","VA","kVA","MVA","GVA","V","A","kV","kA","C","K","l","m3",
        "l/h","m3/h","m3xC","ton","ton/h","h","hh:mm:ss","yy:mm:dd","yyyy:mm:dd",
        "mm:dd","","bar","RTC","ASCII","m3 x 10","ton xr 10","GJ x 10","minutes","Bitfield",
        "s","ms","days","RTC-Q","Datetime"};

unsigned long cntIrPause=0;  	// Counter for IR pause between readings

// Serial data receiption variables
unsigned long rxindex = 0;		// Index counter for received bytes
uint64_t starttime;		// Millisecond variable used for detecting IR receive timeouts

// Variables used for onlyu looping queries every 60 seconds 
uint64_t queryPeriod = 60000; // Interval in milliseconds
uint64_t time_now =0;


// Definition of functions
extern long crc_1021(byte const *inmsg, unsigned int len);
extern float kamReadReg(unsigned short kreg);
extern void kamSend(byte const *msg, int msgsize);
extern float kamDecode(unsigned short const kreg, byte const *msg);

class K382Lx7 : public PollingComponent, public UARTDevice
{

public:
  K382Lx7(UARTComponent *parent) : PollingComponent(60000), UARTDevice(parent){};

  byte recvmsg[40];  			// buffer of bytes to hold the received data
  byte rxdata[50];  			// buffer to hold received data
  bool bResultValid[NUMREGS];	// Array to tell if we have valid measurements (after reboot)

  unsigned short kRegCnt; 		// Counter to indicate where we are in the list of registers
  unsigned short iRegRetryCnt;  // Retry counter - per register query

  bool bReceiveIR = false; 		// Flag to indicate to main loop whether we should try to receive data
  bool bSendIR = false; 			// Flag to indicate to main loop whether it is allowed to send IR
  bool bQueryLoopActive = false; // Flag to indicate whether we are in the process of querying the meter for data
 
  // For each of the values we wish to export, we define a sensor
  Sensor *totEnergyIn_sensor = new Sensor();
#if ENERGYOUT
  Sensor *totEnergyOut_sensor = new Sensor();
#endif
  Sensor *EnergyInHiRes_sensor = new Sensor();
#if ENERGYOUT
  Sensor *EnergyOutHiRes_sensor = new Sensor();
#endif
  Sensor *CurrentPowerIn_sensor = new Sensor();
  Sensor *PowerP1In_sensor = new Sensor();
  Sensor *PowerP2In_sensor = new Sensor();
  Sensor *PowerP3In_sensor = new Sensor();
#if ENERGYOUT
  Sensor *CurrentPowerOut_sensor = new Sensor();
  Sensor *PowerP1Out_sensor = new Sensor();
  Sensor *PowerP2Out_sensor = new Sensor();
  Sensor *PowerP3Out_sensor = new Sensor();
#endif

  Sensor *CurrentP1_sensor = new Sensor();
  Sensor *CurrentP2_sensor = new Sensor();
  Sensor *CurrentP3_sensor = new Sensor();

  Sensor *MaxPower_sensor = new Sensor();

  Sensor *VoltageP1_sensor = new Sensor();
  Sensor *VoltageP2_sensor = new Sensor();
  Sensor *VoltageP3_sensor = new Sensor();

  void setup() override
  {
    // ESPHome takes care of setting up the UART for us
    ESP_LOGD(TAG, "Setting up...");
	for(unsigned int x=0;x<NUMREGS;x++) {
		bResultValid[x]=false;
	}

	// setup kamstrup serial
	pinMode(LED_BUILTIN, OUTPUT); 
	kRegCnt = 0;			// Reset Kamstrup Register Counter
	bReceiveIR = false;		// Disallow receiving IR
	bSendIR = false;		// Disallow sending IR
	bQueryLoopActive = false;	// Query loop not active
	time_now = (esp_timer_get_time()/1000);		// Set current timestamp
	iRegRetryCnt = 0;
  }

  void loop() override
  {

	// Check if we are allowed to send data to the Kamstrup meter to get data from a register
	if(bReceiveIR == false && bSendIR == true && bQueryLoopActive == true) {
		ESP_LOGD(TAG," - Querying register index %d, description: %s - regsiter hex value: 0x%04x",kRegCnt,kregstrings[kRegCnt],kregnums[kRegCnt]);
		iRegRetryCnt++;		// Increment retry counter
		kamReadReg(kRegCnt);
	}
	
	// If send is disabled, and we are allowed to receive
	if (bReceiveIR == true && bSendIR == false && bQueryLoopActive == true) { 
		if(((esp_timer_get_time()/1000)-starttime) > KAMTIMEOUT) {		// Check if to much time has passed for receiving a reply
			ESP_LOGD(TAG,"Timed out listening for data - setting IR receive false, and try next register...");
			bReceiveIR = false;						// Do not process more incoming IR
			rxindex = 0;							// reset index counter for receiption array
			kRegCnt++;								// Increment index counter for Kamstrup register queries
		} else {
			byte r = 0;
			
			// handle incoming data
			if (available()) {						// Check if serial data is available
				// receive byte
				r = read();							
				if(r != 0x40) {  					// don't append if we see the start marker
					// append data
					rxdata[rxindex] = r;
					rxindex++; 
				}
 
				if (r == 0x0d) 						// If EOL received
				{
					// remove escape markers from received data
					unsigned short j = 0;
					for (unsigned short i = 0; i < rxindex -1; i++) {
						if (rxdata[i] == 0x1b) {
							byte v = rxdata[i+1] ^ 0xff;
							if (v != 0x06 and v != 0x0d and v != 0x1b and v != 0x40 and v != 0x80){
							ESP_LOGD(TAG,"Missing escape: %02x ",v);
						  }
						  recvmsg[j] = v;
						  i++; // skip
						} else {
							recvmsg[j] = rxdata[i];
						}
						j++;
					}
					// Do CRC Check
					if (crc_1021(recvmsg,j)) {
						j = 0;
						ESP_LOGW(TAG,"CRC error - iRegRetryCnt = %d",iRegRetryCnt);
						if(iRegRetryCnt > REGISTER_RETRY_COUNT ) { // Check max number of retries per register
							ESP_LOGD(TAG,"Retry count exceeded, move to next register...");
							iRegRetryCnt = 0;
							kRegCnt++;			// Increment index counter for Kamstrup register queries
							bReceiveIR = false;
						} 
					}
					
					if(j != 0){
						float rval;        // this will hold the final value
						
						// decode the received message
						rval = kamDecode(kRegCnt,recvmsg);
						fResultSet[kRegCnt]=rval;
						bResultValid[kRegCnt]=true;
						ESP_LOGD(TAG,"Value read and logged successfully - %s - %f",kregstrings[kRegCnt],rval);
						kRegCnt++;			// Increment index counter for Kamstrup register queries
						iRegRetryCnt = 0;	// Reset retry-counter
					}

					bReceiveIR = false;		// Finished receiving - enable sending
					ESP_LOGD(TAG,"Set receive mode off");
					rxindex = 0;			// Reset receiption array index counter
				}

				if ( rxindex > 49 ) {		// Added for security - so we don't risk writing outside the allocated array
					bReceiveIR = false;		// Finished receiving - enable sending
					ESP_LOGD(TAG,"rxindex exceeded!!!");
					rxindex = 0;			// Reset receiption array index counter
					flush();  				// flush serial buffer - we do not need more data...
				}
			}
		}
	}
	
	// If neither sending or receiving is enabled - delay a bit before querying the meter again
	if ( bSendIR == false && bReceiveIR == false && bQueryLoopActive == true) {
		delay(5);						// delay 5ms
		if(++cntIrPause >= 100 ) {		// If we run through this loop 100 times, it is approx. 0.5 seconds
			cntIrPause = 0;				// reset counter
			bSendIR = true;				// Allow sending of IR again	
			// iRegRetryCnt = 0;			// Reset register retry counter
			ESP_LOGD(TAG,"Set send mode on");
		}
		if ( cntIrPause % 50 == 0 && cntIrPause != 0)	// Just for information - log that we are sleeping
		{
			ESP_LOGD(TAG,"sleeping...");
		}
	}

	if(kRegCnt == NUMREGS)  {			// If we have reached the lasr register...
		kRegCnt= 0;						// Set index counter to zero - to start reading from the first register again
		bQueryLoopActive = false; 		// Set looping inactive 
		ESP_LOGD(TAG,"All registers have been queried - wait until 60 seconds has passed since first query in previous loop...");
	}

	// Only query every 60 seconds
	if((uint64_t)((esp_timer_get_time()/1000) - time_now) > queryPeriod) {
		time_now = (esp_timer_get_time()/1000);		// set new timestamp
		if ( bReceiveIR == false) {
			ESP_LOGD(TAG,"Approx 60 seconds has passed since the last query loop was activated, and receive is not enabled");
			bQueryLoopActive = true;	// Enable query looping
			bSendIR = true;				// Make sure, than sending is enabled again...
		}
	}
	
  }

  void update() override
  {
    // This is the actual sensor reading logic.
    ESP_LOGD(TAG, "Update has been called...");
	unsigned short iCnt = 0;
    
	if(bResultValid[iCnt] == true && fResultSet[iCnt] != 0 ) {
		totEnergyIn_sensor->publish_state(fResultSet[iCnt]);
	} else {
		totEnergyIn_sensor->publish_state(NAN);
	}
	iCnt++;
#if ENERGYOUT
	if(bResultValid[iCnt] == true && fResultSet[iCnt] != 0 ) {
		totEnergyOut_sensor->publish_state(fResultSet[iCnt]);
	} else {
		totEnergyOut_sensor->publish_state(NAN);
	}
	iCnt++;
#endif
	if(bResultValid[iCnt] == true && fResultSet[iCnt] != 0 ) {
		EnergyInHiRes_sensor->publish_state(fResultSet[iCnt]);
	} else {
		EnergyInHiRes_sensor->publish_state(NAN);
	}
	iCnt++;
#if ENERGYOUT
	if(bResultValid[iCnt] == true && fResultSet[iCnt] != 0 ) {
		EnergyOutHiRes_sensor->publish_state(fResultSet[iCnt]);
	} else {
		EnergyOutHiRes_sensor->publish_state(NAN);
	}
	iCnt++;
#endif
	if(bResultValid[iCnt] == true) {
		CurrentPowerIn_sensor->publish_state(fResultSet[iCnt]);
	} else {
		CurrentPowerIn_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		PowerP1In_sensor->publish_state(fResultSet[iCnt]);
	} else {
		PowerP1In_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		PowerP2In_sensor->publish_state(fResultSet[iCnt]);
	} else {
		PowerP2In_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		PowerP3In_sensor->publish_state(fResultSet[iCnt]);
	} else {
		PowerP3In_sensor->publish_state(NAN);
	}
	iCnt++;
#if ENERGYOUT
	if(bResultValid[iCnt] == true) {
		CurrentPowerOut_sensor->publish_state(fResultSet[iCnt]);
	} else {
		CurrentPowerOut_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		PowerP1Out_sensor->publish_state(fResultSet[iCnt]);
	} else {
		PowerP1Out_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		PowerP2Out_sensor->publish_state(fResultSet[iCnt]);
	} else {
		PowerP2Out_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		PowerP3Out_sensor->publish_state(fResultSet[iCnt]);
	} else {
		PowerP3Out_sensor->publish_state(NAN);
	}
	iCnt++;
#endif	
	if(bResultValid[iCnt] == true) {
		CurrentP1_sensor->publish_state(fResultSet[iCnt]);
	} else {
		CurrentP1_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		CurrentP2_sensor->publish_state(fResultSet[iCnt]);
	} else {
		CurrentP2_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		CurrentP3_sensor->publish_state(fResultSet[iCnt]);
	} else {
		CurrentP3_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		MaxPower_sensor->publish_state(fResultSet[iCnt]);
	} else {
		MaxPower_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		VoltageP1_sensor->publish_state(fResultSet[iCnt]);
	} else {
		VoltageP1_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		VoltageP2_sensor->publish_state(fResultSet[iCnt]);
	} else {
		VoltageP2_sensor->publish_state(NAN);
	}
	iCnt++;
	if(bResultValid[iCnt] == true) {
		VoltageP3_sensor->publish_state(fResultSet[iCnt]);
	} else {
		VoltageP3_sensor->publish_state(NAN);
	}

	ESP_LOGD(TAG,"bReceiceIR: %d",bReceiveIR);
	ESP_LOGD(TAG,"bSendIR: %d",bSendIR);
	ESP_LOGD(TAG,"bQueryLoopActive: %d",bQueryLoopActive);
	

  }

private:
	// We store the packet privately and pull values from it when update() is called

	// kamReadReg - read a Kamstrup register
	float kamReadReg(unsigned short kreg)
	{
	  ESP_LOGD(TAG,"Sending request via IR to meter");

	  // prepare message to send and send it
	  byte sendmsg[] = { 0x3f, 0x10, 0x01, (byte)(kregnums[kreg] >> 8), (byte)(kregnums[kreg] & 0xff) };
	  kamSend(sendmsg, 5);

	  bReceiveIR = true; 
	  ESP_LOGD(TAG,"Enable IR receive and disable IR send");
	  bSendIR = false; 
	  flush();  // flush serial buffer - might contain noise
	  starttime = (esp_timer_get_time()/1000);  // Set millis in order to detect timeout
	  return 0;
	}

	// kamSend - send data to Kamstrup meter
	void kamSend(byte const *msg, int msgsize) 
	{

	  // append checksum bytes to message
	  byte newmsg[msgsize+2];
	  for (int i = 0; i < msgsize; i++) { newmsg[i] = msg[i]; }
	  newmsg[msgsize++] = 0x00;
	  newmsg[msgsize++] = 0x00;
	  int c = crc_1021(newmsg, msgsize);
	  newmsg[msgsize-2] = (c >> 8);
	  newmsg[msgsize-1] = c & 0xff;

	  // build final transmit message - escape various bytes
	  byte txmsg[20] = { 0x80 };   // prefix
	  int txsize = 1;
	  for (int i = 0; i < msgsize; i++) {
		if (newmsg[i] == 0x06 or newmsg[i] == 0x0d or newmsg[i] == 0x1b or newmsg[i] == 0x40 or newmsg[i] == 0x80) {
		  txmsg[txsize++] = 0x1b;
		  txmsg[txsize++] = newmsg[i] ^ 0xff;
		} else {
		  txmsg[txsize++] = newmsg[i];
		}
	  }
	  txmsg[txsize++] = 0x0d;  // EOF

	  // send to serial interface
	  for (int x = 0; x < txsize; x++) {
		write(txmsg[x]);
	  }

	}



	// kamDecode - decodes received data
	float kamDecode(unsigned short const kreg, byte const *msg) 
	{

	  // skip if message is not valid
	  if (msg[0] != 0x3f or msg[1] != 0x10) {
		return false;
	  }
	  if (msg[2] != (kregnums[kreg] >> 8) or msg[3] != (kregnums[kreg] & 0xff)) {
		return false;
	  }

	  ESP_LOGD(TAG,"Decode: Unit of measured value: %s",units[msg[4]]);
		
	  // decode the mantissa
	  long x = 0;
	  for (int i = 0; i < msg[5]; i++) {
		x <<= 8;
		x |= msg[i + 7];
	  }
	  
	  // decode the exponent
	  int i = msg[6] & 0x3f;
	  if (msg[6] & 0x40) {
		i = -i;
	  };
	  float ifl = pow(10,i);
	  if (msg[6] & 0x80) {
		ifl = -ifl;
	  }

	  // return final value
	  return (float )(x * ifl);

	}

	// crc_1021 - calculate crc16
	long crc_1021(byte const *inmsg, unsigned int len)
	{
	  long creg = 0x0000;
	  for(unsigned int i = 0; i < len; i++) {
		int mask = 0x80;
		while(mask > 0) {
		  creg <<= 1;
		  if (inmsg[i] & mask){
			creg |= 1;
		  }
		  mask>>=1;
		  if (creg & 0x10000) {
			creg &= 0xffff;
			creg ^= 0x1021;
		  }
		}
	  }
	  return creg;
	}
};
