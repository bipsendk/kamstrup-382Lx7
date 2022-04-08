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

// Kamstrup optical IR serial
#define KAMTIMEOUT 500  // Kamstrup timeout after transmit


// Which Kamstrup register to query (together with short description). 
// If you change registers/descriptions, you will probably have to change the
// sensor defintions from line 53 - and the update code from line 182
word const kregnums[] = { 0x0001,0x000d,0x03ff,0x0438,0x0439,0x043a,0x0434,0x0435,0x0436,0x0027,0x041e,0x041f,0x0420 };
const char* kregstrings[]   = { "EnergyIn","EnergyInHiRes","CurrentPower","PowerP1In","PowerP2In","PowerP3In","CurrentP1","CurrentP2","CurrentP3","MaxPower","VoltageP1","VoltageP2","VoltageP3" };
#define NUMREGS 13     			// Number of registers above
float fResultSet[NUMREGS];		// Array to hold the results of the queries of the registers above

bool bReceiveIR = false; 		// Flag to indicate to main loop whether we should try to receive data
bool bSendIR = false; 			// Flag to indicate to main loop whether it is allowed to send IR
unsigned long cntIrPause=0;  	// Counter for IR pause between readings

// Serial data receiption variables
unsigned long rxindex = 0;		// Index counter for received bytes
unsigned long starttime;		// Millisecond variable used for detecting IR receive timeouts


// Definition of functions
extern long crc_1021(byte const *inmsg, unsigned int len);
extern float kamReadReg(unsigned short kreg);
extern void kamSend(byte const *msg, int msgsize);
extern float kamDecode(unsigned short const kreg, byte const *msg);

class K382Lx7 : public PollingComponent, public UARTDevice
{

public:
  K382Lx7(UARTComponent *parent) : PollingComponent(60000), UARTDevice(parent){};

  byte recvmsg[40];  		// buffer of bytes to hold the received data
  byte rxdata[50];  		// buffer to hold received data

  unsigned short kRegCnt; 	// Counter to indicate where we are in the list of registers

  // For each of the values we wish to export, we define a sensor
  Sensor *totEnergyIn_sensor = new Sensor();
  Sensor *EnergyInHiRes_sensor = new Sensor();
  Sensor *CurrentPower_sensor = new Sensor();

  Sensor *PowerP1In_sensor = new Sensor();
  Sensor *PowerP2In_sensor = new Sensor();
  Sensor *PowerP3In_sensor = new Sensor();

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

	// setup kamstrup serial
	pinMode(LED_BUILTIN, OUTPUT); 
	kRegCnt = 0;			// Reset Kamstrup Register Counter
	bReceiveIR = false;		// Disallow receiving IR
	bSendIR = false;		// Disallow sending IR
  }

  void loop() override
  {

	// Check if we are allowed to send data to the Kamstrup meter to get data from a register
	if(bReceiveIR == false && bSendIR == true) {
		ESP_LOGD(TAG," - Querying register index %d - %s - %04x",kRegCnt,kregstrings[kRegCnt],kregnums[kRegCnt]);
		kamReadReg(kRegCnt);
	}
	
	// If send is disabled, and we are allowed to receive
	if (bReceiveIR == true && bSendIR == false) { 
		if(millis()-starttime > KAMTIMEOUT) {		// Check if to much time has passed for receiving a reply
			ESP_LOGD(TAG,"Timed out listening for data - setting IR receive false...");
			bReceiveIR = false;						// Do not process more incoming IR
			rxindex = 0;							// reset index counter for receiption array
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
						ESP_LOGD(TAG,"CRC error");
						j = 0;
					}
					
					if(j != 0){
						float rval;        // this will hold the final value
						
						// decode the received message
						rval = kamDecode(kRegCnt,recvmsg);
						fResultSet[kRegCnt]=rval;
						ESP_LOGD(TAG,"Value logged successfully - %s - %f",kregstrings[kRegCnt],rval);
						kRegCnt++;			// Increment index counter for Kamstrup register queries
					}

					bReceiveIR = false;		// Finished receiving - enable sending
					ESP_LOGD(TAG,"Set receive mode off");
					rxindex = 0;			// Reset receiption array index counter
				}

				if ( rxindex > 49 ) {		// Added for security - so we don't risk writing outside the allocated array
					bReceiveIR = false;		// Finished receiving - enable sending
					ESP_LOGD(TAG,"rxindex exceeded!!!");
					rxindex = 0;			// Reset receiption array index counter
				}
			}
		}
	}
	
	// If neither sending or receiving is enabled - delay a bit before querying the meter again
	if ( bSendIR == false && bReceiveIR == false ) {
		delay(5);						// delay 5ms
		if(cntIrPause++ > 200 ) {		// If we run through this loop 200 times, it is approx. 1 second
			cntIrPause = 0;				// reset counter
			bSendIR = true;				// Allow sending of IR again	
			ESP_LOGD(TAG,"Set send mode on");
		}
		if ( cntIrPause % 100 == 0 )	// Just for information - log that we are sleeping
		{
			ESP_LOGD(TAG,"sleeping...");
		}
	}
	
	if(kRegCnt == NUMREGS)  {			// If we have reached the lasr register...
		kRegCnt= 0;						// Set index counter to zero - to start reading from the first register again
	}
  }

  void update() override
  {
    // This is the actual sensor reading logic.
    ESP_LOGD(TAG, "Update has been called...");
    
	totEnergyIn_sensor->publish_state(fResultSet[0]);
    EnergyInHiRes_sensor->publish_state(fResultSet[1]);
    CurrentPower_sensor->publish_state(fResultSet[2]);
	
	PowerP1In_sensor->publish_state(fResultSet[3]);
	PowerP2In_sensor->publish_state(fResultSet[4]);
	PowerP3In_sensor->publish_state(fResultSet[5]);
	
	CurrentP1_sensor->publish_state(fResultSet[6]);
	CurrentP2_sensor->publish_state(fResultSet[7]);
	CurrentP3_sensor->publish_state(fResultSet[8]);

	MaxPower_sensor->publish_state(fResultSet[9]);

	VoltageP1_sensor->publish_state(fResultSet[10]);
	VoltageP2_sensor->publish_state(fResultSet[11]);
	VoltageP3_sensor->publish_state(fResultSet[12]);
  }

private:
	// We store the packet privately and pull values from it when update() is called

	// kamReadReg - read a Kamstrup register
	float kamReadReg(unsigned short kreg)
	{
	  ESP_LOGD(TAG,"Sending request via IR");

	  // prepare message to send and send it
	  byte sendmsg[] = { 0x3f, 0x10, 0x01, (byte)(kregnums[kreg] >> 8), (byte)(kregnums[kreg] & 0xff) };
	  kamSend(sendmsg, 5);

	  bReceiveIR = true; 
	  ESP_LOGD(TAG,"Enable IR receive and disable IR send");
	  bSendIR = false; 
	  flush();  // flush serial buffer - might contain noise
	  starttime = millis();  // Set millis in order to detect timeout
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