// =================================================================================================
// eModbus: Copyright 2020 by Michael Harwerth, Bert Melis and the contributors to ModbusClient
//               MIT license - see license.md for details
// =================================================================================================
/* Example code for exchange data between DTSU666 CHINT and DTSU666H Huawei Power Meter
using eMOdbus library from https://github.com/eModbus
when you activate power meter in smartfon aplication the SUN2000 start sending request "0B 03 07 D1 00 01 D5 ED"  where
0B - Power Meter Modbus address (default 11)
03 - Function (READ_HOLD_REGISTER)
07 D1 - register address to read (dec 2001 )
00 01 - how many registers read (1 register mean 2 bytes)
D5 ED - CRC checksum

If register address 0x07D1 = 0x3F80 the sun2000 recognize it as  PowerMeter is active
slave answer should look like this  "0B 03 02 3F 80 30 15" 
after this SUN200 send request  "0B 03 08 36 00 50 A7 32"
and we should send value 80  registers begnining from address 0x0836  (dec 2002)
from time to time SUN2000 send request 0B 03 08 A6 00 0A 27 24 but i do not know what exactly is stored in this 10 registers begining from 0x08A6. I asked support Huawai but they have not answered.
Information about HUAWEI registers addresses
https://forum.huawei.com/enterprise/en/how-to-connect-custom-meter-dtsu666-h-to-smartlogger1000/thread/599712-100027
Information about CHINT registers addresses
https://www.aggsoft.com/serial-data-logger/tutorials/modbus-data-logging/chint-instrument-dsu666-dtsu666.htm
I divided request to CHINT into two part because for one request the number of  registers in answer is to large
*/ 


#include "HardwareSerial.h"  // from the <Arduino.h>  librrary  for Serial, millis()  etc.
#include "Logging.h"
#include "ModbusClientRTU.h"	// header for the Modbus Client RTU style from eModbus Library
#include "ModbusServerRTU.h"
#define RX1_PIN GPIO_NUM_18
#define TX1_PIN GPIO_NUM_19
#define REDE1_PIN GPIO_NUM_23
#define RX2_PIN GPIO_NUM_16
#define TX2_PIN GPIO_NUM_17
#define REDE2_PIN GPIO_NUM_15
#define HUAWEI_ID 0x0B // must be this same address as in setup Huawey inverter
#define CHINT_ID 0x01	// must be this same address as in setup CHINT Power Meter
#define BAUDRATE 9600
#define HUAWEI_START_REG_1 0x0836 //SUN2000 ask for 80 registers beginning from this address	
#define HUAWEI_START_REG_2 0x08A6 //SUN2000 ask for 10 registers but i don't know for what. Huawei did nor answer my question.
#define CHINT_START_REG_1 0x2000  // Low registers
#define CHINT_START_REG_2 0x401E	// High registers
#define CHINT_REQUEST1 82			// number Low registers
#define CHINT_REQUEST2 60			// number High registers
#define READ_INTERVAL 2000  // setup how often read the CHINT Power Meter
bool data_ready = false;
int errors = 1;  // count number errors 
uint32_t request_time;
float Chint_RegData[CHINT_REQUEST1/2+CHINT_REQUEST2/2];  // store Chint registers

int HuaweiTranslate[CHINT_REQUEST1/2+CHINT_REQUEST2/2]=
{6, 7, 8,   	// Phase A,B,C current   0 1 2
24,				// 3  ???
 3, 4, 5,		//Phase A,B,C Voltage   4 5 6
25,				//  7   ???
 0, 1, 2,		// A-B, B-C, C-A line voltage   8 9 10
34, 			// Frequency   11
 9,				// Active Power  12
10,11,12,		//Phase A,B,C Active Power   13 14 15
13,				// Reactive Power   16  
14,	15,16, 		//Reactive Power qa, qb,qc , A, B, C 17, 18 ,19,     
17,18,19,20, 	// Apparent Power total a b c  20  21  22 23
21,22,23,24,	// Power Factor pft, pfa, pfb, pfc  24, 25,26,27
36, 			// Total Active Electricity    28
37,38,39,		// Active Electricity  a b c  29 30 31 
46,				// Total Positive Active Electricity   32
33,34,35,		// 33 34 35
41,				// Total negative Active electricity  36
37,38,39, 		// 37 38 39  
40,	41,			// 40 41 END Lower Chints registers
42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70
}; 

int Divider[CHINT_REQUEST1/2+CHINT_REQUEST2/2]=
{10, 10, 10,   		// Phase A-B,B-C,C-A Voltage  0 1 2
10, 10, 10, 		//Phase A,B,C Voltage		3 4 5
1000,1000, 1000,	// Phase A,B,C current		6 7 8
10, 				// Active Power total			9
10,10,10,			// Phase A,B,C Active Power   10,11,12
10,					// Reactive Power  total  13
10,10,10,			//Phase A,B,C Reactive Power   14 15 16
10,10,10,10,		// Apparent Power total, phase A,B,C  17, 18, 19, 20
10,10,10,10,		// Power Factor total , phase A, B, C 21,22, 23, 24
1,1,1,				// 25, 26, 27   ???
1, 					// 28  			???
1,1,1,    			// 29 30 31		???
1,					// 32			???
1,					// 33			
100,				// 34  Frequency
1,      			// 35			???
1000,				// 36  Total active electricity
1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
}; 


// The RS485 module has no halfduplex,so the second parameter with the DE/RE pin is required! https://emodbus.github.io/modbusserver-rtu-api
//	Create a ModbusRTU Server Slave instance listening on Serial2 with 20000ms timeout RE/DE control pin GPIO_NUM_15  
ModbusServerRTU MBserver(Serial2, 2000, REDE2_PIN);  
//	Create a Modbus RTE Master Client RE/DE control pin GPIO_NUM_21
ModbusClientRTU MbChint(Serial1, REDE1_PIN); 	

// FC03: worker to create answer  Modbus function code 0x03 (READ_HOLD_REGISTER) for Huawei Master Client
ModbusMessage FC03(ModbusMessage request) {
	uint16_t words; 
	uint16_t address;
	float* HuReg;
	HuReg=&Chint_RegData[0];
	ModbusMessage response;  	// declare response message to be sent back to Huawei
	request.get(2, address);
	request.get(4, words);
	response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words*2));	//Set up message with serverID, FC and length of data (1 register is 2 bytes)
	
	// on request for 0x2001 register  "0B 03 07 D1 00 01 D5 ED"  answer 0x3F80 means that Power Meter is active 
	if ((address == 0x07D1)&& (words==0x01))  response.add(uint16_t(0x3F80)); 
	else if ((address==HUAWEI_START_REG_1)&& (words=0x50)) {       // read data part 1 request "0B 03 08 36 00 50 A7 32"
		for (uint16_t i = 0; i < words/2; i++) {
			response.add(*(HuReg+HuaweiTranslate[i]));   // translate CHint on to Huawei addresses
		}
		for (int j = 0; j < words/2; j++)Serial.printf("HUAWEI j=%i            wart=%8.4f: \n", j, *(HuReg+HuaweiTranslate[j]));
		
		}
	else if ((address == HUAWEI_START_REG_2)&& (words==0x0A)) {   //  read data part 2 request "0B 03 08 A6 00 0A 27 24" 
		for (uint16_t i = 0; i <  words/2; i++) {
			response.add(0x0000);  // here is possible put answer for 10 unknown registers, now answer is set to zero for safety
		}}
	else {  // No, either address or words are outside the limits. Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
	Serial.printf("ERROR bad address= %X \n", address);
	}
	return response;
}

// Define an onData handler function to receive the regular responses
// Arguments are received response message and the request's token
// The device has values all as IEEE754 float32 in two consecutive registers
// Read the CHint response in a loop
void handleData1(ModbusMessage response, uint32_t token) {
  uint8_t words;      			// response CHINT number of registers
  response.get(2, words);
  uint16_t offs = 3;   // First data are on pos 3, after server ID, function code and length byte
	if (words==2*CHINT_REQUEST1) {  // is this answer for part1?
	  for (uint8_t i = 0; i < words/2; i++) {
		offs = response.get(offs, Chint_RegData[i]);
		Chint_RegData[i]=Chint_RegData[i]/Divider[i];
	}}
	else if (words==2*CHINT_REQUEST2) {    // is this answer for part2?
		 for (uint8_t i = 0; i < words/2; i++) {
		offs = response.get(offs, Chint_RegData[i+CHINT_REQUEST1/2]);  
		Chint_RegData[i+CHINT_REQUEST1/2]=Chint_RegData[i+CHINT_REQUEST1/2]/Divider[i+CHINT_REQUEST1/2];
	}}
	else{
	Serial.printf("error read from CHINT");
	}
  request_time = token;
  data_ready = true;
}

// Define an onError handler function to receive error responses
// Arguments are the error code returned and a user-supplied token to identify the causing request
void handleError(Error error, uint32_t token) {
  // ModbusError wraps the error code and provides a readable error message for it
  ModbusError me(error);
  Serial.printf("\nError me number  %u  token %u \n", errors++, token);
  LOG_E("Error response: %02X - %s\n", (int)me, (const char *)me);
}

// Setup() - initialization happens here
void setup() {
	Serial1.setRxBufferSize(512);  // default the buffer size is 256
	Serial.begin(115200);  // Init Serial port monitor for development
	while (!Serial) {}
	Serial.print("Serial0 OK __  ");
	
	Serial1.begin(9600, SERIAL_8N2, RX1_PIN, TX1_PIN); //Serial1 connected to Modbus RTU CHINT  
	while (!Serial1) {}
	Serial.print("Serial1 OK __  ");

    MbChint.onDataHandler(&handleData1); 	// Set up ModbusRTU client - provide onData handler function	
    MbChint.onErrorHandler(&handleError);	// - provide onError handler function
    MbChint.setTimeout(2000);			// Set message timeout to 2000ms
    MbChint.begin(0);				// Start ModbusRTU background task on CPU 0

	Serial2.begin(9600, SERIAL_8N2, RX2_PIN, TX2_PIN);  //Serial2 connected to Modbus RTU SUN2000
	while (!Serial2) {}
	Serial.println("Serial2 OK __");
	MBserver.registerWorker(HUAWEI_ID, READ_HOLD_REGISTER, &FC03 ); // Register served function code worker for server 11, FC 0x03
	MBserver.start(1); // Start ModbusRTU background task on CPU 1
}

// loop() - cyclically request the data from CHINT
void loop() {
  static uint32_t next_request = millis();
  if (millis() - next_request > READ_INTERVAL) {
    data_ready = false;
    Error err = MbChint.addRequest(millis(),CHINT_ID, READ_HOLD_REGISTER, CHINT_START_REG_1, CHINT_REQUEST1);
    if (err==SUCCESS) {
		delay(10);    				// delay to get different token ID for request for part2
		err = MbChint.addRequest(millis(),CHINT_ID, READ_HOLD_REGISTER, CHINT_START_REG_2, CHINT_REQUEST2);
	}
    if (err!=SUCCESS) {
		ModbusError e(err);
		LOG_E("Error creating request: %02X - %s\n", (int)e, (const char *)e);
    }
    next_request = millis();    // Save current time to check for next cycle
	} else {
		if (data_ready) {
		data_ready = false;		
		Serial.printf("Loop end time %ld  (ms)\n", millis());
		// if you want to print on monitor port the data received from CHINT
		for (uint8_t j = 0; j < (CHINT_REQUEST1+CHINT_REQUEST2)/2; ++j)Serial.printf("j=%i      LoadresCHINT=%i;      wart=%8.4f: \n", j,(2*j+ CHINT_START_REG_1), Chint_RegData[j]);
		}
	}
}
