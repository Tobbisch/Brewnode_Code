/*
 * BrewnodeV1.h
 *
 *  Created on: 31 mars 2021
 *      Author: Tobias Halldin
 *      Structures common for Brewnode
 */

#ifndef BREWNODEV1_H_
#define BREWNODEV1_H_
#include "Arduino.h"
//----------------------------------------------------------------------
//Define data structures
//----------------------------------------------------------------------
//Datablock in PLC
struct Digital_Sensor {
	float Value;
	uint8_t Status;
	uint8_t Address[8];
	uint8_t Model;
};

struct Analog_Sensor {
	int16_t Value;
	uint8_t Status;
	uint8_t Model;
};

struct Digital_S {
	Digital_Sensor Port1[4];
	Digital_Sensor Port2[4];
	Digital_Sensor Port3[4];
	Digital_Sensor Port4[4];
};

struct Analog_S {
	Analog_Sensor Port5;
	Analog_Sensor Port6;
	Analog_Sensor Port7;
	Analog_Sensor Port8;
};

struct DS18B20_Address {
	uint8_t Address[8];
};

struct Port_Address {
	DS18B20_Address Port1[4];
	DS18B20_Address Port2[4];
	DS18B20_Address Port3[4];
	DS18B20_Address Port4[4];
};

struct Strct_Cfg {
	Port_Address Detected; 		//To PLC detected addresses
	Port_Address Configured;	//From PLC configured addresses
};


struct Datablock {
	uint8_t Heartbeat_To_PLC;
	uint8_t Heartbeat_From_PLC;
	Digital_S Digital_Sensors;
	Analog_S Analog_Sensors;
	uint8_t Digital_Inputs[4];
	uint8_t Relay[5];
	Strct_Cfg Config;

};
//End of datablock in PLC

//Config of brewnode used in code. Storage to/from this into flash.
struct Configuration {
	uint8_t ID = 0;
	IPAddress IP_PLC = {0,0,0,0};
	String SSID = "";
	String PSK = "";
	String Host = "Brewnode_";
	uint8_t BSSID[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
	uint16_t PLC_DB = 0;
	uint8_t PLC_Rack = 0;
	uint8_t PLC_Slot = 0;
	uint8_t Setup_Done = false; //Setup have been performed
};

//Used to show in web page
struct Stat{
	uint8_t IOEXP = 0x00;
	uint8_t ADC = 0x00;
	uint8_t Sensor_Port1_Nr = 0x00;
	uint8_t Sensor_Port1_Error = 0x00;
	uint8_t Sensor_Port2_Nr = 0x00;
	uint8_t Sensor_Port2_Error = 0x00;
	uint8_t Sensor_Port3_Nr = 0x00;
	uint8_t Sensor_Port3_Error = 0x00;
	uint8_t Sensor_Port4_Nr = 0x00;
	uint8_t Sensor_Port4_Error = 0x00;

};
//----------------------------------------------------------------------



#endif /* BREWNODEV1_H_ */
