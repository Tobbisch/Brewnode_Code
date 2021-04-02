/*
 * BrewnodeV1.ino
 *
 *  Created on: 16 aug. 2020
 *  Version: 1.2
 *  Modified: 2 April 2021
 *      Author: Tobias Halldin
 */
//----------------------------------------------------------------------
//Includes
//----------------------------------------------------------------------

#include <Platform.h>
#include <Settimino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PCF8574.h>
#include <ADS1115.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "BrewnodeV1.h"
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//Prototype functions
//----------------------------------------------------------------------
bool Connect();
void CheckError(int);
void MarkTime();
void ShowTime();
void storeAddress(DeviceAddress,uint8_t[]);
void UpdateDigitalSensors();
void UpdateOneDigitalSensor(float,float*,uint8_t);
void UpdateConfigured();
void UpdateAddressArray(uint8_t[], uint8_t[]);
void CheckIO();
void CheckAnalog();
int16_t Const(int16_t);
void Read_DB();
void Write_DB();
void PrintError();
void WriteError(uint8_t);
void ClearError(uint8_t);
void MirrorOutputsEnable();
void Heartbeat_Watchdog(uint8_t);
void User_Setup();
void Print_Setup();
void Store_Setup();
void Fetch_Setup();
String Read_From_Serial();
void web_handle_root();
void web_handle_setup();
String web_Settings();
String web_Onewire();
String web_Analog();
String web_Digital();
//----------------------------------------------------------------------


/* CSS Style */
String style =
"<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}"
"input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#000}"
"#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}"
"#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}"
"form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}"
".btn{background:#3498db;color:#fff;cursor:pointer}</style>";
//----------------------------------------------------------------------


//Declare Global variables
Datablock DB;
Configuration Config;
Stat Status;
unsigned long Elapsed;
int numberOfDevices;
DeviceAddress tempDeviceAddress;
unsigned long previousMillis_PLC = 0;
unsigned long previousMillis_PRC = 0;
const String Version = "1.2"; //Version of code


uint8_t OutputsEnable = 0x00; //Wait for this to be true from PLC, before setting outputs.

//Error Byte
uint8_t Error = 0x00;

//Heartbeat Communication watchdog.
uint8_t Heartbeat_OK = 0x00; 				//Heartbeat is OK
uint8_t Heartbeat_Old = 0x00;				//Heartbeat Edge detect.
unsigned long previousMillis_HB_OK = 0;		//Counter for heartbeat okay


//System defines
#define ONE_WIRE_BUS_Port1 16  	//Onewire Digital Sensors Port1
#define ONE_WIRE_BUS_Port2 17 	//Onewire Digital Sensors Port2
#define ONE_WIRE_BUS_Port3 18  	//Onewire Digital Sensors Port3
#define ONE_WIRE_BUS_Port4 19  	//Onewire Digital Sensors Port4
#define IIC_IOExp_Addr 0x20		//I2C Address of PCF8574 IO Expander
#define IIC_ADC_Addr 0x48		//I2C Address of ADS1115 AD Converter
#define IOEXP_INT 4				//For future use GPIO4
#define ADC_INT 23				//For future use GPIO23
#define Setup_Pin 13			//Set pin low to enter setup

//Configurable
const long PLC_COM_D = 400;		//Milliseconds delay for communication with PLC - DO NOT GO TOO FAST WITH WIFI!
const long PRC_D = 200;			//Milliseconds delay for Processdata
const long HB_LOST_D = 8000;	//Milliseconds timeout delay for Heartbeat lost, should be least (PLC_COM_D X 2)


//Sensor models
#define DS18B20 0x01
#define Voltage_0_5_V 0x02


// Initialize libraries
S7Client Client;
WebServer server(80);
Preferences storage;

//Port1
OneWire oneWire_P1(ONE_WIRE_BUS_Port1);
DallasTemperature sensors_P1(&oneWire_P1);

//Port2
OneWire oneWire_P2(ONE_WIRE_BUS_Port2);
DallasTemperature sensors_P2(&oneWire_P2);

//Port3
OneWire oneWire_P3(ONE_WIRE_BUS_Port3);
DallasTemperature sensors_P3(&oneWire_P3);

//Port4
OneWire oneWire_P4(ONE_WIRE_BUS_Port4);
DallasTemperature sensors_P4(&oneWire_P4);

//IOExpander
PCF8574 IOExp(IIC_IOExp_Addr);

//ADC
ADS1115Scanner ADC(IIC_ADC_Addr);




void setup() {

	// Variable only used in setup scope
	uint8_t errorcode;

	//Open up storage in flash as read/write mode
	storage.begin("Brewnode", false);

	//Snap7 - Default values
	Client.SetConnectionType(OP);

	//Disable wifi powersave
	esp_wifi_set_ps(WIFI_PS_NONE);

	//Use onboard LED for indication of heartbeat
	pinMode(LED_BUILTIN,OUTPUT);

	//Setup the "setup-pin"
	pinMode(Setup_Pin,INPUT_PULLUP);

	// Initialize struct
	memset(&DB,0,sizeof(Datablock));

    // Open serial communications and wait for WIFI:
    Serial.begin(115200);

    //Print the code version
    Serial.print("--- Brewnode_Code Version: ");
    Serial.print(Version);
    Serial.println(" ---");

    //Fetch data from flash
    Fetch_Setup();

    // If user shorts setup-pin or setup is not done, enter setup through serial
    if ((digitalRead(Setup_Pin) == false) or (!Config.Setup_Done) ){
    	User_Setup();
    }

    //Fetch data from flash to make sure we run with stored data
    Fetch_Setup();

    //Close flash storage
    storage.end();

    //Configure brewnode hostname
    Config.Host += Config.ID;

    //Continue booting
    Serial.println("\n------Boot------\n");
    Serial.print("Brewnode MAC Address:  ");
    Serial.println(WiFi.macAddress());
    Serial.print("Connecting to ");
    Serial.println(Config.SSID);

    //Only start wifi locked to BSSID if MAC is set.
    if (Config.BSSID[0] or Config.BSSID[1] or Config.BSSID[2] or Config.BSSID[3] or Config.BSSID[4] or Config.BSSID[5]){
    	 WiFi.begin(Config.SSID.c_str(), Config.PSK.c_str(), 0,Config.BSSID, true);
    }
    else{
    	 WiFi.begin(Config.SSID.c_str(), Config.PSK.c_str());
    }

    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.print("Local IP address : ");
    Serial.println(WiFi.localIP());

//----------------------------------------------------------------------
// Check communication ADC
//----------------------------------------------------------------------
 Wire.begin();
	Wire.beginTransmission(IIC_ADC_Addr);
	errorcode = Wire.endTransmission();
	if (errorcode == 0) {
		Serial.println("ADC I2C Connected");
		Status.ADC = true;
		//Port5-8
		DB.Analog_Sensors.Port5.Model = Voltage_0_5_V;
		DB.Analog_Sensors.Port6.Model = Voltage_0_5_V;
		DB.Analog_Sensors.Port7.Model = Voltage_0_5_V;
		DB.Analog_Sensors.Port8.Model = Voltage_0_5_V;
		DB.Analog_Sensors.Port5.Status = 0x01;
		DB.Analog_Sensors.Port6.Status = 0x01;
		DB.Analog_Sensors.Port7.Status = 0x01;
		DB.Analog_Sensors.Port8.Status = 0x01;
	} else {
		Serial.println("ADC I2C No connection");
	}

//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Check communication IO Expander
//----------------------------------------------------------------------

    Wire.beginTransmission(IIC_IOExp_Addr);
	errorcode = Wire.endTransmission();
	if (errorcode == 0) {
		Status.IOEXP = true;
		Serial.println("IOExpander I2C Connected");
	} else {
		Serial.println("IOExpander I2C No connection");
	}

//----------------------------------------------------------------------


    //Init IO Expander
    IOExp.begin();

    //Init ADC
    ADC.setSpeed(ADS1115_SPEED_8SPS);
    ADC.addChannel(ADS1115_CHANNEL0, ADS1115_RANGE_6144);
    ADC.addChannel(ADS1115_CHANNEL1, ADS1115_RANGE_6144);
    ADC.addChannel(ADS1115_CHANNEL2, ADS1115_RANGE_6144);
    ADC.addChannel(ADS1115_CHANNEL3, ADS1115_RANGE_6144);
    ADC.setSamples(1);
    ADC.start();


//----------------------------------------------------------------------
//Handle update of program over HTTP
//----------------------------------------------------------------------

    /*use MDNS for host name resolution*/
    if (!MDNS.begin(Config.Host.c_str())) {
      Serial.println("Error setting up MDNS responder!");
      while (1) {
        delay(1000);
      }
    }
    Serial.println("mDNS responder started");


    /*return index page which is stored in serverIndex */
    server.on("/", web_handle_root);
    server.on("/serverIndex", web_handle_setup);

    /*handling uploading firmware file */
    server.on("/update", HTTP_POST, []() { // @suppress("Invalid arguments") // @suppress("Ambiguous problem")
      server.sendHeader("Connection", "close");
      server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
      ESP.restart();
    }, []() {
      HTTPUpload& upload = server.upload();
      if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("Update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_WRITE) {
        /* flashing firmware to ESP*/
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
          Update.printError(Serial);
        }
      } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) { //true to set the size to the current progress
          Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
        } else {
          Update.printError(Serial);
        }
      }
    });

server.begin();

//----------------------------------------------------------------------



//------------------------------------------------
//---------- Initialize Digital Sensors ----------
//------------------------------------------------

    sensors_P1.begin();
    sensors_P2.begin();
    sensors_P3.begin();
    sensors_P4.begin();
    sensors_P1.setResolution(12);
    sensors_P2.setResolution(12);
    sensors_P3.setResolution(12);
    sensors_P4.setResolution(12);

    Serial.println("\n---- Initialize Digital_Sensors ----\n");


//----------------------------------------------------------------------
// Grab a count of devices on Port1
//----------------------------------------------------------------------
    numberOfDevices = sensors_P1.getDeviceCount();
    Status.Sensor_Port1_Nr = numberOfDevices;

    Serial.println("Digital_Sensor Locating devices Port1...\n");
    Serial.print("Found ");
    Serial.print(numberOfDevices, DEC);
    Serial.println(" devices.");
    if(numberOfDevices <= 4) {
    	// Loop through each device, print out address
		for(int i=0;i<numberOfDevices; i++){
		  // Search the wire for address
		  if(sensors_P1.getAddress(tempDeviceAddress, i)){
			Serial.print("Found device ");
			Serial.print(i, DEC);
			Serial.print(" with address: ");

			//Store and print for debugging address
			storeAddress(tempDeviceAddress,DB.Config.Detected.Port1[i].Address);
			Serial.println();

		  } else {
			Status.Sensor_Port1_Error = 0x01;
			Serial.print("Found ghost device at ");
			Serial.print(i, DEC);
			Serial.print(" but could not detect address. Check power and cabling");
		  }
		}
    } else {
    	Status.Sensor_Port1_Error = 0x02;
    	Serial.println("Too many devices on Port1. Maximum is 4.");
    }
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Grab a count of devices on Port2
//----------------------------------------------------------------------
    numberOfDevices = sensors_P2.getDeviceCount();
    Status.Sensor_Port2_Nr = numberOfDevices;

    Serial.println("Digital_Sensor Locating devices Port2...\n");
    Serial.print("Found ");
    Serial.print(numberOfDevices, DEC);
    Serial.println(" devices.");
    if(numberOfDevices <= 4) {
    	// Loop through each device, print out address
		for(int i=0;i<numberOfDevices; i++){
		  // Search the wire for address
		  if(sensors_P2.getAddress(tempDeviceAddress, i)){
			Serial.print("Found device ");
			Serial.print(i, DEC);
			Serial.print(" with address: ");

			//Store and print for debugging address
			storeAddress(tempDeviceAddress,DB.Config.Detected.Port2[i].Address);
			Serial.println();

		  } else {
			Status.Sensor_Port2_Error = 0x01;
			Serial.print("Found ghost device at ");
			Serial.print(i, DEC);
			Serial.print(" but could not detect address. Check power and cabling");
		  }
		}
    } else {
    	Status.Sensor_Port2_Error = 0x02;
    	Serial.println("Too many devices on Port2. Maximum is 4.");
    }
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Grab a count of devices on Port3
//----------------------------------------------------------------------
    numberOfDevices = sensors_P3.getDeviceCount();
    Status.Sensor_Port3_Nr = numberOfDevices;

    Serial.println("Digital_Sensor Locating devices Port3...\n");
    Serial.print("Found ");
    Serial.print(numberOfDevices, DEC);
    Serial.println(" devices.");
    if(numberOfDevices <= 4) {
    	// Loop through each device, print out address
		for(int i=0;i<numberOfDevices; i++){
		  // Search the wire for address
		  if(sensors_P3.getAddress(tempDeviceAddress, i)){
			Serial.print("Found device ");
			Serial.print(i, DEC);
			Serial.print(" with address: ");

			//Store and print for debugging address
			storeAddress(tempDeviceAddress,DB.Config.Detected.Port3[i].Address);
			Serial.println();

		  } else {
			Status.Sensor_Port3_Error = 0x01;
			Serial.print("Found ghost device at ");
			Serial.print(i, DEC);
			Serial.print(" but could not detect address. Check power and cabling");
		  }
		}
    } else {
    	Status.Sensor_Port3_Error = 0x02;
    	Serial.println("Too many devices on Port3. Maximum is 4.");
    }
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Grab a count of devices on Port4
//----------------------------------------------------------------------
    numberOfDevices = sensors_P4.getDeviceCount();
    Status.Sensor_Port4_Nr = numberOfDevices;

    Serial.println("Digital_Sensor Locating devices Port4...\n");
    Serial.print("Found ");
    Serial.print(numberOfDevices, DEC);
    Serial.println(" devices.");
    if(numberOfDevices <= 4) {
    	// Loop through each device, print out address
		for(int i=0;i<numberOfDevices; i++){
		  // Search the wire for address
		  if(sensors_P4.getAddress(tempDeviceAddress, i)){
			Serial.print("Found device ");
			Serial.print(i, DEC);
			Serial.print(" with address: ");

			//Store and print for debugging address
			storeAddress(tempDeviceAddress,DB.Config.Detected.Port4[i].Address);
			Serial.println();

		  } else {
			Status.Sensor_Port4_Error = 0x01;
			Serial.print("Found ghost device at ");
			Serial.print(i, DEC);
			Serial.print(" but could not detect address. Check power and cabling");
		  }
		}
    } else {
    	Status.Sensor_Port4_Error = 0x02;
    	Serial.println("Too many devices on Port4. Maximum is 4.");
    }
//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Set start values variables OneWire sensors
//----------------------------------------------------------------------
    //Port1
    DB.Digital_Sensors.Port1[0].Model = DS18B20;
    DB.Digital_Sensors.Port1[1].Model = DS18B20;
    DB.Digital_Sensors.Port1[2].Model = DS18B20;
    DB.Digital_Sensors.Port1[3].Model = DS18B20;

    //Port2
	DB.Digital_Sensors.Port2[0].Model = DS18B20;
	DB.Digital_Sensors.Port2[1].Model = DS18B20;
	DB.Digital_Sensors.Port2[2].Model = DS18B20;
	DB.Digital_Sensors.Port2[3].Model = DS18B20;

	//Port3
	DB.Digital_Sensors.Port3[0].Model = DS18B20;
	DB.Digital_Sensors.Port3[1].Model = DS18B20;
	DB.Digital_Sensors.Port3[2].Model = DS18B20;
	DB.Digital_Sensors.Port3[3].Model = DS18B20;

	//Port4
	DB.Digital_Sensors.Port4[0].Model = DS18B20;
	DB.Digital_Sensors.Port4[1].Model = DS18B20;
	DB.Digital_Sensors.Port4[2].Model = DS18B20;
	DB.Digital_Sensors.Port4[3].Model = DS18B20;

}
//----------------------------------------------------------------------

//----------------------------------------------------------------------
// Web page functions
//----------------------------------------------------------------------
/* Settings */
String web_Settings(){
	String tmp = "";
	tmp += "<p>Brewnode_Code Version: ";
	tmp += Version;
	tmp += "</p>";
	tmp += "<p>ID-Number: ";
	tmp += Config.ID;
	tmp += "</p>";
	tmp += "<p>Hostname: ";
	tmp += Config.Host;
	tmp += "</p>";
	tmp +=	"<p>IP-Brewnode: ";
	tmp += WiFi.localIP()[0];
	tmp += ".";
	tmp += WiFi.localIP()[1];
	tmp += ".";
	tmp += WiFi.localIP()[2];
	tmp += ".";
	tmp += WiFi.localIP()[3];
	tmp += "</p>";
	tmp +=	"<p>IP-PLC: ";
	tmp += Config.IP_PLC[0];
	tmp += ".";
	tmp += Config.IP_PLC[1];
	tmp += ".";
	tmp += Config.IP_PLC[2];
	tmp += ".";
	tmp += Config.IP_PLC[3];
	tmp += "</p>";
	tmp +=	"<p>DB-Number-PLC: ";
	tmp += Config.PLC_DB;
	tmp += "</p>";
	tmp += "<b>S7-300 Rack:0 Slot:2<br>";
	tmp += "S7-400 Follow HW Config<br>";
	tmp += "S7-1200/1500 Rack:0 Slot:0</b>";
	tmp +=	"<p>PLC-Rack: ";
	tmp += Config.PLC_Rack;
	tmp += "</p>";
	tmp +=	"<p>PLC-Slot: ";
	tmp += Config.PLC_Slot;
	tmp += "</p>";

	tmp +=	"<p>PLC-Connection: ";
	if (Client.Connected){
		tmp += "Connected";
	}
	else {
		tmp += "Not Connected";
	}
	tmp += "</p>";

	tmp +=	"<p>ADC-Connection: ";
	if (Status.ADC){
		tmp += "Connected";
	}
	else {
		tmp += "Not Connected";
	}
	tmp += "</p>";

	tmp +=	"<p>IOExp-Connection: ";
	if (Status.IOEXP){
		tmp += "Connected";
	}
	else {
		tmp += "Not Connected";
	}
	tmp += "</p>";
	return tmp;
}

/* Onewire */
String web_Onewire(){
	String tmp = "";
	tmp += "<p>Number of Sensors Port1: ";
	tmp += Status.Sensor_Port1_Nr;
	tmp += " Port2: ";
	tmp += Status.Sensor_Port2_Nr;
	tmp += " Port3: ";
	tmp += Status.Sensor_Port3_Nr;
	tmp += " Port4: ";
	tmp += Status.Sensor_Port4_Nr;
	tmp += "</p>";
	tmp += "<p>Error codes (0=OK) Port1: ";
	tmp += Status.Sensor_Port1_Error;
	tmp += " Port2: ";
	tmp += Status.Sensor_Port2_Error;
	tmp += " Port3: ";
	tmp += Status.Sensor_Port3_Error;
	tmp += " Port4: ";
	tmp += Status.Sensor_Port4_Error;
	tmp += "</p>";

	tmp += "<p>Port1 &degC: ";
	tmp += DB.Digital_Sensors.Port1[0].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port1[1].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port1[2].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port1[3].Value;
	tmp += "&nbsp&nbsp Status: ";
	tmp += DB.Digital_Sensors.Port1[0].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port1[1].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port1[2].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port1[3].Status;
	tmp += "</p>";

	tmp += "<p>Port2 &degC: ";
	tmp += DB.Digital_Sensors.Port2[0].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port2[1].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port2[2].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port2[3].Value;
	tmp += "&nbsp&nbsp Status: ";
	tmp += DB.Digital_Sensors.Port2[0].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port2[1].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port2[2].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port2[3].Status;
	tmp += "</p>";

	tmp += "<p>Port3 &degC: ";
	tmp += DB.Digital_Sensors.Port3[0].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port3[1].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port3[2].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port3[3].Value;
	tmp += "&nbsp&nbsp Status: ";
	tmp += DB.Digital_Sensors.Port3[0].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port3[1].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port3[2].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port3[3].Status;
	tmp += "</p>";

	tmp += "<p>Port4 &degC: ";
	tmp += DB.Digital_Sensors.Port4[0].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port4[1].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port4[2].Value;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port4[3].Value;
	tmp += "&nbsp&nbsp Status: ";
	tmp += DB.Digital_Sensors.Port4[0].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port4[1].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port4[2].Status;
	tmp += " | ";
	tmp += DB.Digital_Sensors.Port4[3].Status;
	tmp += "</p>";

	tmp += "<H2>Detected addresses</H2>";
	tmp += "<p>Port1 #1: ";
	tmp += String(DB.Config.Detected.Port1[0].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[0].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[0].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[0].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[0].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[0].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[0].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[0].Address[7],HEX);
	tmp += "&nbsp&nbsp#2: ";
	tmp += String(DB.Config.Detected.Port1[1].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[1].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[1].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[1].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[1].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[1].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[1].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[1].Address[7],HEX);
	tmp += "&nbsp&nbsp#3: ";
	tmp += String(DB.Config.Detected.Port1[2].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[2].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[2].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[2].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[2].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[2].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[2].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[2].Address[7],HEX);
	tmp += "&nbsp&nbsp#4: ";
	tmp += String(DB.Config.Detected.Port1[3].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[3].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[3].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[3].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[3].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[3].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[3].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port1[3].Address[7],HEX);
	tmp += "</p>";

	tmp += "<p>Port2 #1: ";
	tmp += String(DB.Config.Detected.Port2[0].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[0].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[0].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[0].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[0].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[0].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[0].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[0].Address[7],HEX);
	tmp += "&nbsp&nbsp#2: ";
	tmp += String(DB.Config.Detected.Port2[1].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[1].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[1].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[1].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[1].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[1].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[1].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[1].Address[7],HEX);
	tmp += "&nbsp&nbsp#3: ";
	tmp += String(DB.Config.Detected.Port2[2].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[2].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[2].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[2].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[2].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[2].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[2].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[2].Address[7],HEX);
	tmp += "&nbsp&nbsp#4: ";
	tmp += String(DB.Config.Detected.Port2[3].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[3].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[3].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[3].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[3].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[3].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[3].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port2[3].Address[7],HEX);
	tmp += "</p>";

	tmp += "<p>Port3 #1: ";
	tmp += String(DB.Config.Detected.Port3[0].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[0].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[0].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[0].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[0].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[0].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[0].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[0].Address[7],HEX);
	tmp += "&nbsp&nbsp#2: ";
	tmp += String(DB.Config.Detected.Port3[1].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[1].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[1].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[1].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[1].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[1].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[1].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[1].Address[7],HEX);
	tmp += "&nbsp&nbsp#3: ";
	tmp += String(DB.Config.Detected.Port3[2].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[2].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[2].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[2].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[2].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[2].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[2].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[2].Address[7],HEX);
	tmp += "&nbsp&nbsp#4: ";
	tmp += String(DB.Config.Detected.Port3[3].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[3].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[3].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[3].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[3].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[3].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[3].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port3[3].Address[7],HEX);
	tmp += "</p>";


	tmp += "<p>Port4 #1: ";
	tmp += String(DB.Config.Detected.Port4[0].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[0].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[0].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[0].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[0].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[0].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[0].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[0].Address[7],HEX);
	tmp += "&nbsp&nbsp#2: ";
	tmp += String(DB.Config.Detected.Port4[1].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[1].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[1].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[1].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[1].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[1].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[1].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[1].Address[7],HEX);
	tmp += "&nbsp&nbsp#3: ";
	tmp += String(DB.Config.Detected.Port4[2].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[2].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[2].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[2].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[2].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[2].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[2].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[2].Address[7],HEX);
	tmp += "&nbsp&nbsp#4: ";
	tmp += String(DB.Config.Detected.Port4[3].Address[0],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[3].Address[1],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[3].Address[2],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[3].Address[3],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[3].Address[4],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[3].Address[5],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[3].Address[6],HEX);
	tmp += ".";
	tmp += String(DB.Config.Detected.Port4[3].Address[7],HEX);
	tmp += "</p>";
	return tmp;
}

/* Analog */
String web_Analog(){
	String tmp = "";
	tmp += "<b>Values between 0 - 32768<br>";
	tmp +=	"0-5V or 0-10V. Selected by hardware. ADC Range is 0-6144 mV<br></b>";
	tmp += "<p>Port5 Value: ";
	tmp +=	DB.Analog_Sensors.Port5.Value;
	tmp +=	"</p>";

	tmp += "<p>Port6 Value: ";
	tmp +=	DB.Analog_Sensors.Port6.Value;
	tmp +=	"</p>";

	tmp += "<p>Port7 Value: ";
	tmp +=	DB.Analog_Sensors.Port7.Value;
	tmp +=	"</p>";

	tmp += "<p>Port8 Value: ";
	tmp +=	DB.Analog_Sensors.Port8.Value;
	tmp +=	"</p>";
	return tmp;
}

/* Digital */
String web_Digital(){
	String tmp = "";
	tmp +=	"<H3>Inputs</H3>";
	tmp += "<b>Inputs pull-up to +5v. Switch to GND.</b>";
	tmp += "<p>Port1 Value: ";
	tmp +=	DB.Digital_Inputs[0];
	tmp +=	"</p>";

	tmp += "<p>Port2 Value: ";
	tmp +=	DB.Digital_Inputs[1];
	tmp +=	"</p>";

	tmp += "<p>Port3 Value: ";
	tmp +=	DB.Digital_Inputs[2];
	tmp +=	"</p>";

	tmp += "<p>Port4 Value: ";
	tmp +=	DB.Digital_Inputs[3];
	tmp +=	"</p>";

	tmp +=	"<H3>Relays</H3>";
	tmp +=	"<p>Outputs_Enable Value: ";
	tmp +=	DB.Relay[4];
	tmp +=	"</p>";
	tmp +=	"<p> Relay1 Value: ";
	tmp +=	DB.Relay[0];
	tmp +=	"</p>";
	tmp +=	"<p> Relay2 Value: ";
	tmp +=	DB.Relay[1];
	tmp +=	"</p>";
	tmp +=	"<p> Relay3 Value: ";
	tmp +=	DB.Relay[2];
	tmp +=	"</p>";
	tmp +=	"<p> Relay4 Value: ";
	tmp +=	DB.Relay[3];
	tmp +=	"</p>";
	return tmp;
}

void web_handle_root(){
	/* Login page */
	String loginIndex =
	"<form name=loginForm>"
	"<h1>Brewnode V1 Login</h1>"
	"<input name=userid placeholder='User ID'> "
	"<input name=pwd placeholder=Password type=Password> "
	"<input type=submit onclick=check(this.form) class=btn value=Login></form>"
	"<script>"
	"function check(form) {"
	"if(form.userid.value=='admin' && form.pwd.value=='brewnode')"
	"{window.open('/serverIndex')}"
	"else"
	"{alert('Error Password or Username')}"
	"}"
	"</script>"
	"<head>"
	"<style>"
	"* {"
	"box-sizing: border-box;"
	"}"
	".column {"
	"float: left;"
	"width: 50%;"
	"padding: 10px;"
	"height: 500px;"
	"}"
	".row:after {"
	"content: "";"
	"display: table;"
	"clear: both;"
	"}"
	"</style>"
	"</head>"
	"<div class='row'>"
	"<div class='column' style='background-color:#93c8ec;'>"
	"<h2>Settings/Status</h2>"
	+web_Settings()+
	"</div>"
	"<div class='column' style='background-color:#2283c3;'>"
	"<h2>Onewire Sensors</h2>"
	+web_Onewire()+
	"</div>"
	"<div class='column' style='background-color:#2283c3;'>"
	"<h2>Analog-Inputs</h2>"
	+web_Analog()+
	"</div>"
	"<div class='column' style='background-color:#93c8ec;'>"
	"<h2>Digital-In/Relay-Out</h2>"
	+web_Digital()+
	"</div>"
	"</div>"+ style;
	server.sendHeader("Connection", "close");
	server.send(200, "text/html", loginIndex);
}



void web_handle_setup(){
	/* Server Index Page */
	String serverIndex =
	"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
	"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
	"<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>"
	"<label id='file-input' for='file'>   Choose file...</label>"
	"<input type='submit' class=btn value='Update'>"
	"<br><br>"
	"<div id='prg'></div>"
	"<br><div id='prgbar'><div id='bar'></div></div><br></form>"
	"<script>"
	"function sub(obj){"
	"var fileName = obj.value.split('\\\\');"
	"document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
	"};"
	"$('form').submit(function(e){"
	"e.preventDefault();"
	"var form = $('#upload_form')[0];"
	"var data = new FormData(form);"
	"$.ajax({"
	"url: '/update',"
	"type: 'POST',"
	"data: data,"
	"contentType: false,"
	"processData:false,"
	"xhr: function() {"
	"var xhr = new window.XMLHttpRequest();"
	"xhr.upload.addEventListener('progress', function(evt) {"
	"if (evt.lengthComputable) {"
	"var per = evt.loaded / evt.total;"
	"$('#prg').html('progress: ' + Math.round(per*100) + '%');"
	"$('#bar').css('width',Math.round(per*100) + '%');"
	"}"
	"}, false);"
	"return xhr;"
	"},"
	"success:function(d, s) {"
	"console.log('success!') "
	"},"
	"error: function (a, b, c) {"
	"}"
	"});"
	"});"
	"</script>";
	server.sendHeader("Connection", "close");
	server.send(200, "text/html", serverIndex);
}

//----------------------------------------------------------------------


//----------------------------------------------------------------------
// Setup parameters function
//----------------------------------------------------------------------
void User_Setup(){
	delay(500);
	Serial.println("Entering setup...");
	delay(500);
	Serial.println("\n--------------SETUP--------------\n");
	Serial.println("--Set Serial to Newline (NL) only--\n");

	//Print current settings if settings are done
	if (Config.Setup_Done) {
		Serial.println("Current Settings");
		Print_Setup();
	}

	//Repeat setup routine if not  accepted at the end
	uint8_t getout = 0x00;
	while (!getout){
		Serial.println("New Settings");

		//ID Number
		Serial.print("ID: ");
		Config.ID = Read_From_Serial().toInt();
		Serial.println();

		//PLC IP Address
		Serial.println("PLC-IP_Address");
		Serial.print("First: ");
		Config.IP_PLC[0] = Read_From_Serial().toInt();
		Serial.println();
		Serial.print("Second: ");
		Config.IP_PLC[1] = Read_From_Serial().toInt();
		Serial.println();
		Serial.print("Third: ");
		Config.IP_PLC[2] = Read_From_Serial().toInt();
		Serial.println();
		Serial.print("Fourth: ");
		Config.IP_PLC[3] = Read_From_Serial().toInt();
		Serial.println();

		//SSID
		Serial.print("SSID: ");
		Config.SSID = Read_From_Serial();
		Serial.println();

		//PSK
		Serial.print("PSK: ");
		Config.PSK = Read_From_Serial();
		Serial.println();

		//BSSID
		Serial.println("BSSID is MAC of AP. Put in all zeroes if not used.");
		Serial.println("BSSID");
		Serial.print("Part1: ");
		Config.BSSID[0] = strtoul(Read_From_Serial().c_str(),NULL,16);
		Serial.println();
		Serial.print("Part2: ");
		Config.BSSID[1] = strtoul(Read_From_Serial().c_str(),NULL,16);
		Serial.println();
		Serial.print("Part3: ");
		Config.BSSID[2] = strtoul(Read_From_Serial().c_str(),NULL,16);
		Serial.println();
		Serial.print("Part4: ");
		Config.BSSID[3] = strtoul(Read_From_Serial().c_str(),NULL,16);
		Serial.println();
		Serial.print("Part5: ");
		Config.BSSID[4] = strtoul(Read_From_Serial().c_str(),NULL,16);
		Serial.println();
		Serial.print("Part6: ");
		Config.BSSID[5] = strtoul(Read_From_Serial().c_str(),NULL,16);
		Serial.println();

		//DB Number
		Serial.println("DB Number of PLC:");
		Config.PLC_DB = Read_From_Serial().toInt();

		//PLC-Rack
		Serial.println("PLC Rack:");
		Config.PLC_Rack = Read_From_Serial().toInt();

		//PLC-Slot
		Serial.println("PLC Slot:");
		Config.PLC_Slot = Read_From_Serial().toInt();
		Serial.println();

		//Show the new settings
		Serial.println("\n----This is the new settings----\n");
		Print_Setup();

		//Check if user is OK with settings
		Serial.println("\n---Is this correct ? Y/N ? \n");
		if (Read_From_Serial() == "Y"){
			getout = true;
			Config.Setup_Done = true;
			Serial.println("Storing this settings");
			Store_Setup();
		}
		else{
			Serial.println("Restarting setup...");
		}
	}


}

void Print_Setup(){
	Serial.print("ID: ");
	Serial.println(Config.ID,DEC);
	Serial.print("IP-Address of PLC: ");
	Serial.println(Config.IP_PLC);
	Serial.print("SSID: ");
	Serial.println(Config.SSID.c_str());
	Serial.print("PSK: ");
	Serial.println(Config.PSK.c_str());
	Serial.print("BSSID: ");
	Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X", Config.BSSID[0], Config.BSSID[1], Config.BSSID[2], Config.BSSID[3], Config.BSSID[4], Config.BSSID[5]);
	Serial.println();
	Serial.print("PLC-DB: ");
	Serial.println(Config.PLC_DB);
	Serial.print("PLC-Rack: ");
	Serial.println(Config.PLC_Rack);
	Serial.print("PLC-Slot: ");
	Serial.println(Config.PLC_Slot);
}

String Read_From_Serial(){
	String temp;

	//Just wait here
	while (!Serial.available()){
	}
	temp = Serial.readStringUntil('\n');
	return temp;
}

//----------------------------------------------------------------------
// Fetch/Store data function
//----------------------------------------------------------------------
void Fetch_Setup(){
	//ID
	Config.ID = storage.getUChar("ID", 0x00);

	//IP-PLC
	Config.IP_PLC[0] = storage.getUChar("IP_PLC1", 0x00);
	Config.IP_PLC[1] = storage.getUChar("IP_PLC2", 0x00);
	Config.IP_PLC[2] = storage.getUChar("IP_PLC3", 0x00);
	Config.IP_PLC[3] = storage.getUChar("IP_PLC4", 0x00);

	//SSID
	Config.SSID = storage.getString("SSID", "");

	//PSK
	Config.PSK = storage.getString("PSK", "");

	//BSSID
	Config.BSSID[0] = storage.getUChar("BSSID1", 0x00);
	Config.BSSID[1] = storage.getUChar("BSSID2", 0x00);
	Config.BSSID[2] = storage.getUChar("BSSID3", 0x00);
	Config.BSSID[3] = storage.getUChar("BSSID4", 0x00);
	Config.BSSID[4] = storage.getUChar("BSSID5", 0x00);
	Config.BSSID[5] = storage.getUChar("BSSID6", 0x00);

	//PLC-DB
	Config.PLC_DB = storage.getUShort("PLC_DB", 0);

	//PLC-Rack
	Config.PLC_Rack = storage.getUShort("PLC_Rack", 0);

	//PLC-Slot
	Config.PLC_Slot = storage.getUShort("PLC_Slot", 0);

	//Setup-Done
	Config.Setup_Done = storage.getUChar("Setup_Done", 0x00);
}

void Store_Setup(){
	//ID
	storage.putUChar("ID", Config.ID);

	//IP-PLC
	storage.putUChar("IP_PLC1", Config.IP_PLC[0]);
	storage.putUChar("IP_PLC2", Config.IP_PLC[1]);
	storage.putUChar("IP_PLC3", Config.IP_PLC[2]);
	storage.putUChar("IP_PLC4", Config.IP_PLC[3]);

	//SSID
	storage.putString("SSID", Config.SSID);

	//PSK
	storage.putString("PSK", Config.PSK);

	//BSSID
	storage.putUChar("BSSID1", Config.BSSID[0]);
	storage.putUChar("BSSID2", Config.BSSID[1]);
	storage.putUChar("BSSID3", Config.BSSID[2]);
	storage.putUChar("BSSID4", Config.BSSID[3]);
	storage.putUChar("BSSID5", Config.BSSID[4]);
	storage.putUChar("BSSID6", Config.BSSID[5]);

	//PLC-DB
	storage.putUShort("PLC_DB", Config.PLC_DB);

	//PLC-Rack
	storage.putUShort("PLC_Rack", Config.PLC_Rack);

	//PLC-Slot
	storage.putUShort("PLC_Slot", Config.PLC_Slot);

	//Setup-Done
	storage.putUChar("Setup_Done", Config.Setup_Done);
}

//----------------------------------------------------------------------
// General Error functions
//----------------------------------------------------------------------
void PrintError(){
	Serial.println("Error Occurred,");
	Serial.print("Error Byte: ");
	Serial.print(Error,DEC);
	Serial.print(" - ");
	Serial.println(Error,BIN);
}

void WriteError(uint8_t place){
	Serial.print("Writing error: ");
	Serial.println(place);
	bitSet(Error,place);
}

void ClearError(uint8_t place){
	if bitRead(Error,place) {
		Serial.print("Clearing error: ");
		Serial.println(place);
		bitClear(Error,place);
	}
}

//----------------------------------------------------------------------
// Outputs Enable Mirroring
//----------------------------------------------------------------------
void MirrorOutputsEnable(){
	//If Heartbeat is OK, then use OutputEnable from PLC.
	if ((Heartbeat_OK) and (Error == 0x00)) {
		OutputsEnable=DB.Relay[4];
	}
	else {
		OutputsEnable = 0x00;
	}
}

//----------------------------------------------------------------------
// Heartbeat Watchdog
// Positive edge resets counter.
// Delay until Heartbeat_OK.
//----------------------------------------------------------------------
void Heartbeat_Watchdog(uint8_t Heartbeat){

	unsigned long current_millis = millis();

	//Watch for edge of heartbeat
	if (Heartbeat != Heartbeat_Old) {
		if (Heartbeat) {
			previousMillis_HB_OK = current_millis;
		}
	}
	Heartbeat_Old = Heartbeat;

	//If heartbeat lost longer then this time
	if ((current_millis - previousMillis_HB_OK) <= HB_LOST_D) {
		Heartbeat_OK = 0x01;
	}
	else{
		Heartbeat_OK = 0x00;

		//Constrain counter to remedy rollover
		previousMillis_HB_OK = (current_millis-(HB_LOST_D+10));
	}
}
//----------------------------------------------------------------------
// Connects to the PLC
//----------------------------------------------------------------------
bool Connect()
{
    int Result=Client.ConnectTo(Config.IP_PLC,
                                  Config.PLC_Rack,  // Rack (see the doc.)
                                  Config.PLC_Slot); // Slot (see the doc.)
    Serial.print("Connecting to PLC: ");
    Serial.println(Config.IP_PLC);
    if (Result==0) 
    {
      Serial.print("Connected ! PDU Length = ");
      Serial.println(Client.GetPDULength());

      //Clear Error
      ClearError(1);
      ClearError(2);
    }
    else
    {
    	Serial.println("Connection error");
    	//Error
        WriteError(1);
        PrintError();
    }
    return Result==0;
}

void CheckError(int ErrNo)
{
  Serial.print("Error No. 0x");
  Serial.println(ErrNo, HEX);
  
  // Checks if it's a Severe Error => we need to disconnect
  if (ErrNo & 0x00FF)
  {
    Serial.println("SEVERE ERROR, disconnecting");
    Client.Disconnect();

    WriteError(2);
    PrintError();
  }
}

void MarkTime()
{
  Elapsed=millis();
}

void ShowTime()
{
  // Calcs the time
  Elapsed=millis()-Elapsed;
  Serial.print("Job finished time (ms) : ");
  Serial.println(Elapsed);   
}

//--------------------------------------
//Store and print OneWire Address
//--------------------------------------
void storeAddress(DeviceAddress deviceAddress, uint8_t DB_Address[]) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) {
    	Serial.print("0");
    	DB_Address[i] = 0;
    }
      Serial.print(deviceAddress[i], HEX);
      DB_Address[i] = deviceAddress[i];
  }
}
//--------------------------------------


//--------------------------------------
//Update Digital Sensors
//--------------------------------------
void UpdateDigitalSensors(){

	//Fetch temps port1 - tell sensors to update
	DB.Digital_Sensors.Port1[0].Status=sensors_P1.requestTemperaturesByAddress(DB.Digital_Sensors.Port1[0].Address);
	DB.Digital_Sensors.Port1[1].Status=sensors_P1.requestTemperaturesByAddress(DB.Digital_Sensors.Port1[1].Address);
	DB.Digital_Sensors.Port1[2].Status=sensors_P1.requestTemperaturesByAddress(DB.Digital_Sensors.Port1[2].Address);
	DB.Digital_Sensors.Port1[3].Status=sensors_P1.requestTemperaturesByAddress(DB.Digital_Sensors.Port1[3].Address);

	//Update temps port1 - store new values in struct
	UpdateOneDigitalSensor(sensors_P1.getTempC(DB.Digital_Sensors.Port1[0].Address), &DB.Digital_Sensors.Port1[0].Value, DB.Digital_Sensors.Port1[0].Status);
	UpdateOneDigitalSensor(sensors_P1.getTempC(DB.Digital_Sensors.Port1[1].Address), &DB.Digital_Sensors.Port1[1].Value, DB.Digital_Sensors.Port1[1].Status);
	UpdateOneDigitalSensor(sensors_P1.getTempC(DB.Digital_Sensors.Port1[2].Address), &DB.Digital_Sensors.Port1[2].Value, DB.Digital_Sensors.Port1[2].Status);
	UpdateOneDigitalSensor(sensors_P1.getTempC(DB.Digital_Sensors.Port1[3].Address), &DB.Digital_Sensors.Port1[3].Value, DB.Digital_Sensors.Port1[3].Status);

	//Fetch temps port2 - tell sensors to update
	DB.Digital_Sensors.Port2[0].Status=sensors_P2.requestTemperaturesByAddress(DB.Digital_Sensors.Port2[0].Address);
	DB.Digital_Sensors.Port2[1].Status=sensors_P2.requestTemperaturesByAddress(DB.Digital_Sensors.Port2[1].Address);
	DB.Digital_Sensors.Port2[2].Status=sensors_P2.requestTemperaturesByAddress(DB.Digital_Sensors.Port2[2].Address);
	DB.Digital_Sensors.Port2[3].Status=sensors_P2.requestTemperaturesByAddress(DB.Digital_Sensors.Port2[3].Address);

	//Update temps port2 - store new values in struct
	UpdateOneDigitalSensor(sensors_P2.getTempC(DB.Digital_Sensors.Port2[0].Address), &DB.Digital_Sensors.Port2[0].Value, DB.Digital_Sensors.Port2[0].Status);
	UpdateOneDigitalSensor(sensors_P2.getTempC(DB.Digital_Sensors.Port2[1].Address), &DB.Digital_Sensors.Port2[1].Value, DB.Digital_Sensors.Port2[1].Status);
	UpdateOneDigitalSensor(sensors_P2.getTempC(DB.Digital_Sensors.Port2[2].Address), &DB.Digital_Sensors.Port2[2].Value, DB.Digital_Sensors.Port2[2].Status);
	UpdateOneDigitalSensor(sensors_P2.getTempC(DB.Digital_Sensors.Port2[3].Address), &DB.Digital_Sensors.Port2[3].Value, DB.Digital_Sensors.Port2[3].Status);

	//Fetch temps port3 - tell sensors to update
	DB.Digital_Sensors.Port3[0].Status=sensors_P3.requestTemperaturesByAddress(DB.Digital_Sensors.Port3[0].Address);
	DB.Digital_Sensors.Port3[1].Status=sensors_P3.requestTemperaturesByAddress(DB.Digital_Sensors.Port3[1].Address);
	DB.Digital_Sensors.Port3[2].Status=sensors_P3.requestTemperaturesByAddress(DB.Digital_Sensors.Port3[2].Address);
	DB.Digital_Sensors.Port3[3].Status=sensors_P3.requestTemperaturesByAddress(DB.Digital_Sensors.Port3[3].Address);

	//Update temps port3 - store new values in struct
	UpdateOneDigitalSensor(sensors_P3.getTempC(DB.Digital_Sensors.Port3[0].Address), &DB.Digital_Sensors.Port3[0].Value, DB.Digital_Sensors.Port3[0].Status);
	UpdateOneDigitalSensor(sensors_P3.getTempC(DB.Digital_Sensors.Port3[1].Address), &DB.Digital_Sensors.Port3[1].Value, DB.Digital_Sensors.Port3[1].Status);
	UpdateOneDigitalSensor(sensors_P3.getTempC(DB.Digital_Sensors.Port3[2].Address), &DB.Digital_Sensors.Port3[2].Value, DB.Digital_Sensors.Port3[2].Status);
	UpdateOneDigitalSensor(sensors_P3.getTempC(DB.Digital_Sensors.Port3[3].Address), &DB.Digital_Sensors.Port3[3].Value, DB.Digital_Sensors.Port3[3].Status);

	//Fetch temps port4 - tell sensors to update
	DB.Digital_Sensors.Port4[0].Status=sensors_P4.requestTemperaturesByAddress(DB.Digital_Sensors.Port4[0].Address);
	DB.Digital_Sensors.Port4[1].Status=sensors_P4.requestTemperaturesByAddress(DB.Digital_Sensors.Port4[1].Address);
	DB.Digital_Sensors.Port4[2].Status=sensors_P4.requestTemperaturesByAddress(DB.Digital_Sensors.Port4[2].Address);
	DB.Digital_Sensors.Port4[3].Status=sensors_P4.requestTemperaturesByAddress(DB.Digital_Sensors.Port4[3].Address);

	//Update temps port4 - store new values in struct
	UpdateOneDigitalSensor(sensors_P4.getTempC(DB.Digital_Sensors.Port4[0].Address), &DB.Digital_Sensors.Port4[0].Value, DB.Digital_Sensors.Port4[0].Status);
	UpdateOneDigitalSensor(sensors_P4.getTempC(DB.Digital_Sensors.Port4[1].Address), &DB.Digital_Sensors.Port4[1].Value, DB.Digital_Sensors.Port4[1].Status);
	UpdateOneDigitalSensor(sensors_P4.getTempC(DB.Digital_Sensors.Port4[2].Address), &DB.Digital_Sensors.Port4[2].Value, DB.Digital_Sensors.Port4[2].Status);
	UpdateOneDigitalSensor(sensors_P4.getTempC(DB.Digital_Sensors.Port4[3].Address), &DB.Digital_Sensors.Port4[3].Value, DB.Digital_Sensors.Port4[3].Status);

}
//--------------------------------------


//--------------------------------------
//Update One Digital Sensor
//--------------------------------------
void UpdateOneDigitalSensor(float temp, float* Value, uint8_t Status){
	if (Status == 1){
		*Value = temp;
	}
}
//--------------------------------------


//--------------------------------------
//Update Configured sensors
//Configured -> SensorAddresses
//--------------------------------------
void UpdateConfigured(){

	//Port1
	UpdateAddressArray(DB.Config.Configured.Port1[0].Address,DB.Digital_Sensors.Port1[0].Address);
	UpdateAddressArray(DB.Config.Configured.Port1[1].Address,DB.Digital_Sensors.Port1[1].Address);
	UpdateAddressArray(DB.Config.Configured.Port1[2].Address,DB.Digital_Sensors.Port1[2].Address);
	UpdateAddressArray(DB.Config.Configured.Port1[3].Address,DB.Digital_Sensors.Port1[3].Address);

	//Port2
	UpdateAddressArray(DB.Config.Configured.Port2[0].Address,DB.Digital_Sensors.Port2[0].Address);
	UpdateAddressArray(DB.Config.Configured.Port2[1].Address,DB.Digital_Sensors.Port2[1].Address);
	UpdateAddressArray(DB.Config.Configured.Port2[2].Address,DB.Digital_Sensors.Port2[2].Address);
	UpdateAddressArray(DB.Config.Configured.Port2[3].Address,DB.Digital_Sensors.Port2[3].Address);

	//Port3
	UpdateAddressArray(DB.Config.Configured.Port3[0].Address,DB.Digital_Sensors.Port3[0].Address);
	UpdateAddressArray(DB.Config.Configured.Port3[1].Address,DB.Digital_Sensors.Port3[1].Address);
	UpdateAddressArray(DB.Config.Configured.Port3[2].Address,DB.Digital_Sensors.Port3[2].Address);
	UpdateAddressArray(DB.Config.Configured.Port3[3].Address,DB.Digital_Sensors.Port3[3].Address);

	//Port4
	UpdateAddressArray(DB.Config.Configured.Port4[0].Address,DB.Digital_Sensors.Port4[0].Address);
	UpdateAddressArray(DB.Config.Configured.Port4[1].Address,DB.Digital_Sensors.Port4[1].Address);
	UpdateAddressArray(DB.Config.Configured.Port4[2].Address,DB.Digital_Sensors.Port4[2].Address);
	UpdateAddressArray(DB.Config.Configured.Port4[3].Address,DB.Digital_Sensors.Port4[3].Address);

}
//--------------------------------------

//--------------------------------------
//Update one address array
//--------------------------------------
void UpdateAddressArray(uint8_t Src[], uint8_t Dst[]){
	  for (uint8_t i = 0; i < 8; i++){
		  Dst[i] = Src[i];
	  }
}
//--------------------------------------


//--------------------------------------
//------------ Check In/Out ------------
//--------------------------------------
void CheckIO(){

	//Inputs
	DB.Digital_Inputs[0] = IOExp.read(0);
	DB.Digital_Inputs[1] = IOExp.read(1);
	DB.Digital_Inputs[2] = IOExp.read(2);
	DB.Digital_Inputs[3] = IOExp.read(3);

	if (OutputsEnable) {
		//Outputs - HIGH = Output Off
		IOExp.write(4, !DB.Relay[0]);
		IOExp.write(5, !DB.Relay[1]);
		IOExp.write(6, !DB.Relay[2]);
		IOExp.write(7, !DB.Relay[3]);

	}
	else
	{
		IOExp.write(4, HIGH);
		IOExp.write(5, HIGH);
		IOExp.write(6, HIGH);
		IOExp.write(7, HIGH);
	}

}
//--------------------------------------


//--------------------------------------
//------- Check Analog voltages --------
//--------------------------------------

void CheckAnalog(){
	ADC.update();
	if (ADC.ready()) {
		//Get ADC Values
		DB.Analog_Sensors.Port5.Value = int16_t(ADC.readAverage(0));
		DB.Analog_Sensors.Port6.Value = int16_t(ADC.readAverage(1));
		DB.Analog_Sensors.Port7.Value = int16_t(ADC.readAverage(2));
		DB.Analog_Sensors.Port8.Value = int16_t(ADC.readAverage(3));
		//Trigger new measurement
		ADC.start();

		//Constrain values 0 - 32768
		DB.Analog_Sensors.Port5.Value = Const(DB.Analog_Sensors.Port5.Value);
		DB.Analog_Sensors.Port6.Value = Const(DB.Analog_Sensors.Port6.Value);
		DB.Analog_Sensors.Port7.Value = Const(DB.Analog_Sensors.Port7.Value);
		DB.Analog_Sensors.Port8.Value = Const(DB.Analog_Sensors.Port8.Value);

		//Set status byte
		DB.Analog_Sensors.Port5.Status = 0x01;
		DB.Analog_Sensors.Port6.Status = 0x01;
		DB.Analog_Sensors.Port7.Status = 0x01;
		DB.Analog_Sensors.Port8.Status = 0x01;
	}
}

//--------------------------------------



//--------------------------------------
//--------- Constrain a value ----------
//--------------------------------------
int16_t Const(int16_t temp){
	int32_t temphold;
	temphold = temp;
	return int16_t(constrain(temphold,0,32768));
}
//--------------------------------------


//--------------------------------------
//Read Datablock from PLC
//--------------------------------------

void Read_DB(){
	int DBNum = Config.PLC_DB;
	int Index,Result;
	void *Target;
	Target = NULL; // Uses the internal Buffer (PDU.DATA[])

//--------------------------------------
//---------- Read Heartbeat ------------
//--------------------------------------

	//Check lost connection
	if (Error != 0) {
		return;
	}

	//Heartbeat mirrored and inverted in arduino. Mirrored in PLC
	Serial.println();
	Serial.println("Job Read_Heartbeat start");
	MarkTime();
	Result=Client.ReadArea(S7AreaDB, // We are requesting DB access
						 DBNum,    // DB Number
						 0,        // Start from byte N.0
						 1,     // We need "Size" bytes
						 Target);
	if (Result==0) {
		ShowTime();
		DB.Heartbeat_From_PLC=S7.BitAt(0,1);
	}
	else {
		CheckError(Result);
		//Check lost connection
		if (Error != 0) {
			return;
		}
	}
//--------------------------------------


//--------------------------------------
//------------ Read Relays -------------
//--------------------------------------
	Serial.println("Job Read_Relays start");
	MarkTime();
	Result=Client.ReadArea(S7AreaDB, // We are requesting DB access
						 DBNum,    // DB Number
						 276,        // Start from byte N.0
						 1,     // We need "Size" bytes
						 Target);  // Put them into our target (Buffer or PDU)

	if (Result==0)
	{
		ShowTime();
		DB.Relay[0]=S7.BitAt(0,0);
		DB.Relay[1]=S7.BitAt(0,1);
		DB.Relay[2]=S7.BitAt(0,2);
		DB.Relay[3]=S7.BitAt(0,3);

		//Outputs enabled
		DB.Relay[4]=S7.BitAt(0,4);

		Serial.print("Relay0: ");
		Serial.println(DB.Relay[0]);
		Serial.print("Relay1: ");
		Serial.println(DB.Relay[1]);
		Serial.print("Relay2: ");
		Serial.println(DB.Relay[2]);
		Serial.print("Relay3: ");
		Serial.println(DB.Relay[3]);
		Serial.print("Outputs Enabled: ");
		Serial.println(OutputsEnable);
	}
	else {
		CheckError(Result);
		//Check lost connection
		if (Error != 0) {
			return;
		}
	}
//--------------------------------------


//--------------------------------------
//----- Read Configured addresses ------
//--------------------------------------

	//Part1
	Serial.println("Job Read Configured Addresses Part1 start");
	MarkTime();
	Result=Client.ReadArea(S7AreaDB, // We are requesting DB access
						 DBNum,    // DB Number
						 406,        // Start from byte N.0
						 64,     // We need "Size" bytes
						 Target);  // Put them into our target (Buffer or PDU)

	if (Result==0)
	{
		ShowTime();

		Index=0;
		for(int ix = 0; ix < 4; ix++){
			for(int idx = 0; idx < 8; idx++){
				DB.Config.Configured.Port1[ix].Address[idx]=S7.ByteAt(Index);
				Index++;
			}
		}
		for(int ix = 0; ix < 4; ix++){
			for(int idx = 0; idx < 8; idx++){
				DB.Config.Configured.Port2[ix].Address[idx]=S7.ByteAt(Index);
				Index++;
			}
		}
	}
	else {
		CheckError(Result);
		//Check lost connection
		if (Error != 0) {
			return;
		}
	}

	//Part2
	Serial.println("Job Read Configured Addresses Part2 start");
	MarkTime();
	Result=Client.ReadArea(S7AreaDB, // We are requesting DB access
						 DBNum,    // DB Number
						 470,        // Start from byte N.0
						 64,     // We need "Size" bytes
						 Target);  // Put them into our target (Buffer or PDU)

	if (Result==0)
	{
		ShowTime();

		Index=0;
		for(int ix = 0; ix < 4; ix++){
			for(int idx = 0; idx < 8; idx++){
				DB.Config.Configured.Port3[ix].Address[idx]=S7.ByteAt(Index);
				Index++;
			}
		}
		for(int ix = 0; ix < 4; ix++){
			for(int idx = 0; idx < 8; idx++){
				DB.Config.Configured.Port4[ix].Address[idx]=S7.ByteAt(Index);
				Index++;
			}
		}
	}
	else {
		CheckError(Result);
		//Check lost connection
		if (Error != 0) {
			return;
		}
	}
//--------------------------------------


}



//--------------------------------------
//Write Datablock to PLC
//--------------------------------------

void Write_DB(){
	int DBNum = Config.PLC_DB;
	int Index,Result;
	void *Target;
	byte Buffer[350]; //Temporary storage
	Target = &Buffer; // Pointer to Temporary storage

//--------------------------------------
//---------- Write Heartbeat -----------
//--------------------------------------

	//Heartbeat mirrored and inverted in arduino. Mirrored in PLC

	//Clear buffer
	memset(Buffer,0,sizeof(Buffer));

	//Check lost connection
	if (Error != 0) {
		return;
	}

	Serial.println("Job Write_Heartbeat start");
	//Prepare data in temporary buffer
	S7.SetBitAt(Target, 0, 0, DB.Heartbeat_To_PLC);
	S7.SetBitAt(Target, 0, 1, DB.Heartbeat_From_PLC);
	// Get the current tick
	MarkTime();
	Result=Client.WriteArea(S7AreaDB, // We are requesting DB access
						 DBNum,    // DB Number
						 0,        // Start from byte N.0
						 1,     // We need "Size" bytes
						 Target);
	if (Result==0) {
		ShowTime();
	}
	else {
		CheckError(Result);
		//Check lost connection
		if (Error != 0) {
			return;
		}
	}
//--------------------------------------


//--------------------------------------
//------- Write Digtal_Sensors ---------
//--------------------------------------
	//Clear buffer
	memset(Buffer,0,sizeof(Buffer));

	//Prepare Float values Port1
	Index = 0; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetFloatAt(Target, Index, DB.Digital_Sensors.Port1[ix].Value);
		Index = (Index + 16); //Add offset
	}

	//Prepare Float values Port2
	Index = 64; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetFloatAt(Target, Index, DB.Digital_Sensors.Port2[ix].Value);
		Index = (Index + 16); //Add offset
	}

	//Prepare Float values Port3
	Index = 128; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetFloatAt(Target, Index, DB.Digital_Sensors.Port3[ix].Value);
		Index = (Index + 16); //Add offset
	}

	//Prepare Float values Port4
	Index = 192; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetFloatAt(Target, Index, DB.Digital_Sensors.Port4[ix].Value);
		Index = (Index + 16); //Add offset
	}

	//Prepare Status Byte Port1
	Index = 4; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port1[ix].Status);
		Index = (Index + 16); //Add offset
	}

	//Prepare Status Byte Port2
	Index = 68; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port2[ix].Status);
		Index = (Index + 16); //Add offset
	}

	//Prepare Status Byte Port3
	Index = 132; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port3[ix].Status);
		Index = (Index + 16); //Add offset
	}

	//Prepare Status Byte Port4
	Index = 196; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port4[ix].Status);
		Index = (Index + 16); //Add offset
	}

	//Prepare Address Port1
	Index = 6; //Start offset
	for(int ix = 0; ix < 4; ix++){
		for(int idx = 0; idx < 8; idx++){
			S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port1[ix].Address[idx]);
			Index++;
		}
		Index = (Index + 8); //Add offset
	}

	//Prepare Address Port2
	Index = 70; //Start offset
	for(int ix = 0; ix < 4; ix++){
		for(int idx = 0; idx < 8; idx++){
			S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port2[ix].Address[idx]);
			Index++;
		}
		Index = (Index + 8); //Add offset
	}

	//Prepare Address Port3
	Index = 134; //Start offset
	for(int ix = 0; ix < 4; ix++){
		for(int idx = 0; idx < 8; idx++){
			S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port3[ix].Address[idx]);
			Index++;
		}
		Index = (Index + 8); //Add offset
	}

	//Prepare Address Port4
	Index = 198; //Start offset
	for(int ix = 0; ix < 4; ix++){
		for(int idx = 0; idx < 8; idx++){
			S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port4[ix].Address[idx]);
			Index++;
		}
		Index = (Index + 8); //Add offset
	}

	//Prepare Model Port1
	Index = 14; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port1[ix].Model);
		Index = (Index + 16); //Add offset
	}

	//Prepare Model Port2
	Index = 78; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port2[ix].Model);
		Index = (Index + 16); //Add offset
	}

	//Prepare Model Port3
	Index = 142; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port3[ix].Model);
		Index = (Index + 16); //Add offset
	}

	//Prepare Model Port4
	Index = 206; //Start offset
	for(int ix = 0; ix < 4; ix++){
		S7.SetByteAt(Target, Index, DB.Digital_Sensors.Port4[ix].Model);
		Index = (Index + 16); //Add offset
	}

	Serial.println("Job Write_Digital_Sensors start");
	MarkTime();
	Result=Client.WriteArea(S7AreaDB, // We are requesting DB access
						 DBNum,    // DB Number
						 2,        // Start from byte N.0
						 255,     // We need "Size" bytes
						 Target);  // Put them into our target (Buffer or PDU)
	if (Result==0) {
		ShowTime();
		Serial.println("Wrote Digital_Sensors");
	}
	else {
		CheckError(Result);
		//Check lost connection
		if (Error != 0) {
			return;
		}
	}
//--------------------------------------

//--------------------------------------
//-------- Write Analog_Sensors --------
//--------------------------------------
	//Clear buffer
	memset(Buffer,0,sizeof(Buffer));

	//Prepare Float value Port5
	S7.SetIntAt(Target, 0, DB.Analog_Sensors.Port5.Value);
	//Prepare Float value Port6
	S7.SetIntAt(Target, 4, DB.Analog_Sensors.Port6.Value);
	//Prepare Float value Port7
	S7.SetIntAt(Target, 8, DB.Analog_Sensors.Port7.Value);
	//Prepare Float value Port8
	S7.SetIntAt(Target, 12, DB.Analog_Sensors.Port8.Value);

	//Prepare Status value Port5
	S7.SetByteAt(Target, 2, DB.Analog_Sensors.Port5.Status);
	//Prepare Status value Port6
	S7.SetByteAt(Target, 6, DB.Analog_Sensors.Port6.Status);
	//Prepare Status value Port7
	S7.SetByteAt(Target, 10, DB.Analog_Sensors.Port7.Status);
	//Prepare Status value Port8
	S7.SetByteAt(Target, 14, DB.Analog_Sensors.Port8.Status);

	//Prepare Model value Port5
	S7.SetByteAt(Target, 3, DB.Analog_Sensors.Port5.Model);
	//Prepare Model value Port6
	S7.SetByteAt(Target, 7, DB.Analog_Sensors.Port6.Model);
	//Prepare Model value Port7
	S7.SetByteAt(Target, 11, DB.Analog_Sensors.Port7.Model);
	//Prepare Model value Port8
	S7.SetByteAt(Target, 15, DB.Analog_Sensors.Port8.Model);


	Serial.println("Job Write_Analog_Sensors start");
	MarkTime();


	Result=Client.WriteArea(S7AreaDB, // We are requesting DB access
						 DBNum,    // DB Number
						 258,        // Start from byte N.0
						 16,     // We need "Size" bytes
						 Buffer);  // Put them into our target (Buffer or PDU)
	if (Result==0) {
		ShowTime();
		Serial.println("Wrote Analog_Sensors");
	}
	else {
		CheckError(Result);
		//Check lost connection
		if (Error != 0) {
			return;
		}
	}
//--------------------------------------

//--------------------------------------
//------- Write Digital Inputs ---------
//--------------------------------------
	//Clear buffer
	memset(Buffer,0,sizeof(Buffer));

	//Prepare Digital_Input 1
	S7.SetBitAt(Target, 0, 0, DB.Digital_Inputs[0]);
	//Prepare Digital_Input 2
	S7.SetBitAt(Target, 0, 1, DB.Digital_Inputs[1]);
	//Prepare Digital_Input 3
	S7.SetBitAt(Target, 0, 2, DB.Digital_Inputs[2]);
	//Prepare Digital_Input 4
	S7.SetBitAt(Target, 0, 3, DB.Digital_Inputs[3]);

	Serial.println("Job Write_Digital_Inputs start");
	MarkTime();
	Result=Client.WriteArea(S7AreaDB, // We are requesting DB access
						 DBNum,    // DB Number
						 274,        // Start from byte N.0
						 1,     // We need "Size" bytes
						 Buffer);  // Put them into our target (Buffer or PDU)
	if (Result==0) {
		ShowTime();
		Serial.println("Wrote Digital_Inputs");
	}
	else {
		CheckError(Result);
		//Check lost connection
		if (Error != 0) {
			return;
		}
	}

//--------------------------------------


//--------------------------------------
//----- Write Detected addresses -----
//--------------------------------------
	//Clear buffer
	memset(Buffer,0,sizeof(Buffer));

	//Prepare Detected Addresses Port1
	Index = 0; //Start offset
	for(int ix = 0; ix < 4; ix++){
		for(int idx = 0; idx < 8; idx++){
			S7.SetByteAt(Target, Index, DB.Config.Detected.Port1[ix].Address[idx]);
			Index++;
		}
	}
	//Prepare Detected Addresses Port2
	for(int ix = 0; ix < 4; ix++){
		for(int idx = 0; idx < 8; idx++){
			S7.SetByteAt(Target, Index, DB.Config.Detected.Port2[ix].Address[idx]);
			Index++;
		}
	}
	//Prepare Detected Addresses Port3
	for(int ix = 0; ix < 4; ix++){
		for(int idx = 0; idx < 8; idx++){
			S7.SetByteAt(Target, Index, DB.Config.Detected.Port3[ix].Address[idx]);
			Index++;
		}
	}
	//Prepare Detected Addresses Port4
	for(int ix = 0; ix < 4; ix++){
		for(int idx = 0; idx < 8; idx++){
			S7.SetByteAt(Target, Index, DB.Config.Detected.Port4[ix].Address[idx]);
			Index++;
		}
	}

	Serial.println("Job Write Detected Addresses start");
	MarkTime();
	Result=Client.WriteArea(S7AreaDB, // We are requesting DB access
						 DBNum,    // DB Number
						 278,        // Start from byte N.0
						 128,     // We need "Size" bytes
						 Buffer);  // Put them into our target (Buffer or PDU)
	if (Result==0) {
		ShowTime();
		Serial.println("Wrote Detected Addresses");
	}
	else {
		CheckError(Result);
		//Check lost connection
		if (Error != 0) {
			return;
		}
	}
//--------------------------------------

}

//--------------------------------------
// Main Loop
//--------------------------------------
void loop() 
{
	unsigned long currentMillis = millis();

    //Connnect to PLC if not connected
    if (!Client.Connected)
    {
  	  Serial.println("Not connected");
      if (!Connect())
        delay(500);
    }

    // Only try to communicate if we are connected
	if (Client.Connected){
	  //Communicate with PLC if it's time
		if (currentMillis - previousMillis_PLC >= PLC_COM_D) {
			previousMillis_PLC = currentMillis;
			Read_DB();
			//Mirror Heartbeat
			//Heartbeat mirrored and inverted in arduino. Mirrored in PLC
			DB.Heartbeat_To_PLC = !DB.Heartbeat_From_PLC;
			Write_DB();
		}
	}


  //Heartbeat watch and outputsenable
  Heartbeat_Watchdog(DB.Heartbeat_From_PLC);
  MirrorOutputsEnable();

  //Communicate with Sensors and I/O's if it's time
  if (currentMillis - previousMillis_PRC >= PRC_D) {
      previousMillis_PRC = currentMillis;
	  UpdateDigitalSensors();
	  UpdateConfigured();
	  CheckIO();
	  CheckAnalog();
    }

  //Use onboard LED for indication of heartbeat
  digitalWrite(LED_BUILTIN,DB.Heartbeat_From_PLC);

  //Check for webserver client
  server.handleClient();

}
