//
// pumpmon.h
//
// Contains common functions/definitions used by both sensor node and manager node.
//
// All site specific parameters should go here also so they can be modified in one place.
//
//
/*
 * 2020-08-26 C. Collins Created
 * 2020-09-17 C. Collins hysteresis related changes made
 */

#include "FS.h"
#include <ArduinoJson.h>
#include <TimeLib.h>
#include "timer.h"          // by Michael Contreras

#define PUMPMONCFGFILE  "/pumpmoncfg.json"
#define JSONCONFIGDOCSIZE  3500
#define MAXJSONSIZE 3500
#define MAXPAYLOADSIZE 3500

byte configBuff[JSONCONFIGDOCSIZE];
File configFile;
StaticJsonDocument<JSONCONFIGDOCSIZE> configDoc;
StaticJsonDocument<MAXJSONSIZE> pumpmsg;

int startingSensorNum = -1;
int startingRelayNum = -1;


//
// Site Specific Items =========================================================================================================
//

#define SENDDATADELAY 30000

const char* sitename = " ";

bool dst = false;


// ==== End Site Specific Items ====


//
// Common items
//


//#define NUMSENSORS 1
//#define NUMRELAYS 1
#define MAXNUMSETPOINTS 8

#define MSGBUFFSIZE 3500
#define LEDFLASHTIME 3000

//bool debug = false;
bool ledOn = false;
bool network = false;

int numSensors = -1;
int numRelays = -1;

struct setPoint_t {
  float setP = 0;
  unsigned int setPType = 0; // 1 = cut-in, 0 = cut-out
  unsigned int relayNumber = 0;
  bool hystComp = false;                // hysteresis compensation yes or no
  unsigned int hystSamples = 0;        // hysteresis total smaples to take
  unsigned long hystSampleDelay = 0;   // delay betwee hysteresis samples
  unsigned int hystCount;              // number of hysteresis samples which must be inside setpoint range to yield a "true" result  
  unsigned int action = 0; // 0 = OFF, 1 = ON
};

struct pressureSensor_t {
  unsigned int pin = 0;
  float offset = 0;  // offset for calibrating zero psi
  uint16_t rawV = 0;    // current raw reading
  float minP = 0;    // minimum sensor design pressure
  float maxP = 0;    // maximum sensor design pressure
  float currentP = 0;  // current pressure reading
  unsigned int numSetPoints = -1;
  setPoint_t *setPoints;
  std::uint8_t alarmFlags = 0;
};

struct relay_t {
  unsigned int pin = 0;
  bool stopPump = false;
  bool closed = false;
  std::uint8_t alarmFlags = 0;
};

pressureSensor_t *pressureSensors;

//relay_t relays[NUMRELAYS];

relay_t *relays;


unsigned long int sensorCheckDelay  = 1000;
unsigned long int sendDataDelay = 5000;


// Alarm related bit masks & structs

#define CLEARALARMS    0b00000000
#define HIPALARM       0b00000001
#define LOPALARM       0b00000010
#define PSENSORFAULT   0b00000100
#define RELAYFAULT     0b00001000
#define NUMALARMS 5


struct alarm {
	std::uint8_t alarmType;
	char alarmName[15];
};

alarm alarms[NUMALARMS] = {
  {HIPALARM, "HIP"},
  {LOPALARM, "LOP"},
  {PSENSORFAULT, "PSENSORFAULT"},
  {RELAYFAULT, "RELAYFAULT"},
  {CLEARALARMS, "CLEARALL"}
};

bool globalAlarmFlag = false;

// JSON Message Definition

StaticJsonDocument<MAXJSONSIZE> sensormsg;

/*  JSON data Message Structure
  {
	"n"         // node name
	"p1...n"    // current pressure sensors readings
	"r1...n"    // current relays status
	"aF"        // alarm flags
  }
  
  /*  JSON control Message Structure
  {
	"n"         // node name
	"cmd"       // command (char)
	"cmdData"    // data for command
  }
  
*/


//
// Functions
//

bool loadConfig()
{
	DeserializationError jsonError;
	size_t size = 0;
	int t = 0;

	Serial.print("\nMounting file system...");
	if (!SPIFFS.begin())
	{
		Serial.println("\nFailed to mount file system");
		return false;
	}
	else Serial.print("\nMounted file system");

    Serial.print("\nOpening config file: ");
	Serial.print(PUMPMONCFGFILE);
	configFile = SPIFFS.open(PUMPMONCFGFILE, "r");
	if (!configFile)
	{
		Serial.print("\nFailed to open config file");
		if (!SPIFFS.exists(PUMPMONCFGFILE)) Serial.print("\nFile does not exist");
		return false;
	}
    else
	{
	size = configFile.size();
	Serial.print("\nFile size = ");
	Serial.println(size);
	};

	configFile.read(configBuff, size);

	jsonError = deserializeJson(configDoc, configBuff);
	if (jsonError)
	{
		Serial.println("\nFailed to parse config file");
		return false;
	}
	
	//Serial.flush();
    //serializeJsonPretty(configDoc, Serial);

	sitename = configDoc["site"]["sitename"];
	pssid = configDoc["site"]["pssid"];
	timeZone = configDoc["site"]["timezone"];
	network = configDoc["site"]["network"];
	dst = configDoc["site"]["dst"];
	ppwd = configDoc["site"]["ppwd"];
	wifiTryAlt = configDoc["site"]["usealtssid"];
	assid = configDoc["site"]["altssid"];
	apwd = configDoc["site"]["altpwd"];
	mqttTopicData = configDoc["site"]["mqtt_topic_data"];
	mqttTopicCtrl = configDoc["site"]["mqtt_topic_ctrl"];
	mqttUid = configDoc["site"]["mqtt_uid"];
	mqttPwd = configDoc["site"]["mqtt_pwd"];
	debug = configDoc["site"]["debug"];
	sensorCheckDelay = configDoc["site"]["sensorcheckdelay"];
    sendDataDelay = configDoc["site"]["senddatadelay"];
	numSensors = configDoc["site"]["numSensors"];
	numRelays = configDoc["site"]["numRelays"];
	
	startingSensorNum = configDoc["site"]["startingSensorNum"];
	startingRelayNum = configDoc["site"]["startingRelayNum"];

	pressureSensors = new pressureSensor_t[numSensors];
	relays = new relay_t[numRelays];
		

	for (int i = 0; i < numSensors; i++)
	{
	    pressureSensors[i].pin = configDoc["sensordefs"][i]["pin"];
		pressureSensors[i].minP = configDoc["sensordefs"][i]["minP"];
		pressureSensors[i].maxP = configDoc["sensordefs"][i]["maxP"];
		pressureSensors[i].offset = configDoc["sensordefs"][i]["offset"];
		pressureSensors[i].numSetPoints = configDoc["sensordefs"][i]["numsetpoints"];
		msgn = snprintf(msgbuff, MSGBUFFLEN, "\npressureSensors[%i].numSetPoints = %i", i, pressureSensors[i].numSetPoints);
		outputMsg(msgbuff);
		pressureSensors[i].setPoints = new setPoint_t[pressureSensors[i].numSetPoints];
		for (int s = 0; s < pressureSensors[i].numSetPoints; s++)
		{
			pressureSensors[i].setPoints[s].setP = configDoc["sensordefs"][i]["setpoints"][s]["setp"];
			msgn = snprintf(msgbuff, MSGBUFFLEN, "\npressureSensors[%i].setPoints[%i].setP = %f", i, s, pressureSensors[i].setPoints[s].setP);
			outputMsg(msgbuff);
			pressureSensors[i].setPoints[s].setPType = configDoc["sensordefs"][i]["setpoints"][s]["setptype"];
			pressureSensors[i].setPoints[s].hystComp = configDoc["sensordefs"][i]["setpoints"][s]["hystcomp"];
			Serial.print("\npressureSensors[i].setPoints[s].hystComp=");Serial.println(pressureSensors[i].setPoints[s].hystComp);			
			pressureSensors[i].setPoints[s].hystSamples = configDoc["sensordefs"][i]["setpoints"][s]["hystsamples"];
			pressureSensors[i].setPoints[s].hystSampleDelay = configDoc["sensordefs"][i]["setpoints"][s]["hystsampledelay"];
			pressureSensors[i].setPoints[s].hystCount = configDoc["sensordefs"][i]["setpoints"][s]["hystcount"];
			pressureSensors[i].setPoints[s].relayNumber = configDoc["sensordefs"][i]["setpoints"][s]["relay"];
			pressureSensors[i].setPoints[s].action = configDoc["sensordefs"][i]["setpoints"][s]["action"];	
		}
	}
	
	for (int i = 0; i < numRelays; i++)
	{
	    relays[i].pin = configDoc["relaydefs"][i]["pin"];
	}
	

	Serial.println("\nConfig file loaded");
	return true;
}


int mapAlarm(std::uint8_t alarmType)
{
	switch (alarmType) {
	case HIPALARM:
		return (0);

	case LOPALARM:
		return (1);

	case PSENSORFAULT:
		return (2);

	case RELAYFAULT:
		return (3);

	case CLEARALARMS:
		return (4);

	default: return (-1);

	}
}

