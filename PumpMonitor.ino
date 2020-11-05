
/*
  Pump Monitor Sensor/Relay Node

  2020-08-26 C. Collins Created
  2020-09-17 C. Collins Added hysteresis logic
  2020-10-09 C. Collins added command via MQTT logic

  Todo Log:

  - hysteresis delay is currently a blocking delay() statement. Revise to be non-blocking.
  - why different pressure readings displayed by checkSetPoints depending on setPointType?
  - move all calibration fields to config file
  - Implement MQTT config command acceptanace
      - just send full file, process with loadConfig logic. Edit as text/json file. Build app that just reads file and sends over MQTT as json msg.
  - complete & test alarm processing
  - data logging via MQTT to manager node
    - normal messages
    - relay closed times
    - duty cycle
    ...

*/

#include <cscNetServices.h>
#include <Pumpmon.h>
#include <Adafruit_ADS1015.h>

#define PADJUST 4.2
#define ALLPUMPS 255
#define ALLRELAYS 255

auto timer = timer_create_default();
long unsigned int chipID = 0;
Adafruit_ADS1115 ads;

void printTimestamp()
{
  Serial.print(timeClient.getFormattedTime());
}

void openRelays(int r)
{
  if (r == ALLRELAYS)
  {
    for (int i = 0; i < numRelays; i++)
    {
      digitalWrite(relays[i].pin, LOW);
      relays[i].closed = false;
      msgn = sprintf(msgbuff, "\nRelay % i on pin % i opened", i, relays[i].pin);
      Serial.print(msgbuff);
    }
  }
  else
  {
    digitalWrite(relays[r].pin, LOW);
    relays[r].closed = false;
    msgn = sprintf(msgbuff, "\nRelay % i on pin % i opened", r, relays[r].pin);
    Serial.print(msgbuff);
  }
}

void handleAlarm(std::uint8_t alarmType, int sensor)
{
  globalAlarmFlag = true;

  switch (alarmType) {
    case HIPALARM :
      pressureSensors[sensor].alarmFlags = pressureSensors[sensor].alarmFlags | HIPALARM;
      break;

    case LOPALARM :
      pressureSensors[sensor].alarmFlags = pressureSensors[sensor].alarmFlags | LOPALARM;
      break;

    case PSENSORFAULT :
      pressureSensors[sensor].alarmFlags = pressureSensors[sensor].alarmFlags | PSENSORFAULT;
      break;

    case RELAYFAULT :
      pressureSensors[sensor].alarmFlags = pressureSensors[sensor].alarmFlags | RELAYFAULT;
      break;
  }
}

void clearAlarm(std::uint8_t alarmType, int sensor)
{
  int a = -1;
  std::uint8_t alarmCheck = CLEARALARMS;

  a = mapAlarm(alarmType);
  if (a >= 0)
  {
    pressureSensors[sensor].alarmFlags = pressureSensors[sensor].alarmFlags & (~alarmType); // clear specific alarm flag
  }
  else
  {
    Serial.println();
    printTimestamp();
    Serial.print("Error mapping alarm type to error name in clearAlarm");
  }

  for (int i = 0; i < numSensors; i++)
  {
    alarmCheck = alarmCheck | pressureSensors[i].alarmFlags;
  }

  if (!alarmCheck)
  {
    globalAlarmFlag = false;
  }
}

float calibrateSensor(int sensor)
{
  int samples = 50;
  float sampleSum = 0;
  int discardTime = 10000;  // how long to ignore initial samples

  msgn = snprintf(msgbuff, MSGBUFFLEN, "\nCalibrating pressure sensor %i, discard time = %i,  samples = %i", sensor, discardTime, samples);
  Serial.println(msgbuff);
  Serial.println();

  delay(discardTime);

  for (int i = 0; i < samples; i++)
  {
    sampleSum += analogRead(pressureSensors[sensor].pin);
    delay(1000);
  }

  msgn = snprintf(msgbuff, MSGBUFFLEN, "Calibration results: sampleSum = %f, samples = %i", sampleSum, samples);
  outputMsg(msgbuff);
  sampleSum /= samples;
  pressureSensors[sensor].offset = sampleSum;
  return (sampleSum);
}

void relayAction(int relay, int action)
{
  Serial.println("\nin relayAction");

  if (action == 0)
  {
    digitalWrite(relays[relay].pin, LOW);
    relays[relay].closed = false;
  }

  if (action == 1)
  {
    if (!relays[relay].stopPump)
    {
      digitalWrite(relays[relay].pin, HIGH);
      relays[relay].closed = true;
    }
    else Serial.println("stopPump = true, relay close request ignored");

    msgn = sprintf(msgbuff, "Relay %i closed = %i", relay, relays[relay].closed);
    outputMsg(msgbuff);
  }
}


float readPressure(unsigned int sensor)  // broke out separately so could be called w/out recursion
{
  pressureSensors[sensor].rawV = ads.readADC_SingleEnded(sensor);
  msgn = snprintf(msgbuff, MSGBUFFLEN, "pressureSensors[sensor].rawV = %i", pressureSensors[sensor].rawV);
  outputMsg(msgbuff);
  pressureSensors[sensor].currentP = (((pressureSensors[sensor].rawV - pressureSensors[sensor].offset) * (pressureSensors[sensor].maxP / 65535)) * PADJUST);
  msgn = snprintf(msgbuff, MSGBUFFLEN, "pressureSensors[sensor].currentP = %i", pressureSensors[sensor].currentP);
  outputMsg(msgbuff);
}

bool hysteresisMet(int sensor, int setPoint)
{
  bool hystMet = false;
  int hcount = 0;

  for (int i = 0; i < pressureSensors[sensor].setPoints[setPoint].hystCount; i++)
  {
    readPressure(sensor);
    switch (pressureSensors[sensor].setPoints[setPoint].setPType)
    {
      case 0:  // cut-out
        {
          if (pressureSensors[sensor].currentP >= pressureSensors[sensor].setPoints[setPoint].setP) hcount++;
          break;
        }

      case 1:  // cut-in
        {
          if (pressureSensors[sensor].currentP < pressureSensors[sensor].setPoints[setPoint].setP) hcount++;
          break;
        }

      default: Serial.println("\nERROR: Invalid set point type in checkSetPoints");
    }

    delay(pressureSensors[sensor].setPoints[setPoint].hystSampleDelay);  // change to non-blocking delay
  }

  if (hcount >= pressureSensors[sensor].setPoints[setPoint].hystCount) {
    hystMet = true;
  }
  msgn = sprintf(msgbuff, "\nHcount = %i, hystmet = %i", hcount, hystMet);
  Serial.println(msgbuff);
  return (hystMet);
}

void checkSetPoints(int sensor)
{
  Serial.print("\nIn checkSetPoints(), ");
  Serial.print("sensor = ");
  Serial.println(sensor);
  Serial.print("pressureSensors[sensor].numSetPoints = ");
  Serial.println(pressureSensors[sensor].numSetPoints);

  for (int i = 0; i < pressureSensors[sensor].numSetPoints; i++)
  {
    Serial.println("\ncheckSetPoints(): ");
    msgn = sprintf(msgbuff, "\npressureSensors[%i].setPoints[%i].setPType = % i, pressureSensors[sensor].currentP = % f, pressureSensors[sensor].setPoints[i].setP = % f", sensor, i, pressureSensors[sensor].setPoints[i].setPType, pressureSensors[sensor].currentP, pressureSensors[sensor].setPoints[i].setP);
    Serial.println(msgbuff);

    switch (pressureSensors[sensor].setPoints[i].setPType)
    {
      case 0:  // cut-out
        {
          if (pressureSensors[sensor].currentP >= pressureSensors[sensor].setPoints[i].setP)
          {
            if (pressureSensors[sensor].setPoints[i].hystComp)
            {
              if (hysteresisMet(sensor, i)) relayAction(pressureSensors[sensor].setPoints[i].relayNumber, 0);
            }
            else relayAction(pressureSensors[sensor].setPoints[i].relayNumber, 0);
          }

          break;
        }

      case 1:  // cut-in
        {
          if (pressureSensors[sensor].currentP < pressureSensors[sensor].setPoints[i].setP)
          {
            if (pressureSensors[sensor].setPoints[i].hystComp)
            {
              if (hysteresisMet(sensor, i)) relayAction(pressureSensors[sensor].setPoints[i].relayNumber, 1);
            }
            else relayAction(pressureSensors[sensor].setPoints[i].relayNumber, 1);
          }

          break;
        }

      default: Serial.println("\nERROR: Invalid set point type in checkSetPoints");
    }
  }
}

void checkSensors()
{
  digitalWrite(LED_BUILTIN, LOW);

  for (int i = 0; i < numSensors; i++)
  {
    readPressure(i);
    msgn = sprintf(msgbuff, "\npressureSensors[i].rawV = % f,  pressureSensors[i].currentP = % f",  pressureSensors[i].rawV,  pressureSensors[i].currentP);
    Serial.print(msgbuff);
    checkSetPoints(i);
  }

  digitalWrite(LED_BUILTIN, HIGH);
}


void sendSensorData()
{
  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on (Note that LOW is the voltage level

  sensormsg["node"] = chipID;
  sensormsg["msgtype"] = "P";

  for (int i = 0; i < numSensors; i++)
  {
    sensormsg["sensors"][i]["snum"] = i + startingSensorNum;
    sensormsg["sensors"][i]["rawV"] = pressureSensors[i].rawV;
    sensormsg["sensors"][i]["pressure"] = pressureSensors[i].currentP;
  }

  for (int i = 0; i < numRelays; i++)
  {
    sensormsg["relays"][i]["rnum"] = i + startingRelayNum;
    sensormsg["relays"][i]["closed"] = relays[i].closed;

  }

  if (debug)
  {
    Serial.print("\nsensormsg = ");
    serializeJson(sensormsg, Serial);
    displaySensorData();
  }

  serializeJson(sensormsg, msgbuff);
  if (!mqttClient.publish(mqttTopicData, msgbuff)) Serial.print("MQTT message publish failed!");

  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
}

bool handleCheckSensorsTimer(void *)
{
  checkSensors();
  return true;
}

bool handleSendDataTimer(void *)
{
  sendSensorData();
  return true;
}

bool handleLEDTimer(void *)
{
  if (ledOn)
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }

  ledOn = !ledOn;
  return (true);
}

void displaySensorData()
{
  if (debug)
  {
    Serial.println("\nIn displaySensorData");
  }

  for (int i = 0; i < numSensors; i++)
  {

    Serial.println();
    printTimestamp();
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.println();
    Serial.print("Pressure ");
    Serial.print(pressureSensors[i].currentP);
  }
}

void testRelays(int relay) {

  for (int r = 0; r < 3; r++)
  {
    openRelays(relay);
    if (relay == ALLRELAYS)
    {
      for (int i = 0; i < numRelays; i++)
      {
        relayAction(i, 1);
      }
    }
    else
    {
      relayAction(relay, 1);
    }
    delay(3000);
  }
  openRelays(relay);
}

void acceptConfigFile()
{
  Serial.println("In acceptConfigFile()");
}

void handleMQTTmsg(char* mqtt_topic, byte * payload, unsigned int length)
{
  const char* cmd = "";
  const char* msgtype = "";
  long int targetNode = -1;
  int pumpNum = -1;
  int sensor = -1;
  int relay = -1;

  if (debug)
  {
    msgn = snprintf(msgbuff, MSGBUFFSIZE, "Message arrived, topic [%s]", mqtt_topic);
    outputMsg(msgbuff);
    for (int i = 0; i < length; i++)
    {
      Serial.print((char)payload[i]);
    }
    Serial.println();
  }

  deserializeJson(pumpmsg, payload, MAXJSONSIZE);

  msgtype = pumpmsg["msgtype"];
  Serial.print("\nmsgtype = "); Serial.println(msgtype);

  if (msgtype[0] == 'C')                             // if command msg
  {
    targetNode = pumpmsg["targetnode"];  Serial.print("\ntargetNode = "); Serial.println(targetNode);

    if ((targetNode == chipID) || (targetNode == ALLNODES))
    {
      cmd = pumpmsg["cmd"];
      switch (cmd[0]) {
        case 'B':  // reboot
          {
            Serial.println("Reboot command received, rebooting...");
            ESP.restart();
          }

        case 'C':  // accept new config file
          {
            Serial.println("Accept config file command received, responding...");
            acceptConfigFile();
            break;
          }

        case 'R':  // resume
          {
            Serial.println("Resumecommand received, clearing stop pump flag(s)");
            pumpNum = pumpmsg["pumpnum"];

            if (pumpNum == ALLPUMPS)
            {
              for (int i = 0; i < numRelays; i++) relays[i].stopPump = false;
            }
            else
            {
              if (pumpNum <= numRelays) relays[pumpNum].stopPump = false;
            }
            break;
          }

        case 'S':   // stop
          {
            Serial.println("Stop command received, setting stop pump flag(s)");

            pumpNum = pumpmsg["pumpnum"];
            Serial.print("\nStop pump = "); Serial.println(pumpNum);

            if (pumpNum == ALLPUMPS)
            {
              Serial.println("\nALLPUMPS Stop");
              for (int i = 0; i < numRelays; i++) relays[i].stopPump = true;
              openRelays(ALLRELAYS);
            }
            else
            {
              if (pumpNum <= numRelays)
              {
                relays[pumpNum].stopPump = true;
                openRelays(pumpNum);
              }
            }
            break;
          }

        case 'Q':  // query
          {
            Serial.println("Query command received, responding...");
            sendSensorData();
            break;
          }

        case 'L':  // calibrate sensor
          {
            Serial.print("\nCalibrate command received, calibrating ");
            sensor = pumpmsg["sensor"];
            if ((sensor < 0) || (sensor > numSensors))
            {
              Serial.println("\Invalid sensor number received");
            }
            else
            {
              Serial.print("sensor ");
              Serial.println(sensor);
              calibrateSensor(sensor);
            }
            break;
          }
        case 'T':  // test relays
          {
            outputMsg("Test relays command received, cycling relay(s)....");
            relay = pumpmsg["relay"];
            testRelays(relay);
            break;
          }

        default:
          {
            outputMsg("Invalid command message received, ignored");
          }
      }
    }
    else Serial.println("\nTargetNode in cmd != chipID or ALLNODES, command ignored");
  }
  else Serial.println("\nInvalid message type received");
}


void displayStartHeader()
{
  Serial.println();  Serial.println();
  Serial.print(nodeName);
  Serial.print(" Pump Monitor Sensor Node");
  Serial.print("  Build: ");
  Serial.print(__DATE__);
  Serial.print(" ");
  Serial.print(__TIME__);
  Serial.println();
}


void initRelays()
{
  for (int i = 0; i < numRelays; i++)
  {
    pinMode(relays[i].pin, OUTPUT);
    openRelays(i);
  }
}

void initSensors()
{
  ads.begin();
}

void setup()
{
  delay(5000);                                     // Initial delay to allow intervention
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  chipID = ESP.getChipId();
  msgn = sprintf(nodeName, "SNPM-ESP8266-% X", chipID); // set node name, SNPM (Sensor Node Pump Mon),
  displayStartHeader();

  if (!loadConfig())
  {
    Serial.println("Failed to load configuration file. Halting.");
    while (true);

  };

  initSensors();
  initRelays();
  checkSensors();                                    // get initial readings

  if (network)
  {
    connectWiFi();

    while (!setupNTP(timeZone)) {
      delay(5000);
    };
    while (!setupMdns(nodeName)) {
      delay(5000);
    };
    hostEntry = -1;
    while (hostEntry == -1) {
      hostEntry = findService("_mqtt", "_tcp");
    };
    while (!setupMQTT(MDNS.IP(hostEntry), MDNS.port(hostEntry), true, mqttTopicCtrl, handleMQTTmsg)) {
      Serial.println("Connecting to MQTT...");
      delay(5000);
    };

    timer.every(SENDDATADELAY, handleSendDataTimer);
  }


  timer.every(sensorCheckDelay, handleCheckSensorsTimer);

  timer.every(LEDFLASHTIME, handleLEDTimer);



}

void loop() {

  timer.tick();
  timeClient.update();

  if (network)
  {
    if (!mqttClient.connected()) connectMQTT(true, mqttTopicCtrl, MDNS.IP(hostEntry));
    mqttClient.loop();
  }
}
