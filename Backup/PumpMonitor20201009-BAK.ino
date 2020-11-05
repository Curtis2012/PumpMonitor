
/*
  Pump Monitor Sensor/Relay Node

  2020-08-26 C. Collins Created
  2020-09-17 C. Collins Added hysteresis logic

  Todo Log:

  - hysteresis delay is currently a blocking delay() statement. Revise to be non-blocking.
  - why different pressure readings displayed by checkSetPoints depending on setPointType?
  - move all calibration fields to config file
  - Implement MQTT command acceptanace
    - reboot
    - STOP...open all relays and just wait for next command
    - accept config file updates via MQTT
      - just send full file, its < 500 bytes, process with loadConfig logic. Edit as text/json file. Build app that just reads file and sends over MQTT as json msg.
  - complete & test alarm processing
  - data logging via MQTT to manager node
    - normal messages
    - relay closed times
    - duty cycle
    ...
    


*/

#include <cscNetServices.h>
#include <Pumpmon.h>
ADC_MODE(ADC_TOUT);

#define PADJUST 4.2

auto timer = timer_create_default();

void printTimestamp()
{
  Serial.print(timeClient.getFormattedTime());
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

  for (int i = 0; i < NUMSENSORS; i++)
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

  msgn = (msgbuff, "\nCalibrating pressure sensor %i, discard time = %i,  samples = %i", sensor, discardTime, samples);
  Serial.println(msgbuff);
  Serial.println();

  delay(discardTime);

  for (int i = 0; i < samples; i++)
  {
    sampleSum += analogRead(pressureSensors[sensor].pin);
    delay(1000);
  }

  Serial.println("\nCalibration results: ");
  msgn = (msgbuff, "\nsampleSum = %f, samples = %i", sampleSum, samples);
  Serial.println(msgbuff); Serial.println();
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
    digitalWrite(relays[relay].pin, HIGH);
    relays[relay].closed = true;
  }

  msgn = sprintf(msgbuff, "\nRelay %i closed = %i", relay, relays[relay].closed);
  Serial.println(msgbuff);
}

float readPressure(unsigned int sensor)  // broke out separately so could be called w/out recursion
{
  pressureSensors[sensor].rawV = analogRead(pressureSensors[sensor].pin);
  pressureSensors[sensor].currentP = (((pressureSensors[sensor].rawV - pressureSensors[sensor].offset) * (pressureSensors[sensor].maxP / 1023)) * PADJUST);
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

    delay(pressureSensors[sensor].setPoints[setPoint].hystSampleDelay);
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
    msgn = sprintf(msgbuff, "\npressureSensors[sensor].setPoints[i].setPType = % i, pressureSensors[sensor].currentP = % f, pressureSensors[sensor].setPoints[i].setP = % f", pressureSensors[sensor].setPoints[i].setPType, pressureSensors[sensor].currentP, pressureSensors[sensor].setPoints[i].setP);
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

  for (int i = 0; i < NUMSENSORS; i++)
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

  sensormsg["node"] = ESP.getChipId();

  for (int i = 0; i < NUMSENSORS; i++)
  {
    sensormsg["sensors"][i]["snum"] = i;
    sensormsg["sensors"][i]["rawV"] = pressureSensors[i].rawV;
    sensormsg["sensors"][i]["pressure"] = pressureSensors[i].currentP;
  }

  for (int i = 0; i < NUMRELAYS; i++)
  {
    sensormsg["relays"][i]["rnum"] = i;
    sensormsg["relays"][i]["closed"] = relays[i].closed;

  }

  if (debug)
  {
    Serial.print("\nmsgbuff = (");
    Serial.print(msgbuff);
    Serial.println(")");
    serializeJson(sensormsg, Serial);
    displaySensorData();
  }

  serializeJson(sensormsg, msgbuff);
  mqttClient.publish(mqttTopic, msgbuff);

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

  for (int i = 0; i < NUMSENSORS; i++)
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

void subscribeToMQTTControlMessages();
{
  
}

void handleMQTTmsg(char* mqtt_topic, byte * payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(mqtt_topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
    Serial.println();

/*
 * pcode:
 * 
 * case msg-type 
 * 
 * "R": reboot
 * "C": accept-config-file
 * "Q": send status/config
 * ...
 * 
 */

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
  for (int i = 0; i < NUMRELAYS; i++)
  {
    pinMode(relays[i].pin, OUTPUT);
    digitalWrite(relays[i].pin, LOW);
    relays[i].closed = false;
    msgn = sprintf(msgbuff, "\nRelay % i on pin % i initilized and opened", i, relays[i].pin);
    Serial.print(msgbuff);
  }
}

void initSensors()
{
  for (int i = 0; i < NUMSENSORS; i++)
  {
    //pinMode(pressureSensors[i].pin, INPUT);
    msgn = sprintf(msgbuff, "\nSensor % i pin % i mode set to INPUT", i, pressureSensors[i].pin);
    Serial.print(msgbuff);
  }
}

void setup()
{
  delay(5000);                                     // Initial delay to allow intervention
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  msgn = sprintf(nodeName, "SNPM - ESP8266 - % X", ESP.getChipId());  // set node name, SNPM (Sensor Node Pump Mon),
  displayStartHeader();

  if (!loadConfig())
  {
    Serial.println("Failed to load configuration file. Halting.");
    while (true);

  };

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
    while (!setupMQTT(MDNS.IP(hostEntry), MDNS.port(hostEntry), false, handleMQTTmsg)) {
      delay(5000);
    };

    subscribeToMQTTControlMessages();

    timer.every(SENDDATADELAY, handleSendDataTimer);
  }


  timer.every(sensorCheckDelay, handleCheckSensorsTimer);

  timer.every(LEDFLASHTIME, handleLEDTimer);

  initSensors();

  // Serial.print(calibrateSensor(0)); // ...set up method of invoking calibration in the future...

  initRelays();
  checkSensors();                                    // get initial readings

}

void loop() {

  timer.tick();
  timeClient.update();

  if (network)
  {
    if (!mqttClient.connected()) connectMQTT(false, MDNS.IP(hostEntry));
    mqttClient.loop();
  }
}
