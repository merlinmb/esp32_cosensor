

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>

// Library to disable Brownout detector
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "connectionDetails.h"

#include <merlinNetwork.h>
#include <merlinUpdateWebServer.h>

#include <Wire.h>
#include <SensirionI2CScd4x.h>

#define MQTT_DEVICENAME "espCO2Sensor"

// ER-OLEDM032-1W is the 256x64 white pixels OLED display with adaptor board that simplifies your design,diagonal is only 3.2 inch.The controller ic SSD1322.
#include <u8g2lib.h>
#include <SPI.h>
U8G2_SSD1322_NHD_256X64_1_4W_HW_SPI _display(U8G2_R0, 5, 17, 16);

SensirionI2CScd4x scd4x;

#define DEBUG 1

#define CO_TRIGGER_PIN A0
#define UPDATE_READINGSDATA_INTERVAL_MILLISECS 300000 // Update every 5 minutes
#define UPDATE_DISPLAY_INTERVAL_MILLISECS 600000	  // Update every 5 minutes
#define LED_BLINK_INTERVAL_MILLISECS 10000			  // Update every 10 seconds

#define LARGEFONT u8g2_font_courR18_tf
#define MEDIUMFONT u8g2_font_5x7_tf
#define SMALLFONT u8g2_font_4x6_tf

long _delayCOMeterMillis = UPDATE_READINGSDATA_INTERVAL_MILLISECS;

uint16_t _CO2Val = 0;
float _temp = 0;
float _humidity = 0;
bool _forceUpdate = false;
bool _forceDisplayUpdate = false;

int _range, _min, _max;

long _currentAmbientCOLevel = 0;
String _mqttPostFix = "";

unsigned long _runCurrent;		 // set variable to time store
unsigned long _runCOCheck;		 // set variable to time store
unsigned long _runDisplayUpdate; // set variable to time store
unsigned long _runLEDBlink;		 // set variable to time store

#define MAXBRIGHTNESS 255
#define MINBRIGHTNESS 0
byte _brightness = 100;

String _lastReadingDateTime = "";

#define SCREENHEIGHT 64
#define SCREENWIDTH 256

#define CELLPADDING 4

#define COL1 CELLPADDING
#define COL2 COL1 + CELLPADDING
#define COL3 COL2 + 40
#define COL4 COL3 + 20
#define COL5 COL4 + CELLPADDING * 2
#define COL6 COL5 + 10
#define COL7 COL6 + CELLPADDING
#define COL8 SCREENWIDTH - CELLPADDING

#define ROW1 CELLPADDING
#define ROW2 ROW1 + CELLPADDING
#define ROW3 ROW2 + CELLPADDING
#define ROW4 ROW3 + CELLPADDING
#define ROW5 ROW4 + CELLPADDING * 2
#define ROW6 ROW5 + CELLPADDING
#define ROW7 SCREENHEIGHT / 2
#define ROW8 SCREENHEIGHT - CELLPADDING

#define NUMLASTVALUES 12 // COL8-COL1
#define SPARKLINEHEIGHT ROW8 - ROW7
float lastValues[NUMLASTVALUES];
int lastValuesSparkline[NUMLASTVALUES];

#define UPDATE_SIZE_UNKNOWN 0xFFFFFFFF

#define NTPTIMEOUTVAL 4500
const char *ntpServer = "pool.ntp.org";
const long timezoneOffset = 0; // 0-23
const long gmtOffset_sec = timezoneOffset * 60 * 60;
const int daylightOffset_sec = 0;
unsigned long _epochTime;

String _ip = "";

void setContrast(byte brightnessValue)
{
	DEBUG_PRINTLN("setContrast: " + String(brightnessValue));

	_display.setContrast(brightnessValue);
	if (brightnessValue == 0)
	{
		_display.setPowerSave(1);
	}
	else
	{
		_display.setPowerSave(0);
	}
}

void printUint16Hex(uint16_t value)
{
	DEBUG_PRINT(value < 4096 ? "0" : "");
	DEBUG_PRINT(value < 256 ? "0" : "");
	DEBUG_PRINT(value < 16 ? "0" : "");
	DEBUG_PRINT(value);
	DEBUG_PRINT(HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2)
{
	DEBUG_PRINT("Serial: 0x");
	printUint16Hex(serial0);
	printUint16Hex(serial1);
	printUint16Hex(serial2);
	DEBUG_PRINTLN();
}

void updateCOHistory()
{

	for (byte i = 0; i < NUMLASTVALUES - 1; i++)
	{
		lastValues[i] = lastValues[i + 1];
	}
	lastValues[NUMLASTVALUES - 1] = _CO2Val;

	/*
		for (byte i = 0; i < NUMLASTVALUES; i++)
		{
			lastValues[i] = random(0, 8000);
			DEBUG_PRINTLN("Bar [" + String(i)+"] = "+String(lastValues[i]));
		}
	/**/

	// recalc/normalise the sparkline values
	_min = 9999999, _max = -9999999;
	// find bounds:
	for (byte i = 0; i < NUMLASTVALUES; i++)
	{
		if (lastValues[i] > 0)
		{
			if (lastValues[i] < _min)
			{
				_min = lastValues[i];
			}
			if (lastValues[i] > _max)
			{
				_max = lastValues[i];
			}
		}
	}
	_range = abs(_max - _min);
	// normalise to within Height

	for (byte i = 0; i < NUMLASTVALUES; i++)
	{
		lastValuesSparkline[i] = map(lastValues[i], _min, _max, 0, SPARKLINEHEIGHT);
	}
}

bool getCOValue()
{
	DEBUG_PRINTLN("getCOValue()");

	uint16_t error;
	char errorMessage[256];
	// Read Measurement
	uint16_t co2 = 0;
	float temperature = 0.0f;
	float humidity = 0.0f;
	bool isDataReady = false;
	error = scd4x.getDataReadyFlag(isDataReady);
	if (error)
	{
		DEBUG_PRINT("Error trying to execute getDataReadyFlag(): ");
		errorToString(error, errorMessage, 256);
		DEBUG_PRINTLN(errorMessage);
		return false;
	}
	if (!isDataReady)
	{
		DEBUG_PRINTLN("Data not ready, skipping.");
		return false;
	}
	error = scd4x.readMeasurement(co2, temperature, humidity);
	if (error)
	{
		DEBUG_PRINT("Error trying to execute readMeasurement(): ");
		errorToString(error, errorMessage, 256);
		DEBUG_PRINTLN(errorMessage);
	}
	else if (co2 == 0)
	{
		DEBUG_PRINTLN("Invalid sample detected, skipping.");
	}
	else
	{
		DEBUG_PRINTLN("Valid sample detected.");
		_CO2Val = co2;
		_temp = temperature;
		_humidity = humidity;

		DEBUG_PRINT("Co2:");
		DEBUG_PRINT(co2);
		DEBUG_PRINT("\t");
		DEBUG_PRINT("Temperature:");
		DEBUG_PRINT(temperature);
		DEBUG_PRINT("\t");
		DEBUG_PRINT("Humidity:");
		DEBUG_PRINTLN(humidity);
		return true;
	}
	return false;
}
void updateLocalTime()
{
	struct tm timeinfo;

	if (!getLocalTime(&timeinfo))
	{
		return;
	}

	adjustBST(&timeinfo);

	strftime(timeHour, 3, "%H", &timeinfo);
	strftime(timeMin, 3, "%M", &timeinfo);
	strftime(timeSec, 3, "%S", &timeinfo);
	/*ú
	 strftime(timeWeekDay,10, "%A", &timeinfo);
	 strftime(timeday, 3, "%d", &timeinfo);
	 strftime(timemonth, 10, "%B", &timeinfo);
	 strftime(timeyear, 5, "%Y", &timeinfo);
	*/
}

void mqttTransmitCustomStat()
{
	mqttPublishStat(MQTT_DEVICENAME, "delay", String(_delayCOMeterMillis / 1000));
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
	DEBUG_PRINT("Message arrived [");
	DEBUG_PRINT(topic);
	DEBUG_PRINT("] ");

	char message_buff[MQTT_MAX_PACKET_SIZE];
	int i = 0;
	for (i = 0; i < length; i++)
	{
		message_buff[i] = payload[i];
	}
	message_buff[i] = '\0';
	String __mqttIncomingPayload = String(message_buff);
	__mqttIncomingPayload.trim();
	DEBUG_PRINTLN(__mqttIncomingPayload);

	String __mqttIncomingTopic = String(topic);
	__mqttIncomingTopic.trim();

	_lastMQTTMessage = __mqttIncomingTopic + " " + __mqttIncomingPayload;

	if (__mqttIncomingTopic == "cmnd/" + String(MQTT_DEVICENAME) + "/sendstat")
	{
		mqttTransmitInitStat(MQTT_DEVICENAME);
		return;
	}
	if (__mqttIncomingTopic == "cmnd/" + String(MQTT_DEVICENAME) + "/reset")
	{
		mqttPublishStat(MQTT_DEVICENAME, "restart", __mqttIncomingPayload);
		delay(1000);
		ESP.restart();
	}

	if (__mqttIncomingTopic == "cmnd/mcmddevices/brightness")
	{
		_brightness = __mqttIncomingPayload.toInt();

		int __newBrightness = __mqttIncomingPayload.toInt();

		if (__newBrightness < MINBRIGHTNESS)
			__newBrightness = MINBRIGHTNESS;
		if (__newBrightness > MAXBRIGHTNESS)
			__newBrightness = MAXBRIGHTNESS;

		float __contrastVal = MAXBRIGHTNESS * __newBrightness / 100;

		DEBUG_PRINTLN("Setting Brightness to: " + String(_brightness));
		setContrast(_brightness);
	}

	if (__mqttIncomingTopic == "cmnd/mcmddevices/brightnesspercentage")
	{
		_brightness = __mqttIncomingPayload.toInt();
		_brightness = map(_brightness, 0, 100, 0, 255);
		DEBUG_PRINTLN("Setting Brightness to: " + String(_brightness));
		setContrast(_brightness);
	}

	mqttHandleFriendlyCallback(MQTT_DEVICENAME, __mqttIncomingTopic, __mqttIncomingPayload);
}

void mqttTransmitSensorStat()
{
	mqttPublishStat(MQTT_DEVICENAME, "CO2Value", String(_CO2Val));
	mqttPublishStat(MQTT_DEVICENAME, "Temperature", String(_temp));
	mqttPublishStat(MQTT_DEVICENAME, "Humidity", String(_humidity));
}

/***************************************************
  MQTT
****************************************************/
void mqttTransmitCustomSubscribe() {}
void mqttCustomSubscribe() {}

void doSensorReadMQTTSend()
{
	DEBUG_PRINTLN("doSensorReadMQTTSend()");
	if (getCOValue())
	{
		DEBUG_PRINTLN("getCOValue() success");
		_lastReadingDateTime = _timeClient.getFormattedDate();
		_lastReadingDateTime = _lastReadingDateTime.substring(0, 16);
		_lastReadingDateTime.trim();
		_lastReadingDateTime.replace("T", " ");
		mqttTransmitInitStat(MQTT_DEVICENAME);
		mqttTransmitSensorStat();
		_forceDisplayUpdate = true;
	}
	else
	{
		DEBUG_PRINTLN("getCOValue() failed");
	}
}

void setupWebServer()
{
	DEBUG_PRINTLN("Handling Web Request...");

	_httpServer.on("/", []()
				   {
					   String __infoStr = "";
					   String __mqttConnectedStr = (_mqttClient.connected() ? "True" : "False");

					   __infoStr += "<div align=left><H1><i>" + String(MQTT_DEVICENAME) + "</i></H1>";
					   __infoStr += "<H2>CO<sub>2</sub> value: ";
					   __infoStr += String(_CO2Val);
					   __infoStr += " ppm</H2>";
					   __infoStr += "<H2>Temperature: ";
					   __infoStr += String(_temp);
					   __infoStr += "&deg;C</H2>";
					   __infoStr += "<H2>Humidity: ";
					   __infoStr += String(_humidity);
					   __infoStr += "%</H2>";
					   __infoStr += "Delay:";
					   __infoStr += String(_delayCOMeterMillis/1000,1);
					   __infoStr += " seconds<br>";
					   __infoStr += "Last read time: " + String(_lastReadingDateTime) + "<br>";
					   __infoStr += "<a href=\"/updatedisplay\">Update display</a><br>";
					   __infoStr += "<a href=\"/getraw\">Get raw reading</a><br>";
					   __infoStr += "<a href=\"/sendstat\">MQTT, re-read& send status</a><br>";

					   __infoStr += "<hr>";
					   __infoStr += "<br>Connected to : " + String(WIFI_ACCESSPOINT) + " (" + _rssiQualityPercentage + "%)";
					   __infoStr += "<br>IP Address: " + IpAddress2String(WiFi.localIP());
					   __infoStr += "<br>MAC Address: " + WiFi.macAddress();
					   __infoStr += "<br>MQTT Server: " + String(MQTT_SERVER_IP) + ":" + String(MQTT_SERVER_PORT);
					   __infoStr += "<br>MQTT Connected: " + __mqttConnectedStr;
					   __infoStr += "<br>Last MQTT message recieved: " + _lastMQTTMessage;
					   __infoStr += "<hr>";
					   __infoStr += "<a href='/defaults'>Reset to Defaults</a><br>";
					   __infoStr += "<a href='/reset'>Reboot Device</a><br>";
					   __infoStr += "<a href = \"/firmware\">Upgrade Firmware</a></div>";

					   String __retStr = loginIndex + __infoStr + loginIndex2;

					   _httpServer.send(200, "text/html", __retStr); });

				DEBUG_PRINTLN("Arg(0): " + _httpServer.argName(0));

				_httpServer.on("/sendstat", []()
				{
					doSensorReadMQTTSend();
					_httpServer.send(200, "text/plain", String("{\"state\": \"true\"}")); 
				});

				_httpServer.on("/getraw", []()
				{ 
					_httpServer.send(200, "text/plain", "{\"state\":\"" + String(getCOValue()) + "\"}"); 
				});

				_httpServer.on("/updatedisplay", HTTP_GET, []()
				{
					_forceDisplayUpdate = true; 
					_httpServer.send(200, "text/plain", String("{\"state\": \"true\"}")); 
				});

				_httpServer.on("/serverIndex", HTTP_GET, []()
				{
					_httpServer.sendHeader("Connection", "close");
					_httpServer.send(200, "text/html", serverIndex); 
				});

				_httpServer.on("/reset", []()
				{
					_httpServer.send(200, "text/plain", "Resetting");
					ESP.restart(); 
				});

				/*handling uploading firmware file */
				_httpServer.on("/update", HTTP_POST, []()
				{
					_httpServer.sendHeader("Connection", "close");
					_httpServer.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
					ESP.restart(); },
					[]()
					{
					HTTPUpload &upload = _httpServer.upload();
					if (upload.status == UPLOAD_FILE_START)
					{
					DEBUG_PRINT("Update: ");
					DEBUG_PRINTLN(upload.filename.c_str());
					if (!Update.begin(UPDATE_SIZE_UNKNOWN))
					{ // start with max available size
						Update.printError(Serial);
					}
					}
					else if (upload.status == UPLOAD_FILE_WRITE)
					{
					/* flashing firmware to ESP*/
					if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
					{
						Update.printError(Serial);
					}
					}
					else if (upload.status == UPLOAD_FILE_END)
					{
					if (Update.end(true))
					{ // true to set the size to the current progress
						DEBUG_PRINTLN("Update Success:" + String(upload.totalSize) + "\nRebooting...\n");
					}
					else
					{
						Update.printError(Serial);
					}
					}
				});

	_httpServer.onNotFound(handleSendToRoot);

	DEBUG_PRINTLN("Web Request Completed...");
}

/***************************************************
  Sensor Logic
****************************************************/

void initSensor()
{

	Wire.begin();

	uint16_t error;
	char errorMessage[256];

	scd4x.begin(Wire);

	// stop potentially previously started measurement
	error = scd4x.stopPeriodicMeasurement();
	if (error)
	{
		DEBUG_PRINT("Error trying to execute stopPeriodicMeasurement(): ");
		errorToString(error, errorMessage, 256);
		DEBUG_PRINTLN(errorMessage);
	}

	uint16_t serial0;
	uint16_t serial1;
	uint16_t serial2;
	error = scd4x.getSerialNumber(serial0, serial1, serial2);
	if (error)
	{
		DEBUG_PRINT("Error trying to execute getSerialNumber(): ");
		errorToString(error, errorMessage, 256);
		DEBUG_PRINTLN(errorMessage);
	}
	else
	{
		printSerialNumber(serial0, serial1, serial2);
	}

	// Start Measurement
	error = scd4x.startPeriodicMeasurement();
	if (error)
	{
		DEBUG_PRINT("Error trying to execute startPeriodicMeasurement(): ");
		errorToString(error, errorMessage, 256);
		DEBUG_PRINTLN(errorMessage);
	}

	DEBUG_PRINTLN("Wait for first measurement... (5 sec)");
}

/***************************************************
  Drawing
****************************************************/

void drawGrid()
{

	_display.drawVLine(COL1, 0, SCREENHEIGHT);
	_display.drawVLine(COL2, 0, SCREENHEIGHT);
	_display.drawVLine(COL3, 0, SCREENHEIGHT);
	_display.drawVLine(COL4, 0, SCREENHEIGHT);
	_display.drawVLine(COL5, 0, SCREENHEIGHT);
	_display.drawVLine(COL6, 0, SCREENHEIGHT);
	_display.drawVLine(COL7, 0, SCREENHEIGHT);
	_display.drawVLine(COL8, 0, SCREENHEIGHT);

	_display.drawHLine(0, ROW1, SCREENWIDTH);
	_display.drawHLine(0, ROW2, SCREENWIDTH);
	_display.drawHLine(0, ROW3, SCREENWIDTH);
	_display.drawHLine(0, ROW4, SCREENWIDTH);
	_display.drawHLine(0, ROW5, SCREENWIDTH);
	_display.drawHLine(0, ROW6, SCREENWIDTH);
	_display.drawHLine(0, ROW7, SCREENWIDTH);
	_display.drawHLine(0, ROW8, SCREENWIDTH);
}

void displayLoading()
{
	DEBUG_PRINTLN("displayLoading()");

	_display.firstPage();
	do
	{
		_display.setFont(MEDIUMFONT);
		int16_t tbx, tby;
		uint16_t tbw, tbh; // boundary box window

		String text = "C02 Monitor";
		tbw = _display.getStrWidth(text.c_str()); // get the string width
		tbh = _display.getFontAscent() + _display.getFontDescent();

		// center bounding box by transposition of origin:

		uint16_t x = ((SCREENWIDTH - tbw) / 2) - tbx;
		uint16_t y = ((SCREENHEIGHT - tbh) / 2) - tby;
		_display.drawStr(x, y, text.c_str());
		DEBUG_PRINTLN("Text: " + text + " X: " + String(x) + " Y: " + String(y) + " tbw: " + String(tbw) + " tbh: " + String(tbh));

		text = "Reading initial values...";
		tbw = _display.getStrWidth(text.c_str()); // get the string width
		tbh = _display.getFontAscent() + _display.getFontDescent();

		_display.drawStr(((SCREENWIDTH - tbw) / 2) - tbx, y + tbh + CELLPADDING, "Reading initial values..."); // print some text

	} while (_display.nextPage());
	DEBUG_PRINTLN("displayLoading() finished");
}

void pushDisplayBuffer()
{
	DEBUG_PRINTLN("pushDisplayBuffer()");
	while (_display.nextPage())
		;

	DEBUG_PRINTLN("pushDisplayBuffer() finished");
}

void drawSparkLine()
{
	DEBUG_PRINTLN("drawSparkLine()");
	// skip zeroes
	byte _startPoint = 0;
	for (byte i = 0; i < NUMLASTVALUES; i++)
	{
		if (lastValues[i] > 0)
		{
			_startPoint = i;
			break;
		}
	}

	int __distBetweenPoints = 1;
	if ((NUMLASTVALUES - _startPoint) > 0)
	{
		__distBetweenPoints = (COL8 - COL1) / (NUMLASTVALUES);
	}

	// draw sparkline
	for (byte i = _startPoint; i < NUMLASTVALUES; i++)
	{
		DEBUG_PRINTLN("Drawing Bar: " + String(i) + " Height: " + String(lastValuesSparkline[i]));
		int __curPos = COL1 + (i - _startPoint) * __distBetweenPoints + __distBetweenPoints / 2;
		if (i == NUMLASTVALUES - 1)
		{
			_display.drawBox(__curPos, ROW8 - lastValuesSparkline[i], __distBetweenPoints / 2, lastValuesSparkline[i]);
		}
		_display.drawFrame(__curPos, ROW8 - lastValuesSparkline[i], __distBetweenPoints / 2, lastValuesSparkline[i]);
	}
	DEBUG_PRINTLN("drawSparkLine() finished");
}

void updateDisplayRender()
{
	DEBUG_PRINTLN("updateDisplayRender()");
	//_display.fillScreen(GxEPD_WHITE); // set the background to white (fill the buffer with value for white)
	_display.firstPage();
	do
	{
		int16_t tbx, tby;
		uint16_t tbw, tbh; // boundary box window

		int CO2PPMWidth = 0;

		String __outString = "CO";

		_display.setFont(LARGEFONT);
		_display.drawStr(COL2, ROW2 - 2, __outString.c_str());
		tbw = _display.getStrWidth(__outString.c_str());
		_display.setFont(SMALLFONT);
		_display.drawStr(COL2 + tbw, ROW4, "2");

		_display.setFont(LARGEFONT);

		//_CO2Val=8000;
		_display.drawStr(COL4, ROW2 - 2, String(_CO2Val).c_str());
		tbw = _display.getStrWidth(String(_CO2Val).c_str());

		_display.setFont(SMALLFONT);
		_display.drawStr(COL4 + tbw, ROW4, "PPM");

		CO2PPMWidth += COL4 + tbw + _display.getStrWidth("PPM") + 2;
		_display.drawRFrame(COL1, ROW1, CO2PPMWidth, ROW6 - ROW1, CELLPADDING * 2 + 2);

		_display.setFont(MEDIUMFONT);

		__outString = String(round(_temp), 0) + " C";
		tbw = _display.getStrWidth(__outString.c_str());
		_display.drawStr(COL8 - tbw, ROW1, __outString.c_str());

		__outString = "C";
		tbw = _display.getStrWidth(__outString.c_str());
		_display.drawCircle(COL8 - tbw - 3, ROW1 + 1, 1);

		__outString = String(_humidity, 0) + "%";
		tbw = _display.getStrWidth(__outString.c_str());

		tbh = _display.getFontAscent() + _display.getFontDescent();
		_display.drawStr(COL8 - tbw, ROW2 + tbh + CELLPADDING, __outString.c_str());

		drawSparkLine();

		_display.drawVLine(COL1, ROW7, SPARKLINEHEIGHT);
		_display.drawVLine(COL1 + 1, ROW7, SPARKLINEHEIGHT);

		_display.drawHLine(COL1, ROW8, COL8 - COL1 - 1);
		_display.drawHLine(COL1, ROW8 + 1, COL8 - COL1 - 1);

		// drawGrid();

	} while (_display.nextPage());
	DEBUG_PRINTLN("updateDisplayRender() finished");
}

void initDisplay()
{
	DEBUG_PRINTLN("initDisplay()");

	_display.begin();
	_display.setFont(MEDIUMFONT);
	_display.setFontRefHeightExtendedText();
	_display.setDrawColor(1);
	_display.setFontPosTop();
	_display.setFontDirection(0);

	DEBUG_PRINTLN("Display width: " + String(SCREENWIDTH));
	DEBUG_PRINTLN("Display height: " + String(SCREENHEIGHT));
	DEBUG_PRINTLN("initDisplay() finished");
}

void setup()
{
	Serial.begin(115200);

	// Disable Brownout detector
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

	DEBUG_PRINTLN("Starting...");
	_mqttPostFix = String(random(0xffff), HEX);
	_mqttClientId = MQTT_CLIENTNAME;
	_deviceClientName = MQTT_CLIENTNAME;

	delay(250); // give the screen time to init

	DEBUG_PRINTLN("Starting CO2 Monitor");
	DEBUG_PRINTLN("----------------------------------");
	initSensor();

	DEBUG_PRINTLN("Initialising Display");
	DEBUG_PRINTLN("----------------------------------");
	initDisplay();

	DEBUG_PRINTLN("Initialising WiFi: 1st AP");

	if (!isWiFiConnected(_mqttClientId))
	{
		DEBUG_PRINTLN("Initialising WiFi: 2nd AP");
		flipAPDetails();
		if (!isWiFiConnected(_mqttClientId))
		{
			DEBUG_PRINTLN("WiFi connection failed");
			DEBUG_PRINTLN("Restarting device");
			ESP.restart();
		}
	}
	// DEBUG_PRINT(F("********** Free Heap: "));   DEBUG_PRINTLN(ESP.getFreeHeap());
	DEBUG_PRINTLN("Web Server config");
	setupWebServer();

	// Start the server
	// DEBUG_PRINT(F("********** Free Heap: "));   DEBUG_PRINTLN(ESP.getFreeHeap());
	DEBUG_PRINTLN("Web Server starting");
	_httpServer.begin();

	// DEBUG_PRINT(F("********** Free Heap: "));   DEBUG_PRINTLN(ESP.getFreeHeap());
	DEBUG_PRINTLN("OTA Firmware Setup");
	setupOTA();

	DEBUG_PRINTLN("Configuring MQTT");
	setupMQTT();
	mqttReconnect(_mqttClientId);
	mqttCustomSubscribe();
	mqttSendInitStat();

	// DEBUG_PRINTLN("Setup Time Server");
	// checkBST();

	DEBUG_PRINTLN("Attempting MQTT: ");
	DEBUG_PRINTLN(String(MQTT_SERVERADDRESS) + ":1883");
	_mqttClient.setServer(MQTT_SERVERADDRESS, 1883);
	_mqttClient.setCallback(mqttCallback);

	DEBUG_PRINTLN("Free Heap Memory: " + String(ESP.getFreeHeap()));

	DEBUG_PRINTLN("DNS Setup");
	if (MDNS.begin(_deviceClientName))
	{
		DEBUG_PRINTLN("Connect to:");
		DEBUG_PRINTLN(" http://" + String(_deviceClientName) + ".local");
		MDNS.addService("http", "tcp", 80);
	}
	else
	{
		DEBUG_PRINTLN("DNS Setup failed");
	}
	_ip = WiFi.localIP().toString();
	DEBUG_PRINTLN("IP:" + _ip);

	// DEBUG_PRINTLN("Updating local time");
	// setupTimeClient();
	// configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
	// updateLocalTime();
	displayLoading();

	//updateDisplayRender();
	_forceUpdate = true;
}

void loop()
{

	/* this function will handle incomming chunk of SW, flash and respond sender */
	ArduinoOTA.handle();
	_mqttClient.loop();
	// Check if a client has connected
	_httpServer.handleClient();

	if (!_mqttClient.connected())
	{
		mqttReconnect(MQTT_DEVICENAME);
	}

	_runCurrent = millis(); // sets the counter
	if (_runCurrent - _runCOCheck >= _delayCOMeterMillis || _forceUpdate)
	{
		DEBUG_PRINTLN("Reading CO2 values...");
		_runCOCheck = millis();
		isWiFiConnected(MQTT_DEVICENAME);
		doSensorReadMQTTSend();
		updateCOHistory();
		_forceUpdate = false;
	}

	// if (_runCurrent - _runDisplayUpdate >= UPDATE_DISPLAY_INTERVAL_MILLISECS || _forceDisplayUpdate)
	if (_forceDisplayUpdate)
	{
		updateDisplayRender();
		_runDisplayUpdate = millis();
		_forceDisplayUpdate = false;
	}

	if (_runCurrent - _runLEDBlink >= LED_BLINK_INTERVAL_MILLISECS)
	{
		_runLEDBlink = millis();
		tickLED();
	}
}
