#ifdef ESP32
 #include <WiFi.h>
 #include <ESPmDNS.h>
 #include <SPIFFS.h>
#else
 #include <ESP8266WiFi.h>
 #include <ESP8266mDNS.h>
 #include <FS.h>
#endif
#include "ESPAsyncWebServer.h"
//#include "SPIFFSEditor.h"
#include "SPIFFSEditor_DMD.h"
#include <Wire.h>
#include "AS5600.h"
#include "time.h"
#include "sys/time.h"

//----------------------------------------------------------------------------------------------------------------------------
void getClockTime(int *hour, int *minute, float *herror, float *m10error, float *m1error, float *minute10Angle, float *floatH, float *floatM1, int enablePrint);
int loadCalibration();
int FetchTheTime(struct tm* tinfo, time_t* timeVal);

//----------------------------------------------------------------------------------------------------------------------------
int  SETUP_VERSION  =      10001;
char p_SSID[32] =          "ssid";
char p_PWD[32] =           "password";
int DST_offset =           -1;
int ST_GMT_offset =        -7;
int ENABLE_AUTO_TIME_SET = 0;                   //0=Disable, 1=At reset only, 2=Continuous
//char staticIP[32] =      "192.168.1.167";
char staticIP[32] =      "0";
char gateway[32] =       "192.168.1.1";
char subnet[32] =        "255.255.0.0";
char dns[32] =           "192.168.1.1";
char hostName[32] =      "NodeClock";
char dnsName[32] =       "clock";               //Access server at: dnsName.local
char setTimeZone[32] =   "PST+8PDT,M3.2.0/2,M11.1.0/2"; //Refer to: https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html

//----------------------------------------------------------------------------------------------------------------------------

String SerialCmdStr = "";
char serialBuffer[64];
int serialBufferPtr = 0;

timeval tim;
timezone tzone;
time_t globalRawTime;
struct tm timeinfo;
time_t LastNTPtime = 0;
char ntpServer[32] =     "pool.ntp.org";

unsigned long delayStart = 0;

char http_username[32] = "admin";
char http_password[32] = "admin";

unsigned long lastReconnectAttempt = 0;

int minCount = 0;
int hrCount = 0;
int stepsMin = 0;
int stepsHr = 0;

int nextMinute = -1;
int lastMinute10 = -1;
int lastComputedClockH = 0;
float lastComputedClockM = 0;
int lastComputedClockS = 0;

float angleMin10[120];
float angleMin[120];

int IsCalibrated = 0;
float min10Calibration[12];
float min1Calibration[10];
float hourCalibration[24];

int correctMin = 0;
int correctHr = 0;

AS5600 as5600_1(&Wire);

int NTPsyncAtLeastOnce = 0;
int clockEnabled = 1;
int disableCalibration = 0;

volatile int prevHr;
volatile int prevMin;
volatile int percentDoneMin = 100;
volatile int percentDoneHour = 100;
volatile int percentDoneHrAndMin = 100;
volatile int serverCommand = 1;
char serverCommandStr[64];

unsigned long time_stamp = 0;
int Dprint = 0;

AsyncWebServer server(80);

//--------------------------------------------------------------------------------------------
//Motor to CAM gear ratio = 2.4 * 1.6 = 3.84
//Stepper steps per rev ~= 3072   (3072 / 2 = 1536 / 64 = 24)
//CAM gear 1 rev = 11797.2 steps (11797.2 / 10 = 1179.72)  (11797.2 / 24 = 491.55)
//numMinStepsNom: 1179.72 --> (1179.72 - 1180) * 25 = -7 --> 403.2 counts/day error = 0.342 digits/day error
//numHrStepsNom:  491.55  --> (491.55 - 492) * 20 = -9 --> 10.8 counts/day error = 75.6 counts/wk error = 302.4 counts/mo error = 0.615 digits/mo error

#define MotorStepAngle           (360.0/11800.0)

#define numHrSteps               491.52
#define numHrStepsNom            492
#define numHrAdvCorrectCycles    20  
#define numHrStepsCorrectSteps   -9  

#define numMinSteps              1179.648
#define numMinStepsNom           1180
#define numMinAdvCorrectCycles   25  
#define numMinStepsCorrectSteps  -7

int stepRate = 8000; //microseconds (us)

const char* PARAM_INPUT_1 = "input1";
String serverStatusDisplay = String("Command status: READY");

#define NTP_RESYNC_TIME (60*60)

//----------------------------------------------------------------------------------------------------------------------------
#ifdef ESP32
#define GPIO_EN1  18
#define GPIO_SDA1 25
#define GPIO_EN2  21
#define GPIO_SDA2 26
#define GPIO_SDA3 27
#define GPIO_ST2  22
#define GPIO_ST1  19
#define GPIO_SCL  23
#define GPIO_15   15
#define GPIO_1    1
#define GPIO_3    3
#else
#define GPIO_EN1  16   //D0 (IN / OUT: high at boot)
#define GPIO_SDA1 5    //D1 (IN / OUT)
#define GPIO_EN2  4    //D2 (IN / OUT)
#define GPIO_SDA2 13   //D7 (IN / OUT)
#define GPIO_SDA3 14   //D5 (IN / OUT)
#define GPIO_ST2  12   //D6 (IN / OUT)
#define GPIO_ST1  2    //D4 (IN: pup, boot fail if low / OUT: LED, high at boot)
#define GPIO_SCL  0    //D3 (IN: pup, flash button, fail if low / OUT)
#define GPIO_15   15   //D8 (IN: pdown, boot fail if high / OUT: debug at boot)
#define GPIO_1    1    //TX (OUT: high/debug at boot, boot fail if low)
#define GPIO_3    3    //RX (IN: high at boot)
#endif

//----------------------------------------------------------------------------------------------------------------------------
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

//----------------------------------------------------------------------------------------------------------------------------
String processor(const String& var){
  if(var == "DATAPLACEHOLDER"){
    String msg = "";
    return(msg);
  }
  else return String();
}

//----------------------------------------------------------------------------------------------------------------------------
void InitializeServer()
{
//  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send_P(200, "text/html", index_html, processor);
//  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.htm", String(), false, processor);
  });

  server.on("/status", HTTP_GET, [] (AsyncWebServerRequest *request) {
    request->send_P(200, "text/plain", serverStatusDisplay.c_str());
  });
  
  server.on("/exe", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    String inputParam;
    const char* PARAM_INPUT = "cmd";

    //EXE input1 value on <ESP_IP>/get?input1=<inputMessage>
    if (request->hasParam(PARAM_INPUT)) {
      inputMessage = request->getParam(PARAM_INPUT)->value();
      inputParam = PARAM_INPUT;

      if (serverCommand == 0)
      {
        inputMessage.toCharArray(serverCommandStr,sizeof(serverCommandStr));
        serverCommand = 1;
        serverStatusDisplay = String("Command status: Command Sent");
        request->send_P(200, "text/plain", serverStatusDisplay.c_str());
      }
      else
      {
        serverStatusDisplay = String("Command Status: Error - A command is currently executing");
        request->send_P(200, "text/plain", serverStatusDisplay.c_str());
      }
    }
    Serial.printf("Server EXE --> %s\n", inputMessage.c_str());
  });

  server.on("/time", HTTP_GET, [](AsyncWebServerRequest *request){
    String datStr = "";
    FetchTheTime(&timeinfo, &globalRawTime);
    char str[12];
    sprintf(str,"%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
    datStr += String("<br>Current Time: ") + String(str);
    if (percentDoneMin < 100)
    {
      sprintf(str,"%02d:%02d:%02d", lastComputedClockH, (int)lastComputedClockM, lastComputedClockS);
    }
    else
    {
      sprintf(str,"%02d:%02d", lastComputedClockH, (int)lastComputedClockM);
    }
    datStr += String("<br>Clock Time: ") + String(str);
//      datStr += String("<br>H offset = ") + (int)((eh/360.0)*numHrStepsNom*24.0) + 
//      String("<br>M10 offset = ") + (int)((em10/360.0)*numMinStepsNom*10.0) + 
//      String("<br>M1 offset = ") + (int)((em1/360.0)*numMinStepsNom*10.0);
      String("<br>ClockM = ") + lastComputedClockM;
      String("<br>ClockS = ") + lastComputedClockS;
    datStr += String("<br>");
    if (clockEnabled == 1) datStr += "Clock status: RUNNING<br>";
    else datStr += "Clock status: DISABLED<br>";
    if (IsCalibrated == 1) datStr += "Calibrated: YES<br>";
    else datStr += "Calibrated: NO<br>";
    if (NTPsyncAtLeastOnce == 1) datStr += "NTP Sync: YES";
    else datStr += "NTP Sync: NO";
    request->send_P(200, "text/plain", datStr.c_str());
  });

  server.onNotFound(notFound);

  server.begin();
}

//--------------------------------------------------------------------------------------------
int FetchTheTime(struct tm* tinfo, time_t* timeVal)
{
  int stat = 2; //Get time from RTC
  time_t rawtime;
  struct tm *tinfo_x;
  time(&rawtime);
  tinfo_x = localtime(&rawtime);
  *timeVal = mktime(tinfo_x);
  *tinfo = *tinfo_x;
  double timeSinceLastResync = difftime(rawtime, LastNTPtime);
  if (tinfo->tm_year < 70)  stat = 0;  //Time info not valid --> Get time from NTP server
  else if (timeSinceLastResync > NTP_RESYNC_TIME)  stat = 1; //Time info has not synced in a while -->  Get time from NTP server
  if (stat < 2)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      if (getLocalTime(tinfo, 5000))
      {
        NTPsyncAtLeastOnce = 1;
        stat = 1; //Got time from NTP server
        time(&LastNTPtime);
        Serial.printf("NTP: %02d-%02d-%02d %02d:%02d:%02d\n", tinfo->tm_year + 1900, tinfo->tm_mon + 1, tinfo->tm_mday, tinfo->tm_hour, tinfo->tm_min, tinfo->tm_sec);
      }
      else
      {
        if (stat != 0)  stat = 2; //Failed to get time from NTP server --> Use time from RTC
        else  Serial.println(F("Failed NTP"));
      }
    }
  }
  return (stat);
}

//----------------------------------------------------------------------------------------------------------------------------
int SaveSetup(void)
{
  int stat = 0;

  File fp = SPIFFS.open("/setup.txt", "w");
  if (!fp)
  {
    stat = 1;
  }
  else
  {
    fp.print(F("SETUP_VERSION=")); fp.println(SETUP_VERSION);
    fp.print(F("p_SSID=")); fp.println(p_SSID);
    fp.print(F("p_PWD=")); fp.println(p_PWD);
    fp.print(F("DST_offset=")); fp.println(DST_offset);
    fp.print(F("ST_GMT_offset=")); fp.println(ST_GMT_offset);
    fp.print(F("ENABLE_AUTO_TIME_SET=")); fp.println(ENABLE_AUTO_TIME_SET);
    fp.print(F("staticIP=")); fp.println(staticIP);
    fp.print(F("gateway=")); fp.println(gateway);
    fp.print(F("subnet=")); fp.println(subnet);
    fp.print(F("dns=")); fp.println(dns);
    fp.print(F("hostName=")); fp.println(hostName);
    fp.print(F("dnsName=")); fp.println(dnsName);
    fp.print(F("setTimeZone=")); fp.println(setTimeZone);
    fp.close();
  }
  Serial.printf("SaveSetup=%d\n",stat);
  return(stat);
}

//----------------------------------------------------------------------------------------------------------------------------
int RecallSetup(void)
{
  int stat = 0;
  
  File fp = SPIFFS.open("/setup.txt", "r");
  if (!fp)
  {
    stat = 1;
  }
  else
  {
    fp.readStringUntil('=');
    int Ver = fp.parseInt();
    fp.readStringUntil('\n');
    if (Ver != SETUP_VERSION)  stat = 1;
    else
    {
      fp.readStringUntil('=');
      fp.readStringUntil('\n').toCharArray(p_SSID,32);
      p_SSID[strlen(p_SSID)-1] = 0;
      fp.readStringUntil('=');
      fp.readStringUntil('\n').toCharArray(p_PWD,32);
      p_PWD[strlen(p_PWD)-1] = 0;
      fp.readStringUntil('=');
      DST_offset = fp.parseInt();
      fp.readStringUntil('=');
      ST_GMT_offset = fp.parseInt();
      fp.readStringUntil('=');
      ENABLE_AUTO_TIME_SET = fp.parseInt();
      fp.readStringUntil('=');
      fp.readStringUntil('\n').toCharArray(staticIP,32);
      staticIP[strlen(staticIP)-1] = 0;
      fp.readStringUntil('=');
      fp.readStringUntil('\n').toCharArray(gateway,32);
      gateway[strlen(gateway)-1] = 0;
      fp.readStringUntil('=');
      fp.readStringUntil('\n').toCharArray(subnet,32);
      subnet[strlen(subnet)-1] = 0;
      fp.readStringUntil('=');
      fp.readStringUntil('\n').toCharArray(dns,32);
      dns[strlen(dns)-1] = 0;
      fp.readStringUntil('=');
      fp.readStringUntil('\n').toCharArray(hostName,32);
      hostName[strlen(hostName)-1] = 0;
      fp.readStringUntil('=');
      fp.readStringUntil('\n').toCharArray(dnsName,32);
      dnsName[strlen(dnsName)-1] = 0;
      fp.readStringUntil('=');
      fp.readStringUntil('\n').toCharArray(setTimeZone,32);
      setTimeZone[strlen(setTimeZone)-1] = 0;
    }
    fp.close();
  }
  Serial.printf("RecallSetup=%d\n",stat);
  return(stat);
}

//--------------------------------------------------------------------------------------------
void InitializeWiFi()
{
  WiFi.disconnect();
  delay(250);
#ifdef ESP32
  WiFi.setAutoConnect(false); //* Configure module to automatically connect on power on to the last used access point
  WiFi.setAutoReconnect(true); //* Set whether module will attempt to reconnect to an access point in case it is disconnected
  WiFi.persistent(false);
#endif
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(hostName, "123456789");
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  if (strlen(staticIP) > 3)
  {
    IPAddress sip;
    sip.fromString(staticIP);
    IPAddress gw;
    gw.fromString(gateway);
    IPAddress subn;
    subn.fromString(subnet);
    IPAddress dnsvr;
    dnsvr.fromString(dns);
    if (WiFi.config(sip, gw, subn, dnsvr, dnsvr) == false)
    {
      Serial.println("WiFi configuration failed");
    }
  }

#ifdef ESP32
  WiFi.setHostname(hostName);
  delay(25);
  WiFi.softAPsetHostname(hostName);
  delay(25);
#else
  WiFi.hostname(hostName);
  delay(25);
#endif
  WiFi.begin(p_SSID, p_PWD);
  delay(25);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println(F("Connection to network failed!"));
  }
  else
  {
    Serial.println(F("Connection to network OK!"));
  }
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false); //TODO: ???
  lastReconnectAttempt = millis();

  if (!MDNS.begin(dnsName)) {
    Serial.println(F("Error setting up MDNS responder!"));
  }

  Serial.println(WiFi.localIP());
}

//----------------------------------------------------------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);

  pinMode(GPIO_EN1, OUTPUT);
  pinMode(GPIO_EN2, OUTPUT);
  pinMode(GPIO_ST1, OUTPUT);
  pinMode(GPIO_ST2, OUTPUT);
  digitalWrite(GPIO_EN1, HIGH);
  digitalWrite(GPIO_EN2, HIGH);
  digitalWrite(GPIO_ST1, LOW);
  digitalWrite(GPIO_ST2, LOW);
  
  //--- Initialize SPIFFS File System ---
  SPIFFS.begin();

  sprintf(serverCommandStr, "Enter Command");
  delay(1000);

  int res = RecallSetup();
  if (res != 0)  SaveSetup();

  IsCalibrated = loadCalibration();
  Serial.printf("Calibration status = %d\n", IsCalibrated);

  InitialMinuteSense();

  InitializeWiFi();

  InitializeServer();

  //--- Initialize time/RTC ---
#ifdef ESP32
  configTime(0, 0, "pool.ntp.org");
  setenv("TZ", setTimeZone, 1); //Refer to: https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
  tzset();
#else
  tzone.tz_dsttime = DST_offset * 3600;
  tzone.tz_minuteswest = ST_GMT_offset * 3600;
  configTime(ST_GMT_offset * 3600, DST_offset * 3600, ntpServer);
#endif
  
  if (FetchTheTime(&timeinfo, &globalRawTime) == 0)
  {
    Serial.println(F("Failed to obtain time"));
  }

  printf("\n");

  delayStart = millis();

#ifdef ESP32
  //server.addHandler(new SPIFFSEditor(SPIFFS,http_username,http_password));
  server.addHandler(new SPIFFSEditor_DMD(SPIFFS,http_username,http_password));
#else
  //server.addHandler(new SPIFFSEditor(http_username,http_password,SPIFFS));
  server.addHandler(new SPIFFSEditor_DMD(http_username,http_password,SPIFFS));
#endif  
  server.on("/ace.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/ace.js", String(), true);
  });
  server.on("/mode-css.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/mode-css.js", String(), true);
  });
  server.on("/mode-html.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/mode-html.js", String(), true);
  });
  server.on("/mode-javascript.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/mode-javascript.js", String(), true);
  });
  server.on("/worker-html.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/worker-html.js", String(), true);
  });
  server.on("/ext-searchbox.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/ext-searchbox.js", String(), true);
  });
  server.on("/microajax.js.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/microajax.js", String(), true);
  });

  server.on("/boot", HTTP_GET, [](AsyncWebServerRequest *request){
    ESP.restart();
  });
}

//--------------------------------------------------------------------------------------------
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//--------------------------------------------------------------------------------------------
void OnStepHr(int x)
{
  stepsHr++;
}

//--------------------------------------------------------------------------------------------
void GoToHour(float targetHour)
{
  int x, y;
  int h, m;
  float eh, em10, em1, m10a, fH, fM1;

  getClockTime(&h, &m, &eh, &em10, &em1, &m10a, &fH, &fM1, 0);
  float startTime = fH;
  float deltaTime = targetHour - startTime;
  if (deltaTime < 0) deltaTime += 24.0;
  float previous_toGo = deltaTime;

  percentDoneHour = 0;

  float hrDigitsToAdvance = targetHour - fH;
  if (hrDigitsToAdvance < 0) hrDigitsToAdvance += 24.0;
  float steps2Target = hrDigitsToAdvance * (float)numHrStepsNom;
  int maxSteps = steps2Target + (2 * numHrStepsNom);

  digitalWrite(GPIO_EN2, LOW);
  time_stamp = micros();
  for (x=0; x<maxSteps; x++)
  {
    while (micros() - time_stamp < stepRate) ;
    digitalWrite(GPIO_ST2, HIGH);
    digitalWrite(GPIO_ST2, LOW);
    time_stamp = micros();
    yield();

    OnStepHr(x);

    getClockTime(&h, &m, &eh, &em10, &em1, &m10a, &fH, &fM1, 0);
    float nowTime = fH;
    float doneTime = nowTime - startTime;
    if (doneTime < 0) doneTime += 24.0;
    percentDoneHour = (100.0 * doneTime) / deltaTime;

    float toGo = targetHour - nowTime;
    if (toGo < 0) toGo += 24;
 
    if (Dprint) Serial.printf("h=%d lastComputedClockH=%0.2f toGo=%0.2f fH=%0.2f nowTime=%0.2f doneTime=%0.2f targetHour=%0.2f stepsHr=%d\n",
      h,lastComputedClockH,toGo,fH,nowTime,doneTime,targetHour,stepsHr);

    if ((toGo <= 0) || (abs(previous_toGo - toGo) > 12.0)) break;
    previous_toGo = toGo;
  }
  digitalWrite(GPIO_EN2, HIGH);
  percentDoneHour = 100;
}

//--------------------------------------------------------------------------------------------
void AdvanceHour()
{
  int numHr = numHrStepsNom;
  int x,y;
  if (hrCount++ == numHrAdvCorrectCycles)
  {
    numHr = numHrStepsNom + numHrStepsCorrectSteps;
    hrCount = 0;
  }

  numHr -= correctHr;
  correctHr = 0;
  
  digitalWrite(GPIO_EN2, LOW);
  time_stamp = micros();
  for (x=0; x<numHr; x++)
  {
    while (micros() - time_stamp < stepRate) ;
    digitalWrite(GPIO_ST2, HIGH);
    digitalWrite(GPIO_ST2, LOW);
    time_stamp = micros();
    yield();

    OnStepHr(x);
  }
  digitalWrite(GPIO_EN2, HIGH);
}

//--------------------------------------------------------------------------------------------
void OnStepMin(int x)
{
  stepsMin++;
}

//--------------------------------------------------------------------------------------------
void InitialMinuteSense()
{
  int x, y;
  int h, m;
  float eh, em10, em1, m10a, fH, fM1;

  getClockTime(&h, &m, &eh, &em10, &em1, &m10a, &fH, &fM1, 0);
  Serial.printf("InitialMinuteSense: fM1=%0.2f\n", fM1);
  if (fM1 >= 9.05)
  {
    int maxSteps = 2 * numMinStepsNom;

    digitalWrite(GPIO_EN1, LOW);
    time_stamp = micros();
    for (x=0; x<maxSteps; x++)
    {
      while (micros() - time_stamp < stepRate) ;
      digitalWrite(GPIO_ST1, HIGH);
      digitalWrite(GPIO_ST1, LOW);
      time_stamp = micros();
      yield();

      getClockTime(&h, &m, &eh, &em10, &em1, &m10a, &fH, &fM1, 0);
      if (fM1 < 0) fM1 += 10.0;
      float frac = fM1 - (int)fM1;
      if (frac >= 0.9) break;
    }
    digitalWrite(GPIO_EN1, HIGH);

    Serial.printf("InitialMinuteSense: %0.2f lastMinute10=%d\n", fM1, lastMinute10);
    lastMinute10 = m / 10;
  }
}

//--------------------------------------------------------------------------------------------
void GoToMinute(float targetMinute)
{
  int x, y;
  int h, m;
  float eh, em10, em1, m10a, fH, fM1;

  getClockTime(&h, &m, &eh, &em10, &em1, &m10a, &fH, &fM1, 0);
  float startTime = lastComputedClockM;
  float deltaTime = targetMinute - startTime;
  if (deltaTime < 0) deltaTime += 60.0;
  float previous_toGo = deltaTime;

  percentDoneMin = 0;

  float minDigitsToAdvance = targetMinute - fM1;
  if (minDigitsToAdvance < 0) minDigitsToAdvance += 60.0;
  int steps2Target = minDigitsToAdvance * (float)numMinStepsNom;
  int maxSteps = steps2Target + (2 * numMinStepsNom);

  digitalWrite(GPIO_EN1, LOW);
  time_stamp = micros();
  for (x=0; x<maxSteps; x++)
  {
    while (micros() - time_stamp < stepRate) ;
    digitalWrite(GPIO_ST1, HIGH);
    digitalWrite(GPIO_ST1, LOW);
    time_stamp = micros();
    yield();

    OnStepMin(x);

    getClockTime(&h, &m, &eh, &em10, &em1, &m10a, &fH, &fM1, 0);
    float nowTime = lastComputedClockM;
    float doneTime = nowTime - startTime;
    if (doneTime < 0) doneTime += 60.0;
    percentDoneMin = (100.0 * doneTime) / deltaTime;

    float toGo = targetMinute - nowTime;
    if (toGo < 0) toGo += 60;

    if (Dprint) Serial.printf("m=%d lastComputedClockM=%0.2f toGo=%0.2f fM1=%0.2f nowTime=%0.2f doneTime=%0.2f targetMinute=%0.2f stepsMin=%d\n",
      m,lastComputedClockM,toGo,fM1,nowTime,doneTime,targetMinute,stepsMin);

    if ((toGo <= 0) || (abs(previous_toGo - toGo) > 30.0)) break;
    previous_toGo = toGo;
  }
  digitalWrite(GPIO_EN1, HIGH);
  percentDoneMin = 100;
}

//--------------------------------------------------------------------------------------------
void AdvanceMinute()
{
  int numMin = numMinStepsNom;
  int x, y;

  if (minCount++ == numMinAdvCorrectCycles)
  {
    numMin = numMinStepsNom + numMinStepsCorrectSteps;
    minCount = 0;
  }

  percentDoneMin = 0;
  numMin -= correctMin;
  correctMin = 0;

  digitalWrite(GPIO_EN1, LOW);
  time_stamp = micros();
  for (x=0; x<numMin; x++)
  {
    while (micros() - time_stamp < stepRate) ;
    digitalWrite(GPIO_ST1, HIGH);
    digitalWrite(GPIO_ST1, LOW);
    time_stamp = micros();
    yield();

    OnStepMin(x);
    percentDoneMin = (100 * x) / numMin; //TODO: Verify % done for no sensors present
  }
  digitalWrite(GPIO_EN1, HIGH);
  percentDoneMin = 100;
}

//--------------------------------------------------------------------------------------------
void GoToNextHourAndMinute(int targetHour, int targetMinute)
{
  int x, y;
  int h, m;
  float eh, em10, em1, m10a, fH, fM1;

//TODO: None of this function is complete yet

  percentDoneHrAndMin = 0;

  getClockTime(&h, &m, &eh, &em10, &em1, &m10a, &fH, &fM1, 0);
  float startTimeM = (float)m + ((em1/360.0)*10.0);
  float deltaTimeM = (float)targetMinute - startTimeM;
  if (deltaTimeM < 0) deltaTimeM += 60.0;
//@@@@  lastComputedClockH = *hour;
//@@@@  lastComputedClockM = (float)(10 * (*minute/10)) + fm1;
//@@@@  lastComputedClockS = (int)((fm1 - (float)lastComputedClockM) * 60);
 
  int minDigitsToAdvance = targetMinute - m;
  if (minDigitsToAdvance < 0) minDigitsToAdvance += 60;
  int steps2Target = (minDigitsToAdvance * numMinStepsNom) - (int)((em1/360.0)*numMinStepsNom*10.0);
  int maxSteps = steps2Target + (2 * numMinStepsNom);

  digitalWrite(GPIO_EN1, LOW);
  time_stamp = micros();
  for (x=0; x<maxSteps; x++)
  {
    while (micros() - time_stamp < stepRate) ;
    digitalWrite(GPIO_ST1, HIGH);
    digitalWrite(GPIO_ST1, LOW);
    time_stamp = micros();
    yield();

    OnStepMin(x);

    getClockTime(&h, &m, &eh, &em10, &em1, &m10a, &fH, &fM1, 0);
    if ((m == targetMinute) && (em1 >= 0))
      break;

    float nowTime = (float)m + ((em1/360.0)*10.0);
    float doneTime = nowTime - startTimeM;
    if (doneTime < 0) doneTime += 60.0;
    percentDoneHrAndMin = (100.0 * doneTime) / deltaTimeM;
  }
  digitalWrite(GPIO_EN1, HIGH);
  percentDoneHrAndMin = 100;
}

//--------------------------------------------------------------------------------------------
void AdvanceHourAndMinute()
{
  int numHr = numHrStepsNom;
  int numMin = numMinStepsNom;
  int x,y;
  int hrCnts = 0;

  if (hrCount++ == numHrAdvCorrectCycles)
  {
    numHr = numHrStepsNom + numHrStepsCorrectSteps;
    hrCount = 0;
  }
  if (minCount++ == numMinAdvCorrectCycles)
  {
    numMin = numMinStepsNom + numMinStepsCorrectSteps;
    minCount = 0;
  }
  double d = (double)numHr / (double)numMin;

  percentDoneMin = 0;
  numHr -= correctHr;
  correctHr = 0;
  numMin -= correctMin;
  correctMin = 0;

  digitalWrite(GPIO_EN2, LOW);
  delay(5);
  digitalWrite(GPIO_EN1, LOW);
  time_stamp = micros();
  for (x=0; x<numMin; x++)
  {
    while (micros() - time_stamp < stepRate) ;
    if ((int)((double)x * d) == hrCnts)
    {
      digitalWrite(GPIO_ST1, HIGH);
      digitalWrite(GPIO_ST1, LOW);
      time_stamp = micros();
      delayMicroseconds((int)((((double)stepRate/2.0) + 0.5)));
      digitalWrite(GPIO_ST2, HIGH);
      digitalWrite(GPIO_ST2, LOW);

      hrCnts++;

      OnStepHr(hrCnts);
    }
    else
    {
      digitalWrite(GPIO_ST1, HIGH);
      digitalWrite(GPIO_ST1, LOW);
      time_stamp = micros();
    }
    yield();

    OnStepMin(x);
    percentDoneMin = (100 * x) / numMin;   //TODO: Verify % done for no sensors present
  }
  digitalWrite(GPIO_EN2, HIGH);
  digitalWrite(GPIO_EN1, HIGH);
  percentDoneMin = 100;
}

//--------------------------------------------------------------------------------------------
float getAngle(int sensorNum, int printStatus)
{
  float angle;
  int retryCount = 3;

  do
  {
    if (sensorNum == 1) Wire.begin(GPIO_SDA1, GPIO_SCL);
    else if (sensorNum == 2) Wire.begin(GPIO_SDA2, GPIO_SCL);
    else if (sensorNum == 3) Wire.begin(GPIO_SDA3, GPIO_SCL);
    Wire.setClock(100000);
    as5600_1.begin(GPIO_15);
    angle = as5600_1.readAngle() * AS5600_RAW_TO_DEGREES;
    if (as5600_1._error == AS5600_OK) break;
    else
    {
      #if 0 //ONLY FOR TESTING
      Serial.printf("\nBAD ANGLE READ: RetryCount:%d Sensor:%d Connected:%d Error:%d Angle:%0.2f\n", retryCount, sensorNum, as5600_1.isConnected(), as5600_1._error, angle);
      #endif
      if (retryCount == 1) Serial.printf("\nBAD ANGLE READ\n");
      Wire.end();
    }
  } while (--retryCount > 0);
  
  if (printStatus == 1)
  {
    int stat = as5600_1.readStatus();
    String msg = "";
    if (stat & 0x20) msg += "Detected: ";
    if (stat & 0x10) msg += "Too Weak";
    else if (stat & 0x08) msg += "Too Strong";
    else msg += "OK";
    Serial.printf("%d --> Magnet %s >> Angle: %0.2f\n", sensorNum, msg.c_str(), as5600_1.readAngle() * AS5600_RAW_TO_DEGREES);
  }
#ifdef ESP32
  Wire.end();
#endif
  return(angle);
}

//--------------------------------------------------------------------------------------------
void printAngles()
{
  getAngle(1, 1);
  getAngle(2, 1);
  getAngle(3, 1);
}

//--------------------------------------------------------------------------------------------
void setNowTime(String nowTime)
{
  struct tm tm;
  tm.tm_year = 2024 - 1900;  
  tm.tm_mon = 3 - 1;
  tm.tm_mday = 1;
  tm.tm_hour = getValue(nowTime, ':', 0).toInt();
  tm.tm_min = getValue(nowTime, ':', 1).toInt();
  tm.tm_sec = getValue(nowTime, ':', 2).toInt();
  time_t t = mktime(&tm);

  Serial.printf("Setting time: %02d:%02d:%02d\n", tm.tm_hour, tm.tm_min, tm.tm_sec);
  struct timeval now;
  now.tv_sec = t;
  settimeofday(&now, NULL);

  NTPsyncAtLeastOnce = 1;
}

//--------------------------------------------------------------------------------------------
int setClockTime(int clockHr, int clockMin)
{
    int i;
    int adv;

    if (NTPsyncAtLeastOnce == 0) return(-1);

    FetchTheTime(&timeinfo, &globalRawTime);
    Serial.printf("Changing time %02d:%02d --> %02d:%02d\n", clockHr, clockMin, timeinfo.tm_hour, timeinfo.tm_min);
    adv = timeinfo.tm_hour - clockHr;
    if (adv < 0) adv += 24;
    if (adv > 0)
    {
      Serial.printf("Setting  hour [Advance %d] ", adv);
      for (i=0; i<adv; i++)
      {
        AdvanceHour();
        Serial.printf(".");
        delay(100);    
      }
      Serial.printf(" Done\n");     
    }

    FetchTheTime(&timeinfo, &globalRawTime);
    adv = timeinfo.tm_min - clockMin;
    if (adv < 0) adv += 60;
    if (adv > 0)
    {
      nextMinute = timeinfo.tm_min + 1;
      if (nextMinute == 60) nextMinute = 0;
      Serial.printf("Setting  minute [Advance %d] ", adv);
      int advanceHr = 0;
      for (i=0; i<adv; i++)
      {
        AdvanceMinute();     
        Serial.printf(".");
        delay(100);    
        FetchTheTime(&timeinfo, &globalRawTime);
        if (timeinfo.tm_min == nextMinute)
        {
          if (nextMinute == 0)
          {
            advanceHr = 1;
          }
          adv++;
          nextMinute = timeinfo.tm_min + 1;
          if (nextMinute == 60) nextMinute = 0;
          Serial.printf("+");
        }
      }
      if (advanceHr == 1)
      {
        AdvanceHour();
      }
      Serial.printf(" Done\n");
    }  
    Serial.printf("Set time finished.\n");
    return(0);
}

//--------------------------------------------------------------------------------------------
int loadCalibration()
{
  int calValid = 0;
  
  File fp = SPIFFS.open("/hcal.txt", "r");
  if (fp)
  {
    int i, ival;
    for (i=0; i<24; i++)
    {
      if (!fp.available()) break;
      ival = fp.parseInt();
      hourCalibration[i] = fp.parseFloat();
    }
    if (fp.parseInt() == 1234) calValid++;
    fp.close();
  }

  fp = SPIFFS.open("/mcal2.txt", "r");
  if (fp)
  {
    int i, ival;
    for (i=0; i<12; i++)
    {
      if (!fp.available()) break;
      ival = fp.parseInt();
      min10Calibration[i] = fp.parseFloat();
    }
    if (fp.parseInt() == 1234) calValid++;
    fp.close();
  }

  fp = SPIFFS.open("/mcal1.txt", "r");
  if (fp)
  {
    int i, ival;
    for (i=0; i<10; i++)
    {
      if (!fp.available()) break;
      ival = fp.parseInt();
      min1Calibration[i] = fp.parseFloat();
    }
    if (fp.parseInt() == 1234) calValid++;
    fp.close();
  }

  return((calValid == 3) ? 1 : 0);
}

//--------------------------------------------------------------------------------------------
void getClockTime(int *hour, int *minute, float *herror, float *m10error, float *m1error,
  float *minute10Angle, float *floatH, float *floatM1, int enablePrint)
{
  float h = getAngle(1, 0);
  float m10 = getAngle(2, 0);
  float m1 = getAngle(3, 0);

  int i;
  int minute1 = 0;
  int minute10 = 0;
  float mdiff;

  if ((IsCalibrated == 0) && (disableCalibration == 0))
  { 
    //TODO: Return internally tracked time?
    return;
  }

  mdiff = 99999;
  for (i=0; i<24; i++)
  {
    float diff = h - hourCalibration[i];
    if (diff < -180.0) diff += 360.0;
    else if (diff > 180.0) diff -= 360.0;
    if (abs(diff) < abs(mdiff))
    {
      mdiff = diff;
      *hour = i;
    }
  }
  if (herror != NULL) *herror = mdiff;

  //.............................................
  float frac;
  float num, den;
  int idx;
  float delt = h - hourCalibration[*hour];
  if (delt > 180) delt -=360;
  else if (delt < -180) delt += 360;
  if (delt < 0)
  {
    idx = *hour - 1;
    if (idx < 0) idx += 24;
    num = (hourCalibration[*hour] - h);
    if (num > 180) num -=360;
    else if (num < -180) num += 360;
    den = hourCalibration[idx] - hourCalibration[*hour];
    if (den > 180) den -=360;
    else if (den < -180) den += 360;
    frac = -abs(num / den);
  }
  else
  {
    idx = *hour + 1;
    if (idx >= 24) idx -= 24;
    num = (hourCalibration[*hour] - h);
    if (num > 180) num -=360;
    else if (num < -180) num += 360;
    den = hourCalibration[idx] - hourCalibration[*hour];
    if (den > 180) den -=360;
    else if (den < -180) den += 360;
    frac = abs(num / den);
  }
  float fh = (float)*hour + frac;
  if (floatH != NULL) *floatH = fh;
  if (enablePrint == 1) Serial.printf("floatH=%0.2f h=%0.2f *hour=%d idx=%d\n", fh, h, *hour, idx);
  //.............................................
  
  mdiff = 99999;
  for (i=0; i<10; i++)
  {
    float diff = m1 - min1Calibration[i];
    if (diff < -180.0) diff += 360.0;
    else if (diff > 180.0) diff -= 360.0;
    if (abs(diff) < abs(mdiff))
    {
      mdiff = diff;
      minute1 = i;
    }
  }
  if (m1error != NULL) *m1error = mdiff;

  //.............................................
  delt = m1 - min1Calibration[minute1];
  if (delt > 180) delt -=360;
  else if (delt < -180) delt += 360;
  if (delt < 0)
  {
    idx = minute1 - 1;
    if (idx < 0) idx += 10;
    num = (min1Calibration[minute1] - m1);
    if (num > 180) num -=360;
    else if (num < -180) num += 360;
    den = min1Calibration[idx] - min1Calibration[minute1];
    if (den > 180) den -=360;
    else if (den < -180) den += 360;
    frac = -abs(num / den);
  }
  else
  {
    idx = minute1 + 1;
    if (idx >= 10) idx -= 10;
    num = (min1Calibration[minute1] - m1);
    if (num > 180) num -=360;
    else if (num < -180) num += 360;
    den = min1Calibration[idx] - min1Calibration[minute1];
    if (den > 180) den -=360;
    else if (den < -180) den += 360;
    frac = abs(num / den);
  }
  float fm1 = (float)minute1 + frac;
  if (floatM1 != NULL) *floatM1 = fm1;
  if (enablePrint == 1) Serial.printf("floatM1=%0.2f m1=%0.2f minute1=%d idx=%d\n",
    fm1, m1, minute1, idx);
  //.............................................

  mdiff = 99999;
  for (i=0; i<12; i++)
  {
    float diff = m10 - min10Calibration[i];
    if (diff < -180.0) diff += 360.0;
    else if (diff > 180.0) diff -= 360.0;
    if (abs(diff) < abs(mdiff))
    {
      mdiff = diff;
      if (minute10Angle != NULL) *minute10Angle = min10Calibration[i];
      minute10 = i%6;
    }
  }
  if (m10error != NULL) *m10error = mdiff;

  if (Dprint) Serial.printf("...minute10=%d lastMinute10=%d fm1=%0.2f\n",minute10,lastMinute10,fm1);
  if ((fm1 > 9.0) && (lastMinute10 >= 0)) //Handle [n]9-->[n+1]0 minute switch over
  {
    minute10 = lastMinute10;
  }
  lastMinute10 = minute10;

  *minute = (10*minute10) + minute1;

  lastComputedClockH = *hour;
  lastComputedClockM = (float)(10 * (*minute/10)) + fm1;
  if (lastComputedClockM < 0) lastComputedClockM += 60.0;
  lastComputedClockS = (int)(lastComputedClockM * 60.0) % 60;
  if (lastComputedClockS > 60) lastComputedClockS -= 60;
  
  if (enablePrint == 1)
  {
    Serial.printf("Clock Time=%02d:%02d  ", *hour, (int)lastComputedClockM);
    Serial.printf("Error-> h=%0.2f(%d) m10=%0.2f(%d) m1=%0.2f(%d) %d %0.2f %d\n",
      *herror, (int)((*herror/360.0)*numHrStepsNom*24.0), 
      *m10error, (int)((*m10error/360.0)*numMinStepsNom*10.0), 
      *m1error, (int)((*m1error/360.0)*numMinStepsNom*10.0),
      lastComputedClockH, lastComputedClockM, lastComputedClockS);
  }
}

//--------------------------------------------------------------------------------------------
void fineSet(int show)
{
  int h, m;
  float eh, em10, em1, m10a;
  getClockTime(&h, &m, &eh, &em10, &em1, &m10a, NULL, NULL, 0);
  if (show == 1)
  {
    //Serial.printf("FineSet -> Clock Time=%02d:%02d  ", h, m);
    Serial.printf("FineSet -> Clock Time=%02d:%02d  ", h, (int)lastComputedClockM);
    Serial.printf("Errors: h=%0.2f(%d) m10=%0.2f(%d) m1=%0.2f(%d)\n",
      eh, (int)((eh/360.0)*numHrStepsNom*24.0), em10, (int)((em10/360.0)*numMinStepsNom*10.0), em1, (int)((em1/360.0)*numMinStepsNom*10.0));
  }
  correctMin = (int)((em1/360.0)*numMinStepsNom*10.0);
  correctHr = (int)((eh/360.0)*numHrStepsNom*24.0);
}

//--------------------------------------------------------------------------------------------
int syncTime(String clockTime, String nowTime)
{

   //TODO: Test this for all cases
   
  int clockHr;
  int clockMin;
  if ((clockTime != "") && (clockTime != "-1:-1"))
  {
    clockHr = getValue(clockTime, ':', 0).toInt();
    clockMin = getValue(clockTime, ':', 1).toInt();
  }
  else
  {
    if ((IsCalibrated == 1) && (disableCalibration == 0))
    {
      float eh, em10, em1, m10a;
      getClockTime(&clockHr, &clockMin, NULL, NULL, NULL, NULL, NULL, NULL, 0);
      if (clockTime == "-1:-1")
      {
        setNowTime(nowTime);
      }
    }
    else
    {
      Serial.printf("No calibration!\n");
      return(-1);
    }
  }

  setClockTime(clockHr, clockMin);
  fineSet(0);
  return(0);
}

//--------------------------------------------------------------------------------------------
void MoveHour(int num)
{
  int x;
  int y;
  digitalWrite(GPIO_EN2, LOW);
  time_stamp = micros();
  for (x=0; x<num; x++)
  {
    while (micros() - time_stamp < stepRate) ;
    digitalWrite(GPIO_ST2, HIGH);
    digitalWrite(GPIO_ST2, LOW);
    time_stamp = micros();
    yield();
    OnStepHr(x);
  }
  digitalWrite(GPIO_EN2, HIGH);
}

//--------------------------------------------------------------------------------------------
void calibrateHour(int clockHr, int digitOnly)
{
  int i;

  if (digitOnly == 1)
  {
    hourCalibration[clockHr] = getAngle(1, 0);
  }
  else
  {
    Serial.printf("Hour Cal:\n");
    for (i=0; i<24; i++)
    {
      Serial.printf(".");
      AdvanceHour();
      delay(500);
      clockHr++;
      if (clockHr >= 24) clockHr -= 24;
      hourCalibration[clockHr] = getAngle(1, 0);
    }
    Serial.printf("\nDone.\n");
  }
  
  File fp = SPIFFS.open("/hcal.txt", "w");
  if (fp)
  {
    for (i=0; i<24; i++) fp.printf("%d,%0.2f\n", i, hourCalibration[i]);
    fp.printf("1234\n");
    fp.close();
  }
}

//--------------------------------------------------------------------------------------------
void calibrateMinute(int clockMin, int digitOnly)
{
  File fp;
  int i, j;

  if (digitOnly == 1)
  {
    angleMin10[clockMin] = getAngle(2, 0);
    angleMin[clockMin] = getAngle(3, 0);
  }
  else
  {
    Serial.printf("Minute Cal:\n");
    for (i=0; i<60; i++)
    {
      Serial.printf(".");
      AdvanceMinute();
      delay(250);
      clockMin++;
      if (clockMin >= 120) clockMin -= 120;
      angleMin10[clockMin] = getAngle(2, 0);
      angleMin[clockMin] = getAngle(3, 0);
    }
    Serial.printf("\n");
    for (i=60; i<120; i++)
    {
      Serial.printf("+");
      AdvanceMinute();
      delay(250);
      clockMin++;
      if (clockMin >= 120) clockMin -= 120;
      angleMin10[clockMin] = getAngle(2, 0);
      angleMin[clockMin] = getAngle(3, 0);
    }
    Serial.printf("\nDone.\n");
  }
  
  for (j=0; j<10; j++) min1Calibration[j] = 0;
  float wrapCorrect = 0;
  
  for (i=0; i<120; i+=10)
  {
    for (j=0; j<10; j++)
    {
      if ((i > 0) && (abs(angleMin[i+j] - angleMin[i-10+j]) > 180))
      {
        if (wrapCorrect == 0) wrapCorrect = 360.0;
        else wrapCorrect = 0;
      }
      if (angleMin[j] > 180.0)
        min1Calibration[j] += (angleMin[i+j] + wrapCorrect);
      else
        min1Calibration[j] += (angleMin[i+j] - wrapCorrect);
    }
  }
  for (j=0; j<10; j++)
  {
    min1Calibration[j] /= 12.0;
    if (min1Calibration[j] < 0) min1Calibration[j] += 360.0;
    else if (min1Calibration[j] > 360.0) min1Calibration[j] -= 360.0;
  }

  j = 5;
  for (i=0; i<12; i++)
  {
    min10Calibration[i] = angleMin10[j];
    j += 10;
  }

  fp = SPIFFS.open("/mcal0.txt", "w");
  if (fp)
  {
    for (i=0; i<120; i++) fp.printf("%d,%0.2f,%0.2f\n", i, angleMin10[i], angleMin[i]);
    fp.printf("1234\n");
    fp.close();
  }

  fp = SPIFFS.open("/mcal1.txt", "w");
  if (fp)
  {
    for (i=0; i<10; i++) fp.printf("%d,%0.2f\n", i, min1Calibration[i]);
    fp.printf("1234\n");
    fp.close();
  }

  fp = SPIFFS.open("/mcal2.txt", "w");
  if (fp)
  {
    for (i=0; i<12; i++) fp.printf("%d,%0.2f\n", i, min10Calibration[i]);
    fp.printf("1234\n");
    fp.close();
  }
}

//--------------------------------------------------------------------------------------------
void MoveMinute(int num)
{
  int x;
  int y;
  digitalWrite(GPIO_EN1, LOW);
  time_stamp = micros();
  for (x=0; x<num; x++)
  {
    while (micros() - time_stamp < stepRate) ;
    digitalWrite(GPIO_ST1, HIGH);
    digitalWrite(GPIO_ST1, LOW);
    time_stamp = micros();
    yield();
      
    OnStepMin(x);
  }
  digitalWrite(GPIO_EN1, HIGH);
}

//--------------------------------------------------------------------------------------------
void ProcessSerialCommand(String commandStr)
{
  String cmdStr = getValue(commandStr,' ',0);

  if (cmdStr == "disablecal") 
  {
    disableCalibration = getValue(commandStr, ' ', 1).toInt();
  }
  if (cmdStr == "dprint") 
  {
    Dprint = getValue(commandStr, ' ', 1).toInt();
  }
  if (cmdStr == "enable") 
  {
    clockEnabled = getValue(commandStr, ' ', 1).toInt();
  }
  if (cmdStr == "setfine") 
  {
    fineSet(1);
  }
  if (cmdStr == "clocktime") 
  {
    int h, m;
    float eh, em10, em1, m10a, fH, fM1;
    getClockTime(&h, &m, &eh, &em10, &em1, &m10a, &fH, &fM1, 1);
  }
  if (cmdStr == "angles") 
  {
    printAngles();
  }
  if (cmdStr == "loadcal") 
  {
    loadCalibration();
  }
  if (cmdStr == "calh") //calh [currentClockHour] [1=SetForOnlyCurrentDigit]
  {
    int clockHr = getValue(commandStr, ' ', 1).toInt();
    int digitOnly = 0;
    if (getValue(commandStr, ' ', 2) != "")
      digitOnly = getValue(commandStr, ' ', 2).toInt();
    calibrateHour(clockHr, digitOnly);
  }
  if (cmdStr == "calm") //calm [currentClockHour] [1=SetForOnlyCurrentDigit]
  {
    int clockMin = getValue(commandStr, ' ', 1).toInt();
    int digitOnly = 0;
    if (getValue(commandStr, ' ', 2) != "")
      digitOnly = getValue(commandStr, ' ', 2).toInt();
    calibrateMinute(clockMin, digitOnly);
  }
  if (cmdStr == "time") 
  {
    String nowTime = getValue(commandStr, ' ', 1);
    setNowTime(nowTime);
  }
  if (cmdStr == "set") 
  {
    String clockTime = getValue(commandStr, ' ', 1);
    String nowTime = getValue(commandStr, ' ', 2);
    syncTime(clockTime, nowTime);
  }
  if (cmdStr == "hm") 
  {
      Serial.printf("Advance Hour & minute... ");
      AdvanceHourAndMinute();
      Serial.printf("Done.\n");
  }
  if (cmdStr == "gh") 
  {
    GoToHour(getValue(commandStr, ' ', 1).toFloat());
  }
  if (cmdStr == "h") 
  {
    String param = getValue(commandStr, ' ', 1);
    if (param == "")
    {
      Serial.printf("Advance Hour... ");
      AdvanceHour();
      Serial.printf("Done.\n");
    }
    else
    {
      int num = getValue(commandStr, ' ', 1).toInt();
      Serial.printf("Move Hour... ");
      MoveHour(num);
      Serial.printf("Done.\n");
    }
  }
  if (cmdStr == "gm") 
  {
    GoToMinute(getValue(commandStr, ' ', 1).toFloat());
  }
  if (cmdStr == "m") 
  {
    String param = getValue(commandStr, ' ', 1);
    if (param == "")
    {
      Serial.printf("Advance Minute... ");
      AdvanceMinute();
      Serial.printf("Done.\n");
    }
    else
    {
      int num = getValue(commandStr, ' ', 1).toInt();
      Serial.printf("Move Minute... ");
      MoveMinute(num);
      Serial.printf("Done.\n");
    }
  }
  if (cmdStr == "setstep") 
  {
    stepRate = getValue(commandStr, ' ', 1).toInt();
  }
  if (cmdStr == "x") 
  {
  }
}

//----------------------------------------------------------------------------------------------------------------------------
void loop() {
  int incomingByte;
  int stat;

  //--- Serial port command handling ---
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    serialBuffer[serialBufferPtr++] = incomingByte;
    if (serialBufferPtr > 60) serialBufferPtr = 60;
    if (incomingByte == '\n')
    {
      serialBuffer[serialBufferPtr-1] = 0;
      ProcessSerialCommand(serialBuffer);
      serialBufferPtr = 0; 
    }
  }
  if (serverCommand == 1)
  {
    ProcessSerialCommand(serverCommandStr);
    serverCommand = 0;
    serverStatusDisplay = String("Command status: Command Complete");
  }

  //TODO: Don't advance minute more than 50 minutes, just wait for that time to elapse

  int timeStatus = FetchTheTime(&timeinfo, &globalRawTime);
  if (((timeinfo.tm_sec == 0) || (timeinfo.tm_min == nextMinute)) && (clockEnabled == 1))
  {
    Serial.printf("Time: %02d-%02d-%02d %02d:%02d:%02d - ",
      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    if (timeinfo.tm_min == 0)
    {
      if ((IsCalibrated == 1) && (disableCalibration == 0))
      {
        GoToHour((float)timeinfo.tm_hour); 
        GoToMinute((float)timeinfo.tm_min);
        //GoToNextHourAndMinute((float)timeinfo.tm_hour, (foat)timeinfo.tm_min); //TODO
      }
      else
      {
        AdvanceHourAndMinute();
      }
    }
    else
    {
      if ((IsCalibrated == 1) && (disableCalibration == 0))
      {
        GoToMinute((float)timeinfo.tm_min);
      }
      else
      {
        AdvanceMinute();
      }
    }
    nextMinute = timeinfo.tm_min + 1;
    if (nextMinute == 60) nextMinute = 0;
    
    int h, m;
    float eh, em10, em1, m10a;
    getClockTime(&h, &m, &eh, &em10, &em1, &m10a, NULL, NULL, 0);
    Serial.printf("%d %d %0.2f(%d) %0.2f(%d) %0.2f(%d)", stepsHr, stepsMin,
      getAngle(1, 0), (int)((eh/360.0)*numHrStepsNom*24.0),
      getAngle(2, 0), (int)((em10/360.0)*numMinStepsNom*10.0),
      getAngle(3, 0), (int)((em1/360.0)*numMinStepsNom*10.0));

    if ((IsCalibrated == 1) && (disableCalibration == 0)) //TODO: This section not needed because tile would be set above?
    {
      int clockHr, clockMin;
      getClockTime(&clockHr, &clockMin, NULL, NULL, NULL, NULL, NULL, NULL, 0);
      if ((timeinfo.tm_min != clockMin) && (NTPsyncAtLeastOnce == 1))
      {
        Serial.printf(" - Time not set");
        if (ENABLE_AUTO_TIME_SET >= 1)
        {
          Serial.printf("\n");
          delay(5000);
          GoToHour((float)clockHr);
          GoToMinute((float)clockMin);

          if (ENABLE_AUTO_TIME_SET == 1) ENABLE_AUTO_TIME_SET = 0;
        }
      }
    }
    Serial.printf("\n");
  }

  #if (1)
  if (WiFi.status() != WL_CONNECTED)
  {
    if ((lastReconnectAttempt == 0) || ((unsigned long)(millis() - lastReconnectAttempt) >= (2*60*1000))) //Try to reconnect every 2 minutes
    {
      InitializeWiFi();
      if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        lastReconnectAttempt = millis();
        Serial.println(F("STA: Failed!"));
      }
      else
      {
        lastReconnectAttempt = 0;
      }
    }
  }
#endif

#ifndef ESP32
  MDNS.update();
#endif
}
