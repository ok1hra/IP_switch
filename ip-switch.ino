/*

IP Switch
----------------------
https://remoteqth.com/wiki/index.php?page=IP+Switch+with+ESP32-GATEWAY

___               _        ___ _____ _  _
| _ \___ _ __  ___| |_ ___ / _ \_   _| || |  __ ___ _ __
|   / -_) '  \/ _ \  _/ -_) (_) || | | __ |_/ _/ _ \ '  \
|_|_\___|_|_|_\___/\__\___|\__\_\|_| |_||_(_)__\___/_|_|_|


This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Send test packet
  echo -n -e '\x00ms:bro;' | nc -u -w1 192.168.1.23 88 | hexdump -C
Remote USB access
  screen /dev/ttyUSB0 115200

HARDWARE ESP32-GATEWAY/ESP32-POE

Changelog:
2020-09 - fix enter number in CLI
          add WatchDogTimer
2020-08 - show used gpio in CLI
2020-03 - support XL switch on ESP32-POE https://www.olimex.com/Products/IoT/ESP32/ESP32-POE/
        - support Icom CI-V Band Decoder
        - support TFT LCD by https://www.olimex.com/Products/Modules/LCD/MOD-LCD2-8RTP/
2020-01 - telnet access key generate automaticaly
        - redesign CLI enter value
2019-10 - add #define PCBrev04
2019-09 - disable start snake, show key in serial terminal
2019-06 - telnet with loggin support
2019-05 - reboot and clear output watchdog
2019-04 - group button support (idea TNX SM0MDG)
        - add serial to IP interface (3V3 rx gpio16, tx gpio17)
        - set Switch incoming UDP port from CLI
2019-03 - multi control support (idea TNX SM0MDG)
2019-01 - add CLI
        - redesign UDP communications
        - NET-ID prefix/sufix support
2018-12 - add web status page
        - add OTA
2018-09 - add IP switch support
        - blink LED after DHCP connect and receive sync packet
2018-08 add Band decoder support

ToDo
- esp32 watchdog
- telnet inactivity watchdog > close
- custom name for outputs
- https://github.com/espressif/arduino-esp32/blob/master/libraries/Update/examples/AWS_S3_OTA_Update/AWS_S3_OTA_Update.ino
- https://randomnerdtutorials.com/esp32-esp8266-relay-web-server/
*/
//-------------------------------------------------------------------------------------------------------

// #define PCBrev04                    // Enable for ESP32-GATEWAY PCB revision 0.4 or later
// #define XLswitch                       // Enable for XL switch hardware with ESP32-POE
const char* REV = "20200915";
const char* otaPassword = "remoteqth";

//-------------------------------------------------------------------------------------------------------

int XLswitchANT = 16;                  // number of antenna output
int XLswitchTRX = 4;                   // number of trx output
byte XLswitchOutputs[4][2];          // 4 trx, 2 byte = 16 bit

// #define XLswitchCIV           // Icom CIV
#if defined(XLswitchCIV)
  #define REQUEST        500    // [ms] use TXD output for sending frequency request
  #define CIV_ADRESS    0x56    // CIV input HEX Icom adress (0x is prefix)
  #define CIV_ADR_OUT   0x56    // CIV output HEX Icom adress (0x is prefix)
  int fromAdress = 0xE0;              // 0E
  byte rdI[11];   //read data icom
  String rdIS;    //read data icom string
  long freqPrev1;
  byte incomingByte = 0;
  int state = 1;  // state machine
  bool StateMachineEnd = false;
  int BAND = 0;
  int previousBAND = -1;
  long freq = 0;
  const long Freq2Band[14][2] = {/*
  Freq Hz from       to   Band number
  */   {1810000,   2000000},  // #1 [160m]
       {3500000,   3800000},  // #2  [80m]
       {7000000,   7200000},  // #3  [40m]
      {10100000,  10150000},  // #4  [30m]
      {14000000,  14350000},  // #5  [20m]
      {18068000,  18168000},  // #6  [17m]
      {21000000,  21450000},  // #7  [15m]
      {24890000,  24990000},  // #8  [12m]
      {28000000,  29700000},  // #9  [10m]
      {50000000,  52000000},  // #10  [6m]
      {70000000,  72000000},  // #11  [4m]
     {144000000, 146000000},  // #12  [2m]
     {430000000, 440000000},  // #13  [70cm]
     {1240000000, 1300000000},  // #14  [23cm]
     // {2300000000, 2450000000},  // #15  [13cm]
     // {3300000000, 3500000000},  // #16  [9cm]
     // // {5650000000, 5850000000},  // #16  [6cm]
  };
  long RequestTimeout[2]={0,
    #if defined(REQUEST)
      REQUEST
    #else
      0
    #endif
  };
#endif

#define ETHERNET                       // Enable ESP32 ethernet (DHCP IPv4)
// #define WIFI                        // Enable ESP32 WIFI (DHCP IPv4)
const char* ssid     = "";
const char* password = "";

char key[100];
byte InputByte[21];
// #define Ser2net                  // Serial to ip proxy - DISABLE if board revision 0.3 or lower
#define EnableOTA                // Enable flashing ESP32 Over The Air
bool HW_BCD_SW = 0;              // enable hardware ID board bcd switch (disable if not installed)
int NumberOfEncoderOutputs = 8;  // 2-16
long HW_BCD_SWTimer[2]{0,3000};
byte NET_ID = 0x00;              // Unique ID number [0-F] hex format - over BCD switch
bool EnableSerialDebug     = 0;
#define HTTP_SERVER_PORT  80     // Web server port
int IncomingSwitchUdpPort;
#define ShiftOut                 // Enable ShiftOut register
#define UdpAnswer                // Send UDP answer confirm packet
int BroadcastPort;               // destination broadcast packet port
bool EnableGroupPrefix = 0;      // enable multi controller control
bool EnableGroupButton = 0;      // group to one from
unsigned int GroupButton[8]={1,2,3,4,5,6,7,8};
byte DetectedRemoteSw[16][4];
unsigned int DetectedRemoteSwPort[16];

const int SERIAL_BAUDRATE = 115200; // serial debug baudrate
int SERIAL1_BAUDRATE; // serial1 to IP baudrate
// #if defined(Ser2net) && !defined(XLswitch)
  int incomingByte = 0;   // for incoming serial data
// #endif

#if !defined(Ser2net) && !defined(XLswitch)
  const int BCD[4] = {34, 33, 32, 10};  // BCD encoder PINs
#endif

int i = 0;
#include <WiFi.h>
#include <WiFiUdp.h>
#include "EEPROM.h"
#define EEPROM_SIZE 141   /*
0    -listen source
1    -net ID
2    -encoder range
3    -HW_BCD_SW
4    -EnableGroupPrefix
5    -EnableGroupButton
6-13 -GroupButton
14-17  - SERIAL1_BAUDRATE
18-21 - SerialServerIPport
22-25 - IncomingSwitchUdpPort
26-29 - RebootWatchdog
30-33 - OutputWatchdog
34    - Bank0 storage
35    - Bank1 storage
36    - Bank2 storage
37-40 - Authorised telnet client IP
41-140 - Authorised telnet client key
*/
unsigned int RebootWatchdog;
unsigned int OutputWatchdog;
unsigned long WatchdogTimer=0;

// 73 seconds WDT (WatchDogTimer)
#include <esp_task_wdt.h>
#define WDT_TIMEOUT 73
long WdtTimer=0;

WiFiServer server(HTTP_SERVER_PORT);
bool DHCP_ENABLE = 1;
// Client variables
char linebuf[80];
int charcount=0;
//Are we currently connected?
boolean connected = false;
//The udp library class
WiFiUDP UdpCommand;
uint8_t buffer[50] = "";
unsigned char packetBuffer[10];
int UDPpacketSize;
byte TxUdpBuffer[8];
#include <ETH.h>
static bool eth_connected = false;
IPAddress RemoteSwIP(0, 0, 0, 0);         // remote UDP IP switch - set from UDP DetectRemote array
int RemoteSwPort         = 0;             // remote UDP IP switch port
String HTTP_req;
#if defined(EnableOTA)
  #include <ESPmDNS.h>
  #include <ArduinoOTA.h>
#endif

#if defined(Ser2net) && !defined(XLswitch)
  #define RX1 16
  #define TX1 17
  HardwareSerial Serial_one(1);
#endif
#if defined(XLswitchCIV) && defined(XLswitch)
  #define RX1 35
  #define TX1 33
  HardwareSerial Serial_one(1);
#endif
#if defined(PCBrev04)
    const int ShiftOutDataPin = 32;
    const int ShiftOutLatchPin = 33;
    const int ShiftOutClockPin = 5;
    byte ShiftOutByte[3];
#elif defined(XLswitch)
  const int ShiftOutDataPin = 33;
  const int ShiftOutLatchPin = 32;
  const int ShiftOutClockPin = 4;
  byte ShiftOutByte[5];
#else
    const int ShiftOutDataPin = 17;
    const int ShiftOutLatchPin = 16;
    const int ShiftOutClockPin = 5;
    byte ShiftOutByte[3];
#endif
#if defined(XLswitch)
  const int StatusLedAPin = 5;
  const int StatusLedBPin = 13;
  bool StatusLedB = false;
  long StatusLedBTimer[2] = {0,500};
  int LcdNeedRefresh = 100;

  long LcdFpsTestTimer[2] = {0,1};
  int LcdFpsTestCounter[2]={0,0};
  bool LcdFpsTestForward=true;
// https://learn.adafruit.com/adafruit-gfx-graphics-library/using-fonts

  /***************************************************
    This is an example made by Adafruit and modifed by Olimex for MOD-LCD2.8RTP
    This demo was tested with Olimex MOD-LCD2.8RTP and ESP32-EVB and OLIMEXINO-2560.
    The boards were connected via UEXT connector and cable.

    Make sure to establish proper hardware connections with your board.
    The display requires SPI, the touschreen I2C. Refer to Board_Pinout.h.

    The original example is a GFX example for the Adafruit ILI9341 Breakout and Shield
    ----> http://www.adafruit.com/products/1651

    Check out the link above for Adafruit's tutorials and wiring diagrams
    These displays use SPI to communicate, 4 or 5 pins are required to
    interface (RST is optional)
    Adafruit invests time and resources providing the open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    Written by Limor Fried/Ladyada for Adafruit Industries.
    MIT license, all text above must be included in any redistribution
   ****************************************************/

   // In order to work you have to install Adafruit GFX Library
   // To do so go to:
   // Main menu --> Sketch --> Inlcude Librariy --> Manage Libraries...
   // In the search box filter "Adafruit GFX Library" and install it
   // Tested with version 1.2.3 of the library

  // #include "Board_Pinout.h"
  #include "SPI.h"
  #include "Adafruit_GFX.h"
  #include "Adafruit_ILI9341.h"
  #include "Wire.h"
  #include "Adafruit_STMPE610.h"

  // This is calibration data for the raw touch data to the screen coordinates
  #define TS_MINX 290
  #define TS_MINY 285
  #define TS_MAXX 7520
  #define TS_MAXY 7510
  #define TS_I2C_ADDRESS 0x4d

  // This is pinouts for ESP32-EVB
  #define TFT_DC 15
  #define TFT_CS 5
  #define TFT_MOSI 2
  #define TFT_CLK 14

  Adafruit_STMPE610 ts = Adafruit_STMPE610();

  // Size of the color selection boxes and the paintbrush size
  #define BOXSIZE 40

  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

  uint8_t tp[5];

  /*
  #define ILI9341_BLACK       0x0000  ///<   0,   0,   0
  #define ILI9341_NAVY        0x000F  ///<   0,   0, 123
  #define ILI9341_DARKGREEN   0x03E0  ///<   0, 125,   0
  #define ILI9341_DARKCYAN    0x03EF  ///<   0, 125, 123
  #define ILI9341_MAROON      0x7800  ///< 123,   0,   0
  #define ILI9341_PURPLE      0x780F  ///< 123,   0, 123
  #define ILI9341_OLIVE       0x7BE0  ///< 123, 125,   0
  #define ILI9341_LIGHTGREY   0xC618  ///< 198, 195, 198
  #define ILI9341_DARKGREY    0x7BEF  ///< 123, 125, 123
  #define ILI9341_BLUE        0x001F  ///<   0,   0, 255
  #define ILI9341_GREEN       0x07E0  ///<   0, 255,   0
  #define ILI9341_CYAN        0x07FF  ///<   0, 255, 255
  #define ILI9341_RED         0xF800  ///< 255,   0,   0
  #define ILI9341_MAGENTA     0xF81F  ///< 255,   0, 255
  #define ILI9341_YELLOW      0xFFE0  ///< 255, 255,   0
  #define ILI9341_WHITE       0xFFFF  ///< 255, 255, 255
  #define ILI9341_ORANGE      0xFD20  ///< 255, 165,   0
  #define ILI9341_GREENYELLOW 0xAFE5  ///< 173, 255,  41
  #define ILI9341_PINK        0xFC18  ///< 255, 130, 198
  */
#endif

#define MAX_SRV_CLIENTS 1
int SerialServerIPport;
// WiFiServer SerialServer(SerialServerIPport);
WiFiServer SerialServer;
WiFiClient SerialServerClients[MAX_SRV_CLIENTS];

int TelnetServerIPport = 23;
WiFiServer TelnetServer;
WiFiClient TelnetServerClients[MAX_SRV_CLIENTS];
IPAddress TelnetServerClientAuth;
bool TelnetAuthorized = false;
int TelnetAuthStep=0;
int TelnetAuthStepFails=0;
int TelnetLoginFails=0;
long TelnetLoginFailsBanTimer[2]={0,600000};
int RandomNumber;

#if !defined(XLswitch)
  long MeterRefreshTimer[2]={0,100};
  const int MeterPin[4] = {34,35,36,39};
  int MeterValue[4];
#endif

int CompareInt;
//-------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  while(!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  #if defined(ShiftOut)
    pinMode(ShiftOutLatchPin, OUTPUT);
    pinMode(ShiftOutClockPin, OUTPUT);
    pinMode(ShiftOutDataPin, OUTPUT);
  #endif

  #if !defined(Ser2net) && !defined(XLswitch)
    for (int i = 0; i < 4; i++) {
     pinMode(BCD[i], INPUT);
    }
  #endif

  #if defined(XLswitch)
    pinMode(StatusLedAPin, OUTPUT);
      digitalWrite(StatusLedAPin, LOW);
    pinMode(StatusLedBPin, OUTPUT);
      digitalWrite(StatusLedBPin, LOW);

    delay(1000);
    tft.begin();
    Wire.begin();
    pinMode(TFT_DC, OUTPUT);
    // read diagnostics (optional but can help debug problems)
    //uint8_t x = tft.readcommand8(ILI9341_RDMODE);
    delay(1000);
    ts.begin(TS_I2C_ADDRESS);
    tft.fillScreen(ILI9341_BLACK);
  #endif

  // Listen source
  if (!EEPROM.begin(EEPROM_SIZE)){
    if(EnableSerialDebug==1){
      Serial.println("failed to initialise EEPROM"); delay(1);
    }
  }
  // 0-listen source
  TxUdpBuffer[2] = EEPROM.read(0);
  if(TxUdpBuffer[2]=='o'||TxUdpBuffer[2]=='r'||TxUdpBuffer[2]=='m'||TxUdpBuffer[2]=='e'){
    // OK
  }else{
    TxUdpBuffer[2]='n';
  }

  // 1-net ID
  #if !defined(Ser2net) && !defined(XLswitch)
    if(HW_BCD_SW==true){
      bitClear(NET_ID, 0);
      bitClear(NET_ID, 1);
      bitClear(NET_ID, 2);
      bitClear(NET_ID, 3);
      NET_ID = NET_ID | GetBoardId();
      TxUdpBuffer[0] = NET_ID;
    }else{
  #endif
      NET_ID = EEPROM.read(1);
      TxUdpBuffer[0] = NET_ID;
  #if !defined(Ser2net) && !defined(XLswitch)
    }
  #endif

  // 2-encoder range
  NumberOfEncoderOutputs = EEPROM.read(2);
  if(NumberOfEncoderOutputs < 2 || NumberOfEncoderOutputs > 0x0f){
    NumberOfEncoderOutputs=8;
  }

  #if defined(Ser2net) && !defined(XLswitch)
    HW_BCD_SW = 0;
  #else
    // 3-HW_BCD_SW
    Serial.print("HW BCD on/off ");
    if(EEPROM.read(3)<2){
      HW_BCD_SW = EEPROM.read(3);
      Serial.println("read from EEPROM");
    }else{
      Serial.println("set to OFF");
    }
  #endif

  // 4-EnableGroupPrefix
  // Serial.print("Enable group NET-ID prefix ");
  if(EEPROM.read(4)<2){
    EnableGroupPrefix=EEPROM.read(4);
  }
  // Serial.println(EnableGroupPrefix);

  // 5-EnableGroupButton
  if(EEPROM.read(5)<2){
    EnableGroupButton=EEPROM.read(5);
  }

  // 6-13 ButtonGroup
  if(EnableGroupButton==true){
    for (int i = 0; i < 8; i++) {
      if(EEPROM.read(6+i)<9){
        GroupButton[i]=EEPROM.read(6+i);
      }
    }
  }

  SERIAL1_BAUDRATE=EEPROM.readInt(14);
  SerialServerIPport=EEPROM.readInt(18);
  IncomingSwitchUdpPort=EEPROM.readInt(22);
  BroadcastPort=IncomingSwitchUdpPort;
  RebootWatchdog=EEPROM.readUInt(26);
  if(RebootWatchdog>10080){
    RebootWatchdog=0;
  }
  OutputWatchdog=EEPROM.readUInt(30);
  if(OutputWatchdog>10080){
    OutputWatchdog=0;
  }
  if(RebootWatchdog>0){
    ShiftOutByte[0]=EEPROM.readByte(34);
    ShiftOutByte[1]=EEPROM.readByte(35);
    ShiftOutByte[2]=EEPROM.readByte(36);
  }
  TelnetServerClientAuth[0]=EEPROM.readByte(37);
  TelnetServerClientAuth[1]=EEPROM.readByte(38);
  TelnetServerClientAuth[2]=EEPROM.readByte(39);
  TelnetServerClientAuth[3]=EEPROM.readByte(40);

  // 41-140 key
  // if clear, generate
  if(EEPROM.readByte(41)==255 && EEPROM.readByte(140)==255){
    Serial.println();
    Serial.println("  ** GENERATE KEY **");
    for(int i=41; i<141; i++){
      EEPROM.writeChar(i, RandomChar());
      Serial.print("*");
    }
    Serial.println();
    // to defaults
    IncomingSwitchUdpPort=88;
    EEPROM.writeInt(22, IncomingSwitchUdpPort);
    BroadcastPort=IncomingSwitchUdpPort;
    EEPROM.commit();
  }
  // read
  for(int i=41; i<141; i++){
    key[i-41] = EEPROM.readChar(i);
  }

  // if(EnableSerialDebug==1){
    Serial.println();
    Serial.print("Version: ");
    Serial.println(REV);
    Serial.println("===============================");
    Serial.print("SLAVE DEVICE NET-ID: 0x");
    if(NET_ID <=0x0f){
      Serial.print(F("0"));
    }
    Serial.println(NET_ID, HEX);
    Serial.print("Listen MASTER: ");
    if(TxUdpBuffer[2] == 'o'){
      Serial.println("Open Interface III");
    }
    if(TxUdpBuffer[2] == 'r' ){
      Serial.println("Band decoder MK2");
    }
    if(TxUdpBuffer[2] == 'm' ){
      Serial.println("IP switch master");
    }
    #if !defined(XLswitch)
      if(TxUdpBuffer[2] == 'e' ){
        Serial.println("Meter");
        for (int i = 0; i < 4; i++) {
          pinMode(MeterPin[i], INPUT);
        }
      }
    #endif
    if(TxUdpBuffer[2] == 'n' ){
      Serial.println("none");
    }
    Serial.println("===============================");
    Serial.println("  press '?' for list commands");
    Serial.println();
    Serial.println();
  // }

  #if defined(WIFI)
    if(EnableSerialDebug==1){
      Serial.print("WIFI Connecting to ");
      Serial.print(ssid);
    }
    WiFi.begin(ssid, password);
    // attempt to connect to Wifi network:
    while(WiFi.status() != WL_CONNECTED) {
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      delay(500);
      if(EnableSerialDebug==1){
        Serial.print(".");
      }
    }
    // LED1status = !LED1status;
    // digitalWrite(LED1, LED1status);           // signalize wifi connected
    if(EnableSerialDebug==1){
      Serial.println("");
      Serial.println("WIFI connected");
      Serial.print("WIFI IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("WIFI dBm: ");
      Serial.println(WiFi.RSSI());
    }
    #if !defined(Ser2net) && !defined(XLswitch)
      pinMode(BCD[1], OUTPUT);  // LED
      digitalWrite(BCD[1], HIGH);
      delay(100);
      digitalWrite(BCD[1], LOW);
      delay(100);
      digitalWrite(BCD[1], HIGH);
      delay(100);
      digitalWrite(BCD[1], LOW);
      delay(100);
      pinMode(BCD[1], INPUT);
    #endif
  #endif

  #if defined(ETHERNET)
    WiFi.onEvent(EthEvent);
    ETH.begin();
    if(DHCP_ENABLE==false){
      ETH.config(IPAddress(192, 168, 1, 188), IPAddress(192, 168, 1, 255),IPAddress(255, 255, 255, 0),IPAddress(8, 8, 8, 8));
      //config(IPAddress local_ip, IPAddress gateway, IPAddress subnet, IPAddress dns1 = (uint32_t)0x00000000, IPAddress dns2 = (uint32_t)0x00000000);
    }

  #endif
    server.begin();
    UdpCommand.begin(IncomingSwitchUdpPort);    // incoming udp port

  #if defined(EnableOTA)
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);
    // Hostname defaults to esp3232-[MAC]

    #if defined(XLswitch)
      String StringHostname = "XL-switchID-"+String(NET_ID, HEX);
    #else
      String StringHostname = "IP-relayID-"+String(NET_ID, HEX);
    #endif
    char copy[13];
    StringHostname.toCharArray(copy, 13);

    ArduinoOTA.setHostname(copy);
    ArduinoOTA.setPassword(otaPassword);
    // $ echo password | md5sum
    // ArduinoOTA.setPasswordHash("5587ba7a03b12a409ee5830cea97e079");
    ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
        #if defined(XLswitch)
          tft.fillScreen(ILI9341_ORANGE);
          tft.setRotation(1);
          tft.setTextColor(ILI9341_WHITE);
          tft.setCursor(155,30);
          tft.setTextSize(6);
          tft.println("!");
          tft.setCursor(70,100);
          tft.setTextSize(3);
          tft.println("OTA update");
          tft.drawRect(57, 150, 206, 16, ILI9341_WHITE);
          tft.setCursor(100,180);
          tft.setTextSize(1);
          tft.println("RemoteQTH.com firmware");
        #endif
      })
      .onEnd([]() {
        Serial.println("\nEnd");
        #if defined(XLswitch)
          tft.fillScreen(ILI9341_BLACK);
        #endif
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        #if defined(XLswitch)
          tft.fillRect(60, 153, (progress / (total / 100))*2, 10, ILI9341_WHITE);
        #endif
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
      });

    ArduinoOTA.begin();
  #endif

  #if defined(Ser2net) && !defined(XLswitch)
    Serial_one.begin(SERIAL1_BAUDRATE, SERIAL_8N1, RX1, TX1);
  // Serial2.begin(9600);
  SerialServer.begin(SerialServerIPport);
  SerialServer.setNoDelay(true);
  #endif

  #if defined(XLswitchCIV) && defined(XLswitch)
    Serial_one.begin(SERIAL1_BAUDRATE, SERIAL_8N1, RX1, TX1);
  #endif
  #if defined(XLswitch)
    digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
    shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, B10000000);
    shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, B00000000);
    shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, B00000000);
    shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, B00000000);
    shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, B00000000);
    digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
  #endif

  TelnetServer.begin(TelnetServerIPport);
  // TelnetlServer.setNoDelay(true);

  // WDT
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  WdtTimer=millis();


}

//-------------------------------------------------------------------------------------------------------

void loop() {
  // blank loop 80us
  SerialToIp();
  http();
  RX_UDP();
  RX_UDP_XLswitch();
  CLI();
  Telnet();
  CheckNetId();
  Meter();
  Watchdog();
  CIV();
  LcdDisplay();
  // LcdFpsTest();
  #if defined(EnableOTA)
   ArduinoOTA.handle();
  #endif
  // Demo();
}
// SUBROUTINES -------------------------------------------------------------------------------------------------------
void CIV(){
  #if defined(XLswitchCIV) && defined(XLswitch)
    if(TxUdpBuffer[2]=='I'){
      if (Serial_one.available()) {
          incomingByte = Serial_one.read();
          // if(EnableSerialDebug==1){
          //   Prn(3, 0, String(incomingByte));
          //   // Serial.print(incomingByte);
          //   // Serial.print("|");
          //   // Serial.println(incomingByte, HEX);
          // }
          icomSM(incomingByte);
          rdIS="";
          // if(rdI[10]==0xFD){    // state machine end
          if(StateMachineEnd == true){    // state machine end
            StateMachineEnd = false;
            for (int i=9; i>=5; i-- ){
                if (rdI[i] < 10) {            // leading zero
                    rdIS = rdIS + 0;
                }
                rdIS = rdIS + String(rdI[i], HEX);  // append BCD digit from HEX variable to string
            }
            freq = rdIS.toInt();
            if(EnableSerialDebug==1){
              Prn(3, 0, String(freq));
              Prn(3, 1, " Hz");
            }
            // Serial.println(freq);
            // Serial.println("-------");
            FreqToBandRules();
            // bandSET();
            LcdNeedRefresh=1;
            RequestTimeout[0]=millis();
          }
      }

      #if defined(REQUEST)
        if(REQUEST > 0 && (millis() - RequestTimeout[0] > RequestTimeout[1])){
          txCIV(3, 0, CIV_ADRESS);  // ([command], [freq]) 3=read
          RequestTimeout[0]=millis();
        }
      #endif
    }
  #endif
}

//---------------------------------------------------------------------------------------------------------
#if defined(XLswitchCIV) && defined(XLswitch)

    //---------------------------------------------------------------------------------------------------------
    void txCIV(int commandCIV, long dataCIVtx, int toAddress) {
        //Serial.flush();
        Serial_one.write(254);                                    // FE
        Serial_one.write(254);                                    // FE
        Serial_one.write(toAddress);                              // to adress
        Serial_one.write(fromAdress);                             // from OE
        Serial_one.write(commandCIV);                             // data
        if (dataCIVtx != 0){
            String freqCIVtx = String(dataCIVtx);             // to string
            String freqCIVtxPart;
            while (freqCIVtx.length() < 10) {                 // leding zeros
                freqCIVtx = 0 + freqCIVtx;
            }
            for (int x=8; x>=0; x=x-2){                       // loop for 5x2 char [xx xx xx xx xx]
                freqCIVtxPart = freqCIVtx.substring(x,x+2);   // cut freq to five part
                    Serial_one.write(hexToDec(freqCIVtxPart));    // HEX to DEC, because write as DEC format from HEX variable
            }
        }
        Serial_one.write(253);                                    // FD
        // Serial.flush();
        while(Serial_one.available()){        // clear buffer
          Serial_one.read();
        }
        if(EnableSerialDebug==1){
          Prn(3, 1, "FEFE"+String(toAddress, HEX)+String(fromAdress, HEX)+String(commandCIV, HEX)+"FD");
        }
    }

    //---------------------------------------------------------------------------------------------------------
    unsigned int hexToDec(String hexString) {
        unsigned int decValue = 0;
        int nextInt;
        for (int i = 0; i < hexString.length(); i++) {
            nextInt = int(hexString.charAt(i));
            if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
            if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
            if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
            nextInt = constrain(nextInt, 0, 15);
            decValue = (decValue * 16) + nextInt;
        }
        return decValue;
    }

    //---------------------------------------------------------------------------------------------------------
    void icomSM(byte b){      // state machine
        // This filter solves read from 0x00 0x05 0x03 commands and 00 E0 F1 address used by software
        // Serial.print(b, HEX);
        // Serial.print(" | ");
        // Serial.println(state);
        switch (state) {
            case 1: if( b == 0xFE ){ state = 2; rdI[0]=b; rdI[10]=0x00; }; break;
            case 2: if( b == 0xFE ){ state = 3; rdI[1]=b; }else{ state = 1;}; break;
            // addresses that use different software 00-trx, e0-pc-ale, winlinkRMS, f1-winlink trimode
            case 3: if( b == 0x00 || b == 0xE0 || b == 0x0E || b == 0xF1 ){ state = 4; rdI[2]=b;                       // choose command $03
            }else if( b == CIV_ADRESS ){ state = 6; rdI[2]=b;
                    }else if( b == 0xFE ){ state = 3; rdI[1]=b;      // FE (3x reduce to 2x)
                    }else{ state = 1;}; break;                       // or $05

            case 4: if( b == CIV_ADRESS ){ state = 5; rdI[3]=b; }else{ state = 1;}; break;                      // select command $03
            case 5: if( b == 0x00 || b == 0x03 ){state = 8; rdI[4]=b;  // freq
                    }else if( b == 0x04 ){state = 14; rdI[4]=b;        // mode
                    }else if( b == 0xFE ){ state = 2; rdI[0]=b;        // FE
                    }else{ state = 1;}; break;

            case 6: if( b == 0x00 || b == 0xE0 || b == 0xF1 ){ state = 7; rdI[3]=b; }else{ state = 1;}; break;  // select command $05
            case 7: if( b == 0x00 || b == 0x05 ){ state = 8; rdI[4]=b; }else{ state = 1;}; break;

            case 8: if( b <= 0x99 ){state = 9; rdI[5]=b;             // 10Hz 1Hz
                    }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                    }else{state = 1;}; break;
            case 9: if( b <= 0x99 ){state = 10; rdI[6]=b;            // 1kHz 100Hz
                    }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                    }else{state = 1;}; break;
           case 10: if( b <= 0x99 ){state = 11; rdI[7]=b;            // 100kHz 10kHz
                    }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                    }else{state = 1;}; break;
           case 11: if( b <= 0x52 ){state = 12; rdI[8]=b;            // 10MHz 1Mhz
                    }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                    }else{state = 1;}; break;
           case 12: if( b <= 0x01 || b == 0x04){state = 13; rdI[9]=b; // 1GHz 100MHz  <-- 1xx/4xx MHz limit
                    }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                    }else{state = 1;}; break;
           case 13: if( b == 0xFD ){state = 1; rdI[10]=b; StateMachineEnd = true;
                    }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                    }else{state = 1; rdI[10] = 0x00;}; break;

           case 14: if( b <= 0x12 ){state = 15; rdI[5]=b;
                    }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                    }else{state = 1;}; break;   // Mode
           case 15: if( b <= 0x03 ){state = 16; rdI[6]=b;
                    }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                    }else{state = 1;}; break;   // Filter
           case 16: if( b == 0xFD ){state = 1; rdI[7]=b;
                    }else if( b == 0xFE ){ state = 2; rdI[0]=b;      // FE
                    }else{state = 1; rdI[7] = 0;}; break;
        }
    }
    //-------------------------------------------------------------------------------------------------------
    void FreqToBandRules(){
             if (freq >=Freq2Band[0][0] && freq <=Freq2Band[0][1] )  {BAND=1;}  // 160m
        else if (freq >=Freq2Band[1][0] && freq <=Freq2Band[1][1] )  {BAND=2;}  //  80m
        else if (freq >=Freq2Band[2][0] && freq <=Freq2Band[2][1] )  {BAND=3;}  //  40m
        else if (freq >=Freq2Band[3][0] && freq <=Freq2Band[3][1] )  {BAND=4;}  //  30m
        else if (freq >=Freq2Band[4][0] && freq <=Freq2Band[4][1] )  {BAND=5;}  //  20m
        else if (freq >=Freq2Band[5][0] && freq <=Freq2Band[5][1] )  {BAND=6;}  //  17m
        else if (freq >=Freq2Band[6][0] && freq <=Freq2Band[6][1] )  {BAND=7;}  //  15m
        else if (freq >=Freq2Band[7][0] && freq <=Freq2Band[7][1] )  {BAND=8;}  //  12m
        else if (freq >=Freq2Band[8][0] && freq <=Freq2Band[8][1] )  {BAND=9;}  //  10m
        else if (freq >=Freq2Band[9][0] && freq <=Freq2Band[9][1] ) {BAND=10;}  //   6m
        else if (freq >=Freq2Band[10][0] && freq <=Freq2Band[10][1] ) {BAND=11;}  // 2m
        else if (freq >=Freq2Band[11][0] && freq <=Freq2Band[11][1] ) {BAND=12;}  // 70cm
        else {BAND=0;}                                                // out of range
    }

    //-------------------------------------------------------------------------------------------------------
#endif

//-------------------------------------------------------------------------------------------------------

#if defined(XLswitch)
  void LcdFpsTest() {
    // tft.fillScreen(ILI9341_BLACK);
    tft.setRotation(1);

    // if(millis()-LcdFpsTestTimer[0]>LcdFpsTestTimer[1]){
    //   tft.setTextColor(ILI9341_WHITE);
    //   tft.fillRect(80, 30, 200, 30, ILI9341_BLACK);
    //   tft.setCursor(80,30);
    //   tft.setTextSize(4);
    //   tft.print("fps ");
    //   tft.print(LcdFpsTestCounter[0]);
    //   LcdFpsTestTimer[0]=millis();
    //   LcdFpsTestCounter[0]=0;
    // }
    //
    // // tft.drawRect(57, 150, 206, 16, ILI9341_WHITE);
    // tft.fillRect(60+LcdFpsTestCounter[1], 153, 1, 10, ILI9341_WHITE);
    //
    // tft.fillRect(60+(200-LcdFpsTestCounter[1]), 183, 2, 10, ILI9341_WHITE);
    // tft.fillRect(60+(200-LcdFpsTestCounter[1])+2, 183, 2, 10, ILI9341_BLACK);
    // LcdFpsTestCounter[0]++;
    // LcdFpsTestCounter[1]++;
    // if(LcdFpsTestCounter[1]>200){
    //   tft.fillRect(57, 150, 206, 16, ILI9341_BLACK);
    //   tft.fillRect(57, 180, 206, 16, ILI9341_BLACK);
    //   LcdFpsTestCounter[1]=0;
    // }



    if(LcdFpsTestCounter[0]>2 || LcdFpsTestCounter[0]<1){
      LcdFpsTestForward=!LcdFpsTestForward;
    }
    if(LcdFpsTestForward==true){
      LcdFpsTestCounter[0]--;
    }else{
      LcdFpsTestCounter[0]++;
    }
      tft.setTextColor(ILI9341_WHITE);
      tft.fillRect(80, 30, 200, 30, ILI9341_BLACK);
      tft.setCursor(80,30);
      tft.setTextSize(4);
      tft.print(LcdFpsTestCounter[0]);
      tft.print(" ms");


      tft.fillRect(57, 150, 206, 16, ILI9341_BLACK);
      for (int i = 0; i < 200; i++) {
        tft.fillRect(60+i, 153, 1, 10, ILI9341_WHITE);
        tft.fillRect(60+200-i, 183, 1, 10, ILI9341_WHITE);
        delay(LcdFpsTestCounter[0]);
      }
      for (int i = 200; i > 0; i--) {
        tft.fillRect(60+i, 153, 1, 10, ILI9341_BLACK);
        tft.fillRect(60+200-i, 183, 1, 10, ILI9341_BLACK);
        delay(LcdFpsTestCounter[0]);
      }

  }
#endif
//-------------------------------------------------------------------------------------------------------

void LcdDisplay() { // 320x240 px
  #if defined(XLswitch)
    if(LcdNeedRefresh>0){
      int LineSpace = (226-18)/XLswitchTRX;
      int RowSpace = 320/XLswitchANT;


      // default
      if(LcdNeedRefresh==100){
        // Clear Screen
        tft.fillScreen(ILI9341_BLACK);
        tft.setRotation(1);

        tft.fillRect(0, 0, 320, 18, ILI9341_DARKGREY);
        tft.setTextColor(ILI9341_ORANGE);
        tft.setCursor(0,0);
        tft.setTextSize(2);
        tft.print("XLswitch");
        if(TxUdpBuffer[2]=='I'){
          tft.setCursor(260,0);
          tft.setTextColor(ILI9341_LIGHTGREY);
          tft.setTextSize(2);
          tft.print("Icom");
        }else{
          tft.setCursor(250,0);
          tft.setTextColor(ILI9341_LIGHTGREY);
          tft.setTextSize(2);
          tft.print("ID-");
          tft.print(String(IdPrefix(NET_ID), HEX) );
          tft.print(String(IdSufix(NET_ID), HEX) );
        }
        // tft.drawLine(0,18,340,18, ILI9341_LIGHTGREY);

        // lines + numbers
        for (int i = 0; i < XLswitchTRX+1; i++) {
          tft.drawLine(0,18+i*LineSpace,340,18+i*LineSpace, ILI9341_LIGHTGREY);
          if(i<XLswitchTRX){
            tft.setTextSize(2);
            tft.setTextColor(ILI9341_DARKGREY);
            for (int j = 0; j < XLswitchANT; j++) {
              if(j<10){
                tft.setCursor(RowSpace*j-j*5, LineSpace*i+18+6);
              }else{
                tft.setCursor(RowSpace*j-9*5+(j-9)*6, LineSpace*i+18+6);
              }
              // TX
              tft.print(j+1);
              if(j<10){
                tft.setCursor(RowSpace*j-j*5, LineSpace*i+18+6+LineSpace/2);
              }else{
                tft.setCursor(RowSpace*j-9*5+(j-9)*6, LineSpace*i+18+6+LineSpace/2);
              }
              // RX
              tft.print(j+1);


              // tft.fillRoundRect(5+j*39, 28+i*space+5, 34, 34, 3, ILI9341_LIGHTGREY);
              // tft.fillRoundRect(5+j*39, 28+i*space+5+39, 34, 34, 3, ILI9341_LIGHTGREY);
            }
          }
        }

        // tft.drawLine(0,227,340,227, ILI9341_LIGHTGREY);
        tft.setCursor(0,230);
        tft.setTextColor(ILI9341_LIGHTGREY);
        tft.setTextSize(1);
        tft.print(String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );

        #if defined(XLswitchCIV) && defined(XLswitch)
          tft.setTextSize(1);
          tft.setTextColor(ILI9341_WHITE);
          if(TxUdpBuffer[2]=='I'){
            tft.setCursor(135,230);
            tft.print(String(SERIAL1_BAUDRATE));
            tft.print(" baud ");
            tft.print(String(CIV_ADRESS, HEX) );
            tft.print("h");
          }
          if(TxUdpBuffer[2]=='m'){
            tft.setCursor(130,230);
            tft.print("Manual IP switch");
          }
        #endif

        tft.setCursor(270,230);
        tft.setTextSize(1);
        tft.setTextColor(ILI9341_LIGHTGREY);
        tft.print(String(REV));
      }

      #if defined(XLswitchCIV)
        // CI-V
        if(LcdNeedRefresh==1){
          tft.fillRect(155, 170, 35, 22, ILI9341_BLACK);
          tft.setCursor(155,170);
          tft.setTextColor(ILI9341_LIGHTGREY);
          tft.setTextSize(3);
          tft.print(BAND);

          tft.fillRect(70, 200, 180, 22, ILI9341_BLACK);
          tft.setCursor(70,200);
          tft.setTextColor(ILI9341_GREENYELLOW);
          tft.setTextSize(3);
          int longer=String(freq/1000).length();
          if(longer<5){
            tft.print(" ");
          }
          tft.print(String(freq/1000).substring(0, longer-3));
          tft.print(".");
          tft.print(String(freq/1000).substring(longer-3, longer));
          tft.print(" kHz");
        }
      #endif

      // IP-SW
      if(LcdNeedRefresh==2){
        tft.setTextColor(ILI9341_GREENYELLOW);
        tft.setTextSize(3);

        tft.fillRect(70, 140, 180, 22, ILI9341_BLACK);
        tft.setCursor(70,140);
        tft.print(String(ShiftOutByte[0], BIN));

        tft.fillRect(70, 170, 180, 22, ILI9341_BLACK);
        tft.setCursor(70,170);
        tft.print(String(ShiftOutByte[1], BIN));

        tft.fillRect(70, 200, 180, 22, ILI9341_BLACK);
        tft.setCursor(70,200);
        tft.print(String(ShiftOutByte[2], BIN));
      }

      // 4x16
      if(LcdNeedRefresh==3){
        for (int i = 0; i < XLswitchTRX+1; i++) {
          if(i<XLswitchTRX){
            tft.setTextSize(2);
            for (int j = 0; j < XLswitchANT; j++) {
              if(j<10){
                tft.setCursor(RowSpace*j-j*5, LineSpace*i+18+6);
              }else{
                tft.setCursor(RowSpace*j-9*5+(j-9)*6, LineSpace*i+18+6);
              }
              if(j<8){
                // byte XLswitchOutputs[4][2];          // 4 trx, 2 byte = 16 bit
                if(bitRead(XLswitchOutputs[i][0], j)==1){
                  tft.setTextColor(ILI9341_GREEN);
                }else{
                  tft.setTextColor(ILI9341_DARKGREY);
                }
              }else{
                if(bitRead(XLswitchOutputs[i][1], j-8)==1){
                  tft.setTextColor(ILI9341_GREEN);
                }else{
                  tft.setTextColor(ILI9341_DARKGREY);
                }
              }
              tft.print(j+1);
              // tft.fillRoundRect(5+j*39, 28+i*space+5, 34, 34, 3, ILI9341_LIGHTGREY);
              // tft.fillRoundRect(5+j*39, 28+i*space+5+39, 34, 34, 3, ILI9341_LIGHTGREY);
            }
          }
        }

      }

      LcdNeedRefresh=0;
    }
  #endif
}

#if defined(XLswitch)
  void LcdDisplayOLD() {
    // This is just a draw some data Demo

    // Clear Screen
    tft.fillScreen(ILI9341_BLACK);
    // Set some fancy background
    testFastLines(ILI9341_DARKGREY,ILI9341_DARKCYAN);

    // Print "current date and time"
    tft.setCursor(5,5);
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
    tft.println("29-05-18      11:28"); //TODO: Print the real date and time


    // Print "room temperature"
    tft.setCursor(85,50);
    tft.setTextColor(ILI9341_GREEN);  tft.setTextSize(4);
    tft.println("22");//TODO: Print the real room temperature
    tft.setCursor(148,50);
    tft.println("C");
    tft.drawCircle(138, 54, 4, ILI9341_GREEN);
    tft.drawCircle(138, 54, 5, ILI9341_GREEN);
    tft.setCursor(78,85);
    tft.setTextColor(ILI9341_GREEN);  tft.setTextSize(1);
    tft.println("ROOM TEMPERATURE");


    // Now print Message box wit two yes/no buttons
    tft.fillRoundRect(10,120, 220, 190, 8, ILI9341_OLIVE);
    tft.drawRoundRect(10,120, 220, 190, 8, ILI9341_WHITE);

    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
    tft.fillRoundRect(20,150, 200, 80,8, ILI9341_BLUE);
    tft.setCursor(90, 165);
    tft.println("Save");
    tft.setCursor(40, 190);
    tft.println("new settings?");
    tft.drawRoundRect(20,150, 200, 80, 8, ILI9341_WHITE);
    // Get the choise
    bool answer = Get_yes_no();

    if (answer == true)
    {
      // Some animation while "write to eeprom"
    testFilledRects(ILI9341_DARKGREEN,ILI9341_DARKCYAN);
    tft.setCursor(80, 150);
    tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);
    tft.println("Done!");
    } else   tft.fillScreen(ILI9341_RED);
    // fill screen red to show negative choise
    delay(1000);
  }
#endif

//-------------------------------------------------------------------------------------------------------
void Meter(){
  #if !defined(XLswitch)
  if(TxUdpBuffer[2] == 'e' && millis()-MeterRefreshTimer[0]>MeterRefreshTimer[1]){
    MeterRefreshTimer[0]=millis();
    for (int i = 0; i < 4; i++) {
      MeterValue[i] = analogRead(MeterPin[i]);
      if(EnableSerialDebug==1){
        Serial.print(MeterValue[i]);
        Serial.print(" ");
      }
    }
    if(EnableSerialDebug==1){
      Serial.println();
      Serial.println(MeterValue[0], BIN);
    }
    // int tmp;
    // byte buffer[6];
    // tmp=MeterValue[0];
    // for (int i = 0; i < 4; i++) {
    //
    // }
    // buffer[0]=
    //
    // TxUDP('E', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 1);

  }
  #endif
}
//---------------------------------------------------------------------------------------------------------
// void SplitValue(int A, int B){
//   int tmp;
//   tmp=MeterValue[0];
//   for (int i = 0; i < 4; i++) {
//
//   }
//
//
//   packetBuffer[0];
//
// }

byte HiByte(int ID){
  bitClear(ID, 0);  // ->
  bitClear(ID, 1);
  bitClear(ID, 2);
  bitClear(ID, 3);
  ID = ID >> 4;
  return(ID);
}

//---------------------------------------------------------------------------------------------------------

byte LowByte(int ID){
  bitClear(ID, 4);
  bitClear(ID, 5);
  bitClear(ID, 6);
  bitClear(ID, 7);  // <-
  return(ID);
}

//-------------------------------------------------------------------------------------------------------
void Watchdog(){

  // WDT
  if(millis()-WdtTimer > 60000){
    esp_task_wdt_reset();
    WdtTimer=millis();
    if(EnableSerialDebug==true){
      Prn(3, 0,"WDT reset ");
      Prn(3, 1, String(millis()/1000) );
    }
  }

  if(RebootWatchdog > 0 && millis()-WatchdogTimer > RebootWatchdog*60000){
    Serial.println();
    Serial.println("** Activate reboot watchdog - IP switch will be restarted **");
    EEPROM.writeByte(34, ShiftOutByte[0]);
    EEPROM.writeByte(35, ShiftOutByte[1]);
    EEPROM.writeByte(36, ShiftOutByte[2]);
    EEPROM.commit();
    delay(1000);
    TelnetServerClients[0].stop();
    ESP.restart();
  }

  if(OutputWatchdog > 0 && millis()-WatchdogTimer > OutputWatchdog*60000 && OutputWatchdog < 123456){
    Serial.println();
    Serial.println("** Activate clear output watchdog **");
    ShiftOutByte[0]=0x00;
    ShiftOutByte[1]=0x00;
    ShiftOutByte[2]=0x00;
    EEPROM.writeByte(34, ShiftOutByte[0]);
    EEPROM.writeByte(35, ShiftOutByte[1]);
    EEPROM.writeByte(36, ShiftOutByte[2]);
    EEPROM.commit();
    #if defined(ShiftOut)
      digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
      shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, 0x01);
      shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
      shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
      shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
      digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
    #endif
    // deactivate
    OutputWatchdog=123456;
  }

  #if defined(XLswitch)
    if(StatusLedB==true && millis()-StatusLedBTimer[0]>StatusLedBTimer[1]){
      digitalWrite(StatusLedBPin, LOW);
      StatusLedB = false;
    }
  #endif

}

//-------------------------------------------------------------------------------------------------------
void CheckNetId(){
  #if !defined(Ser2net) && !defined(XLswitch)
  if (HW_BCD_SW==true){
    if(millis()-HW_BCD_SWTimer[0]>HW_BCD_SWTimer[1]){
      bitClear(NET_ID, 0);
      bitClear(NET_ID, 1);
      bitClear(NET_ID, 2);
      bitClear(NET_ID, 3);
      NET_ID = NET_ID | GetBoardId();
      if(NET_ID!=TxUdpBuffer[0]){
        TxUdpBuffer[0] = NET_ID;
        EEPROM.write(1, NET_ID); // address, value
        EEPROM.commit();
        Serial.print("** Now NET-ID change to 0x");
        if(NET_ID <=0x0f){
          Serial.print(F("0"));
        }
        Serial.print(NET_ID, HEX);
        Serial.println(" **");
        if(EnableSerialDebug==1){
          Serial.print("EEPROM read [");
          Serial.print(EEPROM.read(1));
          Serial.println("]");
        }
        TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
        if(TxUdpBuffer[2] == 'm'){
          TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
        }
      }
      HW_BCD_SWTimer[0]=millis();
    }
  }
  #endif
}

//-------------------------------------------------------------------------------------------------------
void CLI(){
  int OUT=2;
  int intBuf=0;
  int mult=1;
  // incomingByte = 0;

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    OUT = 0;
  }

  if(TelnetServerClients[0].connected() && OUT!=0){
    TelnetAuthorized = false;
    if(TelnetServerClients[0].remoteIP()==TelnetServerClientAuth){
        TelnetAuthorized = true;
        // timeout ... > false
    }else{
      TelnetAuth();
    }
    if(TelnetAuthorized==true){
      // incomingByte = TelnetRX();
      if(incomingByte!=0){
        OUT=1;
      }
    }
  }else if(!TelnetServerClients[0].connected()){
    TelnetAuthStep=0;
  }

  if(OUT<2){
    // ?
    if(incomingByte==63){
      ListCommands(OUT);


    // // e
    // }else if(incomingByte==101){
    //   Prn(OUT, 1,"  Change source of control? (y/n)");
    //   EnterChar(OUT);
    //   if(incomingByte==89 || incomingByte==121){
    //     TxUdpBuffer[2] = 'e';
    //     EEPROM.write(0, 'e'); // address, value
    //     EEPROM.commit();
    //     Prn(OUT, 1,"Now control from: Meter");
    //     if(EnableSerialDebug==1){
    //       Prn(OUT, 0,"EEPROM read [");
    //       Prn(OUT, 0, String(EEPROM.read(0)) );
    //       Prn(OUT, 1,"]");
    //     }
    //     TxUDP('e', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
    //     if(TxUdpBuffer[2] == 'm'){
    //       TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
    //     }
    //   }

    // m
  }else if(incomingByte==109){
    Prn(OUT, 1,"  Change source of control? (y/n)");
    EnterChar(OUT);
    if(incomingByte==89 || incomingByte==121){
      TxUdpBuffer[2] = 'm';
      EEPROM.write(0, 'm'); // address, value
      EEPROM.commit();
      Prn(OUT, 1,"Now control from: IP switch master");
      if(EnableSerialDebug==1){
        Prn(OUT, 0,"EEPROM read [");
        Prn(OUT, 0, String(EEPROM.read(0)) );
        Prn(OUT, 1,"]");
      }
      TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
      if(TxUdpBuffer[2] == 'm'){
        TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
      }
      #if defined(XLswitch)
        LcdNeedRefresh=100;
      #endif
    }


    #if defined(XLswitchCIV) && defined(XLswitch)
      // I
      }else if(incomingByte==73){
      Prn(OUT, 1,"  Change source of control? (y/n)");
      EnterChar(OUT);
      if(incomingByte==89 || incomingByte==121){
        TxUdpBuffer[2] = 'I';
        EEPROM.write(0, 'I'); // address, value
        EEPROM.commit();
        Prn(OUT, 1,"Now control from: Icom CI-V");
        if(EnableSerialDebug==1){
          Prn(OUT, 0,"EEPROM read [");
          Prn(OUT, 0, String(EEPROM.read(0)) );
          Prn(OUT, 1,"]");
        }
        LcdNeedRefresh=100;
        // TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
        // if(TxUdpBuffer[2] == 'm'){
        //   TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
        // }
      }
    #endif
    // // r
    // }else if(incomingByte==114){
    //   Prn(OUT, 1,"  Change source of control? (y/n)");
    //   EnterChar(OUT);
    //   if(incomingByte==89 || incomingByte==121){
    //     EnableGroupPrefix=false;
    //         EEPROM.write(4, EnableGroupPrefix);
    //         EEPROM.commit();
    //       EnableGroupButton=false;
    //         EEPROM.write(5, EnableGroupButton);
    //         EEPROM.commit();
    //       TxUdpBuffer[2] = 'r';
    //       EEPROM.write(0, 'r'); // address, value
    //       EEPROM.commit();
    //       Prn(OUT, 1,"Now control from: Band decoder");
    //       if(EnableSerialDebug==1){
    //         Prn(OUT, 0,"EEPROM read [");
    //         Prn(OUT, 0, String(EEPROM.read(0)) );
    //         Prn(OUT, 1,"]");
    //       }
    //       TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
    //       if(TxUdpBuffer[2] == 'm'){
    //         TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
    //       }
    //     }
    // o
    }else if(incomingByte==111){
      Prn(OUT, 1,"  Change source of control? (y/n)");
      EnterChar(OUT);
      if(incomingByte==89 || incomingByte==121){
        EnableGroupPrefix=false;
          EEPROM.write(4, EnableGroupPrefix);
          EEPROM.commit();
        EnableGroupButton=false;
          EEPROM.write(5, EnableGroupButton);
          EEPROM.commit();
        TxUdpBuffer[2] = 'o';
        EEPROM.write(0, 'o'); // address, value
        EEPROM.commit();
        Prn(OUT, 1,"Now control from: Open Interface III");
        if(EnableSerialDebug==1){
          Prn(OUT, 0,"EEPROM read [");
          Prn(OUT, 0, String(EEPROM.read(0)) );
          Prn(OUT, 1,"]");
        }
        TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
        if(TxUdpBuffer[2] == 'm'){
          TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
        }
      }

    // n
    }else if(incomingByte==110){
      Prn(OUT, 1,"  Change source of control? (y/n)");
      EnterChar(OUT);
      if(incomingByte==89 || incomingByte==121){
        EnableGroupPrefix=false;
        EEPROM.write(4, EnableGroupPrefix);
        EnableGroupButton=false;
        EEPROM.write(5, EnableGroupButton);
        TxUdpBuffer[2] = 'n';
        EEPROM.write(0, 'n'); // address, value
        EEPROM.commit();
        Prn(OUT, 1,"Now control from: none");
      }

    // w
    }else if(incomingByte==119 && TxUdpBuffer[2]!='n'){
      Prn(OUT, 0,"Input reboot watchdog in minutes (0-10080), 0-disable and press ");
      if(TelnetAuthorized==true){
        Prn(OUT, 1,"[enter]");
      }else{
        Prn(OUT, 1,"[;]");
      }
      Prn(OUT, 1,"recomended 1440 (1 day)");
      Enter();
      for (int i=InputByte[0]; i>0; i--){
        intBuf = intBuf + ((InputByte[i]-48)*mult);
        mult = mult*10;
      }
      if(intBuf>=0 && intBuf<=10080){
        RebootWatchdog = intBuf;
        EEPROM.writeUInt(26, RebootWatchdog);
        EEPROM.commit();
        Prn(OUT, 0," Set ");
        Prn(OUT, 0, String(EEPROM.readUInt(26)) );
        Prn(OUT, 1," minutes");
      }else{
        Prn(OUT, 0,"Out of range.");
      }

    // W
    }else if(incomingByte==87 && TxUdpBuffer[2]!='n'){
      Prn(OUT, 0,"Input clear output watchdog in minutes (0-10080), 0-disable and press ");
      if(TelnetAuthorized==true){
        Prn(OUT, 1,"[enter]");
      }else{
        Prn(OUT, 1,"[;]");
      }
      Prn(OUT, 1,"note: if you need clear output after reboot watchdog, set smaller than it");
      Enter();
      for (int i=InputByte[0]; i>0; i--){
        intBuf = intBuf + ((InputByte[i]-48)*mult);
        mult = mult*10;
      }
      if(intBuf>=0 && intBuf<=10080){
        OutputWatchdog = intBuf;
        EEPROM.writeUInt(30, OutputWatchdog);
        EEPROM.commit();
        Prn(OUT, 0," Set ");
        Prn(OUT, 0, String(EEPROM.readUInt(30)) );
        Prn(OUT, 1," minutes");
      }else{
        Prn(OUT, 0,"Out of range.");
      }

    // <
    }else if(incomingByte==60 && TxUdpBuffer[2]!='n'){
      Prn(OUT, 0,"write UDP port (1-65535) and press ");
      if(TelnetAuthorized==true){
        Prn(OUT, 1,"[enter]");
      }else{
        Prn(OUT, 1,"[;]");
      }
      Enter();
      for (int i=InputByte[0]; i>0; i--){
        intBuf = intBuf + ((InputByte[i]-48)*mult);
        mult = mult*10;
      }
      if(intBuf>=1 && intBuf<=65535){
        IncomingSwitchUdpPort = intBuf;
        EEPROM.writeInt(22, IncomingSwitchUdpPort);
        EEPROM.commit();
        Prn(OUT, 0," Set ");
        Prn(OUT, 1, String(IncomingSwitchUdpPort) );
        Prn(OUT, 0,"** device will be restarted **");
        delay(1000);
        TelnetServerClients[0].stop();
        ESP.restart();
      }else{
        Prn(OUT, 0,"Out of range.");
      }

    // /
    }else if(incomingByte==47 && TxUdpBuffer[2] == 'm'){
      Prn(OUT, 0,"write encoder rannge number (2-16) and press ");
      if(TelnetAuthorized==true){
        Prn(OUT, 1,"[enter]");
      }else{
        Prn(OUT, 1,"[;]");
      }
      Enter();
      for (int i=InputByte[0]; i>0; i--){
        intBuf = intBuf + ((InputByte[i]-48)*mult);
        mult = mult*10;
      }
      if(intBuf>=2 && intBuf<=16){
        NumberOfEncoderOutputs = intBuf;
/*      EnterChar(OUT);
      // 2-G
      if( (incomingByte>=50 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=103) ){
        if(incomingByte>=50 && incomingByte<=57){
          NumberOfEncoderOutputs = incomingByte-48;
        }else if(incomingByte>=97 && incomingByte<=103){
          NumberOfEncoderOutputs = incomingByte-87;
        }*/
        NumberOfEncoderOutputs--;
        EEPROM.write(2, NumberOfEncoderOutputs); // address, value
        EEPROM.commit();
        Prn(OUT, 0,"** Now Encoder range change to ");
        Prn(OUT, 0, String(NumberOfEncoderOutputs+1) );
        Prn(OUT, 1," **");
        if(EnableSerialDebug==1){
          Prn(OUT, 0,"EEPROM read [");
          Prn(OUT, 0, String(EEPROM.read(2)) );
          Prn(OUT, 1,"]");
        }
        TxUDP('s', packetBuffer[2], 'c', 'f', 'm', 1);    // 0=broadcast, 1= direct to RX IP
        if(TxUdpBuffer[2] == 'm'){
          TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
        }
      }else{
        Prn(OUT, 1,"Out of range");
      }

    // %
    }else if(incomingByte==37){
      EnableGroupButton=!EnableGroupButton;
      Prn(OUT, 0,"** Group buttons (one from) [");
      EEPROM.write(5, EnableGroupButton);
      EEPROM.commit();
      if(EnableGroupButton==true){
        Prn(OUT, 1,"ON] **");
        for (int i = 0; i < 8; i++) {
          if(EEPROM.read(6+i)<9){
            GroupButton[i]=EEPROM.read(6+i);
          }
        }
      }else{
        Prn(OUT, 1,"OFF] **");
      }

    // :
    }else if(incomingByte==58 && EnableGroupButton==true){
      Prn(OUT, 1," List groups");
      for (int i = 0; i < 8; i++) {
        Prn(OUT, 0,"  Button ");
        Prn(OUT, 0, String(i+1) );
        Prn(OUT, 0," in group ");
        Prn(OUT, 1, String(GroupButton[i]) );
      }

    // !
    }else if(incomingByte==33 && EnableGroupButton==true){
      Prn(OUT, 1,"Press button number 1-8...");
      EnterChar(OUT);
      if( (incomingByte>=49 && incomingByte<=56)){
        unsigned int ButtonNumber=incomingByte-48;
        Prn(OUT, 0,"Press Group number 1-8 for button ");
        Prn(OUT, 1, String(ButtonNumber) );
        EnterChar(OUT);
        if( (incomingByte>=49 && incomingByte<=56)){
          unsigned int ButtonGroup=incomingByte-48;
          Prn(OUT, 0," store Button ");
          Prn(OUT, 0, String(ButtonNumber) );
          Prn(OUT, 0," to group ");
          Prn(OUT, 1, String(ButtonGroup) );
          GroupButton[ButtonNumber-1]=ButtonGroup;
          for (int i = 0; i < 8; i++) {
            EEPROM.write(6+i, GroupButton[i]);
          }
          EEPROM.commit();
        }else{
          Prn(OUT, 1," accepts 0-8, exit");
        }
      }else{
        Prn(OUT, 1," accepts 0-8, exit");
      }

    // *
    }else if(incomingByte==42){
      EnableSerialDebug=!EnableSerialDebug;
      Prn(OUT, 0,"** Serial DEBUG ");
      if(EnableSerialDebug==true){
        Prn(OUT, 1,"ENABLE **");
      }else{
        Prn(OUT, 1,"DISABLE **");
      }

    // +
    #if !defined(Ser2net) && !defined(XLswitch)
    }else if(incomingByte==43 && TxUdpBuffer[2]!='n'){
      HW_BCD_SW=!HW_BCD_SW;
      Prn(OUT, 0,"** Net ID sufix by ");
      EEPROM.write(3, HW_BCD_SW);
      EEPROM.commit();
      if(HW_BCD_SW==true){
        Prn(OUT, 1,"EEPROM/[BCD switch] **");
        bitClear(NET_ID, 0);
        bitClear(NET_ID, 1);
        bitClear(NET_ID, 2);
        bitClear(NET_ID, 3);
        NET_ID = NET_ID | GetBoardId();
        TxUdpBuffer[0] = NET_ID;
      }else{
        NET_ID = EEPROM.read(1);
        TxUdpBuffer[0] = NET_ID;
        Prn(OUT, 1,"[EEPROM]/BCD switch **");
      }
    #endif

    // #
    }else if(incomingByte==35 && TxUdpBuffer[2]!='n'){
      if(EnableGroupPrefix==false){
        Prn(OUT, 1,"Press NET-ID X_ prefix 0-f...");
      }else{
        Prn(OUT, 1,"Press NET-ID _X sufix 0-f...");
      }
      EnterChar(OUT);
      if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
        // prefix
        if(EnableGroupPrefix==false){
          bitClear(NET_ID, 4);
          bitClear(NET_ID, 5);
          bitClear(NET_ID, 6);
          bitClear(NET_ID, 7);
          Serial.write(incomingByte);
          Prn(OUT, 1,"");
          if(incomingByte>=48 && incomingByte<=57){
            incomingByte = incomingByte-48;
            incomingByte = (byte)incomingByte << 4;
            NET_ID = NET_ID | incomingByte;
            TxUdpBuffer[0] = NET_ID;
          }else if(incomingByte>=97 && incomingByte<=102){
            incomingByte = incomingByte-87;
            incomingByte = (byte)incomingByte << 4;
            NET_ID = NET_ID | incomingByte;
            TxUdpBuffer[0] = NET_ID;
          }
        }
        // sufix
        if(HW_BCD_SW==false){
          if(EnableGroupPrefix==false){
            Prn(OUT, 1,"Press NET-ID _X sufix 0-f...");
            EnterChar(OUT);
          }
          if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
            bitClear(NET_ID, 0);
            bitClear(NET_ID, 1);
            bitClear(NET_ID, 2);
            bitClear(NET_ID, 3);
            Serial.write(incomingByte);
            Prn(OUT, 1, "");
            if(incomingByte>=48 && incomingByte<=57){
              incomingByte = incomingByte-48;
              NET_ID = NET_ID | incomingByte;
              TxUdpBuffer[0] = NET_ID;
            }else if(incomingByte>=97 && incomingByte<=102){
              incomingByte = incomingByte-87;
              NET_ID = NET_ID | incomingByte;
              TxUdpBuffer[0] = NET_ID;
            }
        // #endif
            EEPROM.write(1, NET_ID); // address, value
            EEPROM.commit();
            Prn(OUT, 0,"** Now NET-ID change to 0x");
            if(NET_ID <=0x0f){
              Prn(OUT, 0, String("0"));
            }
            Prn(OUT, 0, String(NET_ID, HEX) );
            Prn(OUT, 1," **");
            if(EnableSerialDebug==1){
              Prn(OUT, 0,"EEPROM read [");
              Prn(OUT, 0, String(EEPROM.read(1), HEX) );
              Prn(OUT, 1,"]");
            }
            TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
            }
        // #if !defined(HW_BCD_SW)
          }else{
            Prn(OUT, 1," accepts 0-f, exit");
          }
        // #endif
        }
      }else{
        Prn(OUT, 1," accepts 0-f, exit");
      }
      // E
    }else if(incomingByte==69 && OUT==0){
        Prn(OUT, 1,"  Erase whole eeprom (also telnet key)? (y/n)");
        EnterChar(OUT);
        if(incomingByte==89 || incomingByte==121){
          Prn(OUT, 1,"  Stop erase? (y/n)");
          EnterChar(OUT);
          if(incomingByte==78 || incomingByte==110){
            for(int i=0; i<EEPROM_SIZE; i++){
              EEPROM.write(i, 0xff);
              Prn(OUT, 0,".");
            }
            EEPROM.commit();
            Prn(OUT, 1,"");
            Prn(OUT, 1,"  Eeprom erased done");
            ESP.restart();
          }else{
            Prn(OUT, 1,"  Erase aborted");
          }
        }else{
          Prn(OUT, 1,"  Erase aborted");
        }

    // $
    }else if(incomingByte==36){
      EnableGroupPrefix=!EnableGroupPrefix;
      Prn(OUT, 0,"** Group sufix (multi control) [");
      EEPROM.write(4, EnableGroupPrefix);
      EEPROM.commit();
      if(EnableGroupPrefix==true){
        Prn(OUT, 1,"ON] **");
      }else{
        Prn(OUT, 1,"OFF] **");
      }
      if(EnableGroupPrefix==true){
        // clear prefix
        bitClear(NET_ID, 4);
        bitClear(NET_ID, 5);
        bitClear(NET_ID, 6);
        bitClear(NET_ID, 7); // <-
      }
      Prn(OUT, 1,"** IP switch will be restarted **");
      delay(1000);
      TelnetServerClients[0].stop();
      ESP.restart();

    // .
    }else if(incomingByte==46 && TxUdpBuffer[2]=='m' && EnableGroupPrefix){
      Prn(OUT, 1,"List detected IP switch by NET-ID prefix (multi control)");
      for (int i = 0; i < 16; i++) {
        Prn(OUT, 0, String(i, HEX) );
        Prn(OUT, 0, String("  "));
        Prn(OUT, 0, String(DetectedRemoteSw [i] [0]) );
        Prn(OUT, 0, String("."));
        Prn(OUT, 0, String(DetectedRemoteSw [i] [1]) );
        Prn(OUT, 0, String("."));
        Prn(OUT, 0, String(DetectedRemoteSw [i] [2]) );
        Prn(OUT, 0, String("."));
        Prn(OUT, 0, String(DetectedRemoteSw [i] [3]) );
        Prn(OUT, 0, String(":"));
        Prn(OUT, 1, String(DetectedRemoteSwPort [i]) );
      }

    // (
    #if defined(Ser2net) || defined(XLswitchCIV)
    }else if(incomingByte==40){
      Prn(OUT, 0,"Write baudrate and press ");
      if(TelnetAuthorized==true){
        Prn(OUT, 1,"[enter]");
      }else{
        Prn(OUT, 1,"[;]");
      }
      Enter();
      for (int i=InputByte[0]; i>0; i--){
        intBuf = intBuf + ((InputByte[i]-48)*mult);
        mult = mult*10;
      }
      if(intBuf>=80 && intBuf<=5000000){
        SERIAL1_BAUDRATE = intBuf;
        EEPROM.writeInt(14, SERIAL1_BAUDRATE);
        EEPROM.commit();
        Prn(OUT, 0," Set ");
        Prn(OUT, 1, String(SERIAL1_BAUDRATE) );
        Prn(OUT, 0,"** device will be restarted **");
        delay(1000);
        TelnetServerClients[0].stop();
        ESP.restart();
      }else{
        Prn(OUT, 0,"Out of range.");
      }
    #endif

    // )
    #if defined(Ser2net) && !defined(XLswitch)
    }else if(incomingByte==41){
      Prn(OUT, 0,"Write IP port (1-65535) and press ");
      if(TelnetAuthorized==true){
        Prn(OUT, 1,"[enter]");
      }else{
        Prn(OUT, 1,"[;]");
      }
      Enter();
      for (int i=InputByte[0]; i>0; i--){
        intBuf = intBuf + ((InputByte[i]-48)*mult);
        mult = mult*10;
      }
      if(intBuf>=1 && intBuf<=65535){
        SerialServerIPport = intBuf;
        EEPROM.writeInt(18, SerialServerIPport);
        EEPROM.commit();
        Prn(OUT, 0," Set ");
        Prn(OUT, 1, String(SerialServerIPport) );
        Prn(OUT, 0,"** device will be restarted **");
        delay(1000);
        TelnetServerClients[0].stop();
        ESP.restart();
      }else{
        Prn(OUT, 0,"Out of range.");
      }
    #endif

    // &
    }else if(incomingByte==38 && TxUdpBuffer[2]!='n'){
      TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
      if(TxUdpBuffer[2] == 'm'){
        TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
      }

    // q
    }else if(incomingByte==113 && TelnetServerClients[0].connected() ){
      TelnetServerClients[0].stop();
      TelnetAuthorized=false;
      // TelnetServerClientAuth = {0,0,0,0};
      TelnetAuthStep=0;

    // Q
    }else if(incomingByte==81 && TelnetServerClients[0].connected() ){
      TelnetServerClients[0].stop();
      TelnetAuthorized=false;
      TelnetServerClientAuth = {0,0,0,0};
      Serial.println(TelnetServerClientAuth);
      TelnetAuthStep=0;
      EEPROM.write(37, 0); // address, value
      EEPROM.write(38, 0); // address, value
      EEPROM.write(39, 0); // address, value
      EEPROM.write(40, 0); // address, value
      EEPROM.commit();

    // @
    }else if(incomingByte==64){
      Prn(OUT, 1,"** IP switch will be restarted **");
      if(RebootWatchdog > 0){
        Prn(OUT, 1,"   Activate reboot watchdog - store outputs to EEPROM...");
        EEPROM.writeByte(34, ShiftOutByte[0]);
        EEPROM.writeByte(35, ShiftOutByte[1]);
        EEPROM.writeByte(36, ShiftOutByte[2]);
        EEPROM.commit();
        delay(1000);
      }
      TelnetServerClients[0].stop();
      ESP.restart();

    // LF
    }else if(incomingByte==10){
      Prn(OUT, 1,"");

    // anykey
    }else{
      Prn(OUT, 0," [");
      Prn(OUT, 0, String(incomingByte) ); //, DEC);
      Prn(OUT, 1,"] unknown command");
      ListCommands(OUT);
    }
    incomingByte=0;
  }
}
//-------------------------------------------------------------------------------------------------------
void Enter(){
  int OUT;
  if(TelnetAuthorized==true){
    OUT=1;
  }else{
    OUT=0;
  }

  InputByte[0]=0;
  incomingByte = 0;
  bool br=false;
  Prn(OUT, 0,"> ");

  if(OUT==0){
    while(br==false) {
      if(Serial.available()){
        incomingByte=Serial.read();
        if(incomingByte==13 || incomingByte==59){ // CR or ;
          br=true;
          Serial.println("");
        }else{
          Serial.write(incomingByte);
          InputByte[InputByte[0]+1]=incomingByte;
          InputByte[0]++;
        }
        if(InputByte[0]==20){
          br=true;
          Prn(OUT, 1," too long");
        }
      }
    }

  }else if(OUT==1){
    if (TelnetServerClients[0] && TelnetServerClients[0].connected()){

        while(br==false){
          if(TelnetServerClients[0].available()){
            incomingByte=TelnetServerClients[0].read();
            if(incomingByte==13){
              br=true;
              Prn(OUT, 1,"");
            }else{
              TelnetServerClients[0].write(incomingByte);
              InputByte[InputByte[0]+1]=incomingByte;
              InputByte[0]++;
            }
            if(InputByte[0]==20){
              br=true;
              Prn(OUT, 1," too long");
            }
          }
        }
    }
  }

  // Serial.println();
  // for (int i=1; i<InputByte[0]+1; i++){
    // Serial.write(InputByte[i]);
  // }
  // Serial.println();

  // Prn(OUT, 1, "out"+String(CompareInt) );
}
//-------------------------------------------------------------------------------------------------------
void EnterChar(int OUT){
  incomingByte = 0;
  Prn(OUT, 0,">");
  if(OUT==0){
    while (Serial.available() == 0) {
      // Wait
    }
    incomingByte = Serial.read();
  }else if(OUT==1){
    if (TelnetServerClients[0] && TelnetServerClients[0].connected()){
      while(incomingByte==0){
        if(TelnetServerClients[0].available()){
          incomingByte=TelnetServerClients[0].read();
        }
      }
      if(EnableSerialDebug==1){
        Serial.println();
        Serial.print("Telnet rx-");
        Serial.print(incomingByte, DEC);
      }
    }
  }
  Prn(OUT, 1, String(char(incomingByte)) );
}

//-------------------------------------------------------------------------------------------------------

void EnterInt(int OUT){
  incomingByte = 0;
  Prn(OUT, 0,"> ");
  if(OUT==0){
    while(!Serial.available()) {
    }
    delay(3000);
    CompareInt = Serial.parseInt();
  }else if(OUT==1){
    if (TelnetServerClients[0] && TelnetServerClients[0].connected()){
      bool br=true;
      int intField[10];
      int count=0;

      while(incomingByte==0 && br==true){
        if(TelnetServerClients[0].available()){
          incomingByte=TelnetServerClients[0].read();
          // out of 0-9
          if(incomingByte<48 || incomingByte>57){
            br=false;
            intField[count]=0;
            Prn(OUT, 1,"");
          }else{
            intField[count]=incomingByte-48;
            Prn(OUT, 0,String(intField[count]));
            count++;
            incomingByte=0;
          }
        }
      }

      count--;
      CompareInt=0;
      int i=1;
      while(count>-1){
        CompareInt=CompareInt+intField[count]*i;
        // Prn(OUT, 1, String(intField[count])+"*"+String(i)+"="+String(CompareInt) );
        i=i*10;
        count--;
      }
    }
  }
  // Prn(OUT, 1, "out"+String(CompareInt) );
}
//-------------------------------------------------------------------------------------------------------

void EnterIntOld(int OUT){
  Prn(OUT, 0,"> ");
  if(OUT==0){
    while(!Serial.available()) {
    }
    delay(3000);
    CompareInt = Serial.parseInt();
  }else if(OUT==1){
    if (TelnetServerClients[0] && TelnetServerClients[0].connected()){
      delay(5000);
      if(TelnetServerClients[0].available()){
        // while(TelnetServerClients[0].available()){
        //   Prn(OUT, 1, "4" );
        //   // incomingByte=TelnetServerClients[0].read();
        // }
        CompareInt = TelnetServerClients[0].parseInt();
      }
    }
  }
  Prn(OUT, 1, String(CompareInt) );
}

//-------------------------------------------------------------------------------------------------------
void Prn(int OUT, int LN, String STR){
  if(OUT==3){
    if(TelnetAuthorized==true){
      OUT=1;
    }else{
      OUT=0;
    }
  }

  if(OUT==0){
    Serial.print(STR);
    if(LN==1){
      Serial.println();
    }
  }else if(OUT==1){
    size_t len = STR.length()+1;
    // uint8_t sbuf[len];
    char sbuf[len];
    STR.toCharArray(sbuf, len);
    //push data to all connected telnet clients
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (TelnetServerClients[i] && TelnetServerClients[i].connected()){
        TelnetServerClients[i].write(sbuf, len);
        // delay(1);
        if(LN==1){
          TelnetServerClients[i].write(13); // CR
          TelnetServerClients[i].write(10); // LF
        }
      }
    }
  }
}

//-------------------------------------------------------------------------------------------------------
void ListCommands(int OUT){

  #if defined(ETHERNET)
    Prn(OUT, 1,"");
    #if defined(XLswitch)
      Prn(OUT, 1,"  IP XLswitch on ESP32-POE status");
    #else
      Prn(OUT, 1,"  IP relay ESP32-GATEWAY status");
    #endif
    Prn(OUT, 0,"  http://");
    Prn(OUT, 1, String(ETH.localIP()[0])+"."+String(ETH.localIP()[1])+"."+String(ETH.localIP()[2])+"."+String(ETH.localIP()[3]) );
    Prn(OUT, 0,"  ETH  MAC: ");
    Prn(OUT, 0, String(ETH.macAddress()[0], HEX)+"."+String(ETH.macAddress()[1], HEX)+"."+String(ETH.macAddress()[2], HEX)+"."+String(ETH.macAddress()[3], HEX)+"."+String(ETH.macAddress()[4], HEX)+"."+String(ETH.macAddress()[5], HEX) );
    Prn(OUT, 0,"  ");
    Prn(OUT, 0, String(ETH.linkSpeed()) );
    Prn(OUT, 0,"Mbps");
    if (ETH.fullDuplex()) {
      Prn(OUT, 0,", FULL_DUPLEX ");
    }
  #else
    Prn(OUT, 1,"  ETHERNET OFF");
  #endif
  #if defined(WIFI)
    Prn(OUT, 1,"  =================================");
    Prn(OUT, 0,"  http://");
    Prn(OUT, 1, String(WiFi.localIP()[0])+"."+String(WiFi.localIP()[1])+"."+String(WiFi.localIP()[2])+"."+String(WiFi.localIP()[3]) );
    Prn(OUT, 0,"  dBm: ");
    Prn(OUT, 1, String(WiFi.RSSI()) );
  #else
    Prn(OUT, 1,"| WIFI OFF");
  #endif
    if(OUT==0){
      Prn(OUT, 0,"  OTA ");
      #if defined(EnableOTA)
        Prn(OUT, 0,"enable, password: ");
        Prn(OUT, 1, otaPassword);
      #else
        Prn(OUT, 1,"disable");
      #endif
    }
  if(TxUdpBuffer[2]!='n'){
    // Serial.print("  Key: ");
    // Serial.println(String(key));
    if(OUT==0){
      Prn(OUT, 1,"  Key for telnet access:");
      Prn(OUT, 0,"    ");
      for(int i=0; i<100; i++){
        Prn(OUT, 0, String(key[i]));
        if((i+1)%10==0){
          Prn(OUT, 0," ");
        }
      }
      Prn(OUT, 1,"");
    }
    Prn(OUT, 1,"  =================================");
    Prn(OUT, 0,"  Device NET-ID: 0x");
    // if(NET_ID <=0x0f){
    //   Prn(OUT, 0,"0");
    // }
    // Prn(OUT, 0, String(NET_ID, HEX) );
    // if(EnableGroupPrefix==false){
      Prn(OUT, 0, String(IdPrefix(NET_ID), HEX) );
    // }else{
      // Prn(OUT, 0, "_");
    // }
    Prn(OUT, 0, String(IdSufix(NET_ID), HEX) );
    #if !defined(Ser2net) && !defined(XLswitch)
      if(HW_BCD_SW==true){
        Prn(OUT, 0," [BCD-");
        for (int i = 0; i < 4; i++) {
         Prn(OUT, 0,String(digitalRead(BCD[i])) );
        }
        Prn(OUT, 0,"]");
      }
    #endif
    Prn(OUT, 1,"");
    if(EnableGroupPrefix==true){
      Prn(OUT, 1,"  NOTE: If activate Multi control");
      Prn(OUT, 1,"        - ID prefix identifies individual controllers");
      Prn(OUT, 1,"        - ID sufix same at all (relay and also control) devices - one group");
    }
    Prn(OUT, 1,"  =================================");
  }
  if(EnableGroupPrefix==false){
    Prn(OUT, 0,"  Master IP: ");
    Prn(OUT, 0, String(UdpCommand.remoteIP()[0])+"."+String(UdpCommand.remoteIP()[1])+"."+String(UdpCommand.remoteIP()[2])+"."+String(UdpCommand.remoteIP()[3]) );
    Prn(OUT, 0,":");
    Prn(OUT, 1, String(UdpCommand.remotePort()) );
  }
  Prn(OUT, 0,"  Uptime: ");
  Prn(OUT, 0, String(millis()/1000) );
  Prn(OUT, 1," second");

  if(RebootWatchdog > 0){
    Prn(OUT, 0,"> Reboot countdown in ");
    Prn(OUT, 0, String(RebootWatchdog-((millis()-WatchdogTimer)/60000)) );
    Prn(OUT, 1, " minutes");
  }
  if(OutputWatchdog > 0 && OutputWatchdog<123456){
    Prn(OUT, 0,"> Clear output countdown in ");
    Prn(OUT, 0, String(OutputWatchdog-((millis()-WatchdogTimer)/60000)) );
    Prn(OUT, 1," minutes");
  }

  Prn(OUT, 0,"  Version: ");
  Prn(OUT, 1, String(REV));
  if(TxUdpBuffer[2]!='n'){
    Prn(OUT, 0, "  Bank status ABC [LSBFIRST]: ");
    Prn(OUT, 0, String(ShiftOutByte[0], BIN) );
    Prn(OUT, 0, " ");
    Prn(OUT, 0, String(ShiftOutByte[1], BIN) );
    Prn(OUT, 0," ");
    Prn(OUT, 1, String(ShiftOutByte[2], BIN) );
  }
  Prn(OUT, 0, "  ShiftOut GPIO [data, latch, clock]: ");
  Prn(OUT, 1, String(ShiftOutDataPin)+", "+String(ShiftOutLatchPin)+", "+String(ShiftOutClockPin));
  Prn(OUT, 1,"---------------------------------------------");
  Prn(OUT, 1,"  You can change source, with send character:");
  if(TxUdpBuffer[2]=='m'){
    Prn(OUT, 0,"     [m]");
  }else{
    Prn(OUT, 0,"      m ");
  }
  Prn(OUT, 1,"- IP switch master");
  #if defined(XLswitchCIV) && defined(XLswitch)
    if(TxUdpBuffer[2]=='I'){
      Prn(OUT, 0,"     [I]");
    }else{
      Prn(OUT, 0,"      I ");
    }
    Prn(OUT, 1,"- Icom CI-V");
  #endif
  // if(TxUdpBuffer[2]=='r'){
  //   Prn(OUT, 0,"     [r]");
  // }else{
  //   Prn(OUT, 0,"      r ");
  // }
  // Prn(OUT, 1,"- Band decoder");
  // if(TxUdpBuffer[2]=='e'){
  //   Prn(OUT, 0,"     [e]");
  // }else{
  //   Prn(OUT, 0,"      e ");
  // }
  // Prn(OUT, 1,"- Meter");
  if(TxUdpBuffer[2]=='o'){
    Prn(OUT, 0,"     [o]");
  }else{
    Prn(OUT, 0,"      o ");
  }
  Prn(OUT, 1,"- Open Interface III");
  if(TxUdpBuffer[2]=='n'){
    Prn(OUT, 0,"     [n]");
  }else{
    Prn(OUT, 0,"      n ");
  }
  Prn(OUT, 1,"- none");
  Prn(OUT, 1,"");
  Prn(OUT, 1,"  or  ?  list status and commands");
  if(TxUdpBuffer[2]!='n'){
    Prn(OUT, 0,"      w  inactivity reboot watchdog ");
    if(RebootWatchdog>0){
      Prn(OUT, 0,"after [");
      Prn(OUT, 0, String(RebootWatchdog) );
      Prn(OUT, 1,"] minutes");
    }else{
      Prn(OUT, 1,"[disable]");
    }
    Prn(OUT, 0,"      W  inactivity clear output watchdog ");
    if(OutputWatchdog>0){
      Prn(OUT, 0,"after [");
      Prn(OUT, 0, String(OutputWatchdog) );
      Prn(OUT, 1,"] minutes");
    }else{
      Prn(OUT, 1,"[disable]");
    }
    Prn(OUT, 0,"      <  change Switch incoming UDP port [");
    Prn(OUT, 0, String(IncomingSwitchUdpPort) );
    Prn(OUT, 0,"]");
    if(IncomingSwitchUdpPort!=88){
      Prn(OUT, 0,"<-- WARNING! default is 88");
    }
    Prn(OUT, 1,"");
  }
  if(TxUdpBuffer[2] == 'm'){
    Prn(OUT, 0,"      /  set encoder range - now [");
    Prn(OUT, 0, String(NumberOfEncoderOutputs+1) );
    if(NumberOfEncoderOutputs>7){
      Prn(OUT, 1,"] (two bank)");
    }else{
      Prn(OUT, 1,"]");
    }
    Prn(OUT, 0,"      %  group buttons (select one from group) [");
    if(EnableGroupButton==true){
      Prn(OUT, 1,"ON]");
      Prn(OUT, 1,"         !  SET group buttons");
      Prn(OUT, 1,"         :  list group buttons");
    }else{
      Prn(OUT, 1,"OFF]");
    }
  }
  Prn(OUT, 0,"      *  serial debug ");
    if(EnableSerialDebug==true){
      Prn(OUT, 1,"[ON]");
    }else{
      Prn(OUT, 1,"[OFF]");
    }
  #if !defined(Ser2net) && !defined(XLswitch)
    if(TxUdpBuffer[2]!='n'){
      Prn(OUT, 0,"      +  net ID sufix by ");
      if(HW_BCD_SW==true){
        Prn(OUT, 1,"EEPROM/[BCD switch]");
      }else{
        Prn(OUT, 1,"[EEPROM]/BCD switch");
      }
    }
  #endif
  if(TxUdpBuffer[2]!='n'){
    Prn(OUT, 0,"      #  network ID prefix [");
    byte ID = NET_ID;
    bitClear(ID, 0); // ->
    bitClear(ID, 1);
    bitClear(ID, 2);
    bitClear(ID, 3);
    ID = ID >> 4;
    if(EnableGroupPrefix==true && TxUdpBuffer[2]=='m'){
      Prn(OUT, 0,"x");
    }else{
      Prn(OUT, 0, String(ID, HEX) );
    }
    Prn(OUT, 0,"] hex");
    if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
      Prn(OUT, 0," (set only on controllers)");
    }
    Prn(OUT, 1,"");
    if(HW_BCD_SW==false){
      ID = NET_ID;
      bitClear(ID, 4);
      bitClear(ID, 5);
      bitClear(ID, 6);
      bitClear(ID, 7); // <-
      Prn(OUT, 0,"         +network ID sufix [");
      Prn(OUT, 0, String(ID, HEX) );
      Prn(OUT, 0,"] hex");
      if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
        Prn(OUT, 0," (multi control group - same at all)");
      }
      Prn(OUT, 1,"");
    }
    if(TxUdpBuffer[2]=='m'){
      Prn(OUT, 0,"      $  group network ID prefix (multi control) [");
      if(EnableGroupPrefix==true){
        Prn(OUT, 1,"ON]");
      }else{
        Prn(OUT, 1,"OFF]");
      }
    }
    if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
      Prn(OUT, 1,"      .  list detected IP switch (multi control)");
    }
  }
  #if defined(Ser2net) || defined(XLswitchCIV)
    Prn(OUT, 0,"      (  change serial1 baudrate [");
    Prn(OUT, 0, String(SERIAL1_BAUDRATE) );
    Prn(OUT, 1,"]");
  #endif
  #if defined(Ser2net) && !defined(XLswitch)
    Prn(OUT, 0,"      )  change ser2net IP port [");
    Prn(OUT, 0, String(SerialServerIPport) );
    Prn(OUT, 1,"]");
  #endif
  if(TxUdpBuffer[2]!='n'){
    Prn(OUT, 1,"      &  send broadcast packet");
  }
  if(TelnetServerClients[0].connected()){
    Prn(OUT, 0,"      q  disconnect and close telnet [logged from ");
    Prn(OUT, 0, String(TelnetServerClientAuth[0])+"."+String(TelnetServerClientAuth[1])+"."+String(TelnetServerClientAuth[2])+"."+String(TelnetServerClientAuth[3]) );
    Prn(OUT, 1,"]");
    Prn(OUT, 1,"      Q  logout with erase your IP from memory and close telnet");
  }
  Prn(OUT, 1,"      E  erase whole eeprom (telnet key also)");
  Prn(OUT, 1,"      @  restart device");
  Prn(OUT, 1,"---------------------------------------------");
}
//-------------------------------------------------------------------------------------------------------
char RandomChar(){
    int R = random(48, 122);
    if(R>=58 && 64>=R){
      R=R-random(7, 10);
    }
    if(R>=91 && 96>=R){
      R=R+random(6, 26);
    }
    return char(R);
}

//-------------------------------------------------------------------------------------------------------

void Demo(){
  #if defined(ShiftOut)
    ShiftOutByte[0]=0;
    ShiftOutByte[1]=0;
    ShiftOutByte[2]=0;

    for (i = 0; i < 8; i++) {
      bitSet(ShiftOutByte[0], i);
        digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
        digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
        delay(50);
      bitClear(ShiftOutByte[2], i);
    }
    for (i = 0; i < 8; i++) {
      bitSet(ShiftOutByte[1], i);
        digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
        digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
        delay(50);
      bitClear(ShiftOutByte[0], i);
    }
    for (i = 0; i < 8; i++) {
      bitSet(ShiftOutByte[2], i);
        digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
        digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
        delay(50);
      bitClear(ShiftOutByte[1], i);
    }
    for (i = 0; i < 8; i++) {
      // bitSet(ShiftOutByte[0], i);
        digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
        digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
        delay(50);
      bitClear(ShiftOutByte[2], i);
    }
  #endif
}

// http://www.catonmat.net/blog/low-level-bit-hacks-you-absolutely-must-know/
//-------------------------------------------------------------------------------------------------------

byte GetBoardId(){
  byte NETID = 0;
  #if !defined(Ser2net) && !defined(XLswitch)
    if(digitalRead(BCD[0])==0){
      NETID = NETID | (1<<0);    // Set the n-th bit
    }
    if(digitalRead(BCD[1])==0){
      NETID = NETID | (1<<1);    // Set the n-th bit
    }
    if(digitalRead(BCD[2])==0){
      NETID = NETID | (1<<2);    // Set the n-th bit
    }
    if(digitalRead(BCD[3])==0){
      NETID = NETID | (1<<3);    // Set the n-th bit
    }
  #endif
  return NETID;
}
//---------------------------------------------------------------------------------------------------------

byte IdPrefix(byte ID){
  bitClear(ID, 0);  // ->
  bitClear(ID, 1);
  bitClear(ID, 2);
  bitClear(ID, 3);
  ID = ID >> 4;
  return(ID);
}

//---------------------------------------------------------------------------------------------------------

byte IdSufix(byte ID){
  bitClear(ID, 4);
  bitClear(ID, 5);
  bitClear(ID, 6);
  bitClear(ID, 7);  // <-
  return(ID);
}

//-------------------------------------------------------------------------------------------------------
byte AsciiToHex(int ASCII){
  if(ASCII>=48 && ASCII<=57){
    return(ASCII-48);
  }else if(ASCII>=97 && ASCII<=102){
    return(ASCII-87);
  }else{
    return(255);
  }
}

//-------------------------------------------------------------------------------------------------------
/*
ID FROM TO : BROADCAST ;
ID FROM TO : CONFIRM ;
ID FROM TO : A B C ;
ID FROM TO : A B C ;

TX  0ms:b;
RX  0sm:c;
TX  0ms:123;
RX  0sm:123;
*/

void RX_UDP(){
  #if !defined(XLswitch)
    UDPpacketSize = UdpCommand.parsePacket();    // if there's data available, read a packet
    if (UDPpacketSize){
      UdpCommand.read(packetBuffer, 10);      // read the packet into packetBufffer
      // Print RAW
      if(EnableSerialDebug==1){
        Serial.println();
        Serial.print("RXraw [");
        Serial.print(packetBuffer[0], HEX);
        for(int i=1; i<8; i++){
          Serial.print(char(packetBuffer[i]));
        }
        Serial.print(F("] "));
        Serial.print(UdpCommand.remoteIP());
        Serial.print(":");
        Serial.print(UdpCommand.remotePort());
        Serial.println();
      }

      // ID-FROM-TO filter
      if(
        (EnableGroupPrefix==false
        && String(packetBuffer[0], DEC).toInt()==NET_ID
        && packetBuffer[1]==TxUdpBuffer[2]  // FROM Switch
        && packetBuffer[2]== 's'  // TO
        && packetBuffer[3]== B00000000
        // && packetBuffer[3]== ':'
        && packetBuffer[7]== ';')
        ||
        (EnableGroupPrefix==true
        && IdSufix(packetBuffer[0])==IdSufix(NET_ID)
        && packetBuffer[1]==TxUdpBuffer[2]  // FROM Switch
        && packetBuffer[2]== 's'  // TO
        && packetBuffer[3]== B00000000
        // && packetBuffer[3]== ':'
        && packetBuffer[7]== ';')
      ){

        if( EnableGroupPrefix==true && (
          DetectedRemoteSwPort [IdPrefix(packetBuffer[0])] == 0
          || (packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o')
          || (packetBuffer[4]== 'c' && packetBuffer[5]== 'f' && packetBuffer[6]== 'm')
          )
        ){
          IPAddress TmpAddr = UdpCommand.remoteIP();
          DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [0]=TmpAddr[0];     // Switch IP addres storage to array
          DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [1]=TmpAddr[1];
          DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [2]=TmpAddr[2];
          DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [3]=TmpAddr[3];
          DetectedRemoteSwPort [hexToDecBy4bit(IdPrefix(packetBuffer[0]))]=UdpCommand.remotePort();
          if(EnableSerialDebug==1){
            Serial.print("Detect controller ID ");
            Serial.print(IdPrefix(packetBuffer[0]), HEX);
            Serial.print(" on IP ");
            Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [0]);
            Serial.print(".");
            Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [1]);
            Serial.print(".");
            Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [2]);
            Serial.print(".");
            Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [3]);
            Serial.print(":");
            Serial.println(DetectedRemoteSwPort [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] );
          }
          if(TxUdpBuffer[2] == 'm'){
            TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 1);
          }
        }

        // RX Broadcast / CFM
        if((packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o')
          || (packetBuffer[4]== 'c' && packetBuffer[5]== 'f' && packetBuffer[6]== 'm')
          ){
          if(EnableSerialDebug==1){
            Serial.print("RX [");
            Serial.print(packetBuffer[0], HEX);
            for(int i=1; i<8; i++){
              Serial.print(char(packetBuffer[i]));
            }
            Serial.print(F("] "));
            Serial.print(UdpCommand.remoteIP());
            Serial.print(":");
            Serial.println(UdpCommand.remotePort());
          }
          #if !defined(Ser2net)
            pinMode(BCD[1], OUTPUT);
            digitalWrite(BCD[1], HIGH);
            delay(100);
            digitalWrite(BCD[1], LOW);
            delay(100);
            pinMode(BCD[1], INPUT);
          #endif
          if(packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o'){
            TxUDP('s', packetBuffer[2], 'c', 'f', 'm', 1);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 1);
            }
          }

        // RX DATA
        }else{
          if(EnableGroupButton==true){
            CheckGroup();
          }else{
            ShiftOutByte[0] = String(packetBuffer[4], DEC).toInt();    // Bank0
          }
          ShiftOutByte[1] = String(packetBuffer[5], DEC).toInt();    // Bank1
          ShiftOutByte[2] = String(packetBuffer[6], DEC).toInt();    // Bank2

          // SHIFT OUT
          #if defined(ShiftOut)
            digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
            digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
            if(EnableSerialDebug==1){
              Serial.println("ShiftOut");
            }
          #endif

          if(EnableSerialDebug==1){
            // Serial.println();
            Serial.print(F("RX ["));
            Serial.print(packetBuffer[0], HEX);
            for(int i=1; i<4; i++){
              Serial.print(char(packetBuffer[i]));
            }
            Serial.print((byte)packetBuffer[4], BIN);
            Serial.print(F("|"));
            Serial.print((byte)packetBuffer[5], BIN);
            Serial.print(F("|"));
            Serial.print((byte)packetBuffer[6], BIN);
            Serial.print(F(";] "));
            Serial.print(UdpCommand.remoteIP());
            Serial.print(F(":"));
            Serial.println(UdpCommand.remotePort());
          }
          if(UdpCommand.remotePort() != DetectedRemoteSwPort[hexToDecBy4bit(IdPrefix(packetBuffer[0]))] && EnableGroupPrefix==true){
            // if(EnableSerialDebug==1){
              Serial.print(F("** Change ip-port ID "));
              Serial.print(IdPrefix(packetBuffer[0]), HEX);
              Serial.print(F(" (OLD-"));
              Serial.print(DetectedRemoteSwPort[hexToDecBy4bit(IdPrefix(packetBuffer[0]))]);
              Serial.print(F(" NEW-"));
              Serial.print(UdpCommand.remotePort());
              Serial.println(F(") **"));
            // }
            DetectedRemoteSwPort[hexToDecBy4bit(IdPrefix(packetBuffer[0]))]=UdpCommand.remotePort();
          }
          TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
        }
        WatchdogTimer=millis();
        // activate
        if(OutputWatchdog==123456){
          OutputWatchdog=EEPROM.readUInt(27);
        }
      } // filtered end
      else{
        if(EnableSerialDebug==1){
          Serial.println(F("   Different NET-ID, or bad packet format"));
        }
      }
      memset(packetBuffer, 0, sizeof(packetBuffer));   // Clear contents of Buffer

    } //end IfUdpPacketSice
    #endif
}
//-------------------------------------------------------------------------------------------------------

void RX_UDP_XLswitch(){
  #if defined(XLswitch)
    UDPpacketSize = UdpCommand.parsePacket();    // if there's data available, read a packet
    if (UDPpacketSize){
      UdpCommand.read(packetBuffer, 10);      // read the packet into packetBufffer
      // Print RAW
      if(EnableSerialDebug==1){
        Serial.println();
        Serial.print("RXraw [");
        Serial.print(packetBuffer[0], HEX);
        for(int i=1; i<8; i++){
          Serial.print(char(packetBuffer[i]));
        }
        Serial.print(F("] "));
        Serial.print(UdpCommand.remoteIP());
        Serial.print(":");
        Serial.print(UdpCommand.remotePort());
        Serial.println();
      }

      // ID-FROM-TO filter
      if(
        (EnableGroupPrefix==false
        && String(packetBuffer[0], DEC).toInt()==NET_ID
        && packetBuffer[1]==TxUdpBuffer[2]  // FROM Switch
        && packetBuffer[2]== 's'  // TO
        && packetBuffer[3]== B00000000
        // && packetBuffer[3]== ':'
        && packetBuffer[7]== ';')
        ||
        (EnableGroupPrefix==true
        && IdSufix(packetBuffer[0])==IdSufix(NET_ID)
        && packetBuffer[1]==TxUdpBuffer[2]  // FROM Switch
        && packetBuffer[2]== 's'  // TO
        && packetBuffer[3]== B00000000
        // && packetBuffer[3]== ':'
        && packetBuffer[7]== ';')
      ){

        if( EnableGroupPrefix==true && (
          DetectedRemoteSwPort [IdPrefix(packetBuffer[0])] == 0
          || (packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o')
          || (packetBuffer[4]== 'c' && packetBuffer[5]== 'f' && packetBuffer[6]== 'm')
          )
        ){
          IPAddress TmpAddr = UdpCommand.remoteIP();
          DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [0]=TmpAddr[0];     // Switch IP addres storage to array
          DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [1]=TmpAddr[1];
          DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [2]=TmpAddr[2];
          DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [3]=TmpAddr[3];
          DetectedRemoteSwPort [hexToDecBy4bit(IdPrefix(packetBuffer[0]))]=UdpCommand.remotePort();
          if(EnableSerialDebug==1){
            Serial.print("Detect controller ID ");
            Serial.print(IdPrefix(packetBuffer[0]), HEX);
            Serial.print(" on IP ");
            Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [0]);
            Serial.print(".");
            Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [1]);
            Serial.print(".");
            Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [2]);
            Serial.print(".");
            Serial.print(DetectedRemoteSw [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] [3]);
            Serial.print(":");
            Serial.println(DetectedRemoteSwPort [hexToDecBy4bit(IdPrefix(packetBuffer[0]))] );
          }
          if(TxUdpBuffer[2] == 'm'){
            TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 1);
          }
        }

        // RX Broadcast / CFM
        if((packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o')
          || (packetBuffer[4]== 'c' && packetBuffer[5]== 'f' && packetBuffer[6]== 'm')
          ){
          if(EnableSerialDebug==1){
            Serial.print("RX [");
            Serial.print(packetBuffer[0], HEX);
            for(int i=1; i<8; i++){
              Serial.print(char(packetBuffer[i]));
            }
            Serial.print(F("] "));
            Serial.print(UdpCommand.remoteIP());
            Serial.print(":");
            Serial.println(UdpCommand.remotePort());
          }
          #if !defined(Ser2net) && !defined(XLswitch)
            pinMode(BCD[1], OUTPUT);
            digitalWrite(BCD[1], HIGH);
            delay(100);
            digitalWrite(BCD[1], LOW);
            delay(100);
            pinMode(BCD[1], INPUT);
          #endif
          if(packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o'){
            TxUDP('s', packetBuffer[2], 'c', 'f', 'm', 1);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 1);
            }
          }

        // RX DATA
        }else{
          if(EnableGroupButton==true){
            CheckGroup();
          }else{
            ShiftOutByte[0] = String(packetBuffer[4], DEC).toInt();    // Bank0
          }
          // select between A/B
          if(bitRead(ShiftOutByte[0], 0)==0){
            ShiftOutByte[1] = String(packetBuffer[5], DEC).toInt();    // Bank1
            ShiftOutByte[2] = String(packetBuffer[6], DEC).toInt();    // Bank2
          }else{
            ShiftOutByte[3] = String(packetBuffer[5], DEC).toInt();    // Bank1
            ShiftOutByte[4] = String(packetBuffer[6], DEC).toInt();    // Bank2
          }

          // SHIFT OUT
          #if defined(ShiftOut)
            digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[4]);
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[3]);
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
            // shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);  // buttons
            digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
            if(EnableSerialDebug==1){
              Serial.println("ShiftOut");
            }
          #endif

          if(EnableSerialDebug==1){
            // Serial.println();
            Serial.print(F("RX ["));
            Serial.print(packetBuffer[0], HEX);
            for(int i=1; i<4; i++){
              Serial.print(char(packetBuffer[i]));
            }
            // Serial.print((byte)packetBuffer[4], BIN);
            Serial.print((byte)ShiftOutByte[1], BIN);
            Serial.print(F("|"));
            // Serial.print((byte)packetBuffer[5], BIN);
            Serial.print((byte)ShiftOutByte[2], BIN);
            Serial.print(F("|"));
            // Serial.print((byte)packetBuffer[6], BIN);
            Serial.print((byte)ShiftOutByte[3], BIN);
            Serial.print(F("|"));
            Serial.print((byte)ShiftOutByte[4], BIN);
            Serial.print(F(";] "));
            Serial.print(UdpCommand.remoteIP());
            Serial.print(F(":"));
            Serial.println(UdpCommand.remotePort());
          }
          if(UdpCommand.remotePort() != DetectedRemoteSwPort[hexToDecBy4bit(IdPrefix(packetBuffer[0]))] && EnableGroupPrefix==true){
            // if(EnableSerialDebug==1){
              Serial.print(F("** Change ip-port ID "));
              Serial.print(IdPrefix(packetBuffer[0]), HEX);
              Serial.print(F(" (OLD-"));
              Serial.print(DetectedRemoteSwPort[hexToDecBy4bit(IdPrefix(packetBuffer[0]))]);
              Serial.print(F(" NEW-"));
              Serial.print(UdpCommand.remotePort());
              Serial.println(F(") **"));
            // }
            DetectedRemoteSwPort[hexToDecBy4bit(IdPrefix(packetBuffer[0]))]=UdpCommand.remotePort();
          }
          TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
          #if defined(XLswitch)
            if(TxUdpBuffer[2]=='m'){
              LcdNeedRefresh=2;
            }
          #endif
        }
        WatchdogTimer=millis();
        // activate
        if(OutputWatchdog==123456){
          OutputWatchdog=EEPROM.readUInt(27);
        }
      } // filtered end
      else{
        if(EnableSerialDebug==1){
          Serial.println(F("   Different NET-ID, or bad packet format"));
        }
      }
      memset(packetBuffer, 0, sizeof(packetBuffer));   // Clear contents of Buffer

      #if defined(XLswitch)
        digitalWrite(StatusLedBPin, HIGH);
        StatusLedB = true;
        StatusLedBTimer[0]=millis();
      #endif

    } //end IfUdpPacketSice
    #endif
}

//-------------------------------------------------------------------------------------------------------

void CheckGroup(){
  int ChangeBit=9;
  int NumberOfChange=0;
  for (int i=0; i<8; i++){
    if(bitRead(packetBuffer[4], i)!=bitRead(ShiftOutByte[0], i)){
      ChangeBit=i;
      NumberOfChange++;
    }
  }
  // Serial.print("ChangeBit|NumberOfChange ");
  // Serial.print(ChangeBit+1);
  // Serial.print(" ");
  // Serial.println(NumberOfChange);

  ShiftOutByte[0] = String(packetBuffer[4], DEC).toInt();    // Bank0
  if(NumberOfChange==1){
    // Serial.println("clearGroup");
    NumberOfChange=0;
    for (int i=0; i<8; i++){
      if(GroupButton[ChangeBit]==GroupButton[i] && ChangeBit!=i){
        bitClear(ShiftOutByte[0], i);
        NumberOfChange++;
        // Serial.print("Bitclear ");
        // Serial.println(i);
      }
    }
    if(NumberOfChange>0){
      bitSet(ShiftOutByte[0], ChangeBit);
    }
  }
}
//-------------------------------------------------------------------------------------------------------

unsigned char hexToDecBy4bit(unsigned char hex)
// convert a character representation of a hexidecimal digit into the actual hexidecimal value
{
  if(hex > 0x39) hex -= 7; // adjust for hex letters upper or lower case
  return(hex & 0xf);
}

//-------------------------------------------------------------------------------------------------------

void TxUDP(byte FROM, byte TO, byte A, byte B, byte C, int DIRECT){

  // TxUdpBuffer[0] = NET_ID;
  TxUdpBuffer[1] = FROM;
  // TxUdpBuffer[2] = TO;

  // if(TxUdpBuffer[2]=='m' && ( EnableGroupPrefix==true || EnableGroupButton==true ) ){
  //   TxUdpBuffer[3] = B00101101;           // -  multi control || GroupButton
  // }else{
  //   TxUdpBuffer[3] = B00111010;           // :
  // }

  TxUdpBuffer[3] = B00000000;
    // multi control
    if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
      bitSet(TxUdpBuffer[3], 0);
    }
    // group button
    if(TxUdpBuffer[2]=='m' && EnableGroupButton==true){
      bitSet(TxUdpBuffer[3], 1);
    }

  TxUdpBuffer[4] = A;
  TxUdpBuffer[5] = B;
  TxUdpBuffer[6] = C;
  TxUdpBuffer[7] = B00111011;           // ;

  // BROADCAST
  if(A=='b' && B=='r' && C=='o'){  // b r o
    if(TxUdpBuffer[2] == 'm'){
      TxUdpBuffer[6] = NumberOfEncoderOutputs;
    }
    // direct
    if(DIRECT==0){
      RemoteSwIP = ~ETH.subnetMask() | ETH.gatewayIP();
      if(EnableSerialDebug==1){
        Serial.print(F("TX broadcast ["));
      }
    }else{
      RemoteSwIP = UdpCommand.remoteIP();
      if(EnableSerialDebug==1){
        Serial.print(F("TX direct ["));
      }
    }
    UdpCommand.beginPacket(RemoteSwIP, BroadcastPort);

  // CFM
  }else if(A=='c' && B=='f' && C=='m'){  // cfm
      if(TxUdpBuffer[2] == 'm'){
        TxUdpBuffer[6] = NumberOfEncoderOutputs;
      }
      if(EnableSerialDebug==1){
        Serial.print(F("TX direct ["));
      }
      UdpCommand.beginPacket(UdpCommand.remoteIP(), UdpCommand.remotePort());

  // DATA
  }else{
    RemoteSwIP = UdpCommand.remoteIP();
    if(EnableSerialDebug==1){
      Serial.print(F("TX ["));
    }
    UdpCommand.beginPacket(RemoteSwIP, UdpCommand.remotePort());
  }

  // send
  if(EnableGroupPrefix==false){
    UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
    UdpCommand.endPacket();
    if(EnableSerialDebug==1){
      Serial.print(TxUdpBuffer[0], HEX);
      Serial.print(char(TxUdpBuffer[1]));
      Serial.print(char(TxUdpBuffer[2]));
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[3], BIN);
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[4], BIN);
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[5], BIN);
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[6], BIN);
      Serial.print(char(TxUdpBuffer[7]));
      Serial.print(F("] "));
      Serial.print(RemoteSwIP);
      Serial.print(F(":"));
      Serial.print(UdpCommand.remotePort());
      #if defined(WIFI)
        Serial.print(" | dBm: ");
        Serial.print(WiFi.RSSI());
      #endif
      Serial.println();
    }

  // send EnableGroupPrefix
  }else{
    // answer to RX ip
    TxUdpBuffer[0]=packetBuffer[0];   // NET_ID by RX NET_ID
    UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
    UdpCommand.endPacket();
    if(EnableSerialDebug==1){
      Serial.print(TxUdpBuffer[0], HEX);
      for (int i=1; i<4; i++){
        Serial.print(char(TxUdpBuffer[i]));
      }
      Serial.print(TxUdpBuffer[4], BIN);
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[5], BIN);
      Serial.print(F("|"));
      Serial.print(TxUdpBuffer[6], BIN);
      Serial.print(char(TxUdpBuffer[7]));
      Serial.print(F("] "));
      Serial.print(RemoteSwIP);
      Serial.print(F(":"));
      Serial.print(UdpCommand.remotePort());
      #if defined(WIFI)
        Serial.print(" | dBm: ");
        Serial.print(WiFi.RSSI());
      #endif
      Serial.println();
    }
    // send to all ip from storage
    IPAddress ControllerIP = UdpCommand.remoteIP();
    for (int i=0; i<16; i++){
      if(DetectedRemoteSwPort[i]!=0){
        TxUdpBuffer[0]=IdSufix(NET_ID) | i<<4;       // NET_ID by destination device
        RemoteSwIP = DetectedRemoteSw[i];
        RemoteSwPort = DetectedRemoteSwPort[i];
        if(ControllerIP!=RemoteSwIP){
          UdpCommand.beginPacket(RemoteSwIP, RemoteSwPort);
            UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
          UdpCommand.endPacket();

          if(EnableSerialDebug==1){
            Serial.print(F("TX direct ID-"));
            Serial.print(i, HEX);
            Serial.print(IdSufix(NET_ID), HEX);
            Serial.print(F(" "));
            Serial.print(RemoteSwIP);
            Serial.print(F(":"));
            Serial.print(RemoteSwPort);
            Serial.print(F(" ["));
            Serial.print(TxUdpBuffer[0], HEX);
            for (int i=1; i<8; i++){
              Serial.print(char(TxUdpBuffer[i]));
              // Serial.print(F(" "));
            }
            Serial.print("]");
            #if defined(WIFI)
              Serial.print(" WiFi dBm: ");
              Serial.print(WiFi.RSSI());
            #endif
            Serial.println();
          }
        }else{
          if(EnableSerialDebug==1){
            Serial.print(F("noTX - RX prefix "));
            Serial.print(i, HEX);
            Serial.print(F(" "));
            Serial.print(RemoteSwIP);
            Serial.print(F(":"));
            Serial.println(RemoteSwPort);
          }
        }
      }
    }
    // broadcast all prefix
    if(A=='b' && B=='r' && C=='o' && DIRECT==0 && TxUdpBuffer[2] == 'm'){
      if(EnableSerialDebug==1){
        Serial.print("TX all prefix ");
        Serial.print(RemoteSwIP);
        Serial.print(":");
        Serial.print(BroadcastPort);
        Serial.print(F(" ["));
        Serial.print("*");
        for (int i=1; i<8; i++){
          Serial.print(char(TxUdpBuffer[i]));
          // Serial.print(F(" "));
        }
        Serial.println("]");
      }
      Serial.print("*) ");
      for (int i=0; i<16; i++){
        TxUdpBuffer[0]=IdSufix(NET_ID) | (i<<4);
        if(EnableSerialDebug==1){
          Serial.print(TxUdpBuffer[0], HEX);
          Serial.print(" ");
        }
        UdpCommand.beginPacket(RemoteSwIP, BroadcastPort);
        UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
        UdpCommand.endPacket();
      }
      if(EnableSerialDebug==1){
        Serial.println();
      }
    }   // end b r o

  } // end EnableGroupPrefix
}
//-------------------------------------------------------------------------------------------------------

void http(){
  // listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    if(EnableSerialDebug==1){
      Serial.println("WIFI New client");
    }
    #if defined(XLswitch)
      digitalWrite(StatusLedBPin, HIGH);
      StatusLedB = true;
      StatusLedBTimer[0]=millis();
    #endif
    memset(linebuf,0,sizeof(linebuf));
    charcount=0;
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        HTTP_req += c;
        // if(EnableSerialDebug==1){
        //   Serial.write(c);
        // }
        //read char by char HTTP request
        linebuf[charcount]=c;
        if (charcount<sizeof(linebuf)-1) charcount++;
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header

          client.println(F("HTTP/1.1 200 OK"));
          client.println(F("Content-Type: text/html"));
          client.println(F("Connection: close"));
          client.println();
          client.println(F("<!DOCTYPE html>"));
          client.println(F("<html>"));
          client.println(F("<head>"));
          client.print(F("<title>"));
          client.println(F("IP switch NET-ID "));
          if(NET_ID <=0x0f){
            client.print("0");
          }
          client.println(NET_ID, HEX);
          client.println(F("</title>"));
          // client.print(F("<meta http-equiv=\"refresh\" content=\"10"));
          client.print(F("<meta http-equiv=\"refresh\" content=\"10;url=http://"));
          #if defined(ETHERNET)
            client.print(ETH.localIP());
          #endif
          #if defined(WIFI)
            client.print(WiFi.localIP());
          #endif
          client.println(F("\">"));
          client.println(F("<link href='http://fonts.googleapis.com/css?family=Roboto+Condensed:300italic,400italic,700italic,400,700,300&subset=latin-ext' rel='stylesheet' type='text/css'>"));
          client.println(F("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">"));
          client.println(F("<meta name=\"mobile-web-app-capable\" content=\"yes\">"));
          client.println(F("<style type=\"text/css\">"));
          client.println(F("body {font-family: 'Roboto Condensed',sans-serif,Arial,Tahoma,Verdana;background: #ccc;}"));
          client.println(F("a:link  {color: #888;font-weight: bold;text-decoration: none;}"));
          client.println(F("a:visited  {color: #888;font-weight: bold;text-decoration: none;}"));
          client.println(F("a:hover  {color: #888;font-weight: bold;text-decoration: none;}"));
          client.println(F("input {border: 2px solid #aaa;background: #ccc;margin: 10px 5px 0 0;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #333;}"));
          client.println(F("input:hover {border: 2px solid #080;}"));
          client.println(F("input.g {background: #080;color: #fff;}"));
          client.println(F("input.gr {background: #800;color: #fff;}"));
          client.println(F(".box {border: 2px solid #080;background: #ccc;  line-height: 2; margin: 10px 5px 0 5px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #000;}"));
          client.println(F(".boxr {border: 2px solid #800;background: #800; line-height: 2; margin: 10px 5px 0 5px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #fff;}"));
          client.println(F(".boxg {border: 2px solid #888;background: #ccc; line-height: 2; margin: 10px 5px 0 5px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #888;}"));
          // client.println(F(".ptt {border: 2px solid #800;background: #ccc;margin: 10px 15px 0 10px;padding: 1px 7px 1px 7px;-webkit-border-radius: 5px;-moz-border-radius: 5px;border-radius: 5px;color : #800;}"));
          client.println(F("</style></head><body>"));
          String GETOUTPUT = HTTP_req.substring(7, 8);

          //   // SetPin(GETOUTPUT.toInt(), !OutputPinStatus[GETOUTPUT.toInt()]);
          //   ReversePin(GETOUTPUT.toInt());
          if(GETOUTPUT.toInt()==1){
            TxUdpBuffer[2]='o';
            EEPROM.write(0, 'o'); // address, value
            EEPROM.commit();
            TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
            }
          }
          if(GETOUTPUT.toInt()==2){
            TxUdpBuffer[2]='r';
            EEPROM.write(0, 'r'); // address, value
            EEPROM.commit();
            TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
            }
          }
          if(GETOUTPUT.toInt()==3){
            TxUdpBuffer[2]='m';
            EEPROM.write(0, 'm'); // address, value
            EEPROM.commit();
            TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
            }
          }
          // if(GETOUTPUT.toInt()==4){
          //   EnableGroupPrefix=!EnableGroupPrefix;
          //   Serial.print("** Group sufix (multi control) [");
          //   EEPROM.write(4, EnableGroupPrefix);
          //   EEPROM.commit();
          //   if(EnableGroupPrefix==true){
          //     Serial.println("ON] **");
          //   }else{
          //     Serial.println("OFF] **");
          //   }
          //   // if(EnableGroupPrefix==true){
          //   //   // clear prefix
          //   //   bitClear(NET_ID, 4);
          //   //   bitClear(NET_ID, 5);
          //   //   bitClear(NET_ID, 6);
          //   //   bitClear(NET_ID, 7); // <-
          //   // }
          //   // if(TxUdpBuffer[2] == 'm'){
          //   //   TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
          //   // }
          //   Serial.println("** IP switch will be restarted **");
          //   delay(1000);
          // TelnetServerClients[0].stop();
          //   ESP.restart();
          // }

          if(TxUdpBuffer[2]!='n'){
            // client.println("<h1>RemoteSwitch </h1>");
            client.println("<p>Output status<br>");
            if( EnableGroupButton==true){
              client.print(" (Group buttons ENABLE)<br>");
            }
            for (i = 0; i < 3; i++) {
              client.print("Bank ");
              client.print(i+1);
              client.print(" ");
              for (int j = 0; j < 8; j++) {
                client.print("<span class=\"box");
                if( (i==1 && (j>NumberOfEncoderOutputs)) || (i==2 && (j>NumberOfEncoderOutputs-8)) ){
                  // nil
                  client.print("g\">x</span>");
                }else{
                  if(bitRead(ShiftOutByte[i], j)==1){
                    client.print("r");
                  }
                  client.print("\">");
                  // client.print(bitRead(ShiftOutByte[i], j));
                  if(i==2){
                    client.print(j+1+8);
                  }else{
                    client.print(j+1);
                  }
                  // Group button
                  if(i==0 && EnableGroupButton==true){
                    client.print(" group");
                    client.print(GroupButton[j]);
                  }

                  client.println("</span>");

                  // button break
                  if(i==0 && j==3){
                    client.println("<br>");
                    client.print("Bank ");
                    client.print(i+1);
                    client.print(" ");
                  }
                }
              }
              client.println("<br>");
            }
            client.println("</p>");
          }
          client.print("Wifi: ");
          #if defined(WIFI)
            client.print("<b>");
            client.print(ssid);
            client.print("</b> ");
            client.print(WiFi.RSSI());
            client.print(" dBm, ");
            client.print(WiFi.localIP());
          #else
            client.print("OFF");
          #endif

          client.print(" | ETH: ");
          #if defined(ETHERNET)
            client.print(ETH.macAddress());
            client.print(", ");
            client.print(ETH.localIP());
          #else
            client.print("OFF");
          #endif

          client.print(F(" | Version: "));
          client.print(F(REV));
          client.print(F(" | Uptime: "));
          client.print(millis()/1000);
          client.println(F(" s<br>"));
          #if defined(Ser2net) && !defined(XLswitch)
            client.println(F(" Ser2net: ip port "));
            client.print(SerialServerIPport);
            client.print(" &#8644; baudrate ");
            client.print(SERIAL1_BAUDRATE);
            client.println(F(" (rx gpio16, tx gpio17)<br>Source: "));
          #endif
          if(TxUdpBuffer[2] == 'o'){
            client.println("Open Interface III");
          }else if(TxUdpBuffer[2] == 'r'){
            client.println("Band decoder MK2");
          }else if(TxUdpBuffer[2] == 'm'){
            client.println("Manual IP switch MK2");
          }else if(TxUdpBuffer[2] == 'n'){
            client.println("none");
          }
          client.println(F("<br><span style=\"color:Tomato;\">NOTICE: </span>showing this page slows down the device response."));

          if(TxUdpBuffer[2]=='m' && EnableGroupPrefix){
            client.println(F("<form method=\"get\">"));
            client.print("<input type=\"submit\" name=\"S4\" value=\"Multi control\" class=\"");
            if(EnableGroupPrefix==true){
              client.print("g");
            }else{
              client.print("r");
            }

            // client.print("<input type=\"submit\" name=\"S1\" value=\"Open Interface III\" class=\"");
            // if(TxUdpBuffer[2] == 'o'){
            //   client.print("g");
            // }else{
            //   client.print("r");
            // }
            // client.print("\"><input type=\"submit\" name=\"S2\" value=\"Band decoder MK2\" class=\"");
            // if(TxUdpBuffer[2] == 'r'){
            //   client.print("g");
            // }else{
            //   client.print("r");
            // }
            // client.print("\"><input type=\"submit\" name=\"S3\" value=\"Manual IP switch\" class=\"");
            // if(TxUdpBuffer[2] == 'm'){
            //   client.print("g");
            // }else{
            //   client.print("r");
            // }

            // <input type="submit" name="S1" value="[ 1 ]" class="r">
            client.println(F("\"></form>"));

            client.print("<pre>");
                  client.println("List detected IP control by NET-ID prefix");
                  for (int i = 0; i < 16; i++) {
                    if(DetectedRemoteSwPort[i]!=0){
                      client.print(i, HEX);
                      client.print(F("  "));
                      client.print(DetectedRemoteSw [i] [0]);
                      client.print(F("."));
                      client.print(DetectedRemoteSw [i] [1]);
                      client.print(F("."));
                      client.print(DetectedRemoteSw [i] [2]);
                      client.print(F("."));
                      client.print(DetectedRemoteSw [i] [3]);
                      client.print(F(":"));
                      client.println(DetectedRemoteSwPort [i]);
                    }
                  }
                  client.print("</pre>");
            }

          client.println(F("<p><a href=\"https://remoteqth.com/wiki/index.php?page=IP+Switch+with+ESP32-GATEWAY#Web_status_page\"> Wiki</a> | "));
          client.println(F("<p><a href=\".\" onclick=\"window.open( this.href, this.href, 'width=400,height=180,left=0,top=0,menubar=no,location=no,status=no' ); return false;\" > split&#8599;</a></p>"));
          client.println(F("</body></html>"));

          if(EnableSerialDebug==1){
            Serial.print(HTTP_req);
          }
          HTTP_req = "";

          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
          // if (strstr(linebuf,"GET /h0 ") > 0){digitalWrite(GPIOS[0], HIGH);}else if (strstr(linebuf,"GET /l0 ") > 0){digitalWrite(GPIOS[0], LOW);}
          // else if (strstr(linebuf,"GET /h1 ") > 0){digitalWrite(GPIOS[1], HIGH);}else if (strstr(linebuf,"GET /l1 ") > 0){digitalWrite(GPIOS[1], LOW);}

          // you're starting a new line
          currentLineIsBlank = true;
          memset(linebuf,0,sizeof(linebuf));
          charcount=0;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    client.stop();
   if(EnableSerialDebug==1){
     Serial.println("WIFI client disconnected");
   }
  }
}
//-------------------------------------------------------------------------------------------------------

void EthEvent(WiFiEvent_t event)
{
  switch (event) {
    case SYSTEM_EVENT_ETH_START:
      Serial.println("ETH  Started");
      //set eth hostname here
      ETH.setHostname("esp32-ethernet");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("ETH  Connected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.print("ETH  MAC: ");
      Serial.println(ETH.macAddress());
      Serial.println("===============================");
      Serial.print("   IPv4: ");
      Serial.println(ETH.localIP());
      Serial.println("===============================");
      if (ETH.fullDuplex()) {
        Serial.print("FULL_DUPLEX, ");
      }
      Serial.print(ETH.linkSpeed());
      Serial.println("Mbps");
      eth_connected = true;
      #if defined(XLswitch)
        digitalWrite(StatusLedAPin, HIGH);
      #endif
      #if !defined(Ser2net) && !defined(XLswitch)
        pinMode(BCD[1], OUTPUT);
        digitalWrite(BCD[1], HIGH);
        delay(100);
        digitalWrite(BCD[1], LOW);
        delay(100);
        digitalWrite(BCD[1], HIGH);
        delay(100);
        digitalWrite(BCD[1], LOW);
        delay(100);
        digitalWrite(BCD[1], HIGH);
        delay(100);
        delay(100);
        digitalWrite(BCD[1], LOW);
        pinMode(BCD[1], INPUT);
      #endif
      if(TxUdpBuffer[2]!='n'){
        // clear
        // Serial.println("Snake&clear");
        // Demo();
        #if defined(ShiftOut)
          if(RebootWatchdog > 0 && TxUdpBuffer[2]!='n'){
            Serial.println("Restore outputs...");
            digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, 0x01);
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
            shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
            digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
            Serial.print("  Bank status ABC [LSBFIRST]: ");
            Serial.print(ShiftOutByte[0], BIN);
            Serial.print(" ");
            Serial.print(ShiftOutByte[1], BIN);
            Serial.print(" ");
            Serial.println(ShiftOutByte[2], BIN);
          }
        #endif
      }
      // EnableSerialDebug=1;
      TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
      if(TxUdpBuffer[2] == 'm'){
        TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
      }
      // EnableSerialDebug=0;
      #if defined(XLswitch)
          LcdNeedRefresh=100;
      #endif
      break;

    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH  Disconnected");
      eth_connected = false;
      #if defined(XLswitch)
        digitalWrite(StatusLedAPin, LOW);
      #endif
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH  Stopped");
      eth_connected = false;
      #if defined(XLswitch)
        digitalWrite(StatusLedAPin, LOW);
      #endif
      break;
    default:
      break;
  }
}
//-------------------------------------------------------------------------------------------------------

void testClient(const char * host, uint16_t port)
{
  Serial.print("\nETH  connecting to ");
  Serial.println(host);
  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("ETH  connection failed");
    return;
  }
  client.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
  while (client.connected() && !client.available());
  while (client.available()) {
    Serial.write(client.read());
  }
  Serial.println("ETH closing connection\n");
  client.stop();
}

//-------------------------------------------------------------------------------------------------------
void TelnetAuth(){

  switch (TelnetAuthStep) {
    case 0: {
      if(TelnetLoginFails>=3 && millis()-TelnetLoginFailsBanTimer[0]<TelnetLoginFailsBanTimer[1]){
        Prn(1, 1,"");
        Prn(1, 0,"   Ten minutes login ban, PSE QRX ");
        Prn(1, 0,String((TelnetLoginFailsBanTimer[1]-millis()-TelnetLoginFailsBanTimer[0])/1000));
        Prn(1, 1," seconds");
        delay(3000);
        TelnetServerClients[0].stop();
        break;
      }else if(TelnetLoginFails>2 && millis()-TelnetLoginFailsBanTimer[0]>TelnetLoginFailsBanTimer[1]){
        TelnetLoginFails=0;
      }
      if(TelnetLoginFails<=3){
        Prn(1, 1,"Login? [y/n] ");
        TelnetAuthStep++;
        incomingByte=0;
      }
      break; }
    case 1: {
      // incomingByte=TelnetRX();
      if(incomingByte==121 || incomingByte==89){
        Prn(1, 1,String(char(incomingByte)));
        TelnetAuthStep++;
      }else if(incomingByte!=121 && incomingByte!=0){
        TelnetAuthStep=0;
      }
      break; }
    case 2: {
      AuthQ(1, 0);
      TelnetAuthStepFails=0;
      break; }
    case 3: {
      if(incomingByte==key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(2, 0);
      }else if(incomingByte!=0 && incomingByte!=key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(2, 1);
      }
      break; }
    case 4: {
      if(incomingByte==key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(3, 0);
      }else if(incomingByte!=0 && incomingByte!=key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(3, 1);
      }
      break; }
    case 5: {
      if(incomingByte==key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(4, 0);
      }else if(incomingByte!=0 && incomingByte!=key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        AuthQ(4, 1);
      }
      break; }
    case 6: {
      if(incomingByte==key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        TelnetAuthStep++;
        incomingByte=0;
      }else if(incomingByte!=0 && incomingByte!=key[RandomNumber]){
        Prn(1, 1, String(char(incomingByte)) );
        TelnetAuthStep++;
        incomingByte=0;
        TelnetAuthStepFails++;
      }
      break; }
    case 7: {
      if(TelnetAuthStepFails==0){
        TelnetAuthorized = true;
        TelnetServerClientAuth = TelnetServerClients[0].remoteIP();
        Prn(1, 1,"Login OK");
        ListCommands(1);
        TelnetAuthStep++;
        incomingByte=0;
      }else{
        TelnetAuthorized = false;
        TelnetServerClientAuth = {0,0,0,0};
        Prn(1, 1,"Access denied");
        TelnetAuthStep=0;
        incomingByte=0;
        TelnetLoginFails++;
        TelnetLoginFailsBanTimer[0]=millis();
      }
      EEPROM.write(37, TelnetServerClientAuth[0]); // address, value
      EEPROM.write(38, TelnetServerClientAuth[1]); // address, value
      EEPROM.write(39, TelnetServerClientAuth[2]); // address, value
      EEPROM.write(40, TelnetServerClientAuth[3]); // address, value
      EEPROM.commit();
      break; }
  }
}

//-------------------------------------------------------------------------------------------------------

void AuthQ(int NR, bool BAD){
  Prn(1, 0,"What character is at ");
  RandomNumber=random(0, strlen(key));
  Prn(1, 0, String(RandomNumber+1) );
  Prn(1, 0," position, in key? (");
  Prn(1, 0,String(NR));
  Prn(1, 1,"/4)");
  // Prn(1, 1, String(key[RandomNumber]) );
  TelnetAuthStep++;
  incomingByte=0;
  if(BAD==true){
    TelnetAuthStepFails++;
  }
}

//-------------------------------------------------------------------------------------------------------
void Telnet(){
  uint8_t i;
  // if (wifiMulti.run() == WL_CONNECTED) {
  if (eth_connected==true) {

    //check if there are any new clients
    if (TelnetServer.hasClient()){
      for(i = 0; i < MAX_SRV_CLIENTS; i++){
        //find free/disconnected spot
        if (!TelnetServerClients[i] || !TelnetServerClients[i].connected()){
          if(TelnetServerClients[i]) TelnetServerClients[i].stop();
          TelnetServerClients[i] = TelnetServer.available();
          if (!TelnetServerClients[i]) Serial.println("Telnet available broken");
          if(EnableSerialDebug==1){
            Serial.println();
            Serial.print("New Telnet client: ");
            Serial.print(i); Serial.print(' ');
            Serial.println(TelnetServerClients[i].remoteIP());
          }
          break;
        }
      }
      if (i >= MAX_SRV_CLIENTS) {
        //no free/disconnected spot so reject
        TelnetServer.available().stop();
      }
    }

    //check clients for data
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (TelnetServerClients[i] && TelnetServerClients[i].connected()){
        if(TelnetServerClients[i].available()){
          //get data from the telnet client and push it to the UART
          // while(TelnetServerClients[i].available()) Serial_one.write(TelnetServerClients[i].read());
          if(EnableSerialDebug==1){
            Serial.println();
            Serial.print("TelnetRX ");
          }

          while(TelnetServerClients[i].available()){
            incomingByte=TelnetServerClients[i].read();
            // Serial_one.write(RX);
            if(EnableSerialDebug==1){
              // Serial.write(RX);
              Serial.print(char(incomingByte));
            }
          }
        }
      }else{
        if (TelnetServerClients[i]) {
          TelnetServerClients[i].stop();
          TelnetAuthorized=false;
          // TelnetServerClientAuth = {0,0,0,0};
        }
      }
    }

    //check UART for data
    // if(Serial_one.available()){
    //   size_t len = Serial_one.available();
    //   uint8_t sbuf[len];
    //   Serial_one.readBytes(sbuf, len);
    //   //push UART data to all connected telnet clients
    //   for(i = 0; i < MAX_SRV_CLIENTS; i++){
    //     if (TelnetServerClients[i] && TelnetServerClients[i].connected()){
    //       TelnetServerClients[i].write(sbuf, len);
    //       // delay(1);
    //       if(EnableSerialDebug==1){
    //         Serial.println();
    //         Serial.print("Telnet tx-");
    //         Serial.write(sbuf, len);
    //       }
    //     }
    //   }
    // }

  }else{
    if(EnableSerialDebug==1){
      Serial.println("Telnet not connected!");
    }
    for(i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (TelnetServerClients[i]) TelnetServerClients[i].stop();
    }
    delay(1000);
  }
}

//-------------------------------------------------------------------------------------------------------

void SerialToIp(){
  #if defined(Ser2net) && !defined(XLswitch)
  uint8_t i;
  // if (wifiMulti.run() == WL_CONNECTED) {
  if (eth_connected==true) {
    //check if there are any new clients
    if (SerialServer.hasClient()){
      for(i = 0; i < MAX_SRV_CLIENTS; i++){
        //find free/disconnected spot
        if (!SerialServerClients[i] || !SerialServerClients[i].connected()){
          if(SerialServerClients[i]) SerialServerClients[i].stop();
          SerialServerClients[i] = SerialServer.available();
          if (!SerialServerClients[i]) Serial.println("available broken");
          if(EnableSerialDebug==1){
            Serial.println();
            Serial.print("New Ser2net client: ");
            Serial.print(i); Serial.print(' ');
            Serial.println(SerialServerClients[i].remoteIP());
          }
          break;
        }
      }
      if (i >= MAX_SRV_CLIENTS) {
        //no free/disconnected spot so reject
        SerialServer.available().stop();
      }
    }
    //check clients for data
    for(i = 0; i < MAX_SRV_CLIENTS; i++){
      if (SerialServerClients[i] && SerialServerClients[i].connected()){
        if(SerialServerClients[i].available()){
          //get data from the telnet client and push it to the UART
          // while(SerialServerClients[i].available()) Serial_one.write(SerialServerClients[i].read());
          if(EnableSerialDebug==1){
            Serial.println();
            Serial.print("rx-");
          }

/*
Zapinaci pakety
< 2019/04/17 22:05:29.670771  length=1 from=2009 to=2009
 0d
< 2019/04/17 22:05:29.734563  length=3 from=2010 to=2012
 30 31 0d
> 2019/04/17 22:05:29.924267  length=2 from=3338 to=3339
 0d 0d
> 2019/04/17 22:05:29.972263  length=3 from=3340 to=3342
 30 31 0d
< 2019/04/17 22:05:30.086153  length=1 from=2013 to=2013
 0d
> 2019/04/17 22:05:30.249406  length=3 from=3343 to=3345
 30 31 0d
< 2019/04/17 22:05:30.261738  length=2 from=2014 to=2015
 ff 0d

Udrzovaci pakety
> 2019/04/17 21:59:00.549347  length=2 from=3319 to=3320
 ff 0d
< 2019/04/17 21:59:00.559100  length=2 from=1998 to=1999
 ff 0d
< 2019/04/17 21:59:02.555837  length=1 from=2000 to=2000
 0d
< 2019/04/17 21:59:04.568329  length=1 from=2001 to=2001
 0d
> 2019/04/17 21:59:05.549832  length=2 from=3321 to=3322
 ff 0d
< 2019/04/17 21:59:05.558790  length=2 from=2002 to=2003
 ff 0d

Vypinaci Paket
 < 2019/04/17 21:59:10.382425  length=3 from=2006 to=2008
 30 30 0d
> 2019/04/17 21:59:10.388125  length=15 from=3323 to=3337
 0d 0d 0d 0d 0d 0d 0d 0d 0d 0d 0d 0d 30 30 0d

*/


          while(SerialServerClients[i].available()){
            byte RX;
            RX=SerialServerClients[i].read();
            Serial_one.write(RX);
            if(EnableSerialDebug==1){
              Serial.write(RX);
            }
          }
        }
      }
      else {
        if (SerialServerClients[i]) {
          SerialServerClients[i].stop();
        }
      }
    }
    //check UART for data
    if(Serial_one.available()){
      size_t len = Serial_one.available();
      uint8_t sbuf[len];
      Serial_one.readBytes(sbuf, len);
      //push UART data to all connected telnet clients
      for(i = 0; i < MAX_SRV_CLIENTS; i++){
        if (SerialServerClients[i] && SerialServerClients[i].connected()){
          SerialServerClients[i].write(sbuf, len);
          // delay(1);
          if(EnableSerialDebug==1){
            Serial.println();
            Serial.print("tx-");
            Serial.write(sbuf, len);
          }
        }
      }
    }
  }
  else {
    if(EnableSerialDebug==1){
      Serial.println("Ser2net not connected!");
    }
    for(i = 0; i < MAX_SRV_CLIENTS; i++) {
      if (SerialServerClients[i]) SerialServerClients[i].stop();
    }
    delay(1000);
  }
  #endif
}

//-------------------------------------------------------------------------------------------------------
#if defined(XLswitch)
  unsigned long testFastLines(uint16_t color1, uint16_t color2) {
    unsigned long start;
    int           x, y, w = tft.width(), h = tft.height();

    tft.fillScreen(ILI9341_BLACK);
    start = micros();
    for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
    for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

    return micros() - start;
  }

  unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
    unsigned long start, t = 0;
    int           n, i, i2,
                  cx = tft.width()  / 2 - 1,
                  cy = tft.height() / 2 - 1;

    tft.fillScreen(ILI9341_BLACK);
    n = min(tft.width(), tft.height());
    for(i=n; i>0; i-=6) {
      i2    = i / 2;
      start = micros();
      tft.fillRect(cx-i2, cy-i2, i, i, color1);
      t    += micros() - start;
      // Outlines are not included in timing results
      tft.drawRect(cx-i2, cy-i2, i, i, color2);
      yield();
    }

    return t;
  }

  bool Get_yes_no(void){
  TS_Point p;
      tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(3);

      tft.fillRoundRect(20,250, 100, 50,8, ILI9341_RED);
      tft.setCursor(56, 265);
      tft.println("NO");
      tft.drawRoundRect(20,250, 100, 50, 8, ILI9341_WHITE);

      tft.fillRoundRect(120,250, 100, 50,8, ILI9341_GREEN);
      tft.setCursor(144, 265);
      tft.println("YES");
      tft.drawRoundRect(120,250, 100, 50, 8, ILI9341_WHITE);


  while (1){
        delay(50);
      p = ts.getPoint();

      if (p.z != 129){


        p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
        p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
        p.y = 320 - p.y;

        //  tft.fillCircle(p.x, p.y, 5, ILI9341_YELLOW);


      if ((p.y > 250) && (p.y<300)){

        if ((p.x> 20) && (p.x < 220)){
              if (p.x>120)
              {
                tft.fillRoundRect(120,250, 100, 50,8, ILI9341_OLIVE);
                tft.setCursor(144, 265);
                tft.println("YES");
                tft.drawRoundRect(120,250, 100, 50, 8, ILI9341_WHITE);

                delay(500);
                return true;
              }
                     else{

                       tft.fillRoundRect(20,250, 100, 50,8, ILI9341_OLIVE);
                       tft.setCursor(56, 265);
                       tft.println("NO");
                       tft.drawRoundRect(20,250, 100, 50, 8, ILI9341_WHITE);

                          delay(500);
                       return false;
                     }
         }

      }

    }
  }
  }
#endif
