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

HARDWARE ESP32-GATEWAY

Changelog:
2019-04 - group button support (idea TNX SM0MDG)
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
- cli configure first bank button groups

*/

//-------------------------------------------------------------------------------------------------------

#define ETHERNET                    // Enable ESP32 ethernet (DHCP IPv4)
// #define WIFI                          // Enable ESP32 WIFI (DHCP IPv4)
// const char* ssid     = "";
// const char* password = "";

//-------------------------------------------------------------------------------------------------------
const char* REV = "20190406";
// #define EnableOTA                // Enable flashing ESP32 Over The Air
bool HW_BCD_SW = 0;              // enable hardware ID board bcd switch (disable if not installed)
int NumberOfEncoderOutputs = 8;  // 2-16
long HW_BCD_SWTimer[2]{0,3000};
byte NET_ID = 0x00;              // Unique ID number [0-F] hex format - over BCD switch
bool EnableSerialDebug     = 0;
#define HTTP_SERVER_PORT  80     // Web server port
#define INCOMING_UDP_PORT 88     // command:
#define ShiftOut                 // Enable ShiftOut register
#define UdpAnswer                // Send UDP answer confirm packet
int BroadcastPort       = 88;    // destination broadcast packet port
bool EnableGroupPrefix = 0;      // enable multi controller control
bool EnableGroupButton = 0;      // group to one from
unsigned int GroupButton[8]={1,2,3,4,5,6,7,8};
byte DetectedRemoteSw[16][4];
unsigned int DetectedRemoteSwPort[16];

const int SERIAL_BAUDRATE = 115200; // serial debud baudrate
int incomingByte = 0;   // for incoming serial data
#if defined(ShiftOut)
  const int ShiftOutDataPin = 17;
  const int ShiftOutLatchPin = 16;
  const int ShiftOutClockPin = 5;
  byte ShiftOutByte[3];
#endif

const int BCD[4] = {34, 33, 32, 10};  // BCD encoder PINs

int i = 0;
#include <WiFi.h>
#include <WiFiUdp.h>
#include "EEPROM.h"
#define EEPROM_SIZE 14   /*
0    -listen source
1    -net ID
2    -encoder range
3    -HW_BCD_SW
4    -EnableGroupPrefix
5    -EnableGroupButton
6-13 -GroupButton */

WiFiServer server(HTTP_SERVER_PORT);
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

  for (int i = 0; i < 4; i++) {
   pinMode(BCD[i], INPUT);
  }

  // Listen source
  if (!EEPROM.begin(EEPROM_SIZE)){
    if(EnableSerialDebug==1){
      Serial.println("failed to initialise EEPROM"); delay(1);
    }
  }
  // 0-listen source
  TxUdpBuffer[2] = EEPROM.read(0);
  if(TxUdpBuffer[2]=='o'||TxUdpBuffer[2]=='r'||TxUdpBuffer[2]=='m'){
    // OK
  }else{
    TxUdpBuffer[2]='m';
  }

  // 1-net ID
  if(HW_BCD_SW==true){
    bitClear(NET_ID, 0);
    bitClear(NET_ID, 1);
    bitClear(NET_ID, 2);
    bitClear(NET_ID, 3);
    NET_ID = NET_ID | GetBoardId();
    TxUdpBuffer[0] = NET_ID;
  }else{
      NET_ID = EEPROM.read(1);
      TxUdpBuffer[0] = NET_ID;
  }

  // 2-encoder range
  NumberOfEncoderOutputs = EEPROM.read(2);
  if(NumberOfEncoderOutputs < 2 || NumberOfEncoderOutputs > 0x0f){
    NumberOfEncoderOutputs=8;
  }

  // 3-HW_BCD_SW
  Serial.print("HW BCD on/off ");
  if(EEPROM.read(3)<2){
    HW_BCD_SW = EEPROM.read(3);
    Serial.println("read from EEPROM");
  }else{
    Serial.println("set to OFF");
  }

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

  #if defined(ETHERNET)
    WiFi.onEvent(EthEvent);
    ETH.begin();
  #endif
    server.begin();
    UdpCommand.begin(INCOMING_UDP_PORT);    // incoming udp port

  #if defined(EnableOTA)
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);
    // Hostname defaults to esp3232-[MAC]

    String StringHostname = "IP-relayID-"+String(NET_ID, HEX);
    char copy[13];
    StringHostname.toCharArray(copy, 13);

    ArduinoOTA.setHostname(copy);
    // ArduinoOTA.setPassword("remoteqth");
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
      })
      .onEnd([]() {
        Serial.println("\nEnd");
      })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
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

}

//-------------------------------------------------------------------------------------------------------

void loop() {
  http();
  RX_UDP();
  SerialCLI();
  CheckNetId();

  #if defined(EnableOTA)
   ArduinoOTA.handle();
  #endif
}

// SUBROUTINES -------------------------------------------------------------------------------------------------------
void CheckNetId(){
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
}

//-------------------------------------------------------------------------------------------------------
void SerialCLI(){
  if (Serial.available() > 0) {
          incomingByte = Serial.read();

          // m
          if(incomingByte==109){
            TxUdpBuffer[2] = 'm';
            EEPROM.write(0, 'm'); // address, value
            EEPROM.commit();
            Serial.println("Now control from: IP switch master");
            if(EnableSerialDebug==1){
              Serial.print("EEPROM read [");
              Serial.print(EEPROM.read(0));
              Serial.println("]");
            }
            TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
            }
          // r
          }else if(incomingByte==114){
            EnableGroupPrefix=false;
              EEPROM.write(4, EnableGroupPrefix);
              EEPROM.commit();
            EnableGroupButton=false;
              EEPROM.write(5, EnableGroupButton);
              EEPROM.commit();
            TxUdpBuffer[2] = 'r';
            EEPROM.write(0, 'r'); // address, value
            EEPROM.commit();
            Serial.println("Now control from: Band decoder");
            if(EnableSerialDebug==1){
              Serial.print("EEPROM read [");
              Serial.print(EEPROM.read(0));
              Serial.println("]");
            }
            TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
            }
          // o
          }else if(incomingByte==111){
            EnableGroupPrefix=false;
              EEPROM.write(4, EnableGroupPrefix);
              EEPROM.commit();
            EnableGroupButton=false;
              EEPROM.write(5, EnableGroupButton);
              EEPROM.commit();
            TxUdpBuffer[2] = 'o';
            EEPROM.write(0, 'o'); // address, value
            EEPROM.commit();
            Serial.println("Now control from: Open Interface III");
            if(EnableSerialDebug==1){
              Serial.print("EEPROM read [");
              Serial.print(EEPROM.read(0));
              Serial.println("]");
            }
            TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
            }

          // *
          }else if(incomingByte==42){
            EnableSerialDebug=!EnableSerialDebug;
            Serial.print("** Serial DEBUG ");
            if(EnableSerialDebug==true){
              Serial.println("ENABLE **");
            }else{
              Serial.println("DISABLE **");
            }

        // &
        }else if(incomingByte==38){
          TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
          if(TxUdpBuffer[2] == 'm'){
            TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
          }

          // @
        }else if(incomingByte==64){
            Serial.print("** IP switch will be restarted **");
            ESP.restart();

        // +
        }else if(incomingByte==43){
            HW_BCD_SW=!HW_BCD_SW;
            Serial.print("** Net ID sufix by ");
            EEPROM.write(3, HW_BCD_SW);
            EEPROM.commit();
            if(HW_BCD_SW==true){
              Serial.println("EEPROM/[BCD switch] **");
              bitClear(NET_ID, 0);
              bitClear(NET_ID, 1);
              bitClear(NET_ID, 2);
              bitClear(NET_ID, 3);
              NET_ID = NET_ID | GetBoardId();
              TxUdpBuffer[0] = NET_ID;
            }else{
              NET_ID = EEPROM.read(1);
              TxUdpBuffer[0] = NET_ID;
              Serial.println("[EEPROM]/BCD switch **");
            }

    // %
    }else if(incomingByte==37){
        EnableGroupButton=!EnableGroupButton;
        Serial.print("** Group buttons (one from) [");
        EEPROM.write(5, EnableGroupButton);
        EEPROM.commit();
        if(EnableGroupButton==true){
          Serial.println("ON] **");
          for (int i = 0; i < 8; i++) {
            if(EEPROM.read(6+i)<9){
              GroupButton[i]=EEPROM.read(6+i);
            }
          }
        }else{
          Serial.println("OFF] **");
        }

    // :
    }else if(incomingByte==58 && EnableGroupButton==true){
      Serial.println(" List groups");
      for (int i = 0; i < 8; i++) {
        Serial.print("  Button ");
        Serial.print(i+1);
        Serial.print(" in group ");
        Serial.println(GroupButton[i]);
      }

    // !
    }else if(incomingByte==33 && EnableGroupButton==true){
          Serial.println("Press button number 1-8...");
          Serial.print("> ");
          while (Serial.available() == 0) {
            // Wait
          }
          incomingByte = Serial.read();

          if( (incomingByte>=49 && incomingByte<=56)){
            unsigned int ButtonNumber=incomingByte-48;
            Serial.print("Press Group number 1-8 for button ");
            Serial.print(ButtonNumber);
            Serial.println(" ...");
            Serial.print("> ");
            while (Serial.available() == 0) {
              // Wait
            }
            incomingByte = Serial.read();
            if( (incomingByte>=49 && incomingByte<=56)){
              unsigned int ButtonGroup=incomingByte-48;
              Serial.print(" store Button ");
              Serial.print(ButtonNumber);
              Serial.print(" to group ");
              Serial.println(ButtonGroup);
              GroupButton[ButtonNumber-1]=ButtonGroup;
              for (int i = 0; i < 8; i++) {
                EEPROM.write(6+i, GroupButton[i]);
              }
              EEPROM.commit();
            }else{
              Serial.println(" accepts 0-8, exit");
            }
          }else{
            Serial.println(" accepts 0-8, exit");
          }


      // $
      }else if(incomingByte==36){
            EnableGroupPrefix=!EnableGroupPrefix;
            Serial.print("** Group sufix (multi control) [");
            EEPROM.write(4, EnableGroupPrefix);
            EEPROM.commit();
            if(EnableGroupPrefix==true){
              Serial.println("ON] **");
            }else{
              Serial.println("OFF] **");
            }
            if(EnableGroupPrefix==true){
              // clear prefix
              bitClear(NET_ID, 4);
              bitClear(NET_ID, 5);
              bitClear(NET_ID, 6);
              bitClear(NET_ID, 7); // <-
            }
            Serial.println("** IP switch will be restarted **");
            delay(1000);
            ESP.restart();

      // #
      }else if(incomingByte==35){
          if(EnableGroupPrefix==false){
            Serial.println("Press NET-ID X_ prefix 0-f...");
          }else{
            Serial.println("Press NET-ID _X sufix 0-f...");
          }
          Serial.print("> ");
          while (Serial.available() == 0) {
            // Wait
          }
          incomingByte = Serial.read();

          if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
            // prefix
            if(EnableGroupPrefix==false){
              bitClear(NET_ID, 4);
              bitClear(NET_ID, 5);
              bitClear(NET_ID, 6);
              bitClear(NET_ID, 7);
              Serial.write(incomingByte);
              Serial.println();
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
                Serial.println("Press NET-ID _X sufix 0-f...");
                Serial.print("> ");
                while (Serial.available() == 0) {
                  // Wait
                }
                incomingByte = Serial.read();
              }
              if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
                bitClear(NET_ID, 0);
                bitClear(NET_ID, 1);
                bitClear(NET_ID, 2);
                bitClear(NET_ID, 3);
                Serial.write(incomingByte);
                Serial.println();
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
                Serial.print("** Now NET-ID change to 0x");
                if(NET_ID <=0x0f){
                  Serial.print(F("0"));
                }
                Serial.print(NET_ID, HEX);
                Serial.println(" **");
                if(EnableSerialDebug==1){
                  Serial.print("EEPROM read [");
                  Serial.print(EEPROM.read(1), HEX);
                  Serial.println("]");
                }
                TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
                if(TxUdpBuffer[2] == 'm'){
                  TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
                }
            // #if !defined(HW_BCD_SW)
              }else{
                Serial.println(" accepts 0-f, exit");
              }
            // #endif
            }
          }else{
            Serial.println(" accepts 0-f, exit");
          }

      // .
    }else if(incomingByte==46 && TxUdpBuffer[2]=='m' && EnableGroupPrefix){
            Serial.println("List detected IP switch by NET-ID prefix (multi control)");
            for (int i = 0; i < 16; i++) {
              Serial.print(i, HEX);
              Serial.print(F("  "));
              Serial.print(DetectedRemoteSw [i] [0]);
              Serial.print(F("."));
              Serial.print(DetectedRemoteSw [i] [1]);
              Serial.print(F("."));
              Serial.print(DetectedRemoteSw [i] [2]);
              Serial.print(F("."));
              Serial.print(DetectedRemoteSw [i] [3]);
              Serial.print(F(":"));
              Serial.println(DetectedRemoteSwPort [i]);
            }

      // /
    }else if(incomingByte==47 && TxUdpBuffer[2] == 'm'){
          Serial.println("press 2-g for encoder rannge number 2-16...");
          Serial.print(">");
          while (Serial.available() == 0) {
            // Wait
          }
          incomingByte = Serial.read();
          // 2-G
          if( (incomingByte>=50 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=103) ){
            if(incomingByte>=50 && incomingByte<=57){
              NumberOfEncoderOutputs = incomingByte-48;
            }else if(incomingByte>=97 && incomingByte<=103){
              NumberOfEncoderOutputs = incomingByte-87;
            }
            NumberOfEncoderOutputs--;
            EEPROM.write(2, NumberOfEncoderOutputs); // address, value
            EEPROM.commit();
            Serial.print("** Now Encoder range change to ");
            Serial.print(NumberOfEncoderOutputs+1);
            Serial.println(" **");
            if(EnableSerialDebug==1){
              Serial.print("EEPROM read [");
              Serial.print(EEPROM.read(2));
              Serial.println("]");
            }
            TxUDP('s', packetBuffer[2], 'c', 'f', 'm', 1);    // 0=broadcast, 1= direct to RX IP
            if(TxUdpBuffer[2] == 'm'){
              TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
            }
          }else{
            Serial.println("must be 0-f");
          }


          // ?
        }else if(incomingByte==63){
            ListCommands();
        }else{
          Serial.print(" [");
          Serial.write(incomingByte); //, DEC);
          Serial.println("] unknown command");
          ListCommands();
        }
  }
}

//-------------------------------------------------------------------------------------------------------
void ListCommands(){
  Serial.println();
  #if defined(ETHERNET)
    Serial.println("  IP relay ESP32-GATEWAY status");
    // Serial.println("  =================================");
    Serial.print("  http://");
    Serial.println(ETH.localIP());
    Serial.print("  ETH  MAC: ");
    Serial.println(ETH.macAddress());
    Serial.print("  ");
    Serial.print(ETH.linkSpeed());
    Serial.print("Mbps");
    if (ETH.fullDuplex()) {
      Serial.print(", FULL_DUPLEX ");
    }
    Serial.println();
  #else
    Serial.println("  ETHERNET OFF");
  #endif
  #if defined(WIFI)
    Serial.println("  =================================");
    Serial.print("  http://");
    Serial.println(WiFi.localIP());
    Serial.print("  dBm: ");
    Serial.println(WiFi.RSSI());
  #else
    Serial.println("  WIFI OFF");
  #endif
  Serial.println("  =================================");
  Serial.print("  Device NET-ID: 0x");
  if(NET_ID <=0x0f){
    Serial.print(F("0"));
  }
  Serial.print(NET_ID, HEX);
  if(HW_BCD_SW==true){
    Serial.print(" [BCD-");
    for (int i = 0; i < 4; i++) {
     Serial.print(digitalRead(BCD[i]));
    }
    Serial.print("]");
  }
  Serial.println();
  Serial.println("  =================================");
  if(EnableGroupPrefix==false){
    Serial.print("  Master IP: ");
    Serial.print(UdpCommand.remoteIP());
    Serial.print(":");
    Serial.println(UdpCommand.remotePort());
  }
  Serial.print("  Uptime: ");
  Serial.print(millis()/1000);
  Serial.println(" second");
  Serial.print("  Version: ");
  Serial.println(REV);
  Serial.print("  Bank status ABC [LSBFIRST]: ");
  Serial.print(ShiftOutByte[0], BIN);
  Serial.print(" ");
  Serial.print(ShiftOutByte[1], BIN);
  Serial.print(" ");
  Serial.println(ShiftOutByte[2], BIN);
  Serial.println("---------------------------------------------");
  Serial.println("  You can change source, with send character:");
  if(TxUdpBuffer[2]=='m'){
    Serial.print("     [m]");
  }else{
    Serial.print("      m ");
  }
  Serial.println("- IP switch master");
  if(TxUdpBuffer[2]=='r'){
    Serial.print("     [r]");
  }else{
    Serial.print("      r ");
  }
  Serial.println("- Band decoder");
  if(TxUdpBuffer[2]=='o'){
    Serial.print("     [o]");
  }else{
    Serial.print("      o ");
  }
  Serial.println("- Open Interface III");
  Serial.println();
  Serial.println("  or  ?  list status and commands");
  if(TxUdpBuffer[2] == 'm'){
    Serial.print("      /  set encoder range - now [");
    Serial.print(NumberOfEncoderOutputs+1);
    Serial.println("]");

    Serial.print("      %  group buttons (select one from group) [");
    if(EnableGroupButton==true){
      Serial.println("ON]");
      Serial.println("         !  SET group buttons");
      Serial.println("         :  list group buttons");
    }else{
      Serial.println("OFF]");
    }
  }
  Serial.print("      *  serial debug on/off [");
    if(EnableSerialDebug==true){
      Serial.println("ON]");
    }else{
      Serial.println("OFF]");
    }
  Serial.print("      +  net ID sufix by ");
    if(HW_BCD_SW==true){
      Serial.println("EEPROM/[BCD switch]");
    }else{
      Serial.println("[EEPROM]/BCD switch");
    }

  Serial.print("      #  network ID prefix [");
  byte ID = NET_ID;
  bitClear(ID, 0); // ->
  bitClear(ID, 1);
  bitClear(ID, 2);
  bitClear(ID, 3);
  ID = ID >> 4;
  if(EnableGroupPrefix==true && TxUdpBuffer[2]=='m'){
    Serial.print("x");
  }else{
    Serial.print(ID, HEX);
  }
  Serial.print("] hex");
  if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
    Serial.print(" (set only on controllers)");
  }
  Serial.println();
  if(HW_BCD_SW==false){
    ID = NET_ID;
    bitClear(ID, 4);
    bitClear(ID, 5);
    bitClear(ID, 6);
    bitClear(ID, 7); // <-
    Serial.print("         +network ID sufix [");
    Serial.print(ID, HEX);
    Serial.print("] hex");
    if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
      Serial.print(" (multi control group - same at all)");
    }
    Serial.println();
  }
  if(TxUdpBuffer[2]=='m'){
    Serial.print("      $  group network ID prefix (multi control) [");
    if(EnableGroupPrefix==true){
      Serial.println("ON]");
    }else{
      Serial.println("OFF]");
    }
  }
  if(TxUdpBuffer[2]=='m' && EnableGroupPrefix==true){
    Serial.println("      .  list detected IP switch (multi control)");
  }
  Serial.println("      &  send broadcast packet");
  Serial.println("      @  restart IP switch");
  Serial.println("---------------------------------------------");
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
        pinMode(BCD[1], OUTPUT);
        digitalWrite(BCD[1], HIGH);
        delay(100);
        digitalWrite(BCD[1], LOW);
        delay(100);
        pinMode(BCD[1], INPUT);
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
        if(UdpCommand.remotePort() != DetectedRemoteSwPort[hexToDecBy4bit(IdPrefix(packetBuffer[0]))]){
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
    } // filtered end
    else{
      if(EnableSerialDebug==1){
        Serial.println(F("   Different NET-ID, or bad packet format"));
      }
    }
    memset(packetBuffer, 0, sizeof(packetBuffer));   // Clear contents of Buffer
  } //end IfUdpPacketSice
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
          //   ESP.restart();
          // }

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
          client.println(F(" s<br>Source: "));

          if(TxUdpBuffer[2] == 'o'){
            client.println("Open Interface III");
          }else if(TxUdpBuffer[2] == 'r'){
            client.println("Band decoder MK2");
          }else if(TxUdpBuffer[2] == 'm'){
            client.println("Manual IP switch MK2");
          }

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
      // clear
      Serial.println("Snake&clear");
      Demo();

      // EnableSerialDebug=1;
      TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
      if(TxUdpBuffer[2] == 'm'){
        TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
      }
      // EnableSerialDebug=0;
      break;

    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH  Disconnected");
      eth_connected = false;
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("ETH  Stopped");
      eth_connected = false;
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
