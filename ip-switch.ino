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
2019-01 - add CLI
        - redesign UDP communications
        - NET-ID prefix/sufix support
2018-12 - web suport
        - add OTA
2018-09 - add IP switch support
        - blink LED after DHCP connect and receive sync packet
2018-08 add Band decoder support
*/

//-------------------------------------------------------------------------------------------------------

#define ETHERNET                    // Enable ESP32 ethernet (DHCP IPv4)
// #define WIFI                          // Enable ESP32 WIFI (DHCP IPv4)
// const char* ssid     = "";
// const char* password = "";

//-------------------------------------------------------------------------------------------------------
const char* REV = "20190120";
#define EnableOTA                // Enable flashing ESP32 Over The Air
#define HW_BCD_SW                // enable hardware ID board bcd switch (disable if not installed)
int NumberOfEncoderOutputs = 8;  // 2-16
long HW_BCD_SWTimer[2]{0,3000};
byte NET_ID = 0x00;              // Unique ID number [0-F] hex format - over BCD switch
bool EnableSerialDebug     = 0;
#define HTTP_SERVER_PORT  80     // Web server port
#define INCOMING_UDP_PORT 88     // command:
#define ShiftOut                 // Enable ShiftOut register
#define UdpAnswer                // Send UDP answer confirm packet
int BroadcastPort       = 88;    // destination broadcast packet port

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
#define EEPROM_SIZE 3   // 1-listen source, 2-net ID, 3-encoder range

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
String HTTP_req;
#if defined(EnableOTA)
  #include <ESPmDNS.h>
  #include <ArduinoOTA.h>
#endif
//-------------------------------------------------------------------------------------------------------

void setup() {
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
  TxUdpBuffer[2] = EEPROM.read(0);
  if(TxUdpBuffer[2]=='o'||TxUdpBuffer[2]=='r'||TxUdpBuffer[2]=='m'){
    // OK
  }else{
    TxUdpBuffer[2]='m';
  }

  #if defined(HW_BCD_SW)
    bitClear(NET_ID, 0);
    bitClear(NET_ID, 1);
    bitClear(NET_ID, 2);
    bitClear(NET_ID, 3);
    NET_ID = NET_ID | GetBoardId();
    TxUdpBuffer[0] = NET_ID;
  #else
      NET_ID = EEPROM.read(1);
      TxUdpBuffer[0] = NET_ID;
  #endif

  NumberOfEncoderOutputs = EEPROM.read(2);
  if(NumberOfEncoderOutputs < 2 || NumberOfEncoderOutputs > 0x0f){
    NumberOfEncoderOutputs=8;
  }

  // if(EnableSerialDebug==1){
    Serial.begin(SERIAL_BAUDRATE);
    while(!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println();
    Serial.print("Version: ");
    Serial.println(REV);
    Serial.println("===============================");
    Serial.print("SLAVE DEVICE NET-ID: 0x0");
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
    Serial.println("---------------------------------------------");
    Serial.println("  You can change source, with send character:");
    Serial.println("      m - IP switch master");
    Serial.println("      r - Band decoder");
    Serial.println("      o - Open Interface III");
    Serial.println("  or '?' for info");
    Serial.println("     '*' serial debug on/off");
    Serial.println("    #0-f network ID prefix [hex]");
    #if !defined(HW_BCD_SW)
      Serial.println("         +network ID sufix [hex]");
    #endif
    Serial.print("    n2-g set encoder range (now ");
    Serial.print(NumberOfEncoderOutputs+1);
    Serial.println(")");
    Serial.println("---------------------------------------------");
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
  #if defined(HW_BCD_SW)
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
  #endif
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

        // // 0-9 a-f
        // #if !defined(HW_BCD_SW)
        // }else if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
        //   if(incomingByte>=48 && incomingByte<=57){
        //     NET_ID = incomingByte-48;
        //     TxUdpBuffer[0] = NET_ID;
        //   }else if(incomingByte>=97 && incomingByte<=102){
        //     NET_ID = incomingByte-87;
        //     TxUdpBuffer[0] = NET_ID;
        //   }
        //   EEPROM.write(1, NET_ID); // address, value
        //   EEPROM.commit();
        //   Serial.print("** Now NET-ID change to 0x0");
        //   Serial.print(NET_ID, HEX);
        //   Serial.println(" **");
        //   if(EnableSerialDebug==1){
        //     Serial.print("EEPROM read [");
        //     Serial.print(EEPROM.read(1));
        //     Serial.println("]");
        //   }
        //   TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
        //   if(TxUdpBuffer[2] == 'm'){
        //     TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
        //   }
        // #endif

      // #
      }else if(incomingByte==35){
          Serial.println("Press NET-ID X_ prefix 0-f...");
          Serial.print("> ");
          while (Serial.available() == 0) {
            // Wait
          }
          incomingByte = Serial.read();
          if( (incomingByte>=48 && incomingByte<=57) || (incomingByte>=97 && incomingByte<=102) ){
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
            // sufix
            #if !defined(HW_BCD_SW)
              Serial.println("Press NET-ID _X sufix 0-f...");
              Serial.print("> ");
              while (Serial.available() == 0) {
                // Wait
              }
              incomingByte = Serial.read();
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
            #endif
                EEPROM.write(1, NET_ID); // address, value
                EEPROM.commit();
                Serial.print("** Now NET-ID change to 0x");
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
            #if !defined(HW_BCD_SW)
              }else{
                Serial.println(" accepts 0-f, exit");
              }
            #endif
          }else{
            Serial.println(" accepts 0-f, exit");
          }

      // n
      }else if(incomingByte==110 && TxUdpBuffer[2] == 'm'){
          Serial.println("Press 2-g for encoder rannge number 2-16...");
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

            Serial.println();
            #if defined(ETHERNET)
              Serial.println("  IP Switch with ESP32-GATEWAY");
              Serial.println("  =================================");
              Serial.print("     http://");
              Serial.println(ETH.localIP());
              Serial.println("  =================================");
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
              Serial.print("    http://");
              Serial.println(WiFi.localIP());
              Serial.println("  =================================");
              Serial.print("  dBm: ");
              Serial.println(WiFi.RSSI());
            #else
              Serial.println("  WIFI OFF");
            #endif
            Serial.print("  Device NET-ID: 0x");
            Serial.print(NET_ID, HEX);
            Serial.print(" [BCD-");
            for (int i = 0; i < 4; i++) {
             Serial.print(digitalRead(BCD[i]));
            }
            Serial.println("]");
            Serial.print("  Listen master: ");
            Serial.println(char(TxUdpBuffer[2]));
            Serial.print("  Master IP: ");
            Serial.print(UdpCommand.remoteIP());
            Serial.print(":");
            Serial.println(UdpCommand.remotePort());
            Serial.print("  Uptime: ");
            Serial.print(millis()/1000);
            Serial.println(" second");
            Serial.print("  Version: ");
            Serial.println(REV);


            Serial.print("  Serial debug: ");
            if(EnableSerialDebug==true){
              Serial.println("ENABLE");
            }else{
              Serial.println("DISABLE");
            }
            Serial.print("  Bank status ABC [LSBFIRST]: ");
            Serial.print(ShiftOutByte[0], BIN);
            Serial.print(" ");
            Serial.print(ShiftOutByte[1], BIN);
            Serial.print(" ");
            Serial.println(ShiftOutByte[2], BIN);
            Serial.println("---------------------------------------------");
            Serial.println("  You can change source, with send character:");
            Serial.println("      m - IP switch master");
            Serial.println("      r - Band decoder");
            Serial.println("      o - Open Interface III");
            Serial.println("  or '?' for info");
            Serial.println("     '*' serial debug on/off");
            Serial.println("    #0-f network ID prefix [hex]");
            #if !defined(HW_BCD_SW)
              Serial.println("         +network ID sufix [hex]");
            #endif
            Serial.print("    n2-g set encoder range (now ");
            Serial.print(NumberOfEncoderOutputs+1);
            Serial.println(")");
            Serial.println("---------------------------------------------");

          }else{
            Serial.print(" [");
            Serial.write(incomingByte); //, DEC);
            Serial.println("] unknown command");
          }
  }
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
      Serial.print(F("RXraw ["));
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
    if(String(packetBuffer[0], DEC).toInt()==NET_ID
      && packetBuffer[1]==TxUdpBuffer[2]  // FROM Switch
      && packetBuffer[2]== 's'  // TO
      && packetBuffer[3]== ':'
      && packetBuffer[7]== ';'
    ){

      // RX Broadcast / CFM
      if((packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o')
        || (packetBuffer[4]== 'c' && packetBuffer[5]== 'f' && packetBuffer[6]== 'm')
        ){

        if(EnableSerialDebug==1){
          Serial.print(F("RX ["));
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
        pinMode(BCD[1], INPUT);
        if(packetBuffer[4]== 'b' && packetBuffer[5]== 'r' && packetBuffer[6]== 'o'){
          TxUDP('s', packetBuffer[2], 'c', 'f', 'm', 1);    // 0=broadcast, 1= direct to RX IP
          if(TxUdpBuffer[2] == 'm'){
            TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
          }
        }

      // RX DATA
      }else{
        ShiftOutByte[2] = String(packetBuffer[6], DEC).toInt();    // Bank2
        ShiftOutByte[1] = String(packetBuffer[5], DEC).toInt();    // Bank1
        ShiftOutByte[0] = String(packetBuffer[4], DEC).toInt();    // Bank0

        // SHIFT OUT
        #if defined(ShiftOut)
          digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
          shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
          shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
          shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
          digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
        #endif

        if(EnableSerialDebug==1){
          Serial.println();
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

void TxUDP(byte FROM, byte TO, byte A, byte B, byte C, bool DIRECT){

  // TxUdpBuffer[0] = NET_ID;
  TxUdpBuffer[1] = FROM;
  // TxUdpBuffer[2] = TO;
  TxUdpBuffer[3] = B00111010;           // :
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
      UdpCommand.beginPacket(RemoteSwIP, UdpCommand.remotePort());

  // DATA
  }else{
    RemoteSwIP = UdpCommand.remoteIP();
    if(EnableSerialDebug==1){
      Serial.print(F("TX ["));
    }
    UdpCommand.beginPacket(RemoteSwIP, UdpCommand.remotePort());
  }

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

}

//---------------------------------------------
/*
void RX_UDP2(){
  UDPpacketSize = UdpCommand.parsePacket();    // if there's data available, read a packet
  if (UDPpacketSize){
    UdpCommand.read(packetBuffer, 10);      // read the packet into packetBufffer

    if(EnableSerialDebug==1){
      Serial.print("RX: ");
      for (i = 0; i < sizeof(packetBuffer); i++) {
        Serial.print(packetBuffer[i]);
        Serial.print("[");
        Serial.print(packetBuffer[i], HEX);
        Serial.print("/");
        Serial.print(packetBuffer[i], BIN);
        Serial.print("]");
        // Serial.print("|");
      }
      Serial.println();
    }

    //------------------------------------------------------------------------
    // QUERY SWITCH value
    if (packetBuffer[0] == 's' && packetBuffer[1] == ':' && packetBuffer[2] == 'q' && packetBuffer[3] == ';'){

        UdpCommand.beginPacket(UdpCommand.remoteIP(), UdpCommand.remotePort());   // Send to IP and port from recived UDP command
          UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
        UdpCommand.endPacket();

        if(EnableSerialDebug==1){
          Serial.print("TX answer query ");
          Serial.print(UdpCommand.remoteIP());
          Serial.print(":");
          Serial.print(UdpCommand.remotePort());
          Serial.print("   ");
          Serial.print("BankA: ");
          Serial.print(TxUdpBuffer[0], BIN);
          Serial.print(" BankB: ");
          Serial.print(TxUdpBuffer[1], BIN);
          Serial.print(" BankC: ");
          Serial.println(TxUdpBuffer[2], BIN);
        }
    }

    //------------------------------------------------------------------------
    // RX switch Bank0-2  [mro]X:###;
    if ( (packetBuffer[0] == TxUdpBuffer[0])  // Listen source
      && String(packetBuffer[1]).toInt()==NET_ID  // NET-ID
      && packetBuffer[2] == ':' ){

      if(packetBuffer[6] == ';'){
        ShiftOutByte[2] = packetBuffer[5];    // Bank2
        ShiftOutByte[1] = packetBuffer[4];    // Bank1
        ShiftOutByte[0] = packetBuffer[3];    // Bank0
      }else if(packetBuffer[5] == ';'){
        ShiftOutByte[2] = 0x00;
        ShiftOutByte[1] = packetBuffer[4];    // Bank1
        ShiftOutByte[0] = packetBuffer[3];    // Bank0
      }else if(packetBuffer[4] == ';'){
        ShiftOutByte[2] = 0x00;
        ShiftOutByte[1] = 0x00;
        ShiftOutByte[0] = packetBuffer[3];    // Bank0
      }

      // SHIFT OUT
      #if defined(ShiftOut)
        digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
        shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
        digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
      #endif

      // ANSWER CONFIRM PACKET
      #if defined(UdpAnswer)
        TxUdpBuffer[3]=ShiftOutByte[0];    // set buffer
        TxUdpBuffer[4]=ShiftOutByte[1];
        TxUdpBuffer[5]=ShiftOutByte[2];
        UdpCommand.beginPacket(UdpCommand.remoteIP(), UdpCommand.remotePort());   // Send to IP and port from recived UDP command
          UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
          // Serial.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
        UdpCommand.endPacket();

        if(EnableSerialDebug==1){
        // Serial.print("RX  packetBuffer[2] (Char) ");
        // Serial.println(packetBuffer[2]);
        // Serial.print("SET ShiftOutByte[0] (byte) ");
        // Serial.println(ShiftOutByte[0]);
        // Serial.print("TX  TxUdpBuffer[2]  (byte) ");
        // Serial.println(TxUdpBuffer[2]);
          Serial.print("TX ");
          Serial.print(UdpCommand.remoteIP());
          Serial.print(":");
          Serial.print(UdpCommand.remotePort());
          Serial.print(" ");
          // Serial.print("BankA: ");
          Serial.print(char(TxUdpBuffer[0]));
          Serial.print(char(TxUdpBuffer[1]));
          Serial.print(char(TxUdpBuffer[2]));
          Serial.print(TxUdpBuffer[3], BIN);
          Serial.print(" | ");
          Serial.print(TxUdpBuffer[4], BIN);
          Serial.print(" | ");
          Serial.print(TxUdpBuffer[5], BIN);
          Serial.print(char(TxUdpBuffer[6]));
          #if defined(WIFI)
            Serial.print(" | dBm: ");
            Serial.print(WiFi.RSSI());
          #endif
          Serial.println();
        }
      #endif
    }

    //------------------------------------------------------------------------
    // ANSWER TO querry Broadcast
    if ( packetBuffer[0] == 'b'
      && packetBuffer[1] == ':'
      && packetBuffer[2] == TxUdpBuffer[0]
      && ( (TxUdpBuffer[0]=='m' && String(packetBuffer[3]).toInt()==NET_ID)
        || (TxUdpBuffer[0]=='r' && String(packetBuffer[3]).toInt()==NET_ID)
        || (TxUdpBuffer[0]=='o') )
      && packetBuffer[4] == ';'
       ){

      if(EnableSerialDebug==1){
        Serial.print("RX b:");
        Serial.print(packetBuffer[2]);
        Serial.print(String(packetBuffer[3]).toInt() );
        // Serial.print("/");
        // Serial.print(NET_ID, HEX);
        Serial.print(";");
        Serial.print(UdpCommand.remoteIP());
        Serial.print(":");
        Serial.println(UdpCommand.remotePort());
      }
      digitalWrite(BCD[1], HIGH);
      delay(100);
      digitalWrite(BCD[1], LOW);
      TxUDP('s', packetBuffer[1], 'b', 'r', 'o', 1);    // 0=broadcast, 1= direct to RX IP
    }

    memset(packetBuffer,0,sizeof(packetBuffer));    // clear array
  }
}*/
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

          // client.println("<h1>RemoteSwitch </h1>");
          client.println("<p>Output status<br>");
          for (i = 0; i < 3; i++) {
            client.print("Bank ");
            client.print(i+1);
            client.print(" ");
            for (int j = 0; j < 8; j++) {
              client.print("<span class=\"box");
              if(bitRead(ShiftOutByte[i], j)==1){
                client.print("r");
              }
              client.print("\">");
              // client.print(bitRead(ShiftOutByte[i], j));
              client.print(j+1);
              client.println("</span>");
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
          client.println(F(" s"));

          client.println(F("<form method=\"get\">Source: "));
          client.print("<input type=\"submit\" name=\"S1\" value=\"Open Interface III\" class=\"");
          if(TxUdpBuffer[2] == 'o'){
            client.print("g");
          }else{
            client.print("r");
          }
          client.print("\"><input type=\"submit\" name=\"S2\" value=\"Band decoder MK2\" class=\"");
          if(TxUdpBuffer[2] == 'r'){
            client.print("g");
          }else{
            client.print("r");
          }
          client.print("\"><input type=\"submit\" name=\"S3\" value=\"Manual IP switch\" class=\"");
          if(TxUdpBuffer[2] == 'm'){
            client.print("g");
          }else{
            client.print("r");
          }

          // <input type="submit" name="S1" value="[ 1 ]" class="r">
          client.println(F("\"></form><p><a href=\".\" onclick=\"window.open( this.href, this.href, 'width=400,height=180,left=0,top=0,menubar=no,location=no,status=no' ); return false;\" > split&#8599;</a></p>"));
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
      digitalWrite(BCD[1], LOW);
      pinMode(BCD[1], INPUT);
      // clear
      Serial.println("Snake&clear");
      Demo();

      EnableSerialDebug=1;
      TxUDP('s', packetBuffer[2], 'b', 'r', 'o', 0);    // 0=broadcast, 1= direct to RX IP
      if(TxUdpBuffer[2] == 'm'){
        TxUDP('s', packetBuffer[2], ShiftOutByte[0], ShiftOutByte[1], ShiftOutByte[2], 0);
      }
      EnableSerialDebug=0;
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
/*
void SendBroadcastUdp(bool DIRECT){
  if(DIRECT==0){
    BroadcastIP = ~ETH.subnetMask() | ETH.gatewayIP();
    Serial.print("TX UDP broadcast packet [b:s");
    Serial.print(char(TxUdpBuffer[0]));
    Serial.print(NET_ID, HEX);
    Serial.print(";] to ");
    Serial.print(BroadcastIP);
    Serial.print(":");
    Serial.println(BroadcastPort);

    UdpCommand.beginPacket(BroadcastIP, BroadcastPort);   // Send to IP and port from recived UDP command
    // UdpCommand.beginMulticast(UdpCommand.BroadcastIP(), BroadcastPort, ETH.localIP()).
      UdpCommand.print("b:s");
      UdpCommand.print(char(TxUdpBuffer[0]));
      UdpCommand.print(NET_ID, HEX);
      UdpCommand.print(";");
    UdpCommand.endPacket();

  // DIRECT
  }else{
    Serial.print("TX UDP direct packet [b:s");
    Serial.print(char(TxUdpBuffer[0]));
    Serial.print(NET_ID, HEX);
    Serial.print(";] to ");
    Serial.print(UdpCommand.remoteIP());
    Serial.print(":");
    Serial.println(UdpCommand.remotePort());

    UdpCommand.beginPacket(UdpCommand.remoteIP(), UdpCommand.remotePort());   // Send to IP and port from recived UDP command
      UdpCommand.print("b:s");
      UdpCommand.print(char(TxUdpBuffer[0]));
      UdpCommand.print(NET_ID, HEX);
      UdpCommand.print(";");
    UdpCommand.endPacket();
  }
}*/
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
