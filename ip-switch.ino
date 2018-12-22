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

HARDWARE ESP32-GATEWAY

Changelog:
2018-12 - web suport
        - add OTA
2018-09 - add IP switch support
        - blink LED after DHCP connect and receive sync packet
2018-08 add Band decoder support
*/

// Receive UDP broadcast master device ID  --------------------------------------------------------------
const char* REV = "20181222";

char  Listens = 'm' ;   // default, if eeprom not set/initialized
            //  [o] Open Interface 3
            //  [d] Band decoder
            //  [m] IP switch master
            //  [s] IP switch slave * this device *
//-------------------------------------------------------------------------------------------------------

#define ETHERNET                    // Enable ESP32 ethernet (DHCP IPv4)
// #define WIFI                          // Enable ESP32 WIFI (DHCP IPv4)
#define EnableOTA                     // Enable ESP32 WIFI (DHCP IPv4)
const int SERIAL_BAUDRATE = 115200; // serial debud baudrate
int incomingByte = 0;   // for incoming serial data

const char* ssid     = "";
const char* password = "";
#define HTTP_SERVER_PORT  80    // Web server port
#define INCOMING_UDP_PORT 88    // command:
#define ShiftOut                // Enable ShiftOut register
#define UdpAnswer               // Send UDP answer confirm packet
#define SERIAL_DEBUG            // Enable serial debug [115200 baud]
int BroadcastPort       = 88;   // destination broadcast packet port

#define HW_BCD_SW                   // enable hardware ID board bcd switch (disable if not installed)
byte RELAY_BOARD_ID        = 0x00;  // Unique ID number [0-F] hex format - over BCD switch
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
#define EEPROM_SIZE 1

WiFiServer server(HTTP_SERVER_PORT);
// Client variables
char linebuf[80];
int charcount=0;
//Are we currently connected?
boolean connected = false;
//The udp library class
WiFiUDP UdpCommand;
uint8_t buffer[50] = "";
char packetBuffer[50];
int UDPpacketSize;
byte TxUdpBuffer[] = { Listens, B00111010, 0, 0, 0, B00111011}; // s, :, Bank0, Bank1, Bank2, ;
#include <ETH.h>
static bool eth_connected = false;
IPAddress BroadcastIP(0, 0, 0, 0);   // Broadcast IP address// #endif
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

  #if defined(HW_BCD_SW)
    GetBoardId();
    pinMode(BCD[1], OUTPUT);  // LED
    digitalWrite(BCD[1], LOW);
  #endif

  if (!EEPROM.begin(EEPROM_SIZE)){
    #if defined(SERIAL_DEBUG)
      Serial.println("failed to initialise EEPROM"); delay(1);
    #endif
  }
  Listens = EEPROM.read(0);
  if(Listens=='o'||Listens=='d'||Listens=='m'){
    // OK
  }else{
    Listens='m';
  }

  #if defined(SERIAL_DEBUG)
    Serial.begin(SERIAL_BAUDRATE);
    while(!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println();
    Serial.println("===============================");
    Serial.print("Version: ");
    Serial.println(REV);
    Serial.print("SLAVE DEVICE ID: ");
    Serial.println(RELAY_BOARD_ID, HEX);
    Serial.print("Listen MASTER: ");
    if(Listens == 'o'){
      Serial.println("Open Interface III");
    }
    if(Listens == 'd' ){
      Serial.println("Band decoder MK2");
    }
    if(Listens == 'm' ){
      Serial.println("IP switch master");
    }
    Serial.println("===============================");
    Serial.println("You can change with send character:");
    Serial.println("    m - IP switch master");
    Serial.println("    d - Band decoder");
    Serial.println("    o - Open Interface III");
    Serial.println("or 'h' for info");
    Serial.println();
    Serial.print("Incoming UDP port: ");
      Serial.println(INCOMING_UDP_PORT);
    Serial.print("Broadcast port: ");
      Serial.println(BroadcastPort);
  #endif

  #if defined(WIFI)
    #if defined(SERIAL_DEBUG)
      Serial.print("WIFI Connecting to ");
      Serial.print(ssid);
    #endif
    WiFi.begin(ssid, password);
    // attempt to connect to Wifi network:
    while(WiFi.status() != WL_CONNECTED) {
      // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
      delay(500);
      #if defined(SERIAL_DEBUG)
        Serial.print(".");
      #endif
    }
    // LED1status = !LED1status;
    // digitalWrite(LED1, LED1status);           // signalize wifi connected
    #if defined(SERIAL_DEBUG)
      Serial.println("");
      Serial.println("WIFI connected");
      Serial.print("WIFI IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("WIFI dBm: ");
      Serial.println(WiFi.RSSI());
    #endif
    digitalWrite(BCD[1], HIGH);
    delay(100);
    digitalWrite(BCD[1], LOW);
    delay(100);
    digitalWrite(BCD[1], HIGH);
    delay(100);
    digitalWrite(BCD[1], LOW);
  #endif

  #if defined(ETHERNET)
    WiFi.onEvent(EthEvent);
    ETH.begin();
  #endif
    server.begin();
    UdpCommand.begin(INCOMING_UDP_PORT);    // incoming udp port
  #if defined(WIFI)
    SendBroadcastUdp(0);    // 0=broadcast, 1= direct to RX IP
  #endif

  #if defined(EnableOTA)
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);
    // Hostname defaults to esp3232-[MAC]

    String StringHostname = "IP-relayID-"+String(RELAY_BOARD_ID, HEX);
    char copy[13];
    StringHostname.toCharArray(copy, 13);

    ArduinoOTA.setHostname(copy);
    ArduinoOTA.setPassword("remoteqth");
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

  #if defined(EnableOTA)
   ArduinoOTA.handle();
  #endif


  if (Serial.available() > 0) {
          incomingByte = Serial.read();

          if(incomingByte==109){
            Listens = 'm';
            EEPROM.write(0, 'm'); // address, value
            EEPROM.commit();
            Serial.println("Now control from: IP switch master");
            SendBroadcastUdp(0);    // 0=broadcast, 1= direct to RX IP
          }else if(incomingByte==100){
            Listens = 'd';
            EEPROM.write(0, 'd'); // address, value
            EEPROM.commit();
            Serial.println("Now control from: Band decoder");
            SendBroadcastUdp(0);    // 0=broadcast, 1= direct to RX IP
          }else if(incomingByte==111){
            Listens = 'o';
            EEPROM.write(0, 'o'); // address, value
            EEPROM.commit();
            Serial.println("Now control from: Open Interface III");
            SendBroadcastUdp(0);    // 0=broadcast, 1= direct to RX IP
          }else if(incomingByte==104){  // h

            Serial.println("-----------------------");
            Serial.print("Version: ");
            Serial.println(REV);
            Serial.print("SLAVE DEVICE ID: ");
            Serial.println(RELAY_BOARD_ID, HEX);
            Serial.print("Listen MASTER: ");
            Serial.println(Listens);
            #if defined(WIFI)
              Serial.print("WIFI IP address: ");
              Serial.print(WiFi.localIP());
              Serial.print(" | dBm: ");
              Serial.println(WiFi.RSSI());
            #else
              Serial.println("WIFI OFF");
            #endif

            #if defined(ETHERNET)
              Serial.print("ETH  MAC: ");
              Serial.print(ETH.macAddress());
              Serial.print(", IPv4: ");
              Serial.print(ETH.localIP());
              if (ETH.fullDuplex()) {
                Serial.print(", FULL_DUPLEX, ");
              }
              Serial.print(ETH.linkSpeed());
              Serial.println("Mbps");
            #else
              Serial.println("ETHERNET OFF");
            #endif
            Serial.println("-----------------------");

          }else{
            Serial.print(" [");
            Serial.write(incomingByte); //, DEC);
            Serial.println("] unknown command");
          }
  }

  // Demo();
}

// SUBROUTINES -------------------------------------------------------------------------------------------------------

void Demo(){
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
      delay(25);
    bitClear(ShiftOutByte[2], i);
  }
  for (i = 0; i < 8; i++) {
    bitSet(ShiftOutByte[1], i);
      digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
      shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
      shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
      shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
      digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
      delay(25);
    bitClear(ShiftOutByte[0], i);
  }
  for (i = 0; i < 8; i++) {
    bitSet(ShiftOutByte[2], i);
      digitalWrite(ShiftOutLatchPin, LOW);  // když dáme latchPin na LOW mužeme do registru poslat data
      shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[2]);
      shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[1]);
      shiftOut(ShiftOutDataPin, ShiftOutClockPin, LSBFIRST, ShiftOutByte[0]);
      digitalWrite(ShiftOutLatchPin, HIGH);    // jakmile dáme latchPin na HIGH data se objeví na výstupu
      delay(25);
    bitClear(ShiftOutByte[1], i);
  }
}


// http://www.catonmat.net/blog/low-level-bit-hacks-you-absolutely-must-know/

void GetBoardId(){
  RELAY_BOARD_ID = 0;
  if(digitalRead(BCD[0])==0){
    RELAY_BOARD_ID = RELAY_BOARD_ID | (1<<0);    // Set the n-th bit
  }
  if(digitalRead(BCD[1])==0){
    RELAY_BOARD_ID = RELAY_BOARD_ID | (1<<1);    // Set the n-th bit
  }
  if(digitalRead(BCD[2])==0){
    RELAY_BOARD_ID = RELAY_BOARD_ID | (1<<2);    // Set the n-th bit
  }
  if(digitalRead(BCD[3])==0){
    RELAY_BOARD_ID = RELAY_BOARD_ID | (1<<3);    // Set the n-th bit
  }
}
//-------------------------------------------------------------------------------------------------------

void RX_UDP(){
  UDPpacketSize = UdpCommand.parsePacket();    // if there's data available, read a packet
  if (UDPpacketSize){
    UdpCommand.read(packetBuffer, 50);      // read the packet into packetBufffer

    // QUERY SWITCH value
    if (packetBuffer[0] == 's' && packetBuffer[1] == ':' && packetBuffer[2] == 'q' && packetBuffer[3] == ';'){

        UdpCommand.beginPacket(UdpCommand.remoteIP(), UdpCommand.remotePort());   // Send to IP and port from recived UDP command
          UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
        UdpCommand.endPacket();

        #if defined(SERIAL_DEBUG)
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
        #endif
    }

    // RX Bank0-2  r:###;
    // if ( (packetBuffer[0] == 's' || packetBuffer[0] == 'm')
    if ( (packetBuffer[0] == Listens)
      && packetBuffer[1] == ':' && packetBuffer[2] != 'q'){
      if(packetBuffer[5] == ';'){
        ShiftOutByte[2] = packetBuffer[4];    // Bank2
        ShiftOutByte[1] = packetBuffer[3];    // Bank1
        ShiftOutByte[0] = packetBuffer[2];    // Bank0
      }else if(packetBuffer[4] == ';'){
        ShiftOutByte[1] = packetBuffer[3];    // Bank1
        ShiftOutByte[0] = packetBuffer[2];    // Bank0
      }else if(packetBuffer[3] == ';'){
        ShiftOutByte[0] = packetBuffer[2];    // Bank0
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
        TxUdpBuffer[2]=ShiftOutByte[0];    // set buffer
        TxUdpBuffer[3]=ShiftOutByte[1];
        TxUdpBuffer[4]=ShiftOutByte[2];
        UdpCommand.beginPacket(UdpCommand.remoteIP(), UdpCommand.remotePort());   // Send to IP and port from recived UDP command
          UdpCommand.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
          // Serial.write(TxUdpBuffer, sizeof(TxUdpBuffer));   // send buffer
        UdpCommand.endPacket();

        #if defined(SERIAL_DEBUG)
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
          Serial.print(TxUdpBuffer[2], BIN);
          Serial.print(" | ");
          Serial.print(TxUdpBuffer[3], BIN);
          Serial.print(" | ");
          Serial.print(TxUdpBuffer[4], BIN);
          #if defined(WIFI)
            Serial.print(" | dBm: ");
            Serial.print(WiFi.RSSI());
          #endif
          Serial.println();
        #endif
      #endif
    }

    // ANSWER TO querry Broadcast
    if ( packetBuffer[0] == 'b'
      && packetBuffer[1] == ':'
      && packetBuffer[2] == Listens
      && ( (Listens=='m' && String(packetBuffer[3]).toInt()==RELAY_BOARD_ID)
        || (Listens=='d' && String(packetBuffer[3]).toInt()==RELAY_BOARD_ID)
        || (Listens=='o') )
      && packetBuffer[4] == ';'
       ){

      #if defined(SERIAL_DEBUG)
        Serial.print("RX b:");
        Serial.print(packetBuffer[2]);
        Serial.print(String(packetBuffer[3]).toInt() );
        // Serial.print("/");
        // Serial.print(RELAY_BOARD_ID, HEX);
        Serial.print(";");
        Serial.print(UdpCommand.remoteIP());
        Serial.print(":");
        Serial.println(UdpCommand.remotePort());
      #endif
      digitalWrite(BCD[1], HIGH);
      delay(100);
      digitalWrite(BCD[1], LOW);
      SendBroadcastUdp(1);    // 0=broadcast, 1= direct to RX IP
    }

    memset(packetBuffer,0,sizeof(packetBuffer));    // clear array
  }
}
//-------------------------------------------------------------------------------------------------------

void http(){
  // listen for incoming clients
  WiFiClient client = server.available();
  if (client) {
    #if defined(SERIAL_DEBUG)
      Serial.println("WIFI New client");
    #endif
    memset(linebuf,0,sizeof(linebuf));
    charcount=0;
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        HTTP_req += c;
        // #if defined(SERIAL_DEBUG)
        //   Serial.write(c);
        // #endif
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
          client.println(F("IP switch ID#"));
          client.println(RELAY_BOARD_ID);
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
            Listens='o';
            EEPROM.write(0, 'o'); // address, value
            EEPROM.commit();
            SendBroadcastUdp(0);    // 0=broadcast, 1= direct to RX IP
          }
          if(GETOUTPUT.toInt()==2){
            Listens='d';
            EEPROM.write(0, 'd'); // address, value
            EEPROM.commit();
            SendBroadcastUdp(0);    // 0=broadcast, 1= direct to RX IP
          }
          if(GETOUTPUT.toInt()==3){
            Listens='m';
            EEPROM.write(0, 'm'); // address, value
            EEPROM.commit();
            SendBroadcastUdp(0);    // 0=broadcast, 1= direct to RX IP
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
          client.println(F(REV));

          client.println(F("<form method=\"get\">Source: "));
          client.print("<input type=\"submit\" name=\"S1\" value=\"Open Interface III\" class=\"");
          if(Listens == 'o'){
            client.print("g");
          }else{
            client.print("r");
          }
          client.print("\"><input type=\"submit\" name=\"S2\" value=\"Band decoder MK2\" class=\"");
          if(Listens == 'd'){
            client.print("g");
          }else{
            client.print("r");
          }
          client.print("\"><input type=\"submit\" name=\"S3\" value=\"Manual IP switch\" class=\"");
          if(Listens == 'm'){
            client.print("g");
          }else{
            client.print("r");
          }

          // <input type="submit" name="S1" value="[ 1 ]" class="r">
          client.println(F("\"></form><p><a href=\".\" onclick=\"window.open( this.href, this.href, 'width=400,height=180,left=0,top=0,menubar=no,location=no,status=no' ); return false;\" > split&#8599;</a></p>"));
          client.println(F("</body></html>"));

          #if defined(SERIAL_DEBUG)
            Serial.print(HTTP_req);
          #endif
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
   #if defined(SERIAL_DEBUG)
     Serial.println("WIFI client disconnected");
   #endif
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

      SendBroadcastUdp(0);    // 0=broadcast, 1= direct to RX IP
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

void SendBroadcastUdp(bool DIRECT){
  if(DIRECT==0){
    BroadcastIP = ~ETH.subnetMask() | ETH.gatewayIP();
    Serial.print("TX UDP broadcast packet [b:s");
    Serial.print(Listens);
    Serial.print(RELAY_BOARD_ID, HEX);
    Serial.print(";] to ");
    Serial.print(BroadcastIP);
    Serial.print(":");
    Serial.println(BroadcastPort);

    UdpCommand.beginPacket(BroadcastIP, BroadcastPort);   // Send to IP and port from recived UDP command
    // UdpCommand.beginMulticast(UdpCommand.BroadcastIP(), BroadcastPort, ETH.localIP()).
      UdpCommand.print("b:s");
      UdpCommand.print(Listens);
      UdpCommand.print(RELAY_BOARD_ID, HEX);
      UdpCommand.print(";");
    UdpCommand.endPacket();

  // DIRECT
  }else{
    Serial.print("TX UDP direct packet [b:s");
    Serial.print(Listens);
    Serial.print(RELAY_BOARD_ID, HEX);
    Serial.print(";] to ");
    Serial.print(UdpCommand.remoteIP());
    Serial.print(":");
    Serial.println(UdpCommand.remotePort());

    UdpCommand.beginPacket(UdpCommand.remoteIP(), UdpCommand.remotePort());   // Send to IP and port from recived UDP command
      UdpCommand.print("b:s");
      UdpCommand.print(Listens);
      UdpCommand.print(RELAY_BOARD_ID, HEX);
      UdpCommand.print(";");
    UdpCommand.endPacket();
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
