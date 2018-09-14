/*

IP Switch
----------------------
https://remoteqth.com/wiki/index.php?page=IP+Switch+with+ESP32-GATEWAY
2018-09 by OK1HRA
rev 0.2

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

Features:
HARDWARE ESP32-GATEWAY

Changelog:
2018-09 - add IP switch support
        - blink LED after DHCP connect and receive sync packet
2018-08 add Band decoder support
*/

// Receive UDP broadcast master device ID  --------------------------------------------------------------
char  Listens = 'm' ;
            //  [o] Open Interface 3
            //  [d] Band decoder
            //  [m] IP switch master
            //  [s] IP switch slave * this device *
//-------------------------------------------------------------------------------------------------------

// #define ETHERNET                    // Enable ESP32 ethernet (DHCP IPv4)
#define WIFI                     // Enable ESP32 WIFI (DHCP IPv4)
const int SERIAL_BAUDRATE = 115200; // serial debud baudrate
int incomingByte = 0;   // for incoming serial data

// const char* ssid     = "";
// const char* password = "";
#define HTTP_SERVER_PORT  80    // Web server port
#define INCOMING_UDP_PORT 88    // command:
#define ShiftOut                // Enable ShiftOut register
#define UdpAnswer               // Send UDP answer confirm packet
#define SERIAL_DEBUG            // Enable serial debug [115200 baud]
int BroadcastPort       = 88;   // destination broadcast packet port

#define HW_BCD_SW                   // enable hardware ID board bcd switch (disable if not installed)
byte RELAY_BOARD_ID        = 0x00;  // Unique ID number [0-F] hex format,
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

  #if defined(SERIAL_DEBUG)
    Serial.begin(SERIAL_BAUDRATE);
    while(!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println();
    Serial.println("===============================");
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
    Serial.print("Incoming UDP port: ");
      Serial.println(INCOMING_UDP_PORT);
    Serial.print("Broadcast port: ");
      Serial.println(BroadcastPort);
    Serial.println();
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
    SendBroadcastUdp();
  #endif
}
//-------------------------------------------------------------------------------------------------------

void loop() {
  http();
  RX_UDP();

  if (Serial.available() > 0) {
          incomingByte = Serial.read();

          if(incomingByte==109){
            Listens = 'm';
            Serial.println("Now control from: IP switch master");
          }else if(incomingByte==100){
            Listens = 'd';
            Serial.println("Now control from: Band decoder");
          }else if(incomingByte==111){
            Listens = 'o';
            Serial.println("Now control from: Open Interface III");
          }else{
            Serial.print(" [");
            Serial.write(incomingByte); //, DEC);
            Serial.println("] unknown command");
          }
  }
}

// SUBROUTINES -------------------------------------------------------------------------------------------------------
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
    if ( (packetBuffer[0] == 's' || packetBuffer[0] == 'm')
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
          Serial.print("TX ");
          Serial.print(UdpCommand.remoteIP());
          Serial.print(":");
          Serial.print(UdpCommand.remotePort());
          Serial.print(" ");
          // Serial.print("BankA: ");
          Serial.print(ShiftOutByte[0], BIN);
          Serial.print(" | ");
          Serial.print(ShiftOutByte[1], BIN);
          Serial.print(" | ");
          Serial.println(ShiftOutByte[2], BIN);
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
      SendBroadcastUdp();
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
        #if defined(SERIAL_DEBUG)
          Serial.write(c);
        #endif
        //read char by char HTTP request
        linebuf[charcount]=c;
        if (charcount<sizeof(linebuf)-1) charcount++;
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          client.println();
          client.println("<!DOCTYPE HTML><html><head>");
          client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"></head>");
          client.println("<h1>RemoteSwitch </h1>");
          for (i = 0; i < 6; i++) {
            client.println("<p>GPIO ");
            // client.println(GPIOS[i]);
            client.println(" <a href=\"h");
            client.println(i);
            client.println("\"><button>ON</button></a>&nbsp;<a href=\"l");
            client.println(i);
            client.println("\"><button>OFF</button></a></p>");
          }
          // client.println("<p>LED1 (gpio33) <a href=\"on1\"><button>ON</button></a>&nbsp;<a href=\"off1\"><button>OFF</button></a></p>");
          // client.println("<p>GPIO 5 <a href=\"on2\"><button>ON</button></a>&nbsp;<a href=\"off2\"><button>OFF</button></a></p>");
          client.print("Wifi: <b>");
          client.print(ssid);
          client.print("</b> ");
          client.print(WiFi.RSSI());
          client.print(" dBm, ");
          client.print(WiFi.localIP());
          client.print("</p><p>ETH: <b>");
          client.print(ETH.macAddress());
          client.print("</b>, ");
          client.print(ETH.localIP());
          client.println("</p></html>");
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
      Serial.print(ETH.macAddress());
      Serial.print(", IPv4: ");
      Serial.print(ETH.localIP());
      if (ETH.fullDuplex()) {
        Serial.print(", FULL_DUPLEX");
      }
      Serial.print(", ");
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

      SendBroadcastUdp();
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

void SendBroadcastUdp(){
  BroadcastIP = ~ETH.subnetMask() | ETH.gatewayIP();
  Serial.print("TX UDP broadcast packet [b:s");
  Serial.print(RELAY_BOARD_ID, HEX);
  Serial.print(";] to ");
  Serial.print(BroadcastIP);
  Serial.print(":");
  Serial.println(BroadcastPort);

  UdpCommand.beginPacket(BroadcastIP, BroadcastPort);   // Send to IP and port from recived UDP command
  // UdpCommand.beginMulticast(UdpCommand.BroadcastIP(), BroadcastPort, ETH.localIP()).
    UdpCommand.print("b:s");
    UdpCommand.print(RELAY_BOARD_ID, HEX);
    UdpCommand.print(";");
  UdpCommand.endPacket();
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
