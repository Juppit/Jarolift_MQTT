/*  Controlling Jarolift TDEF 433MHZ radio shutters via ESP8266 and CC1101 Transceiver Module in asynchronous mode.
    Copyright (C) 2017-2018 Steffen Hille et al.

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
*/


// Changelog: see CHANGES.md

/*
  Kanal  S/N           DiscGroup_8-16             DiscGroup_1-8     SN(last two digits)
  0       0            0000 0000                   0000 0001           0000 0000
  1       1            0000 0000                   0000 0010           0000 0001
  2       2            0000 0000                   0000 0100           0000 0010
  3       3            0000 0000                   0000 1000           0000 0011
  4       4            0000 0000                   0001 0000           0000 0100
  5       5            0000 0000                   0010 0000           0000 0101
  6       6            0000 0000                   0100 0000           0000 0110
  7       7            0000 0000                   1000 0000           0000 0111
  8       8            0000 0001                   0000 0000           0000 0111
  9       9            0000 0010                   0000 0000           0000 0111
  10      10           0000 0100                   0000 0000           0000 0111
  11      11           0000 1000                   0000 0000           0000 0111
  12      12           0001 0000                   0000 0000           0000 0111
  13      13           0010 0000                   0000 0000           0000 0111
  14      14           0100 0000                   0000 0000           0000 0111
  15      15           1000 0000                   0000 0000           0000 0111
*/


#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <SPI.h>
#include <FS.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <DoubleResetDetector.h>
#include <simpleDSTadjust.h>
#include <coredecls.h>              // settimeofday_cb()

#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

#include "helpers.h"
#include "global.h"
#include "html_api.h"

extern "C" {
#include "user_interface.h"
#include "Arduino.h"
#include "cc1101.h"
#include <KeeloqLib.h>
}

// host name used for OTA and mDNS
#define host_name "esp8266"

// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10

// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0

// User configuration
#define Lowpulse         400    // Defines pulse-width in microseconds. Adapt for your use...
#define Highpulse        800

uint32_t device_key_msb  = 0x0; // stores cryptkey MSB
uint32_t device_key_lsb  = 0x0; // stores cryptkey LSB

byte disc_low[16]        = {0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0,  0x0,  0x0,  0x0};
byte disc_high[16]       = {0x0, 0x0, 0x0, 0x0, 0x0,  0x0,  0x0,  0x0, 0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80};
byte serials[16]         = {0x0, 0x1, 0x2, 0x3, 0x4,  0x5,  0x6,  0x7, 0x8, 0x9, 0xA, 0xB, 0xC,  0xD,  0xE,  0xF }; // Represents last serial digit in binary 1234567[8] = 0xF
byte adresses[]          = {5, 11, 17, 23, 29, 35, 41, 47, 53, 59, 65, 71, 77, 85, 91, 97 }; // Defines start addresses of channel data stored in EEPROM 4bytes s/n.

// Commands
#define CMD_UP         0x8
#define CMD_SET_SHADE  0x6
#define CMD_STOP_SHADE 0x5
#define CMD_STOP       0x4
#define CMD_SHADE      0x3
#define CMD_DOWN       0x2
#define CMD_LEARN      0x1

char commands[16][12] {"0","learn","down","shade","stop","stop_shade","set_shade","7","up","9","up+down","11","12","13","14","15"};

// RX variables and defines
#define debounce         200              // Ignoring short pulses in reception... no clue if required and if it makes sense ;)
#define bufsize          216              // Pulsebuffer
#define TX_PORT            4              // Outputport for transmission
#define RX_PORT            5              // Inputport for reception

uint32_t rx_device_key_msb = 0x0;         // stores cryptkey MSB
uint32_t rx_device_key_lsb = 0x0;         // stores cryptkey LSB

volatile byte pbwrite;
volatile uint32_t lowbuf[bufsize];        // ring buffer storing LOW pulse lengths
volatile uint32_t highbuf[bufsize];       // ring buffer storing HIGH pulse lengths
volatile bool rx_full = false;            // flag for buffer is full

boolean time_is_set_first = true;

DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);
CC1101 cc1101;                            // The connection to the hardware chip CC1101 the RF Chip

// forward declarations
void ICACHE_RAM_ATTR radio_rx_measure();

//####################################################################
// sketch initialization routine
//####################################################################
void setup()
{
  byte syncWord = 199;
  InitLog();
  EEPROM.begin(4096);
  Serial.begin(115200);
  settimeofday_cb(time_is_set);
  updateNTP(); // Init the NTP time
  WriteLog("[INFO] - starting Jarolift Dongle " + (String)PROGRAM_VERSION, true);
  char chipId[12];
  sprintf(chipId, "0x%06x", ESP.getChipId());
  WriteLog("[INFO] - ESP-ID " + (String)chipId + " // ESP-Core " + ESP.getCoreVersion() + " // SDK Version " + ESP.getSdkVersion(), true);
  //WriteLog("[INFO] - ESP-ID " + (String)ESP.getChipId() + " // ESP-Core " + ESP.getCoreVersion() + " // SDK Version " + ESP.getSdkVersion(), true);

  // callback functions for WiFi connect and disconnect
  // placed as early as possible in the setup() function to get the connect
  // message catched when the WiFi connect is really fast
  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP & event)
  {
    WriteLog("[INFO] - WiFi station connected - IP: " + WiFi.localIP().toString(), true);
    wifi_disconnect_log = true;
  });

  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected & event)
  {
    if (wifi_disconnect_log) {
      WriteLog("[INFO] - WiFi station disconnected", true);
      // turn off logging disconnect events after first occurrence, otherwise the log is filled up
      wifi_disconnect_log = false;
    }
  });

  InitializeConfigData();
  EEPROM.get(cntadr, devcnt);

  // initialize the transceiver chip
  WriteLog("[INFO] - initializing the CC1101 transceiver", true);
  cc1101.init();
  cc1101.setSyncWord(syncWord, false);
  cc1101.setCarrierFreq(CFREQ_433);
  cc1101.disableAddressCheck();   // if not specified, will only display "packet received"

  pinMode(led_pin, OUTPUT);   // prepare LED on ESP-Chip

  // test if the WLAN SSID is on default
  // or DoubleReset detected
  if ((drd.detectDoubleReset()) || (config.ssid == "MYSSID")) {
    digitalWrite(led_pin, LOW);  // if yes then turn on LED
    AdminEnabled = true;         // and go to Admin-Mode
  } else {
    digitalWrite(led_pin, HIGH); // turn LED off
  }

  // enable access point mode if Admin-Mode is enabled
  if (AdminEnabled)
  {
    WriteLog("[WARN] - Admin-Mode enabled!", true);
    WriteLog("[WARN] - starting soft-AP ... ", false);
    wifi_disconnect_log = false;
    WiFi.mode(WIFI_AP);
    WriteLog(WiFi.softAP(ACCESS_POINT_NAME, ACCESS_POINT_PASSWORD) ? "Ready" : "Failed!", true);
    WriteLog("[WARN] - Access Point <" + (String)ACCESS_POINT_NAME + "> activated. WPA password is " + ACCESS_POINT_PASSWORD, true);
    WriteLog("[WARN] - you have " + (String)AdminTimeOut + " seconds time to connect and configure!", true);
    WriteLog("[WARN] - configuration webserver is http://" + WiFi.softAPIP().toString(), true);
  }
  else
  {
    // establish Wifi connection in station mode
    ConfigureWifi();
  }

  // configure webserver and start it
  server.on ( "/api", html_api );                       // command api
  SPIFFS.begin();                                       // Start the SPI flash filesystem
  server.onNotFound([]() {                              // If the client requests any URI
    if (!handleFileRead(server.uri())) {                // send it if it exists
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
      Serial.println(" File " + server.uri() + " not found: did you upload the data directory?");
    }
  });

  server.begin();
  WriteLog("[INFO] - HTTP server started", true);
  tkHeartBeat.attach(1, HeartBeat);
  // configure MQTT client
  if (config.mqtt_broker_addr[0] + config.mqtt_broker_addr[1] + config.mqtt_broker_addr[2] + config.mqtt_broker_addr[3]) {
    mqtt_client.setServer(IPAddress(config.mqtt_broker_addr[0], config.mqtt_broker_addr[1],
                                    config.mqtt_broker_addr[2], config.mqtt_broker_addr[3]),
                                    config.mqtt_broker_port.toInt());
    mqtt_client.setCallback(mqtt_callback);   // define Handler for incoming messages
    mqttLastConnectAttempt = 0;
  }

  // TX Pin
  pinMode(TX_PORT, OUTPUT);
  // RX Pin
  pinMode(RX_PORT, INPUT_PULLUP);

  if (MDNS.begin(host_name)) {
    WriteLog("[INFO] - mDNS server started for \"" + String(host_name) + ".local\" (MacOS and linux)", true);
    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", 80);
  }

  ArduinoOTA.setHostname(host_name);
  ArduinoOTA.onStart(otastart);
  ArduinoOTA.begin();
  WriteLog("[INFO] - OTA  server started for \"" + String(host_name) + "\"", true);

  attachInterrupt(RX_PORT, radio_rx_measure, CHANGE); // Interrupt on change of RX_PORT
} // void setup

//####################################################################
// Callback routine for OTA start
//####################################################################
void otastart()
{
  Serial.println("OTA started...");
} // void otastart

//####################################################################
// main loop
//####################################################################
void loop()
{
  // Call the double reset detector loop method every so often,
  // so that it can recognise when the timeout expires.
  // You can also call drd.stop() when you wish to no longer
  // consider the next reset as a double reset.
  drd.loop();

  // disable Admin-Mode after AdminTimeOut
  if (AdminEnabled)
  {
    if (AdminTimeOutCounter > AdminTimeOut / HEART_BEAT_CYCLE)
    {
      AdminEnabled = false;
      digitalWrite(led_pin, HIGH);   // turn LED off
      WriteLog("[WARN] - Admin-Mode disabled, soft-AP terminate ...", false);
      WriteLog(WiFi.softAPdisconnect(true) ? "success" : "fail!", true);
      ConfigureWifi();
    }
  }
  server.handleClient();

  // clear the flag if a message arrived at last loop
  if (rx_full) {
    cc1101.cmdStrobe(CC1101_SCAL);
    delay(50);
    enterrx();
    rx_full = false;
    delay(200);
    attachInterrupt(RX_PORT, radio_rx_measure, CHANGE); // Interrupt on change of RX_PORT
  }

  // check if first pulse is a header and RX buffer is full
  if ((lowbuf[0] > 3650) && (lowbuf[0] < 4300) && (pbwrite >= 65) && (pbwrite <= 75)) {    // Decode received data...
    byte value = ReadRSSI();
    rx_full = true;
    pbwrite = 0;

    // encrypted code data
    uint32_t rx_hopcode=0;
    // fixed code data
    uint32_t rx_serial=0;       // serial number
    uint8_t  rx_function=0;     // button status
    uint8_t  rx_disc_h=0;       //
    // decoded data
    uint32_t decoded;           // decoded hop code
    uint16_t rx_counter=0;      // sync value
    uint8_t rx_disc_low[4];

    for (int i = 0; i <= 31; i++) {                        // extracting Hopcode
      if (lowbuf[i + 1] < highbuf[i + 1]) {
        rx_hopcode = rx_hopcode & ~(1 << i) | (0 << i);
      } else {
        rx_hopcode = rx_hopcode & ~(1 << i) | (1 << i);
      }
    }
    for (int i = 0; i <= 27; i++) {                        // extracting Serialnumber
      if (lowbuf[i + 33] < highbuf[i + 33]) {
        rx_serial = rx_serial & ~(1 << i) | (0 << i);
      } else {
        rx_serial = rx_serial & ~(1 << i) | (1 << i);
      }
    }
    for (int i = 0; i <= 3; i++) {                         // extracting function code
      if (lowbuf[61 + i] < highbuf[61 + i]) {
        rx_function = rx_function & ~(1 << i) | (0 << i);
      } else {
        rx_function = rx_function & ~(1 << i) | (1 << i);
      }
    }
    for (int i = 0; i <= 1; i++) {                         // extracting high disc
      if (lowbuf[65 + i] < highbuf[65 + i]) {
        rx_disc_h = rx_disc_h & ~(1 << i) | (0 << i);
      } else {
        rx_disc_h = rx_disc_h & ~(1 << i) | (1 << i);
      }
    }

    rx_keygen(rx_serial);                    // now create the deccrypt key from serial
    decoded = rx_decoder(rx_hopcode);        // and decode the received hopcode
    rx_counter = decoded & 0xFFFF;           // mask out the counter value

    rx_function &= 0xF;                      // note command array size!
    uint32_t serial = config.serial_number;  // this is the first serial number
    if (serial == (rx_serial & 0xfff0)) {    // check if this is our own number
      int channel = rx_serial & 0xf;
        WriteLog("[INFO] - received command for channel " + (String)channel + config.channel_name[channel], true);
    } else {
      //if (debug_log_radio_receive_all)
      {
        char line[60];
        snprintf(line, sizeof(line), " sender: 0x%06x, counter: %i button: %s",
                 rx_serial, rx_counter, commands[rx_function]);
        WriteLog("[INFO] - received data (RSSI: " + (String)value + line, true);
      }
    }

    // send mqtt message with received Data:
    if (mqtt_client.connected() && mqtt_send_radio_receive_all) {
      rx_disc_low[0] = (decoded >> 24) & 0xFF;
      rx_disc_low[1] = (decoded >> 16) & 0xFF;
      rx_disc_low[2] = (decoded >> 8) & 0xFF;
      rx_disc_low[3] = decoded & 0xFF;
      String Topic = "stat/" + config.mqtt_devicetopic + "/received";
      const char * msg = Topic.c_str();
      char payload[220];
      snprintf(payload, sizeof(payload),
               "{\"serial\":\"0x%08x\", \"rx_function\":\"0x%x\", \"rx_disc_low\":%d, \"rx_disc_high\":%d, \"RSSI\":%d, \"counter\":%d, \"rx_device_key_lsb\":\"0x%08x\", \"rx_device_key_msb\":\"0x%08x\", \"decoded\":\"0x%08x\"}",
               rx_serial, rx_function, rx_disc_low[0], rx_disc_h, value, rx_disc_low[3], rx_device_key_lsb, rx_device_key_msb, decoded );
      mqtt_client.publish(msg, payload);
    }
  }

  // If you do not use a MQTT broker so configure the address 0.0.0.0
  if (config.mqtt_broker_addr[0] + config.mqtt_broker_addr[1] + config.mqtt_broker_addr[2] + config.mqtt_broker_addr[3]) {
    // establish connection to MQTT broker
    if (WiFi.status() == WL_CONNECTED) {
      if (!mqtt_client.connected()) {
        // calculate time since last connection attempt
        uint32_t now = millis();
        // possible values of mqttLastReconnectAttempt:
        // 0  => never attempted to connect
        // >0 => at least one connect attempt was made
        if ((mqttLastConnectAttempt == 0) || (now - mqttLastConnectAttempt > MQTT_Reconnect_Interval)) {
          mqttLastConnectAttempt = now;
          // attempt to connect
          mqtt_connect();
        }
      } else {
        // client is connected, call the mqtt loop
        mqtt_client.loop();
      }
    }
  }

  // run a CMD whenever a web_cmd event has been triggered
  if (web_cmd != "") {
/*
    iset = true;
    detachInterrupt(RX_PORT); // Interrupt on change of RX_PORT
    delay(1);
*/
//    WriteLog("[INFO] - received web_cmd for channel " + String(web_cmd_channel)+ " command: " + web_cmd, true);
    if (web_cmd == "up") {
      cmd_up(web_cmd_channel);
    } else if (web_cmd == "down") {
      cmd_down(web_cmd_channel);
    } else if (web_cmd == "stop") {
      cmd_stop(web_cmd_channel);
    } else if (web_cmd == "set shade") {
      cmd_set_shade_position(web_cmd_channel);
    } else if (web_cmd == "shade") {
      cmd_shade(web_cmd_channel);
    } else if (web_cmd == "learn") {
      cmd_learn(web_cmd_channel);
    } else if (web_cmd == "updown") {
      cmd_updown(web_cmd_channel);
    } else if (web_cmd == "save") {
      Serial.println("main loop: in web_cmd save");
      cmd_save_config();
    } else if (web_cmd == "restart") {
      Serial.println("main loop: in web_cmd restart");
      cmd_restart();
    } else {
      WriteLog("[ERR ] - received unknown command from web_cmd.", true);
    }
    web_cmd = "";
  }
  ArduinoOTA.handle();
} // void loop


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// CC1101 radio functions group
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//####################################################################
// Receive Routine
//####################################################################
void ICACHE_RAM_ATTR radio_rx_measure()
{
  static uint32_t LineUp, LineDown, Timeout;
  uint32_t LowVal, HighVal;
  int pinstate = digitalRead(RX_PORT); // Read current pin state
  if (micros() - Timeout > 3500) {     // header detected
    pbwrite = 0;                       // reset buffer
  }
  if (pinstate)                        // pin is now HIGH, was low
  {
    LineUp = micros();                 // Get actual time in LineUp
    LowVal = LineUp - LineDown;        // calculate the LOW pulse time
    if (LowVal < debounce) return;
    if ((LowVal > 300) && (LowVal < 4300))
    {
      if ((LowVal > 3650) && (LowVal < 4300)) {
        Timeout = micros();
        pbwrite = 0;
        lowbuf[pbwrite] = LowVal;
        pbwrite++;
      }
      if ((LowVal > 300) && (LowVal < 1000)) {
        lowbuf[pbwrite] = LowVal;
        pbwrite++;
        Timeout = micros();
      }
    }
  }
  else
  {
    LineDown = micros();               // line went LOW after being HIGH
    HighVal = LineDown - LineUp;       // calculate the HIGH pulse time
    if (HighVal < debounce) return;
    if ((HighVal > 300) && (HighVal < 1000))
    {
      highbuf[pbwrite] = HighVal;
    }
  }
} // void ICACHE_RAM_ATTR radio_rx_measure

//####################################################################
// Generation the encrypted message (Hopcode)
//####################################################################
uint32_t keeloq (byte disc) {
  Keeloq k(device_key_msb, device_key_lsb);
  uint32_t result = (disc << 16) | devcnt;  // append counter value to discrimination value
  return k.encrypt(result);
} // uint32_t keeloq

//####################################################################
// Keygen generates the device crypt key in relation to the masterkey and provided serial number.
// Here normal key-generation is used according to 00745a_c.PDF Appendix G.
// https://github.com/hnhkj/documents/blob/master/KEELOQ/docs/AN745/00745a_c.pdf
//####################################################################
void keygen (uint32_t serial) {
  Keeloq k(config.ulMasterMSB, config.ulMasterLSB); // create Keeloq object
  device_key_lsb  = k.decrypt(serial | 0x20000000); // decrypt devicekey lsb
  device_key_msb  = k.decrypt(serial | 0x60000000); // decrypt devicekey msb
} // void keygen

//####################################################################
// Simple TX routine. Repetitions for simulate continuous button press.
// Send code two times. In case a shutter did not "hear" the command.
//####################################################################
void radio_tx(int channel, byte command, int repetitions) {
  uint32_t serial;
  EEPROM.get(adresses[channel], serial);
  byte disc_l = disc_low[channel];
  byte disc = (disc_l << 8) | serials[channel];
  uint32_t tx_hop = keeloq(disc);
  uint64_t command_long = command;     // 64bit to shift left
  uint64_t serial_long = serial;       // 64bit to shift left
  uint64_t pack = (command_long << 60) | (serial_long << 32) | tx_hop;
  for (int a = 0; a < repetitions; a++)
  {
    digitalWrite(TX_PORT, LOW);        // CC1101 in TX Mode+
    delayMicroseconds(1150);
    radio_tx_frame(13);                // change 28.01.2018 default 10
    delayMicroseconds(3500);

    for (int i = 0; i < 64; i++) {

      int out = ((pack >> i) & 0x1);   // Bitmask to get MSB and send it first
      if (out == 0x1)
      {
        digitalWrite(TX_PORT, LOW);    // Simple encoding of bit state 1
        delayMicroseconds(Lowpulse);
        digitalWrite(TX_PORT, HIGH);
        delayMicroseconds(Highpulse);
      }
      else
      {
        digitalWrite(TX_PORT, LOW);    // Simple encoding of bit state 0
        delayMicroseconds(Highpulse);
        digitalWrite(TX_PORT, HIGH);
        delayMicroseconds(Lowpulse);
      }
    }
    radio_tx_group_h(channel);         // Last 8Bit. For motor 8-16.

    delay(16);                         // delay in loop context is save for wdt
  }
} // void radio_tx

//####################################################################
// Sending high_group_bits 8-16
//####################################################################
void radio_tx_group_h(int channel) {
  byte disc_h;
  disc_h = disc_high[channel];
  for (int i = 0; i < 8; i++) {
    int out = ((disc_h >> i) & 0x1);   // Bitmask to get MSB and send it first
    if (out == 0x1)
    {
      digitalWrite(TX_PORT, LOW);      // Simple encoding of bit state 1
      delayMicroseconds(Lowpulse);
      digitalWrite(TX_PORT, HIGH);
      delayMicroseconds(Highpulse);
    }
    else
    {
      digitalWrite(TX_PORT, LOW);      // Simple encoding of bit state 0
      delayMicroseconds(Highpulse);
      digitalWrite(TX_PORT, HIGH);
      delayMicroseconds(Lowpulse);
    }
  }
} // void radio_tx_group_h

//####################################################################
// Generates sync-pulses
//####################################################################
void radio_tx_frame(int len) {
  for (int i = 0; i < len; ++i) {
    digitalWrite(TX_PORT, LOW);
    delayMicroseconds(400);            // change 28.01.2018 default highpulse
    digitalWrite(TX_PORT, HIGH);
    delayMicroseconds(380);            // change 28.01.2018 default lowpulse
  }
} // void radio_tx_frame

//####################################################################
// Calculate device code from received serial number
//####################################################################
void rx_keygen (uint32_t serial) {
  Keeloq k(config.ulMasterMSB, config.ulMasterLSB);
  rx_device_key_lsb = k.decrypt(serial | 0x20000000);
  rx_device_key_msb = k.decrypt(serial | 0x60000000);
} // void rx_keygen

//####################################################################
// Decoding the hopping code
//####################################################################
uint32_t rx_decoder (uint32_t rx_hopcode) {
  Keeloq k(rx_device_key_msb, rx_device_key_lsb);
  return k.decrypt(rx_hopcode);
} // uint32_t rx_decoder

//####################################################################
// transmit one command
//####################################################################
void tx_command(int channel, byte command, int repetitions)
{
  uint32_t serial;
  byte cmd_new = command;
  if (cmd_new == CMD_STOP_SHADE)
    cmd_new = CMD_STOP;
  if (cmd_new == CMD_SET_SHADE)
    cmd_new = CMD_DOWN;
  if (cmd_new == CMD_SHADE)
    cmd_new = CMD_DOWN;
  EEPROM.get(adresses[channel], serial);   // get the stored serial number
  keygen(serial);                          // generate key for this channel
  EEPROM.get(cntadr, devcnt);              // load last device counter
  entertx();                               // enter transmit mode
  radio_tx(channel, cmd_new, repetitions); // transmit the command
  devcnt++;                                // increment the counter
  devcnt_handler(false);                   // store new device counter
  enterrx();                               // enter receive mode again
  WriteLog("[INFO] - sent to channel " + (String)channel + " (" + config.channel_name[channel] + ") command: " + (String)commands[cmd_new], false);
  if (command != cmd_new)
    WriteLog("(" + (String)commands[command] + ")", false);
  WriteLog("", true);
} // void tx_command

//####################################################################
// calculate RSSI value (Received Signal Strength Indicator)
//####################################################################
byte ReadRSSI()
{
  byte rssi = 0;
  byte value = 0;
  rssi = (cc1101.readReg(CC1101_RSSI, CC1101_STATUS_REGISTER));
  if (rssi >= 128)
  {
    value = 255 - rssi;
    value /= 2;
    value += 74;
  }
  else
  {
    value = rssi / 2;
    value += 74;
  }
  return value;
} // byte ReadRSSI

//####################################################################
// put CC1101 to receive mode
//####################################################################
void enterrx() {
  byte marcState;
  uint32_t rx_time;
  cc1101.setRxState();
  delay(2);
  rx_time = micros();
  while (((marcState = cc1101.readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x0D )
  {
    if (micros() - rx_time > 50000) break; // Quit when marcState does not change...
  }
} // void enterrx

//####################################################################
// put CC1101 to send mode
//####################################################################
void entertx() {
  byte marcState;
  uint32_t rx_time;
  cc1101.setTxState();
  delay(2);
  rx_time = micros();
  while (((marcState = cc1101.readStatusReg(CC1101_MARCSTATE)) & 0x1F) != 0x13 && 0x14 && 0x15)
  {
    if (micros() - rx_time > 50000) break; // Quit when marcState does not change...
  }
} // void entertx


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Webserver functions group
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// void html_api() -> see html_api.h

//####################################################################
// convert the file extension to the MIME type
//####################################################################
String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  return "text/plain";
} // String getContentType

//####################################################################
// send the right file to the client (if it exists)
//####################################################################
bool handleFileRead(String path) {
  if (debug_webui) Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";         // If a folder is requested, send the index file
  String contentType = getContentType(path);            // Get the MIME type
  if (SPIFFS.exists(path)) {                            // If the file exists
    File file = SPIFFS.open(path, "r");                 // Open it
    size_t sent = server.streamFile(file, contentType); // And send it to the client
    file.close();                                       // Then close the file again
    return true;
  }
  if (debug_webui) Serial.println("\tFile Not Found");
  return false;                                         // If the file doesn't exist, return false
} // bool handleFileRead

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// MQTT functions group
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//####################################################################
// Callback for incoming MQTT messages
//####################################################################
void mqtt_callback(char* topic, byte* payload, unsigned int length) {

  if (debug_mqtt) {
    Serial.printf("mqtt in: %s - ", topic);
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
  }

  // extract channel id from topic name
  int channel = 999;
  char * token = strtok(topic, "/");   // initialize token
  token = strtok(NULL, "/");           // now token = 2nd token
  token = strtok(NULL, "/");           // now token = 3rd token, "shutter" or so
  if (debug_mqtt) Serial.printf("command token: %s\n", token);
  if (strncmp(token, "shutter", 7) == 0) {
    token = strtok(NULL, "/");
    if (token != NULL) {
      channel = atoi(token);
    }
  } else if (strncmp(token, "sendconfig", 10) == 0) {
    WriteLog("[INFO] - incoming MQTT command: sendconfig", true);
    mqtt_send_config();
    return;
  } else {
    WriteLog("[ERR ] - incoming MQTT command unknown: " + (String)token, true);
    return;
  }

  // convert payload in string
  payload[length] = '\0';
  String cmd = String((char*)payload);

  // print serial message
  WriteLog("[INFO] - incoming MQTT command: channel " + (String) channel + ":", false);
  WriteLog(cmd, true);

  if (channel <= 15) {
/*
    iset = true;
    detachInterrupt(RX_PORT); // Interrupt @Inputpin
    delay(1);
*/
    if (cmd == "UP" || cmd == "0") {
      cmd_up(channel);
    } else if (cmd == "DOWN"  || cmd == "100") {
      cmd_down(channel);
    } else if (cmd == "STOP") {
      cmd_stop(channel);
    } else if (cmd == "SETSHADE") {
      cmd_set_shade_position(channel);
    } else if (cmd == "SHADE" || cmd == "90") {
      cmd_shade(channel);
    } else if (cmd == "LEARN") {
      cmd_learn(channel);
    } else if (cmd == "UPDOWN") {
      cmd_updown(channel);
    } else {
      WriteLog("[ERR ] - incoming MQTT payload unknown.", true);
    }
  } else {
    WriteLog("[ERR ] - invalid channel, choose one of 0-15", true);
  }
} // void mqtt_callback

//####################################################################
// increment and store devcnt, send devcnt as mqtt state topic
//####################################################################
void devcnt_handler(boolean do_increment) {
  if (do_increment)
    devcnt++;
  EEPROM.put(cntadr, devcnt);
  EEPROM.commit();
  if (mqtt_client.connected()) {
    String Topic = "stat/" + config.mqtt_devicetopic + "/devicecounter";
    const char * msg = Topic.c_str();
    char devcntstr[10];
    itoa(devcnt, devcntstr, 10);
    mqtt_client.publish(msg, devcntstr, true);
  }
} // void devcnt_handler

//####################################################################
// send status via mqtt
//####################################################################
void mqtt_send_percent_closed_state(int channelNum, int percent, String command) {
  if (percent > 100) percent = 100;
  if (percent < 0) percent = 0;
  if (mqtt_client.connected()) {
    char percentstr[4];
    itoa(percent, percentstr, 10);
    String Topic = "stat/" + config.mqtt_devicetopic + "/shutter/" + (String)channelNum;
    const char * msg = Topic.c_str();
    mqtt_client.publish(msg, percentstr);
    WriteLog("[INFO] - MQTT command " + command + " for channel " + (String)channelNum + " (" + config.channel_name[channelNum] + ") sent", true);
  }
} // void mqtt_send_percent_closed_state

//####################################################################
// send config via mqtt
//####################################################################
void mqtt_send_config() {
  String Payload;
  int configCnt = 0, lineCnt = 0;
  char numBuffer[25];
  uint32_t serial;
  if (mqtt_client.connected()) {

    // send config of the shutter channels
    for (int channelNum = 0; channelNum <= 15; channelNum++) {
      if (config.channel_name[channelNum] != "") {
        if (lineCnt == 0) {
          Payload = "{\"channel\":[";
        } else {
          Payload += ", ";
        }
        EEPROM.get(adresses[channelNum], serial);
        sprintf(numBuffer, "0x%08x", serial);
        Payload += "{\"id\":" + String(channelNum) + ", \"name\":\"" + config.channel_name[channelNum] + "\", "
                   + "\"serial\":\"" + numBuffer +  "\"}";
        lineCnt++;

        if (lineCnt >= 4) {
          Payload += "]}";
          mqtt_send_config_line(configCnt, Payload);
          lineCnt = 0;
        }
      } // if (config.channel_name[channelNum] != "")
    } // for

    // handle last item
    if (lineCnt > 0) {
      Payload += "]}";
      mqtt_send_config_line(configCnt, Payload);
    }

    // send most important other config info
    snprintf(numBuffer, 15, "%d", devcnt);
    Payload = "{\"serialprefix\":\"" + config.serial + "\", "
              + "\"mqtt-clientid\":\"" + config.mqtt_broker_client_id + "\", "
              + "\"mqtt-devicetopic\":\"" + config.mqtt_devicetopic + "\", "
              + "\"devicecounter\":" + (String)numBuffer + ", "
              + "\"new_learn_mode\":" + (String)config.learn_mode + "}";
    mqtt_send_config_line(configCnt, Payload);
    WriteLog("[INFO] - MQTT command send_config done", true);
  } // if (mqtt_client.connected())
} // void mqtt_send_config

//####################################################################
// send one config telegram via mqtt
//####################################################################
void mqtt_send_config_line(int & counter, String Payload) {
  String Topic = "stat/" + config.mqtt_devicetopic + "/config/" + (String)counter;
  if (debug_mqtt) Serial.println("mqtt send: " + Topic + " - " + Payload);
  mqtt_client.publish(Topic.c_str(), Payload.c_str());
  counter++;
  yield();
} // void mqtt_send_config_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// execute cmd_ functions group
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//####################################################################
// function to move the shutter up
//####################################################################
void cmd_up(int channel) {
  tx_command(channel, CMD_UP, 2);
  mqtt_send_percent_closed_state(channel, 0, "UP");
} // void cmd_up

//####################################################################
// function to move the shutter down
//####################################################################
void cmd_down(int channel) {
  tx_command(channel, CMD_DOWN, 2);
  mqtt_send_percent_closed_state(channel, 100, "DOWN");
} // void cmd_down

//####################################################################
// function to stop the shutter
//####################################################################
void cmd_stop(int channel) {
  tx_command(channel, CMD_STOP, 2);
  if (shadeLearn)
  {
    shadeLearn = false;
    if ((shadeLearnTime > 0) && (shadeLearnTime < MAX_SHADE_TIME))
    {
      shadeTime[channel] = shadeLearnTime;
      WriteLog("[INFO] - set for channel " + (String)channel + " (" + config.channel_name[channel] + ") shadeTime to " + (String)shadeTime[channel] + " ", true);
    }
  }
} // void cmd_stop

//####################################################################
// function to move shutter to shade position
//####################################################################
void cmd_shade(int channel) {
  uint16_t seconds = 1;
  //WriteLog("[INFO] - putting channel " + (String)channel + " (" + config.channel_name[channel] + ") into mode shade", true);
  tx_command(channel, CMD_SHADE, 2);
  WriteLog("[INFO] - use for channel " + (String)channel + " (" + config.channel_name[channel] + ") shadeTime " + (String)shadeTime[channel] + " ", false);
  if ((shadeTime[channel] > 0) && (shadeTime[channel] < MAX_SHADE_TIME))
    seconds = shadeTime[channel];
  for (int i = 0; i < seconds; i++)
  {
    WriteLog(".", false);
    delay(1000);
  }
  WriteLog("", true);
  tx_command(channel, CMD_STOP_SHADE, 2);
  mqtt_send_percent_closed_state(channel, 90, "SHADE");
} // void cmd_shade

//####################################################################
// function to set the learn/set the shade position
//####################################################################
void cmd_set_shade_position(int channel) {
  shadeLearnTime = 0;
  shadeLearn = true;
  tx_command(channel, CMD_SET_SHADE, 1);
  delay(2000); // Safety time to prevent accidentally erase of end-points.
  WriteLog("[INFO] - wait on channel " + (String)channel + " (" + config.channel_name[channel] + ") for command stop", true);
} // void cmd_set_shade_position

//####################################################################
// function to put the dongle into the learn mode and
// send learning packet.
//####################################################################
void cmd_learn(int channel) {
  WriteLog("[INFO] - putting channel " +  (String) channel +  " (" + config.channel_name[channel] + ") into learning mode...", true);
  if (config.learn_mode == true)
    tx_command(channel, CMD_UP+CMD_DOWN, 1);     // New learn method. Up+Down followd by Stop.
  else
    tx_command(channel, CMD_LEARN, 1);           // Old learn method for receiver before Mfg date 2010.
  if (config.learn_mode == true) {
    delay(1000);
    tx_command(channel, CMD_STOP, 1);
  }
  WriteLog("[INFO] - end for channel " +  (String) channel +  " (" + config.channel_name[channel] + ") mode learning", true);
} // void cmd_learn

//####################################################################
// function to send UP+DOWN button at same time
//####################################################################
void cmd_updown(int channel) {
  tx_command(channel, CMD_UP+CMD_DOWN, 1);
} // void cmd_updown

//####################################################################
// webUI save config function
//####################################################################
void cmd_save_config() {
  WriteLog("[CFG ] - save config initiated from WebUI", true);
  // check if mqtt_devicetopic was changed
  if (config.mqtt_devicetopic_new != config.mqtt_devicetopic) {
    // in case the devicetopic has changed, the LWT state with the old devicetopic should go away
    WriteLog("[CFG ] - devicetopic changed, gracefully disconnect from mqtt server", true);
    // first we send an empty message that overwrites the retained "Online" message
    String topicOld = "tele/" + config.mqtt_devicetopic + "/LWT";
    mqtt_client.publish(topicOld.c_str(), "", true);
    // next: remove retained "devicecounter" message
    topicOld = "stat/" + config.mqtt_devicetopic + "/devicecounter";
    mqtt_client.publish(topicOld.c_str(), "", true);
    delay(200);
    // finally we disconnect gracefully from the mqtt broker so the stored LWT "Offline" message is discarded
    mqtt_client.disconnect();
    config.mqtt_devicetopic = config.mqtt_devicetopic_new;
    delay(200);
  }
  if (config.set_and_generate_serial) {
    WriteLog("[CFG ] - set and generate new serial, user input: " + config.new_serial, true);
    if ((config.new_serial[0] == '0') && (config.new_serial[1] == 'x')) {
      Serial.println("config.serial is hex");
      // string serial stores only highest 3 bytes,
      // add lowest byte with a shift operation for config.serial_number
      config.serial_number = strtoul(config.new_serial.c_str(), NULL, 16) << 8;
      // be safe an convert number back to clean 6-digit hex string
      char serialNumBuffer[11];
      snprintf(serialNumBuffer, 11, "0x%06x", (config.serial_number >> 8));
      config.serial = serialNumBuffer;
      Serial.printf("config.serial: %08u = 0x%08x \n", config.serial_number, config.serial_number);
      cmd_generate_serials(config.serial_number);
    } else {
      server.send ( 200, "text/plain", "Set new Serial not successful, not a hexadecimal number.");
      return;
    }
  }
  if (config.set_devicecounter) {
    uint16_t new_devcnt = strtoul(config.new_devicecounter.c_str(), NULL, 10);
    WriteLog("[CFG ] - set devicecounter to " + String(new_devcnt), true);
    devcnt = new_devcnt;
    devcnt_handler(false);
  }
  WriteConfig();
  server.send ( 200, "text/plain", "Configuration has been saved, system is restarting. Please refresh manually in about 30 seconds.." );
  cmd_restart();
} // void cmd_save_config

//####################################################################
// webUI restart function
//####################################################################
void cmd_restart() {
  server.send ( 200, "text/plain", "System is restarting. Please refresh manually in about 30 seconds." );
  delay(500);
  wifi_disconnect_log = false;
  ESP.restart();
} // void cmd_restart

//####################################################################
// generates 16 serial numbers
//####################################################################
void cmd_generate_serials(uint32_t sn) {
  WriteLog("[CFG ] - Generate serial numbers starting from" + String(sn), true);
  uint32_t z = sn;
  for (int i = 0; i <= 15; ++i) {      // generate 16 serial numbers and storage in EEPROM
    EEPROM.put(adresses[i], z);        // Serial 4Bytes
    z++;
  }
  devcnt = 0;
  devcnt_handler(false);
  delay(100);
} // void cmd_generate_serials

//####################################################################
// connect function for MQTT broker
// called from the main loop
//####################################################################
boolean mqtt_connect() {
  const char* client_id = config.mqtt_broker_client_id.c_str();
  const char* username = config.mqtt_broker_username.c_str();
  const char* password = config.mqtt_broker_password.c_str();
  String willTopic = "tele/" + config.mqtt_devicetopic + "/LWT"; // connect with included "Last-Will-and-Testament" message
  uint8_t willQos = 0;
  boolean willRetain = true;
  const char* willMessage = "Offline";           // LWT message says "Offline"
  String subscribeString = "cmd/" + config.mqtt_devicetopic + "/#";

  WriteLog("[INFO] - trying to connect to MQTT broker", true);
  // try to connect to MQTT
  if (mqtt_client.connect(client_id, username, password, willTopic.c_str(), willQos, willRetain, willMessage )) {
    WriteLog("[INFO] - MQTT connect success", true);
    // subscribe the needed topics
    mqtt_client.subscribe(subscribeString.c_str());
    // publish telemetry message "we are online now"
    mqtt_client.publish(willTopic.c_str(), "Online", true);
  } else {
    WriteLog("[ERR ] - MQTT connect failed, rc =" + (String)mqtt_client.state(), true);
  }
  return mqtt_client.connected();
} // boolean mqtt_connect

//####################################################################
// NTP update
//####################################################################
void updateNTP() {
  configTime(TIMEZONE * 3600, 0, NTP_SERVERS);
} // void updateNTP

//####################################################################
// callback function when time is set via SNTP
//####################################################################
void time_is_set(void) {
  if (time_is_set_first) {    // call WriteLog only once for the initial time set
    time_is_set_first = false;
    WriteLog("[INFO] - time set from NTP server", true);
  }
} // void time_is_set
