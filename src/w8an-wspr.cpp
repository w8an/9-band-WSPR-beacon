/* 
 * W8AN_WSPR
 *
 * Nine band WSPR transmitter with OLED display
 * Copyright (C)2024, Steven R. Stuart, W8AN
 * ----------------------------------------------------------------------------
 * Nine band WSPR beacon for Espressif ESP32 using the Silicon Labs Si5351A clock 
 * generator and nine QRP Labs low pass filter kits installed in two of their 
 * Ultimate relay-switched LPF kits.
 *
 * WiFi parameters, WSPR station information and Si5351A frequency calibration are 
 * entered via a wifi device (phone, laptop, etc.) into a web page which is
 * available when connecting to the ESP32 device wireless portal.
 * 
 * Some code is based on Feld Hell beacon for Arduino by Mark Vandewettering K6HX, 
 * adapted for the Si5351A by Robert Liesenfeld AK6L
 */

#include <int.h>
#include <string.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <si5351.h>
#include <JTEncode.h>
#include <rs_common.h>
#include <SSD1306Wire.h>
#include <Preferences.h>
#include <WiFiManager.h>

// mode defines
#define WSPR_TONE_SPACING       146          // ~1.46 Hz
#define WSPR_DELAY              683          // Delay value for WSPR

// frequencies (bands 0..9)
#define WWV_FREQ                10000000UL    // WWV 10MHz
#define WSPR_10M_FREQ           28126000UL    // 28126.000 - 28126.200  
#define WSPR_12M_FREQ           24926000UL    // 24926.000 - 24926.200  
#define WSPR_15M_FREQ           21096000UL    // 21096.000 - 21096.200  
#define WSPR_17M_FREQ           18106000UL    // 18106.000 - 18106.200  
#define WSPR_20M_FREQ           14097000UL    // 14097.000 - 14097.200  
#define WSPR_30M_FREQ           10140100UL    // 10140.100 - 10140.300  
#define WSPR_40M_FREQ            7040000UL    //  7040.000 -  7040.200  
#define WSPR_80M_FREQ            3594000UL    //  3594.000 -  3594.200  
#define WSPR_160M_FREQ           1838000UL    //  1838.000 -  1838.200  
#define WSPR_WNDO_CTR                100      // center of wspr window
const int freqArray [] = { 
  WWV_FREQ, WSPR_10M_FREQ, WSPR_12M_FREQ, WSPR_15M_FREQ, WSPR_17M_FREQ, \
  WSPR_20M_FREQ, WSPR_30M_FREQ, WSPR_40M_FREQ, WSPR_80M_FREQ, WSPR_160M_FREQ };
long biasHertz[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // array of correction freqs

// low pass filters connection pins 
#define FILTER_1_PIN           18  //  10M    
#define FILTER_2_PIN           27  //  12M      
#define FILTER_3_PIN           12  //  15M      
#define FILTER_4_PIN           14  //  17M      
#define FILTER_5_PIN           25  //  20M      
#define FILTER_6_PIN           32  //  30M        
#define FILTER_7_PIN           33  //  40M        
#define FILTER_8_PIN           26  //  80M        
#define FILTER_9_PIN           23  // 160M      
#define FILTER_0_PIN           19  // Bypass filter bank 0

// hardware defines
#define BAND_BUTTON_PIN        34  // configuration band select
#define PORTAL_PIN             15  // wifi portal when low on reset      
#define LED_PIN                13  // On-The-Air lamp

#define PREFS_RO             true  // preferences read-only flag
#define PREFS_RW            false  // preferences read/write flag

const char* portalName = "WSPR-PORTAL";    // portal ssid
const char* prefs_station = "station";     // general purpose namespace in prefs
const char* station_callsign = "callsign"; // prefs station key name
const char* station_locator = "locator";   // prefs locator key name
const char* prefs_bias = "bias";           // frequency bias values namespace
const char biasband[10][5] =               // prefs_bias keys
  {"wwv", "10m", "12m", "15m", "17m", "20m", "30m", "40m", "80m", "160m"};

// Class instantiation
Preferences prefs;                         // persistent data store
Si5351 si5351;
JTEncode jtencode;
SSD1306Wire display(0x3C, 21, 22);
WiFiUDP udp;
IPAddress timeServer(192, 5, 41, 41);        // tock.usno.navy.mil
//IPAddress timeServer(192, 48, 105, 15);    // us.pool.ntp.org 
//IPAddress timeServer(208, 113, 130, 146);  // us.pool.ntp.org alt

// NTP
const int timeZone = 0;             // UTC
const int NTP_PACKET_SIZE = 48;     // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

// global variables
int band = 0;
bool tog = false;    // used to toggle oled data page
unsigned long freq;
char call[7] = "NOCALL";   // maximum 6 character station call sign
char loc[5] = "AA00";      // station maidenhead grid position (4 chars)
char timebuf[12], freqbuf[16];
uint8_t dbm = 3;   // 2.1mW Calculated from observed output (0.92mV p-p @ 50 ohm)
uint8_t tx_buffer[255];
bool call_for_portal = false;
bool calibration_mode = false;     // calibration transmit mode

// function prototypes
void encode(unsigned long);
void calibrate(unsigned long, int);
void set_tx_buffer(void);
void sendNTPpacket(IPAddress &address);
time_t getNtpTime(void);
void getTime(void);
void printTime(void);
void setFreqbuf(int);
void initFilterPins(void);
void lowpass(uint);            
void blinkLed();              

/*
 *
 *    Set Up 
 */
void setup()
{
  pinMode(LED_PIN, OUTPUT);                // "On the Air" indicator.
  blinkLed();                              // system starting indicator
  pinMode(PORTAL_PIN, INPUT_PULLUP);       // call for web portal
  pinMode(BAND_BUTTON_PIN, INPUT_PULLUP);  // call for calibration mode
  initFilterPins();                        // set up low pass filter pins

  Serial.begin(115200);  
  delay(500);      // 0.5 seconds

  Serial.println("=");
  if( digitalRead(PORTAL_PIN) == LOW ) {
    // hold the portal button on boot to launch web configuration
    call_for_portal = true;
    Serial.println(F("==> CONFIGURATION PORTAL"));
  }
  else { 
    // hold the band button on boot to launch calibration mode 
    if( digitalRead(BAND_BUTTON_PIN) == LOW) {
      calibration_mode = true;
      Serial.println(F("==> CALIBRATION MODE"));
    }
    else Serial.println(F("==> OPERATION MODE"));
  }
  Serial.println("=");

  // get station identification and grid location
  prefs.begin(prefs_station, PREFS_RO);
  prefs.getString(station_callsign, call, sizeof(call));
  prefs.getString(station_locator,  loc,  sizeof(loc));
  prefs.end();

  // sign on
  Serial.println(F("---[ W8AN ]---"));
  Serial.println(F("Nine Band WSPR Transmitter"));
  Serial.println(F(__FILE__));
  Serial.print(F(__DATE__));
  Serial.print(" ");
  Serial.println(F(__TIME__));
  Serial.print(F("PlatformIO v."));
  Serial.println(F(__VERSION__));
  Serial.println();
  Serial.print("Station: ");
  Serial.println(call);
  Serial.print("Locator: ");
  Serial.println(loc);
  Serial.println();

  // start up the oled display
  display.init();      
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.clear();                     
  if (calibration_mode) {
    display.drawString(0, 0, "Calibration");
    display.drawString(0, 20, "Mode");
    display.drawString(0, 40, call);
  }
  else {
    display.drawString(70, 0, "W8AN");
    display.drawString(0, 20, "v.");
    display.drawString(35, 20, __DATE__);
  }
  display.display();

  // get frequency correction settings  
  prefs.begin(prefs_bias, PREFS_RO); 
  for ( band=0; band<10; band++) 
    biasHertz[band] = prefs.getLong(biasband[band], 0);
  prefs.end();

  if (! calibration_mode) {

    // operation mode

    WiFiManager wifiMan;  // start up the wifi
    wifiMan.setDebugOutput(false);  // true if you want send to serial debug
    //--  wifiMan.resetSettings();  // force wifi set up portal
    if(wifiMan.autoConnect(portalName)) {   // portal at 192.168.4.1
      if (call_for_portal) { // user call for portal
        // retrieve the current Wi-Fi configuration
        wifi_config_t conf;
        if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK) {
          Serial.printf("SSID: %s\n", (char*)conf.sta.ssid);
          Serial.printf("Password: %s\n", (char*)conf.sta.password);
        } else Serial.println(F("Failed to get WiFi config"));
        
        // set up portal station parameters and calibration settings  
        WiFiManagerParameter callsign(station_callsign, "Station call sign", call, sizeof(call));
        wifiMan.addParameter(&callsign);

        WiFiManagerParameter locator(station_locator, "Maidenhead grid locator", loc, sizeof(loc));
        wifiMan.addParameter(&locator);

        char buffer [11];
        itoa(WSPR_10M_FREQ + WSPR_WNDO_CTR - biasHertz[1], buffer, 10);
        WiFiManagerParameter bias10(biasband[1], "10M freq", buffer, 9); 
        wifiMan.addParameter(&bias10);

        itoa(WSPR_12M_FREQ + WSPR_WNDO_CTR - biasHertz[2], buffer, 10);
        WiFiManagerParameter bias12(biasband[2], "12M freq", buffer, 9); 
        wifiMan.addParameter(&bias12);

        itoa(WSPR_15M_FREQ + WSPR_WNDO_CTR - biasHertz[3], buffer, 10);
        WiFiManagerParameter bias15(biasband[3], "15M freq", buffer, 9);
        wifiMan.addParameter(&bias15);

        itoa(WSPR_17M_FREQ + WSPR_WNDO_CTR - biasHertz[4], buffer, 10);
        WiFiManagerParameter bias17(biasband[4], "17M freq", buffer, 9);
        wifiMan.addParameter(&bias17);

        itoa(WSPR_20M_FREQ + WSPR_WNDO_CTR - biasHertz[5], buffer, 10);
        WiFiManagerParameter bias20(biasband[5], "20M freq", buffer, 9);
        wifiMan.addParameter(&bias20);

        itoa(WSPR_30M_FREQ + WSPR_WNDO_CTR - biasHertz[6], buffer, 10);
        WiFiManagerParameter bias30(biasband[6], "30M freq", buffer, 9);
        wifiMan.addParameter(&bias30);

        itoa(WSPR_40M_FREQ + WSPR_WNDO_CTR - biasHertz[7], buffer, 10);
        WiFiManagerParameter bias40(biasband[7], "40M freq", buffer, 9);
        wifiMan.addParameter(&bias40);

        itoa(WSPR_80M_FREQ + WSPR_WNDO_CTR - biasHertz[8], buffer, 10);
        WiFiManagerParameter bias80(biasband[8], "80M freq", buffer, 9);
        wifiMan.addParameter(&bias80);

        itoa(WSPR_160M_FREQ + WSPR_WNDO_CTR - biasHertz[9], buffer, 10);
        WiFiManagerParameter bias160(biasband[9], "160M freq", buffer, 9);
        wifiMan.addParameter(&bias160);

        itoa(WWV_FREQ - biasHertz[0], buffer, 10);
        WiFiManagerParameter biaswwv(biasband[0], "wwv freq", buffer, 9);  
        wifiMan.addParameter(&biaswwv);

        display.clear();
        display.drawString(0,0,F("Portal On-Line"));
        display.drawString(0,20,portalName);
        display.drawString(0,40,F("192.168.4.1"));
        display.display();

        wifiMan.startConfigPortal(portalName); 

        // store the user input values from portal page to prefs database
        // station info
        prefs.begin(prefs_station, PREFS_RW);
        prefs.putString(station_callsign, callsign.getValue());
        prefs.putString(station_locator, locator.getValue());
        // reload to get any changed value(s)
        prefs.getString(station_callsign, call, 6); 
        prefs.getString(station_locator, loc, 4);
        prefs.end();
        // frequency adjustments
        prefs.begin(prefs_bias, PREFS_RW); 
        prefs.putLong(biasband[1], WSPR_10M_FREQ + WSPR_WNDO_CTR - atol(bias10.getValue())); // 10M
        prefs.putLong(biasband[2], WSPR_12M_FREQ + WSPR_WNDO_CTR - atol(bias12.getValue())); // 12M
        prefs.putLong(biasband[3], WSPR_15M_FREQ + WSPR_WNDO_CTR - atol(bias15.getValue())); // 15M
        prefs.putLong(biasband[4], WSPR_17M_FREQ + WSPR_WNDO_CTR - atol(bias17.getValue())); // 17M
        prefs.putLong(biasband[5], WSPR_20M_FREQ + WSPR_WNDO_CTR - atol(bias20.getValue())); // 20M
        prefs.putLong(biasband[6], WSPR_30M_FREQ + WSPR_WNDO_CTR - atol(bias30.getValue())); // 30M
        prefs.putLong(biasband[7], WSPR_40M_FREQ + WSPR_WNDO_CTR - atol(bias40.getValue())); // 40M
        prefs.putLong(biasband[8], WSPR_80M_FREQ + WSPR_WNDO_CTR - atol(bias80.getValue())); // 80M
        prefs.putLong(biasband[9], WSPR_160M_FREQ + WSPR_WNDO_CTR - atol(bias160.getValue())); // 160M
        prefs.putLong(biasband[0], WWV_FREQ - atol(biaswwv.getValue())); // wwv
        prefs.end();

        // now load biasHertz array from pref entries
        prefs.begin(prefs_bias, PREFS_RO);
        for (band=0; band<10; band++) 
          biasHertz[band] = prefs.getLong(biasband[band], 0);
        prefs.end();
      }
    } 
    else {
      Serial.println(F("Failed to connect to WiFi. Please restart."));
      display.clear();
      display.drawString(0, 20, F("WiFi Failed"));
      ESP.restart();
    }

    // start the ntp time client
    Serial.println(F("Waiting for NTP sync"));
    delay(10000);
    setSyncProvider(getNtpTime);
    setSyncInterval(300000); //bogus interval, we will reset later
    delay(5000);
    printTime();
  }
  
  // cycle the filter relays as a test
  Serial.print(F("Testing filter relays: "));
  for (band=1; band<10; band++ )  //nine filters
  { 
    Serial.print(band);
    lowpass(band);
    delay(1000);
  }
  lowpass(-1); // disable all filters
  Serial.println();

  // Initialize the Si5351
  // Change the 2nd parameter in init if using a ref osc other than 25 MHz
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);

  // Set CLK0 output (power level: SSI5351_DRIVE_2MA, _4MA, _6MA, _8MA) 
  if (calibration_mode) si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);
  else si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0, 0); // Disable the clock initially

  // get the si5351 frequency adjustments
  prefs.begin(prefs_bias, PREFS_RO); // load biasHertz array from pref entries
  for (int i=0; i<10; i++)   
    biasHertz[i] = prefs.getLong(biasband[i], 0);
  prefs.end();

  // encode the message in the transmit buffer
  set_tx_buffer();

  if (calibration_mode) {
    // set initial calibration frequency
    band = 0;
    lowpass(band);
  }
}



/*
 *
 *    Loop 
 */
void loop() { 
  if (calibration_mode) {
    /*
     * Press band button to select the next calibration band and
     * collect a list of the frequencies at the 2-second tone mark.
     * Those frequencies are to be entered into the portal fields.
     */
    if(digitalRead(BAND_BUTTON_PIN) == LOW) {
      delay(50);   // delay to debounce
      if (digitalRead(BAND_BUTTON_PIN) == LOW) {
        band++;
        if (band > 9) band = 0;  // rotate
        lowpass(band);           // set the band filter
        Serial.print(F("Calibration freq: "));
        Serial.print(freqArray[band] + WSPR_WNDO_CTR);  //offset to center of wspr window
        Serial.print(F("  bias "));
        Serial.print(biasHertz[band]);
        Serial.print(F("  Error freq: "));
        Serial.println(freqArray[band] + WSPR_WNDO_CTR - biasHertz[band]);
      }
    }
    setFreqbuf(freqArray[band] + WSPR_WNDO_CTR);
    display.clear();
    display.drawString(0, 0, F("-- CALIBRATE --"));
    display.drawString(0, 20, freqbuf);
    //display.drawString(0, 40, F("press BAND"));
    display.display();
  
    digitalWrite(LED_PIN, HIGH);
    calibrate(freqArray[band] + WSPR_WNDO_CTR, 2000);   //xmit no freq correction for 2 secs
    digitalWrite(LED_PIN, LOW);
    calibrate(freqArray[band] + WSPR_WNDO_CTR + biasHertz[band], 1000); //xmit corrected freq 1 sec
  }
  else {
    // operation mode

    if (timeStatus() == timeSet &&       // if the time is correct,
        minute() % 2 == 0 &&             // run at the top of every even minute
        second() == 0)      
    {
      band++;  // next band: 1=10M 2=12M 3=15M 4=17M 5=20M 6=30M 7=40M 8=80M 9=160M 
      if (band > 9) band = 1;   // rotate

      int offset = random(1,199);   // hop around within the WSPR window 

      Serial.println(F("--"));
      Serial.print(F("Random offset  "));
      Serial.print(offset);
      Serial.println(F(" Hz"));

      setFreqbuf(freqArray[band] + offset);
      Serial.print(F("Frequency  "));
      Serial.println(freqbuf);
      
      lowpass(band);  // set the band filter
      
      delay(1500);   // transmission to begin at approx 2 secs past the even minute
      getTime();     // populate timebuf

      display.clear();                  
      int r = random(0,70);  // randomize the horiz pos to avoid oled burn-in
      display.drawString(r, 0, F(call) );
      r = random(0,40);
      display.drawString(r, 20, timebuf);
      r = random(0,10);
      display.drawString(r, 40, freqbuf);
      display.display();

      // transmit a report
      Serial.print(F("Transmission begin    "));
      printTime();
      encode(freqArray[band] + biasHertz[band] + offset);
      Serial.print(F("Transmission complete "));
      getTime();
      printTime();
      delay(1000);
    }
    else { 
      //show time while waiting for next transmission
      display.clear();                  
      int r = random(0,70);  // randomize oled x-position
      display.drawString(r, 0, call);
      getTime();             // populate timebuf
      r = random(0,30);
      display.drawString(r, 20, timebuf);
      if (tog) display.drawString(0, 40, WiFi.localIP().toString()); 
      else display.drawString(0, 40, "RSSI: " + String(WiFi.RSSI())); 
      display.display();
      tog = !tog;
    }
    delay(1000);
  }
}


void encode(unsigned long xfreq) {

  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  digitalWrite(LED_PIN, HIGH); // on-the-air indicator
  si5351.output_enable(SI5351_CLK0, 1);

  // Loop through the string, transmitting one character at a time.
  for(i = 0; i < WSPR_SYMBOL_COUNT; i++) { // WSPR_ consts are from the JTEncode lib
    si5351.set_freq((xfreq * 100) + (tx_buffer[i] * WSPR_TONE_SPACING), SI5351_CLK0);
    delay(WSPR_DELAY);
  }

  // Turn off the output
  si5351.output_enable(SI5351_CLK0, 0);
  digitalWrite(LED_PIN, LOW);
}

void calibrate(unsigned long calfreq, int xtime) {

  uint8_t i;

  // Reset the tone to the base frequency and turn on the output
  si5351.set_freq(calfreq * 100, SI5351_CLK0);
  si5351.output_enable(SI5351_CLK0, 1); // transmit on
  delay(xtime);  // transmission time in ms
  si5351.output_enable(SI5351_CLK0, 0); // transmit off
}

void set_tx_buffer() {
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);
  // Set the proper frequency and timer CTC 
  jtencode.wspr_encode(call, loc, dbm, tx_buffer);
}

void sendNTPpacket(IPAddress &address) {
  // send an NTP request to the time server at the given address
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}

void getTime() {  
  //populate timebuf
  sprintf(timebuf, "%02d:%02d:%02d z", hour(), minute(), second());
}

void printTime() {
  //Serial.println(String(hour()) + ":" + String(minute()) + ":" + String(second()) + " z");
  Serial.println(timebuf);
}  

void setFreqbuf(int freq) { 
  // populate freqbuf 
  int mhz = freq / 1000000;
  int khz = freq / 1000 - mhz * 1000;
  int hz  = freq - mhz * 1000000 - khz * 1000;
  sprintf(freqbuf, "%d.%03d.%03d MHz", mhz, khz, hz);
}

void initFilterPins() {
  // Set the filter relay control pins
  pinMode(FILTER_1_PIN, OUTPUT);
  pinMode(FILTER_2_PIN, OUTPUT);
  pinMode(FILTER_3_PIN, OUTPUT);
  pinMode(FILTER_4_PIN, OUTPUT);
  pinMode(FILTER_5_PIN, OUTPUT);
  pinMode(FILTER_6_PIN, OUTPUT);
  pinMode(FILTER_7_PIN, OUTPUT);
  pinMode(FILTER_8_PIN, OUTPUT);
  pinMode(FILTER_9_PIN, OUTPUT);
  pinMode(FILTER_0_PIN, OUTPUT);
}

void lowpass(uint filter) {
  // Set the low pass filters as needed for the band, -1 = none
  digitalWrite(FILTER_1_PIN, LOW);    // all off
  digitalWrite(FILTER_2_PIN, LOW);
  digitalWrite(FILTER_3_PIN, LOW);
  digitalWrite(FILTER_4_PIN, LOW);
  digitalWrite(FILTER_5_PIN, LOW);
  digitalWrite(FILTER_6_PIN, LOW);
  digitalWrite(FILTER_7_PIN, LOW);
  digitalWrite(FILTER_8_PIN, LOW);
  digitalWrite(FILTER_9_PIN, LOW);
  digitalWrite(FILTER_0_PIN, LOW);
  
  switch (filter) { 
    // Set filter pin and bank 0 bypass pin 
    //   FILTER_0_PIN = HIGH - Bypass all filters on 160M-30M bank
    //   FILTER_1_PIN = HIGH - enables 10M filter for bands 20M-10M 
    case 1:           // 10M
      digitalWrite(FILTER_1_PIN, HIGH); digitalWrite(FILTER_0_PIN, HIGH); break;
    case 2:           // 12M
      digitalWrite(FILTER_2_PIN, HIGH); digitalWrite(FILTER_0_PIN, HIGH); break;
    case 3:           // 15M
      digitalWrite(FILTER_3_PIN, HIGH); digitalWrite(FILTER_0_PIN, HIGH); break;
    case 4:           // 17M
      digitalWrite(FILTER_4_PIN, HIGH); digitalWrite(FILTER_0_PIN, HIGH); break;
    case 5:           // 20M
      digitalWrite(FILTER_5_PIN, HIGH); digitalWrite(FILTER_0_PIN, HIGH); break;
    case 6 | 0:       // 30M | wwv
      digitalWrite(FILTER_6_PIN, HIGH); digitalWrite(FILTER_1_PIN, HIGH); break;
    case 7:           // 40M
      digitalWrite(FILTER_7_PIN, HIGH); digitalWrite(FILTER_1_PIN, HIGH); break;
    case 8:           // 80M
      digitalWrite(FILTER_8_PIN, HIGH); digitalWrite(FILTER_1_PIN, HIGH); break;
    case 9:           // 160M
      digitalWrite(FILTER_9_PIN, HIGH); digitalWrite(FILTER_1_PIN, HIGH); break;
  }
}

time_t getNtpTime() {
  while (udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println(F("Transmit NTP Request"));
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println(F("Receive NTP Response"));
      udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println(F("No NTP Response"));
  return 0; // return 0 when unable to get the time
}

void blinkLed() { 
  // 3 blinks
  for(int i = 0; i<3; i++) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(100);
  }
}

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
// ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
// CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// eof