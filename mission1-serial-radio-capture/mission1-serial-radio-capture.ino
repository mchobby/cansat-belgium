/***************************************************************************
  Cansat-Belgium - Mission 1 - Data Emitter

  This is sketch Collect the TMP36 & BMP280 data and send it over 
  the RFM69HCW Radio transmitter to the Ground Station Receiver.

  See the Wiki : https://wiki.mchobby.be/index.php?title=ENG-CANSAT-MISSION1-CAPTURE
  All tutorials: http://cansat.mchobby.be
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_NeoPixel.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 433.0

#if defined(ARDUINO_SAMD_FEATHER_M0) 
  // UPDATE for Feather M0 EXPRESS with RFM69HCW radio module
  // G0 is the Radio Module interrupt pin
  #define RFM69_CS      6
  #define RFM69_INT     9 
  #define RFM69_RST     10
  #define RADIO_LED     13 // light up while transmitting
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
/********* EOF Radio Setup *************/

// NeoPixel on the board
#define NEOPIXEL       8 // on pin 8
#define NUMPIXELS      1 // nbr of pixels

// where is wired the TMP36 on analogue input
#define temperaturePin A3 

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BMP280 bme; // I2C

// Pixels collection reduced to ONE single LED
Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, NEOPIXEL, NEO_GRB + NEO_KHZ800);
  
void setup() {
  Serial.begin(9600);

  // wait until serial console is open, remove if not tethered to computer
  while (!Serial) { delay(1); } 
 
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  init_radio_module();

  // everything is right! So switch off neopixel
  pixel.begin();
  pixel.setPixelColor(0, pixel.Color(0,0,0)); // switch off
  pixel.show();
}



bool header_send = false; // if header has not been send yet... 
int16_t packetnum = 0; // packet number increment at each data emission
void loop() {
    // send columns header
    if( !(header_send) ){
        send_header();
        header_send = true;
    } 
    
    // read the voltage of TMP36
    float voltage = getVoltage(temperaturePin);
    // convert voltage to temperature
    //   Degrees = (voltage - 500mV) multiplied by 100
    float temperature = (voltage - .5) *100;

    float bme_temp = bme.readTemperature();
    float bme_hpa  = bme.readPressure();

    unsigned long ms = millis();
    packetnum += 1; // increment 

    // char radiopacket[40] = "Hello World #";
    String packet_str = String( ":"+String(packetnum,DEC)+"|" );
    packet_str.concat( String(ms,DEC)+"|" );
    packet_str.concat( String( temperature, 2 )+"|" );
    packet_str.concat( String( bme_hpa, 2 )+"|" );
    packet_str.concat( String( bme_temp, 2 )+";\r\n" );

    // send to Serial
    Serial.print( packet_str.c_str() );
    // Send over Radio
    rf69.send((uint8_t *)(packet_str.c_str()), packet_str.length());
    rf69.waitPacketSent();

    // Now wait for a reply
    uint8_t buf[4]; // We limit the quantity received data
    uint8_t len = sizeof(buf);

    if (rf69.waitAvailableTimeout(500))  { 
      // Should be a reply message for us now   
      if (rf69.recv(buf, &len)) {
          Serial.print(": ");
          Serial.println((char*)buf);
          Blink(RADIO_LED, 50, 1); //blink LED once, 50ms between blinks
      } else {
          Serial.println("Receive failed");
          Blink(RADIO_LED, 50, 2 ); //blink LED twice, 50ms between blinks
      }
    } else {
        Serial.println("No reply, is another RFM69 listening?");
        Blink(RADIO_LED, 50, 3 ); // blink 3 times, 50ms between blinks
    }

    // Going to next round
}

void init_radio_module() {
  pinMode(RADIO_LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Cansat-Belgium - Mission 1 - Data Emitter");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  
  pinMode(RADIO_LED, OUTPUT);

  Serial.print("RFM69 radio @");
  Serial.print((int)RF69_FREQ);
  Serial.println(" MHz");
}

void send_header() {
  // Send header about the data  Serial.println(F("***START***"));
  String s1 = String( F("***HEADER***\r\n") );
  Serial.print( s1 );
  // use : as begin of data and ; as end of data
  String s2 = String( F(":counter|time_ms|temperature|pressure_hpa|temp2;\r\n") );
  Serial.print(s2);
  String s3 = String( F("***DATA***\r\n") );
  Serial.print( s3 );
  
}

void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
  // exit: wait 3 times the delay
  delay( 3* DELAY_MS );
}

/*
 * getVoltage() - return the voltage of an analog pin 
 */
float getVoltage(int pin){
   // AS the sketch does not call the analogReadResolution()
   //    function to change the analog reading resolution 
   // THEN Arduino use the defaut 12 bits resolution!
   // Under 12 bits resolution, the analogRead() returns
   //    a value between 0 & 1024.
   //
   // Convert digital value between 0 & 1024 to 
   //    voltage between 0 & 3.3 volts.
   //    (each unit equal 3.3 / 1024 = 3.2 millivolts)
   return (analogRead(pin) * .0032);
}
