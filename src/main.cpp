/*******************************************************************************
 * ToDo:
 * - set NWKSKEY (value from staging.thethingsnetwork.com)
 * - set APPKSKEY (value from staging.thethingsnetwork.com)
 * - set DEVADDR (value from staging.thethingsnetwork.com)
 * - optionally comment #define DEBUG
 * - optionally comment #define SLEEP
 * - set TX_INTERVAL in seconds
 *
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <CayenneLPP.h>
#include "DHT.h"

// BME280 I2C address is 0x76(108)
#define Addr 0x76

#define DHTPIN 9     // digital pin DHT22

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// LoRaWAN NwkSKey, your network session key, 16 bytes (from staging.thethingsnetwork.org)
static const PROGMEM u1_t NWKSKEY[16] = {  };
// LoRaWAN AppSKey, application session key, 16 bytes  (from staging.thethingsnetwork.org)
static const u1_t PROGMEM APPSKEY[16] = {  };
// LoRaWAN end-device address (DevAddr),  (from staging.thethingsnetwork.org)
static const u4_t DEVADDR = 0x0; // <-- Change this address for every node!

// show debug statements; comment next line to disable debug statements
#define DEBUG
// use low power sleep; comment next line to not use low power sleep
#define SLEEP

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10*60;

CayenneLPP lpp(51);
DHT dht(DHTPIN, DHTTYPE);
double cTemp0;
double cTemp1;
double pressure;
double humidity;

#ifdef SLEEP
  #include "LowPower.h"
  bool next = false;
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

void getSensorData(){
  unsigned int b1[24];
  unsigned int data[8];
  unsigned int dig_H1 = 0;
  for(int i = 0; i < 24; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((136+i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);

    // Read 24 bytes of data
    if(Wire.available() == 1)
    {
      b1[i] = Wire.read();
    }
  }

  // Convert the data
  // temp coefficients
  unsigned int dig_T1 = (b1[0] & 0xff) + ((b1[1] & 0xff) * 256);
  int dig_T2 = b1[2] + (b1[3] * 256);
  int dig_T3 = b1[4] + (b1[5] * 256);

  // pressure coefficients
  unsigned int dig_P1 = (b1[6] & 0xff) + ((b1[7] & 0xff ) * 256);
  int dig_P2 = b1[8] + (b1[9] * 256);
  int dig_P3 = b1[10] + (b1[11] * 256);
  int dig_P4 = b1[12] + (b1[13] * 256);
  int dig_P5 = b1[14] + (b1[15] * 256);
  int dig_P6 = b1[16] + (b1[17] * 256);
  int dig_P7 = b1[18] + (b1[19] * 256);
  int dig_P8 = b1[20] + (b1[21] * 256);
  int dig_P9 = b1[22] + (b1[23] * 256);

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select data register
  Wire.write(161);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Request 1 byte of data
  Wire.requestFrom(Addr, 1);

  // Read 1 byte of data
  if(Wire.available() == 1)
  {
    dig_H1 = Wire.read();
  }

  for(int i = 0; i < 7; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((225+i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);

    // Read 7 bytes of data
    if(Wire.available() == 1)
    {
      b1[i] = Wire.read();
    }
  }

  // Convert the data
  // humidity coefficients
  int dig_H2 = b1[0] + (b1[1] * 256);
  unsigned int dig_H3 = b1[2] & 0xFF ;
  int dig_H4 = (b1[3] * 16) + (b1[4] & 0xF);
  int dig_H5 = (b1[4] / 16) + (b1[5] * 16);
  int dig_H6 = b1[6];

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select control humidity register
  Wire.write(0xF2);
  // Humidity over sampling rate = 1
  Wire.write(0x01);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select control measurement register
  Wire.write(0xF4);
  // Normal mode, temp and pressure over sampling rate = 1
  Wire.write(0x27);
  // Stop I2C Transmission
  Wire.endTransmission();

  // Start I2C Transmission
  Wire.beginTransmission(Addr);
  // Select config register
  Wire.write(0xF5);
  // Stand_by time = 1000ms
  Wire.write(0xA0);
  // Stop I2C Transmission
  Wire.endTransmission();

  for(int i = 0; i < 8; i++)
  {
    // Start I2C Transmission
    Wire.beginTransmission(Addr);
    // Select data register
    Wire.write((247+i));
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 1 byte of data
    Wire.requestFrom(Addr, 1);

    // Read 8 bytes of data
    if(Wire.available() == 1)
    {
      data[i] = Wire.read();
    }
  }

  // Convert pressure and temperature data to 19-bits
  long adc_p = (((long)(data[0] & 0xFF) * 65536) + ((long)(data[1] & 0xFF) * 256) + (long)(data[2] & 0xF0)) / 16;
  long adc_t = (((long)(data[3] & 0xFF) * 65536) + ((long)(data[4] & 0xFF) * 256) + (long)(data[5] & 0xF0)) / 16;
  // Convert the humidity data
  long adc_h = ((long)(data[6] & 0xFF) * 256 + (long)(data[7] & 0xFF));

  // Temperature offset calculations
  double var1 = (((double)adc_t) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  double var2 = ((((double)adc_t) / 131072.0 - ((double)dig_T1) / 8192.0) *
  (((double)adc_t)/131072.0 - ((double)dig_T1)/8192.0)) * ((double)dig_T3);
  double t_fine = (long)(var1 + var2);
  cTemp0 = (var1 + var2) / 5120.0;

  // Pressure offset calculations
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
  var1 = (((double) dig_P3) * var1 * var1 / 524288.0 + ((double) dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
  double p = 1048576.0 - (double)adc_p;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double) dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double) dig_P8) / 32768.0;
  pressure = (p + (var1 + var2 + ((double)dig_P7)) / 16.0) / 100;

  cTemp1 = dht.readTemperature();
  humidity = dht.readHumidity();
}

void onEvent (ev_t ev) {
    #ifdef DEBUG
      Serial.println(F("Enter onEvent"));
    #endif

    switch(ev) {
        case EV_TXCOMPLETE:
            #ifdef DEBUG
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            #endif
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                #ifdef DEBUG
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
                #endif
            }
            // Schedule next transmission
            #ifndef SLEEP
                        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            #else
                        next = true;
            #endif

            break;

         default:
            #ifdef DEBUG
            Serial.println(F("Unknown event"));
            #endif
            break;
    }
    #ifdef DEBUG
      Serial.println(F("Leave onEvent"));
    #endif

}

void do_send(osjob_t* j){

    #ifdef DEBUG
      Serial.println(F("Enter do_send"));
    #endif

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {

        getSensorData();
        // Prepare upstream data transmission at the next possible time.
        lpp.reset();
        lpp.addTemperature(1, cTemp0);
      //  lpp.addTemperature(2, cTemp1);
        lpp.addBarometricPressure(2, pressure);
        lpp.addRelativeHumidity(3, humidity);

        LMIC.txChnl = 0;

        LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
        #ifdef DEBUG
        Serial.println(F("Packet queued"));
        Serial.println(LMIC.freq);
        #endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
    #ifdef DEBUG
      Serial.println(F("Leave do_send"));
    #endif

}


int main(int argc, char const *argv[]) {
  init();

  #ifdef DEBUG
  Serial.begin(9600);
  #endif

  // Initialise I2C communication as MASTER
  Wire.begin();

  dht.begin();

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));

  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);

  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.


  // Disable link check validation
  
   LMIC_setLinkCheckMode(0);
  // Uncomment to use channel 868100000 only
  // for(int i=0; i<9; i++) { // For EU; for US use i<71
  //     if(i != 0) {
  //         LMIC_disableChannel(i);
  //     }
  // }

  // Set data rate (SF) and transmit power for uplink
  LMIC_setDrTxpow(DR_SF12, 14);

  // Start job
  do_send(&sendjob);

  #ifdef DEBUG
    Serial.println(F("Leave setup"));
  #endif

  while(1) {

    #ifndef SLEEP

      os_runloop_once();

    #else
      extern volatile unsigned long timer0_overflow_count;

      if (next == false) {

        os_runloop_once();

      } else {

        int sleepcycles = TX_INTERVAL / 8;  // calculate the number of sleepcycles (8s) given the TX_INTERVAL
        #ifdef DEBUG
          Serial.print(F("Enter sleeping for "));
          Serial.print(sleepcycles);
          Serial.println(F(" cycles of 8 seconds"));
        #endif
        Serial.flush(); // give the serial print chance to complete
        for (int i=0; i<sleepcycles; i++) {
          // Enter power down state for 8 s with ADC and BOD module disabled
          LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
          //LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);

          // LMIC uses micros() to keep track of the duty cycle, so
          // hack timer0_overflow for a rude adjustment:
          cli();
          timer0_overflow_count+= 8 * 64 * clockCyclesPerMicrosecond();
          sei();
        }
        #ifdef DEBUG
          Serial.println(F("Sleep complete"));
        #endif
        next = false;
        // Start job
        do_send(&sendjob);
      }

    #endif

  }

  return 0;
}
