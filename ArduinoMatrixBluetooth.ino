/*********************************************************************
 This code is built on top of one of Adafruit's examples. The text originally
 included with the example is below:
 
  | This is an example for our nRF51822 based Bluefruit LE modules
  |
  | Pick one up today in the adafruit shop!
  | 
  | Adafruit invests time and resources providing this open source code,
  | please support Adafruit and open-source hardware by purchasing
  | products from Adafruit!
  |
  | MIT license, check LICENSE for more information
  | All text above, and the splash screen below must be included in
  | any redistribution
*********************************************************************/

/*
 * This first portion deals with configuring the LED matrix
 */
#include "Adafruit_HT1632.h"
// orange wire
#define HT_DATA 2
// green wire
#define HT_WR   3
// white wire
// skip pin 4 because it's a software SPI pin or something 
// and it causes the bluefruit to shit the bed.
#define HT_CS   5

//use only one matrix
Adafruit_HT1632LEDMatrix matrix = Adafruit_HT1632LEDMatrix(HT_DATA, HT_WR, HT_CS);

// include the images
#include "bitmaps.c"

/*
 * This next portion goes into all of the configuration for the Bluefruit LE
 */


#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// some global variables
String lastMessage = "ERROR";

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /*
   * Configure some basic bluetooth settings
   */
  Serial.println( F("Setting power level to 0db"));
  ble.sendCommandCheckOK("AT+BLEPOWERLEVEL=0");
  Serial.println( F("Setting device name") );
  ble.sendCommandCheckOK("AT+GAPDEVNAME=Wireless LED Screen");
  // reset the device so name takes effect.
  ble.sendCommandCheckOK("ATZ");

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  Serial.println(F("Initializing LED Matrix"));
  matrix.begin(ADA_HT1632_COMMON_16NMOS);  
  matrix.fillScreen();
  matrix.clearScreen();

  /* Wait for connection */
  reconnect();

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }
  

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));


  delay(500);
  matrix.clearScreen(); 
  Serial.println(F("LED Matrix ready 2 go"));
  matrix.drawXBitmap(0, 0, bluetooth_bits, default_width, default_height, 1);
  matrix.writeScreen();
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Check for user input
  char n, inputs[BUFSIZE+1];

  if (Serial.available())
  {
    n = Serial.readBytes(inputs, BUFSIZE);
    inputs[n] = 0;
    // Send characters to Bluefruit
    Serial.print("Sending: ");
    Serial.println(inputs);

    // Send input data to host via Bluefruit
    ble.print(inputs);
    Serial.println("Sent");
  }

  if(!ble.isConnected()) {
    reconnect();
  }
  
  if( ble.available() ) {
    String msg = ble.readString();
    msg.trim();
    Serial.print(F("Got: ")); Serial.println(msg);
    matrix.clearScreen();
    if(msg == "PHONE") {
      matrix.drawXBitmap(0, 0, phone_bits, default_width, default_height, 1);
    } else if(msg == "MUSIC") {
      matrix.drawXBitmap(0, 0, music_bits, default_width, default_height, 1);
    } else if(msg == "MUTE") {
      matrix.drawXBitmap(0, 0, mute_bits, default_width, default_height, 1);
    } else if(msg == "UNMUTE") {
      matrix.drawXBitmap(0, 0, unmute_bits, default_width, default_height, 1);
    } else if(msg == "NEWMSG") {
      matrix.drawXBitmap(0, 0, newmsg_bits, default_width, default_height, 1);
    } else if(msg == "WIFI1") {
      matrix.drawXBitmap(0, 0, wifi1_bits, default_width, default_height, 1);
    } else if(msg == "WIFI2") {
      matrix.drawXBitmap(0, 0, wifi2_bits, default_width, default_height, 1);
    } else if(msg == "WIFI3") {
      matrix.drawXBitmap(0, 0, wifi3_bits, default_width, default_height, 1);
    } else if(msg == "BLUETOOTH") {
      matrix.drawXBitmap(0, 0, bluetooth_bits, default_width, default_height, 1);
    } else {
      matrix.drawXBitmap(0,0, error_bits, error_width, error_height, 1);
    }
    matrix.writeScreen();
    Serial.print(F("Got: ")); Serial.println(msg);
  }
}

void reconnect() {
  int i = 0;
  while(!ble.isConnected()) {
    matrix.clearScreen();
    if((i % 3) == 0) { 
      matrix.drawXBitmap(0, 0, wifi1_bits, wifi1_width, wifi1_height, 1);
    } else if((i % 3) == 1) {
      matrix.drawXBitmap(0, 0, wifi2_bits, wifi2_width, wifi2_height, 1);
    } else {
      matrix.drawXBitmap(0, 0, wifi3_bits, wifi3_width, wifi3_height, 1);
    }
    matrix.writeScreen();
    i++;
    delay(500);
  }
  matrix.clearScreen();
  matrix.drawXBitmap(0, 0, bluetooth_bits, default_width, default_height, 1);
  matrix.writeScreen();
}


