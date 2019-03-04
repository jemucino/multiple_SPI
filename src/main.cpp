#include <string.h>

#include <Arduino.h>
#include <SPI.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

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
// -----------------------------------------------------------------------------
// Create the bluefruit object
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
int parseint(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
// -----------------------------------------------------------------------------

#define ENCODER_SPI_CS 5

// Encoder declarations for ABI interface
#define c_EncoderPinA A3
#define c_EncoderPinB A4
#define c_EncoderPinI 3 // PD0 -> INT0
#define EncoderIsReversed false
volatile long _EncoderTicks = 0;

// Helper variable
unsigned long now;

// Define encoder variables
uint16_t receivedVal16;
int raw_angle;
float measured_angle;
float lsb=360.0/pow(2,14);

// Function declarations
void handle_encoder_rollover();

void setup() {
  // Initialize encoder
  pinMode(c_EncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_EncoderPinB, INPUT_PULLUP);      // sets pin B as input
  attachInterrupt(0, handle_encoder_rollover, FALLING);

  // Setup SPI communications with AS5047D
  pinMode(ENCODER_SPI_CS, OUTPUT);
  digitalWrite(ENCODER_SPI_CS, HIGH); // Disconnect AS5047D from SPI bus
  // SPI.begin();

// -----------------------------------------------------------------------------
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

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

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  Serial.print("Waiting for connection");
  while (! ble.isConnected()) {
      delay(500);
      Serial.print(".");
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
// -----------------------------------------------------------------------------

// initialize helper variable
now = millis();
}

void loop() {
// -----------------------------------------------------------------------------
  /* Wait for new data to arrive */
  // Serial.println(millis()-now);
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  // Serial.println(millis()-now);
  if (len != 0) {
    /* Got a packet! */
    // printHex(packetbuffer, len);

    // Buttons
    if (packetbuffer[1] == 'B') {
      uint8_t buttnum = packetbuffer[2] - '0';
      boolean pressed = packetbuffer[3] - '0';
      Serial.print ("Button "); Serial.print(buttnum);
      if (pressed) {
        Serial.println(" pressed");
      } else {
        Serial.println(" released");
      }
    }

    // SeekBar
    if (packetbuffer[1] == 'S') {
    int progress = parseint(packetbuffer+2);
    Serial.print ("SeekBar "); Serial.println(progress);
    }
  }
  // Serial.println(millis()-now);
//------------------------------------------------------------------------------

  // Perform SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ENCODER_SPI_CS, LOW);
  receivedVal16 = SPI.transfer16(0xFFFF);
  digitalWrite(ENCODER_SPI_CS, HIGH);
  SPI.endTransaction();
  // Serial.println(millis()-now);

  // Convert counts to degrees
  raw_angle = int (receivedVal16 & 0x3FFF);
  measured_angle = (_EncoderTicks*360.0 + raw_angle*lsb)/32.0;

  // Print value to screen
  Serial.println(measured_angle);

  // Wait before next loop
  while (millis()-now < 20);

  Serial.println(millis()-now);
  now = millis();
}

// Interrupt service routines for the encoder
void handle_encoder_rollover()
{
  // Increment or decrement rollover counter
  #ifdef EncoderIsReversed
    _EncoderTicks -= digitalReadFast(c_EncoderPinA) > digitalReadFast(c_EncoderPinB) ? -1 : +1;
  #else
    _EncoderTicks += digitalReadFast(c_EncoderPinA) > digitalReadFast(c_EncoderPinB) ? -1 : +1;
  #endif
}
