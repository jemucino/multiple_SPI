/*
  AS5047D.h - Library for reading angle from AS5047D magnetic encoder
  Created by J. Eduardo Mucino on 03 March 2019.
*/

#include "AS5047D.h"

#include <SPI.h>

#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/


// Initialize variables
long AS5047D::_EncoderTurns = 0;

AS5047D::AS5047D() {
  // Initialize encoder
  pinMode(c_EncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_EncoderPinB, INPUT_PULLUP);      // sets pin B as input
  attachInterrupt(0, handle_encoder_rollover, FALLING);

  // Setup SPI communications with AS5047D
  pinMode(ENCODER_SPI_CS, OUTPUT);
  digitalWrite(ENCODER_SPI_CS, HIGH); // Disconnect AS5047D from SPI bus
  SPI.begin();

  // Get initial measurement
  get_new_measurement();
}

float AS5047D::get_new_measurement() {
  // Perform SPI transaction
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
  digitalWrite(ENCODER_SPI_CS, LOW);
  receivedVal16 = SPI.transfer16(0xFFFF);
  digitalWrite(ENCODER_SPI_CS, HIGH);
  SPI.endTransaction();

  // Convert turns and counts to degrees
  raw_angle = int (receivedVal16 & 0x3FFF);
  measured_angle = (_EncoderTurns*360.0 + raw_angle*LSB)/32.0;

  return measured_angle;
}

// Interrupt service routines for the encoder
void AS5047D::handle_encoder_rollover()
{
  // Increment or decrement rollover counter
  #ifdef EncoderIsReversed
    _EncoderTurns -= digitalReadFast(c_EncoderPinA) > digitalReadFast(c_EncoderPinB) ? -1 : +1;
  #else
    _EncoderTurns += digitalReadFast(c_EncoderPinA) > digitalReadFast(c_EncoderPinB) ? -1 : +1;
  #endif
}
