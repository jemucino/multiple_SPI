/*
  AS5047D.h - Library for reading angle from AS5047D magnetic encoder
  Created by J. Eduardo Mucino on 03 March 2019.
*/
#ifndef AS5047D_h
#define AS5047D_h

#include <Arduino.h>

// Pin definitions for SPI interface
#define ENCODER_SPI_CS 5

// Pin definitions for ABI interface
#define c_EncoderPinA A3
#define c_EncoderPinB A4
#define c_EncoderPinI 3 // PD0 -> INT0

// Define encoder polarity
#define EncoderIsReversed false

// Encoder lsb
#define LSB (360.0/pow(2,14))

class AS5047D {
  public:
    static long _EncoderTurns;

    int raw_angle;
    float measured_angle;

    AS5047D();
    float get_new_measurement();
  private:
    uint16_t receivedVal16;

    static void handle_encoder_rollover();
};

#endif
