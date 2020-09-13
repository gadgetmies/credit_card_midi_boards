/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// Workaround for wrong definitions in "iom32u4.h".
// This should be fixed in the AVR toolchain.
#undef UHCON
#undef UHINT
#undef UHIEN
#undef UHADDR
#undef UHFNUM
#undef UHFNUML
#undef UHFNUMH
#undef UHFLEN
#undef UPINRQX
#undef UPINTX
#undef UPNUM
#undef UPRST
#undef UPCONX
#undef UPCFG0X
#undef UPCFG1X
#undef UPSTAX
#undef UPCFG2X
#undef UPIENX
#undef UPDATX
#undef TCCR2A
#undef WGM20
#undef WGM21
#undef COM2B0
#undef COM2B1
#undef COM2A0
#undef COM2A1
#undef TCCR2B
#undef CS20
#undef CS21
#undef CS22
#undef WGM22
#undef FOC2B
#undef FOC2A
#undef TCNT2
#undef TCNT2_0
#undef TCNT2_1
#undef TCNT2_2
#undef TCNT2_3
#undef TCNT2_4
#undef TCNT2_5
#undef TCNT2_6
#undef TCNT2_7
#undef OCR2A
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7
#undef OCR2B
#undef OCR2_0
#undef OCR2_1
#undef OCR2_2
#undef OCR2_3
#undef OCR2_4
#undef OCR2_5
#undef OCR2_6
#undef OCR2_7

#define NUM_DIGITAL_PINS  44
#define NUM_ANALOG_INPUTS 12

/* TODO: no leds on board, does 0 here work?
   These are used in USBCore.cpp and need to be defined */
#define TX_RX_LED_INIT		0
#define TXLED0			0
#define TXLED1			0
#define RXLED0			0
#define RXLED1			0

#define PIN_WIRE_SDA         (19)
#define PIN_WIRE_SCL         (18)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define LED_BUILTIN 32

/* TODO: no leds on board and constants not in use -> can these be removed?
#define LED_BUILTIN_RX 17
#define LED_BUILTIN_TX 30
*/

#define PIN_SPI_SS    (8)
#define PIN_SPI_MOSI  (10)
#define PIN_SPI_MISO  (11)
#define PIN_SPI_SCK   (9)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// Mapping of analog pins as digital I/O
// A6-A11 share with digital pins
#define PIN_A0   (41)
#define PIN_A1   (40)
/* ACD2 and ACD3 are used for differential input? */
// #define PIN_A2   ()
// #define PIN_A3   ()
#define PIN_A4   (39)
#define PIN_A5   (38)
#define PIN_A6   (37)
#define PIN_A7   (36)
#define PIN_A8   (25)
#define PIN_A9   (26)
#define PIN_A10  (27)
#define PIN_A11  (28)
#define PIN_A12  (29)
#define PIN_A13  (30)

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
// static const uint8_t A2 = PIN_A2;
// static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
static const uint8_t A6 = PIN_A6;
static const uint8_t A7 = PIN_A7;
static const uint8_t A8 = PIN_A8;
static const uint8_t A9 = PIN_A9;
static const uint8_t A10 = PIN_A10;
static const uint8_t A11 = PIN_A11;
static const uint8_t A12 = PIN_A12;
static const uint8_t A13 = PIN_A13;

/* TODO: what are these? How should these be set? */
#define digitalPinToPCICR(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    ((((p) >= 8 && (p) <= 11) || ((p) >= 14 && (p) <= 17) || ((p) >= A8 && (p) <= A10)) ? (&PCMSK0) : ((uint8_t *)0))
#define digitalPinToPCMSKbit(p) ( ((p) >= 8 && (p) <= 11) ? (p) - 4 : ((p) == 14 ? 3 : ((p) == 15 ? 1 : ((p) == 16 ? 2 : ((p) == 17 ? 0 : (p - A8 + 4))))))

//	__AVR_ATmega32U4__ has an unusual mapping of pins to channels
extern const uint8_t PROGMEM analog_pin_to_channel_PGM[];
#define analogPinToChannel(P)  ( pgm_read_byte( analog_pin_to_channel_PGM + (P) ) )

#define digitalPinHasPWM(p)         ((p) == 12 || (p) == 18 || (p) == 26 || (p) == 27 || (p) == 29 || (p) == 30 || (p) == 31 || (p) == 32)

#define digitalPinToInterrupt(p) ((p) == 18 ? 0 : ((p) == 19 ? 1 : ((p) == 20 ? 2 : ((p) == 21 ? 3 : NOT_AN_INTERRUPT))))

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
};

// const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
// 	PB, // D0 - PB0 / SS	/ PCINT0					// LED_LATCH
// 	PB, // D1 - PB1 / SCKL	/ PCINT1					// LED_CLK
// 	PB, // D2 - PB2 / MOSI	/ PCINT2	/ PDI				// LED_DATA
// 	PB, // D3 - PB3 / MISO	/ PCINT3	/ PDO				// STROBE
// 	PB, // D4 - PB4 / ADC11	/ PCINT11					// CROSSPOINT_DATA
// 	PB, // D5 - PB5 / ADC12	/ PCINT5 	/ OC1A	/ O!C4B	/ PWM		// !OE
// 	PB, // D6 - PB6 / ADC13	/ PCINT6	/ OC1B	/ OC4B	/ PWM		// STCP
// 	PB, // D7 - PB7 / RTS	/ PCINT7	/ OC1C	/ OC0A	/ PWM

// 	PC, // D8 - PC6 / OC3A	/ O!C4A				/ PWM		// SHCP
// 	PC, // D9 - PC7 / ICP3	/ CLK0				/ PWM		// !MR

// 	PD, // D10 - PD0 / SCL	/ OC0B 		/ INT0		/ PWM		// !INT3
// 	PD, // D11 - PD1 / SDA			/ INT1				// !INT2
// 	PD, // D12 - PD2 / RXD1			/ INT2				// !INT1
// 	PD, // D13 - PD3 / TXD1			/ INT3				// !E3
// 	PD, // D14 - PD4 / ADC8	/ ICP1						// !E2
// 	PD, // D15 - PD5 / CTS	/ XCK1						// !E1
// 	PD, // D16 - PD6 / ADC9 / T1		/ O!C4D		/ PWM
// 	PD, // D17 - PD7 / ADC10/ T0		/ OC4D		/ PWM

// 	PE, // D18 - PE2 / !HWB
// 	PE, // D19 - PE6 / AIN0	/ INT6

// 	PF, // D20 - PF0 / ADC0							// A
// 	PF, // D21 - PF1 / ADC1							// B
// 	PF, // D22 - PF4 / ADC4	/ TCK						// C
// 	PF, // D23 - PF5 / ADC5 / TMS						// OUT3
// 	PF, // D24 - PF6 / ADC6 / TDO						// OUT2
// 	PF, // D25 - PF7 / ADC7 / TDI						// OUT1
// };

// DIRECT MAPPING
const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	NOT_A_PIN, // no such pin (0)
	PE, // pin 1 - PE6
	NOT_A_PIN, // pin 2 - UVcc
	NOT_A_PIN, // pin 3 - D-
	NOT_A_PIN, // pin 4 - D+
	NOT_A_PIN, // pin 5 - UGnd
	NOT_A_PIN, // pin 6 - UCap
	NOT_A_PIN, // pin 7 - VBus
	PB, // pin 8 - PB0
	PB, // pin 9 - PB1
	PB, // pin 10 - PB2
	PB, // pin 11 - PB3
	PB, // pin 12 - PB7
	NOT_A_PIN, // pin 13 - /RESET
	NOT_A_PIN, // pin 14 - VCC
	NOT_A_PIN, // pin 15 - GND
	NOT_A_PIN, // pin 16 - XTAL2
	NOT_A_PIN, // pin 17 - XTAL1
	PD, // pin 18 - PD0
	PD, // pin 19 - PD1
	PD, // pin 20 - PD2
	PD, // pin 21 - PD3
	PD, // pin 22 - PD5
	NOT_A_PIN, // pin 23 - GND
	NOT_A_PIN, // pin 24 - AVCC
	PD, // pin 25 - PD4
	PD, // pin 26 - PD6
	PD, // pin 27 - PD7
	PB, // pin 28 - PB4
	PB, // pin 29 - PB5
	PB, // pin 30 - PB6
	PC, // pin 31 - PC6
	PC, // pin 32 - PC7
	PE, // pin 33 - PE2
	NOT_A_PIN, // pin 34 - VCC
	NOT_A_PIN, // pin 35 - GND
	PF, // pin 36 - PF7
	PF, // pin 37 - PF6
	PF, // pin 38 - PF5
	PF, // pin 39 - PF4
	PF, // pin 40 - PF1
	PF, // pin 41 - PF0
	NOT_A_PIN, // pin 42 - AREF
	NOT_A_PIN, // pin 43 - GND
	NOT_A_PIN, // pin 44 - AVCC
};

// const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
//         _BV(0), // D0 - PB0
//         _BV(1), // D1 - PB1
//         _BV(2), // D2 - PB2
//         _BV(3), // D3 - PB3
//         _BV(4), // D4 - PB4
//         _BV(5), // D5 - PB5
//         _BV(6), // D6 - PB6
//         _BV(7), // D7 - PB7

//         _BV(6), // D8 - PC6
//         _BV(7), // D9 - PC7

//         _BV(0), // D10 - PD0
//         _BV(1), // D11 - PD1
//         _BV(2), // D12 - PD2
//         _BV(3), // D13 - PD3
//         _BV(4), // D14 - PD4
//         _BV(5), // D15 - PD5
//         _BV(6), // D16 - PD6
//         _BV(7), // D17 - PD7

//         _BV(2), // D18 - PE2
//         _BV(6), // D19 - PE6

//         _BV(0), // D20 - PF0
//         _BV(1), // D21 - PF1
//         _BV(4), // D22 - PF4
//         _BV(5), // D23 - PF5
//         _BV(6), // D24 - PF6
//         _BV(7), // D25 - PF7
// };

// DIRECT MAPPING
const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	0, // no such pin (0)
	_BV(6), // pin 1 - PE6
	0, // pin 2 - UVcc
	0, // pin 3 - D-
	0, // pin 4 - D+
	0, // pin 5 - UGnd
	0, // pin 6 - UCap
	0, // pin 7 - VBus
	_BV(0), // pin 8 - PB0
	_BV(1), // pin 9 - PB1
	_BV(2), // pin 10 - PB2
	_BV(3), // pin 11 - PB3
	_BV(7), // pin 12 - PB7
	0, // pin 13 - /RESET
	0, // pin 14 - VCC
	0, // pin 15 - GND
	0, // pin 16 - XTAL2
	0, // pin 17 - XTAL1
	_BV(0), // pin 18 - PD0
	_BV(6), // pin 19 - PD1
	_BV(2), // pin 20 - PD2
	_BV(3), // pin 21 - PD3
	_BV(5), // pin 22 - PD5
	0, // pin 23 - GND
	0, // pin 24 - AVCC
	_BV(4), // pin 25 - PD4
	_BV(6), // pin 26 - PD6
	_BV(7), // pin 27 - PD7
	_BV(4), // pin 28 - PB4
	_BV(5), // pin 29 - PB5
	_BV(6), // pin 30 - PB6
	_BV(6), // pin 31 - PC6
	_BV(7), // pin 32 - PC7
	_BV(2), // pin 33 - PE2
	0, // pin 34 - VCC
	0, // pin 35 - GND
	_BV(7), // pin 36 - PF7
	_BV(6), // pin 37 - PF6
	_BV(5), // pin 38 - PF5
	_BV(4), // pin 39 - PF4
	_BV(1), // pin 40 - PF1
	_BV(0), // pin 41 - PF0
	0, // pin 42 - AREF
	0, // pin 43 - GND
	0, // pin 44 - AVCC
};

//TODO
const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER, // no such pin (0)
	NOT_ON_TIMER, // // pin 1 - PE6
	NOT_ON_TIMER, // pin 2 - UVcc
	NOT_ON_TIMER, // pin 3 - D-
	NOT_ON_TIMER, // pin 4 - D+
	NOT_ON_TIMER, // pin 5 - UGnd
	NOT_ON_TIMER, // pin 6 - UCap
	NOT_ON_TIMER, // pin 7 - VBus
	NOT_ON_TIMER, // // pin 8 - PB0
	NOT_ON_TIMER, // // pin 9 - PB1
	NOT_ON_TIMER, // // pin 10 - PB2
	NOT_ON_TIMER, // // pin 11 - PB3
	NOT_ON_TIMER, // // pin 12 - PB7
	NOT_ON_TIMER, // pin 13 - /RESET
	NOT_ON_TIMER, // pin 14 - VCC
	NOT_ON_TIMER, // pin 15 - GND
	NOT_ON_TIMER, // pin 16 - XTAL2
	NOT_ON_TIMER, // pin 17 - XTAL1
	NOT_ON_TIMER, // // pin 18 - PD0
	NOT_ON_TIMER, // // pin 19 - PD1
	NOT_ON_TIMER, // // pin 20 - PD2
	NOT_ON_TIMER, // // pin 21 - PD3
	NOT_ON_TIMER, // // pin 22 - PD5
	NOT_ON_TIMER, // pin 23 - GND
	NOT_ON_TIMER, // pin 24 - AVCC
	NOT_ON_TIMER, // // pin 25 - PD4
	NOT_ON_TIMER, // // pin 26 - PD6
	NOT_ON_TIMER, // // pin 27 - PD7
	NOT_ON_TIMER, // // pin 28 - PB4
	NOT_ON_TIMER, // // pin 29 - PB5
	NOT_ON_TIMER, // // pin 30 - PB6
	NOT_ON_TIMER, // // pin 31 - PC6
	NOT_ON_TIMER, // // pin 32 - PC7
	NOT_ON_TIMER, // // pin 33 - PE2
	NOT_ON_TIMER, // pin 34 - VCC
	NOT_ON_TIMER, // pin 35 - GND
	NOT_ON_TIMER, // // pin 36 - PF7
	NOT_ON_TIMER, // // pin 37 - PF6
	NOT_ON_TIMER, // // pin 38 - PF5
	NOT_ON_TIMER, // // pin 39 - PF4
	NOT_ON_TIMER, // // pin 40 - PF1
	NOT_ON_TIMER, // // pin 41 - PF0
	NOT_ON_TIMER, // pin 42 - AREF
	NOT_ON_TIMER, // pin 43 - GND
	NOT_ON_TIMER, // pin 44 - AVCC
};

// direct mapping in analog ports, comments are from arduino
const uint8_t PROGMEM analog_pin_to_channel_PGM[] = {
        0,
        1,
        255, // ADC2 & ADC3 do not exist in the microcontroller
        255,
        4,
        5,
        6,
        7,
        8,
        9,
        10,
        11,
        12,
        13
};

#endif /* ARDUINO_MAIN */

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR        Serial
#define SERIAL_PORT_USBVIRTUAL     Serial
#define SERIAL_PORT_HARDWARE       Serial1
#define SERIAL_PORT_HARDWARE_OPEN  Serial1

// Alias SerialUSB to Serial
#define SerialUSB SERIAL_PORT_USBVIRTUAL

/* Mixer functionality */
// Shared outputs
static const uint8_t CM_MIXER_X1 = 37; // One segment of EQs, left segment of xfader
static const uint8_t CM_MIXER_X2 = 36; // One segment of EQs, right segment of xfader
static const uint8_t CM_MIXER_X3 = 38; // One segment of EQs, middle segment of xfader

// Analog inputs
static const uint8_t CM_MIXER_HP_CUE_L = 4; // ADC4 Y1
static const uint8_t CM_MIXER_HP_CUE_R = 1; // ADC1 Y2
static const uint8_t CM_MIXER_EQ_HIGH_L = 0; // ADC0 Y3
static const uint8_t CM_MIXER_EQ_HIGH_R = 8; // ADC8 Y4
static const uint8_t CM_MIXER_EQ_LOW_R = 10; // ADC10 Y5
static const uint8_t CM_MIXER_XFADER = 11; // ADC11 Y6
static const uint8_t CM_MIXER_EQ_LOW_L = 12; // ADC12 Y7
static const uint8_t CM_MIXER_EQ_MID_L = 13; // ADC13 Y8
static const uint8_t CM_MIXER_EQ_MID_R = 9; // ADC9 Y9

static const uint8_t CM_MIXER_LED_HP_CUE_L = 33;
static const uint8_t CM_MIXER_LED_HP_CUE_R = 32;

#endif /* Pins_Arduino_h */
