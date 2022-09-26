/*
 * Author:          Niklas Menke - https://www.mennik.de/ - CC BY-NC-SA (https://creativecommons.org/licenses/by-nc-sa/4.0/)
 * 
 * Project:         SML Interface
 * Description:     This code provides some functions to receive data from a SML interface. The code looks for supported OBIS codes
 *                  and save supported values in a buffer register. If a SML telegram is completely received, the buffer register
 *                  will be write to the original modbus register.
 * 
 * Supported MCU:   ATtiny 1626
 * 
 * File:            sml.h
 * Created:         20.07.2021
 * Needed files:    sml.c, modbus_RTU.h, modbus_RTU.c
 * 
 * Version:         1.1.0
 * Updated:         13.03.2021
 * 
 * Changelog:       1.0.0:
 *                      - First release
 *                  1.0.1:
 *                      - Revised macros
 *                  1.1.0:
 *                      - New OBIS list syntax
 *                      - New register table (with calculation of some values)
 * 
 * To-Do:           - Optimizations
 *                  - Correct minor errors
 *                  - Ideas or problems? --> https://www.mennik.de/kontakt/
 * 
 * F_CPU MUST BE SET!
 */





#ifndef SML_H
#define	SML_H

#include <avr/io.h>         // Register macros of the used microcontroller
#include <avr/interrupt.h>  // Routines for interrupts
#include <stdlib.h>         // Functions for absolute values
#include "modbus_RTU.h"     // Modbus functions


// Check with preprocessor if F_CPU is set
#ifndef F_CPU
#error "F_CPU must be set!"
#endif

#define SUPPORTED_OBIS_CODES_NUMBER 21  // Number of supported obis codes
#define SML_RX_BUFFER_SIZE          32  // Size of the buffer for received bytes of the smartmeter

// Variable type to save information about the supported OBIS codes
typedef struct
{
    uint32_t obis_code;         // OBIS code
    uint16_t register_start;    // ID of the first modbus register
    uint8_t register_number;    // Data type of the obis list entry
    uint8_t register_type;      // Data type of the obis list entry
    int8_t scaler;              // Scaler in the register table
} OBIS_CODES;

extern OBIS_CODES obis_legal;           // See sml.c row 211

// Register types
#define REG_SIGNED      0x50    // Register is signed
#define REG_UNSIGNED    0x60    // Register is unsigned
//#define REG_POS         0x01    // Value is only written to the register if it is positive
//#define REG_NEG         0x02    // Value is only written to the register if it is negative
#define REG_STRING      0x00    // Register is a string array

// Number of regisers for integer types
#define REG_BOOL    0x01
#define REG_INT8    0x01
#define REG_INT16   0x01
#define REG_INT32   0x02
#define REG_INT64   0x04

// Makros for sml_sin_cos(...);
#define ANGLE_SIN   0x00
#define ANGLE_COS   0x01

// Makros for sml_values_calc(...)
#define PHASE_L1    40
#define PHASE_L2    80
#define PHASE_L3    120


void sml_init(void);    // Configure USART interface

void sml_rx_enable(void);                                   // Enable Rx mode
void sml_rx_disable(void);                                  // Disable Rx mode
uint8_t sml_rx_get_byte();                                  // Get byte from the buffer and delete byte from the buffer
uint8_t sml_rx_read_byte();                                 // Read byte from the buffer without delete them from the buffer
void sml_rx_get_bytes(uint8_t *storage, uint8_t number);    // Get bytes from the buffer and delete bytes from the buffer
void sml_rx_read_bytes(uint8_t *storage, uint8_t number);   // Read bytes from the buffer without delete them from the buffer
uint8_t sml_rx_avail(void);                                 // Return the number of available bytes in the buffer
void sml_rx_delete(void);                                   // Delete alle bytes from the buffer

void sml_tx_send(const uint8_t *bytes, uint8_t number); // Transmit an array of bytes

void sml_baud_set(void);    // Get the baud rate from the modbus register and write it to the baud register

uint16_t sml_crc_calc(const uint8_t *bytes, uint8_t number, const uint16_t start);  // CRC calculation for modbus

void sml_analyse(void);                         // Analyse received bytes from the smartmeter
uint8_t sml_analyse_wait(const uint8_t count);  // Wait until enough bytes are received. Return 1 if timeout

void sml_values_calc(const uint8_t phase);                      // Calculate additional values
int16_t sml_sin_cos(const uint8_t type, const uint16_t angle);  // Return value of sin(angle) or cos(angle) with four decimal places (sin(1) = 0,0175 -> 174 -> 0x00ae)

#endif