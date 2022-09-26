/*
 * Author:          Niklas Menke - https://www.mennik.de/ - CC BY-NC-SA (https://creativecommons.org/licenses/by-nc-sa/4.0/)
 * 
 * Project:         Modbus RTU
 * Description:     This code provides some functions to service a Modbus RTU interface. It use the hardware USART interface.
 * 
 * Supported MCU:   ATtiny 1626
 * 
 * File:            modbus_RTU.h
 * Created:         20.07.2021
 * Needed files:    modbus_RTU.c
 * 
 * Version:         1.1.0
 * Updated:         13.03.2022
 * 
 * Changelog:       1.0.0:
 *                      - First release
 *                  1.0.1:
 *                      - Revised macros
 *                  1.1.0:
 *                      - New table syntax
 *                      - New functions to read/write registers
 *                      - Stop bits and parity can be configured using the coding switches
 * 
 * To-Do:           - Optimizations
 *                  - Correct minor errors
 *                  - Ideas or problems? --> https://www.mennik.de/kontakt/
 * 
 * F_CPU must be set!
 */




 
#ifndef MODBUS_RTU_H
#define	MODBUS_RTU_H

#include <avr/io.h>         // Register macros of the used microcontroller
#include <avr/interrupt.h>  // Routines for interrupts
#include <util/delay.h>     // Delays
#include <avr/eeprom.h>     // Functions to read/write EEPROM
#include "sml.h"


// Check with preprocessor if F_CPU is set
#ifndef F_CPU
#error "F_CPU must be set!"
#endif


#define MODBUS_ADDRESS_MANUFACTURER 8192
#define MODBUS_ADDRESS_DEVICE       8193
#define MODBUS_ADDRESS_TEMPERATURE  8206
#define MODBUS_ADDRESS_POWER        8207
#define MODBUS_ADDRESS_UNIX         8208
#define MODBUS_ADDRESS_RTCCALIB     8210
#define MODBUS_ADDRESS_PIN          8211

#define MODBUS_ADDRESS_WRITEABLE_1   8193
#define MODBUS_ADDRESS_WRITEABLE_2   8207

#define MODBUS_EEPROM_STOPBITS      (uint8_t*)0
#define MODBUS_EEPROM_PARITY        (uint8_t*)1
#define MODBUS_EEPROM_POWER         (uint8_t*)2
#define MODBUS_EEPROM_MANUFACTURER  (uint16_t*)3
#define MODBUS_EEPROM_DEVICE        (uint16_t*)5

#define MODBUS_TABLE_COUNT      11  // Number of register tables


// Struct for a register table
typedef struct
{
    const uint16_t address_first;  // Address offset of the first register of this table
    const uint16_t address_last;  // Number of registers in this table
    uint16_t* storage;      // Stroage for the register entrys -> This registers can read via Modbus
    uint16_t* buffer;       // Buffer for the register entrys -> This registers can't read via Modbus
} MODBUS_REGISTER_TABLE;


#define MODBUS_RX_BUFFER_SIZE   16  // Size of the buffer for received bytes of the smartmeter

#define MODBUS_BUFFER   0
#define MODBUS_STORAGE  1

#ifndef NULL
#define NULL (void *) 0
#endif


void modbus_init(/*const uint8_t coding*/); // Configure USART interface for a RS485 one-wire communication

void modbus_rx_enable(void);                            // Enable Rx mode
void modbus_rx_disable(void);                           // Disable Rx mode
void modbus_rx_get(uint8_t *storage, uint8_t number);   // Get byte(s) from the buffer and delete the byte(s) from the buffer
void modbus_rx_read(uint8_t *storage, uint8_t number);  // Read byte(s) from the buffer without delete them from the buffer
uint8_t modbus_rx_avail(void);                          // Return the number of available bytes in the buffer
void modbus_rx_delete(void);                            // Delete alle bytes from the buffer

void modbus_tx_send(const uint8_t *bytes, uint8_t number);  // Transmit an array of bytes

void modbus_adr_set(void);      // Get the current pin state of the coding switches for the modbus device address
uint8_t modbus_adr_get(void);   // Return the modbus device address

void modbus_baud_set(void); // Get the selected baud rate from the 2x2 pin header and write it to the baud register

uint16_t modbus_crc_calc(const uint8_t *bytes, uint8_t number, const uint16_t start);   // CRC calculation for modbus

uint8_t modbus_frame_avail(void);                                       // Return if a complete modbus frame is available in the modbus buffer
uint8_t modbus_frame_send(const uint8_t *bytes, const uint8_t number);  // Analyse received modbus frame and send a response

uint8_t modbus_table_number(const uint16_t reg_adr, const uint8_t reg_num);

// FC03: Read Holding Registers for internal use
void modbus_fc03_16(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, uint16_t *data);
void modbus_fc03_32(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, uint32_t *data);
void modbus_fc03_64(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, uint64_t *data);

// FC16: Preset Multiple Registers for internal use
void modbus_fc16_16(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, const uint16_t *data);
void modbus_fc16_32(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, const uint32_t *data);
void modbus_fc16_64(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, const uint64_t *data);

void modbus_register_switch(const uint8_t reg_tab); // Toggle array address of buffer and storage. Clear buffer
void modbus_register_clear(const uint8_t reg_tab);  // Clear buffer of each register table

#endif