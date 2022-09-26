/*
 * Author:          Niklas Menke - https://www.mennik.de/ - CC BY-NC-SA (https://creativecommons.org/licenses/by-nc-sa/4.0/)
 * 
 * Project:         Smart meter reading head V2.1.0 / V2.2.0
 * Description:     The reading head gets a SML message from a smart meter.
 *                  The SML message is searched for supported OBIS codes.
 *                  The values of the supported OBIS codes will be saved in registers.
 *                  The registers can be read via a RS485 interface by using the
 *                  Modbus RTU protocol. A regierter table is located in the documentation.
 * 
 * File:            main_functions.h
 * Created:         19.03.2022
 * Needed files:    main_functions.c
 * 
 * Version:         1.0.0
 * Updated:         -
 * 
 * Changelog:       1.0.0:
 *                      - First release
 * 
 * To-Do:           - Ideas? --> https://www.mennik.de/kontakt/
 * 
 * Compiler settings:
 *      Preprocessing:
 *          Macros: F_CPU=2666666UL (OSC: 16MHz, PRE: 6 --> F_CPU: 2666666)
 */





#ifndef MAIN_FUNCTIONS_H
#define	MAIN_FUNCTIONS_H

#include <avr/io.h>         // Register macros of the used microcontroller
#include <avr/interrupt.h>  // Routines for interrupts
#include <util/delay.h>     // Delays
#include <avr/eeprom.h>     // Functions to read/write EEPROM
#include "modbus_RTU.h"

#define RTC_CORRECTION_FACTOR_ADDRESS   (uint8_t*)7 // EEPROM Address for the RTC correction factor

void general_init(void);        // General initialisation of the device
void serial_number_set(void);   // Write the serial number of this device to the modbus register

void temperature_get(void);     // Get chip temperature and write it to the modbus register

void rtc_correction(void);      // Set the RTC correction factor
void smartmeter_pin(void);      // Transmit a PIN to the smartmeter
void modbus_power(void);        // Select which active power will write to the total active power registers
void manufacturer_device(void); // Ckeck if the manufacturer ID or the device ID was changed


#endif

