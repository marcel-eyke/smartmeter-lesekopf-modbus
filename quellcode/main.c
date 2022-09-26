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
 * File:            main.c
 * Created:         20.07.2021
 * Needed files:    modbus_RTU.h, modbus_RTU.c, sml.h, sml.c
 * 
 * Version:         1.1.0
 * Updated:         13.03.2022
 * 
 * Changelog:       1.0.0:
 *                      - First release
 *                  1.1.0:
 *                      - Added temperature measurement
 *                      - Support new modbus_RTU
 * 
 * To-Do:           - Clean up
 *                  - Bug fixing
 *                  - Ideas? --> https://www.mennik.de/kontakt/
 * 
 * Compiler settings:
 *      Preprocessing:
 *          Macros: F_CPU=2666666UL (OSC: 16MHz, PRE: 6 --> F_CPU: 2666666)
 */





// ********** GLOBAL CONFIGURATION BEGIN ********** //

// Include needed files
#include <avr/io.h>         // Register macros of the used microcontroller
#include <avr/interrupt.h>  // Routines for interrupts
#include <util/delay.h>     // Delays
#include "main_functions.h"
#include "modbus_RTU.h"
#include "sml.h"

// Set fuse bits
FUSES = {
	.WDTCFG = 0x00, // WDTCFG {PERIOD=OFF, WINDOW=OFF}
	.BODCFG = 0x00, // BODCFG {SLEEP=DIS, ACTIVE=DIS, SAMPFREQ=1KHZ, LVL=BODLEVEL0}
	.OSCCFG = 0x7D, // OSCCFG {FREQSEL=16MHZ, OSCLOCK=CLEAR}
	.SYSCFG0 = 0xF6, // SYSCFG0 {EESAVE=CLEAR, RSTPINCFG=UPDI, TOUTDIS=SET, CRCSRC=NOCRC}
	.SYSCFG1 = 0xFF, // SYSCFG1 {SUT=64MS}
	.APPEND = 0x00, // APPEND {APPEND=User range:  0x0 - 0xFF}
	.BOOTEND = 0x00, // BOOTEND {BOOTEND=User range:  0x0 - 0xFF}
};
LOCKBITS = 0xC5; // {LB=NOLOCK}

// ********** GLOBAL CONFIGURATION END ********** //





// ********** MAIN FUNCTION BEGIN ********** //

int main(void) {
    general_init();         // Configure IO-Pins, RTC and EEPROM
    serial_number_set();    // Write the serial number of this device to the modbus register
    
    modbus_init();      // Initialize USART0 interface for the RS485/Modbus RTU
    modbus_rx_enable(); // Enable modbus receiver
    
    sml_init();         // Initialize USART1 interface for the communication with the smartmeter
    sml_rx_enable();    // Enable SML receiver
    
    // Show that the device is ready
    for(uint8_t i = 0; i < 6; i++) {
        PORTA.OUTTGL = PIN1_bm | PIN2_bm;
        _delay_ms(500);
    }

    sei();  // Enable interrupts globally
    
    while(1) {
        // A modbus frame was received
        if(modbus_frame_avail()) {
            temperature_get();                              // Get current temperature
            
            uint8_t frame_number = modbus_rx_avail();       // Number of available bytes in the modbus buffer
            uint8_t frame_bytes[frame_number];              // Storage for the received bytes of the modbus frame
            modbus_rx_get(frame_bytes, frame_number);       // Get the bytes from the modbus buffer
            
            sml_rx_disable();                               // Disable serial communication with the smartmeter
            modbus_frame_send(frame_bytes, frame_number);   // Analyse received modbus frame and send a response
            sml_rx_enable();                                // Enable serial communication with the smartmeter

            smartmeter_pin();                               // Check if the PIN of the smartmeter was written to the modbus register
            rtc_correction();                               // Check if RTC correction factor was changed inside the modbus register
            modbus_power();                                 // Check if the selection of the active power was changed
            manufacturer_device();                          // Ckeck if the manufacturer ID or the device ID was changed
        }
        
        // A byte from a smartmeter was received
        if(sml_rx_avail()) sml_analyse();
    }
}

// ********** MAIN FUNCTION END ********** //