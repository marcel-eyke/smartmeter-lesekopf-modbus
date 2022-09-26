/*
 * Author:          Niklas Menke - https://www.mennik.de/ - CC BY-NC-SA (https://creativecommons.org/licenses/by-nc-sa/4.0/)
 * 
 * File:            main_functions.c
 * Needed files:    main_functions.h
 * 
 * MORE INFORMATION INSIDE THE HEADER FILE
 */





// ********** GLOBAL CCONFIGURATION BEGIN ********** //

#include "main_functions.h"

// ********** GLOBAL CCONFIGURATION END ********** //





// ********** FUNCTION DEFINITION BEGIN ********** //

// General initialisation of the device
void general_init(void) {
    // Check if stop bits and parity settings in the EEPROM are legal
    uint8_t stop_bits = eeprom_read_byte(MODBUS_EEPROM_STOPBITS);
    if((stop_bits != USART_SBMODE_1BIT_gc) && (stop_bits != USART_SBMODE_2BIT_gc)) {
        //eeprom_write_byte(MODBUS_EEPROM_STOPBITS, USART_SBMODE_1BIT_gc);   // DEFAULT STOPP BITS: 1
        eeprom_write_byte(MODBUS_EEPROM_STOPBITS, USART_SBMODE_2BIT_gc);   // DEFAULT STOPP BITS: 2
    }
    uint8_t parity = eeprom_read_byte(MODBUS_EEPROM_PARITY);
    if((parity != USART_PMODE_DISABLED_gc) && (parity != USART_PMODE_EVEN_gc) && (parity != USART_PMODE_ODD_gc)) {
        eeprom_write_byte(MODBUS_EEPROM_PARITY, USART_PMODE_DISABLED_gc);    // DEFAULT PARITY: NONE
        //eeprom_write_byte(MODBUS_EEPROM_PARITY, USART_PMODE_EVEN_gc);        // DEFAULT PARITY: EVEN
        //eeprom_write_byte(MODBUS_EEPROM_PARITY, USART_PMODE_ODD_gc);         // DEFAULT PARITY: ODD
    }
    
    // Write values from EEPROM to the modbus register
    uint16_t val = (uint16_t) eeprom_read_byte(MODBUS_EEPROM_POWER);
    if(val < 2) modbus_fc16_16(MODBUS_STORAGE, MODBUS_ADDRESS_POWER, 1, &val);
    val = eeprom_read_word(MODBUS_EEPROM_MANUFACTURER);
    modbus_fc16_16(MODBUS_STORAGE, MODBUS_ADDRESS_MANUFACTURER, 1, &val);
    val = eeprom_read_word(MODBUS_EEPROM_DEVICE);
    modbus_fc16_16(MODBUS_STORAGE, MODBUS_ADDRESS_DEVICE, 1, &val);
    val = eeprom_read_byte(RTC_CORRECTION_FACTOR_ADDRESS);
    modbus_fc16_16(MODBUS_STORAGE, MODBUS_ADDRESS_RTCCALIB, 1, &val);
    
    // Set LED and unused PA3 pin as output
    PORTA.DIR = PIN1_bm | PIN2_bm | PIN3_bm;
    
    // RTC
    RTC.CTRLA = RTC_RUNSTDBY_bm | RTC_PRESCALER_DIV1024_gc | RTC_RTCEN_bm;  // RTC is enabled in standby, Presacler is 1024 and RTC is enabled
    //RTC.OVF = UINT16_MAX;                                                   
    RTC.INTCTRL = RTC_OVF_bm;                                               // Enable overflow interrupt
    RTC.INTFLAGS = 0xff;                                                    // Clear all interrupt flags of the RTC
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;                                       // Select internal 1024 kHz oscillator as clock
    RTC.CNT = 0;                                                            // Clear RTC counter
    rtc_correction();
}



// Write the serial number of this device to the modbus register
void serial_number_set(void) {
    uint16_t ser[5] = {
        ((uint16_t) SIGROW.SERNUM9<<8) | SIGROW.SERNUM8,
        ((uint16_t) SIGROW.SERNUM7<<8) | SIGROW.SERNUM6,
        ((uint16_t) SIGROW.SERNUM5<<8) | SIGROW.SERNUM4,
        ((uint16_t) SIGROW.SERNUM3<<8) | SIGROW.SERNUM2,
        ((uint16_t) SIGROW.SERNUM1<<8) | SIGROW.SERNUM0,
    };
    
    for(uint8_t i = 0; i < 5; i++) modbus_fc16_16(MODBUS_STORAGE, 8196+i, 1, &ser[i]);
}



void temperature_get(void) {
    ADC0.CTRLC = ADC_REFSEL_1024MV_gc;      // Configure the voltage reference to internal 1.024V
    ADC0.MUXPOS = ADC_MUXPOS_TEMPSENSE_gc;  // Select the temperature sensor as input
    ADC0.CTRLE = 0x28;                      // Configure the ADC Sample Duration
    //ADC0.INTFLAGS = 0xff;                   // Clear interrupt flags
    //ADC0.INTCTRL = ADC_RESRDY_bm;           // Eneable result ready interrupt
    
    ADC0.CTRLA = ADC_ENABLE_bm;                                         // Enable ADC
    ADC0.COMMAND = ADC_MODE_SINGLE_12BIT_gc | ADC_START_IMMEDIATE_gc;   // Running a 12-bit Single-Ended conversion
    
    //ADC0.INTFLAGS = 0xff;           // Clear interrupt flags
    while(ADC0.STATUS&ADC_ADCBUSY_bm);  // Wait until conversion is completed
    ADC0.CTRLA &= ~ADC_ENABLE_bm;       // Disable ADC
    
    int8_t sigrow_offset = SIGROW.TEMPSENSE1;       // Read signed offset from signature row
    uint8_t sigrow_gain = SIGROW.TEMPSENSE0;        // Read unsigned gain/slope from signature row
    uint16_t adc_reading = ADC0.RESULT >> 2;        // 10-bit MSb of ADC result with 1.024V internal reference
    uint32_t temp = adc_reading - sigrow_offset;
    temp *= sigrow_gain;                            // Result might overflow 16-bit variable (10-bit + 8-bit)
    temp += 0x80;                                   // Add 256/2 to get correct integer rounding on division below
    temp >>= 8;                                     // Divide result by 256 to get processed temperature in Kelvin
    uint16_t temperature_in_K = temp;
    
    modbus_fc16_16(MODBUS_STORAGE, MODBUS_ADDRESS_TEMPERATURE, 1, &temperature_in_K);   // Write temperature to the modbus register
}



// Set the RTC correction factor
void rtc_correction(void) {
    //static uint16_t rtc_correction = 0xff;    // Current value for the RTC correction inside the modbus register
    uint16_t val;
    modbus_fc03_16(MODBUS_STORAGE, MODBUS_ADDRESS_RTCCALIB, 1, &val);
    
    if(val != eeprom_read_byte(RTC_CORRECTION_FACTOR_ADDRESS)) {    
        if(val&0x8000) {                    // Negative value
            val = (val ^ UINT16_MAX) + 1;   // Remove sign
            val |= 0x80;                    // Set last bit to one to show that the value is negative
        }
        else {              // Positive value
            val &= ~0x80;   // Set last bit to zero to show that the value is positive
        }
                
        eeprom_write_byte(RTC_CORRECTION_FACTOR_ADDRESS, (uint8_t) val); // Save correction factor to EEPROM
        modbus_fc16_16(MODBUS_STORAGE, MODBUS_ADDRESS_RTCCALIB, 1, &val);
        RTC.CALIB = (uint8_t) val;                                       // Set correction factor
        if(val) RTC.CTRLA |= RTC_CORREN_bm;
        else RTC.CTRLA &= ~RTC_CORREN_bm;
    }
}



// Transmit a PIN to the smartmeter
void smartmeter_pin(void) {
    uint16_t pin;
    
    modbus_fc03_16(MODBUS_STORAGE, MODBUS_ADDRESS_PIN, 1, &pin);
    if(pin) {
        PORTA.OUTSET = PIN1_bm | PIN2_bm;   // Turn on both status LEDs

        // Disable both receiver
        modbus_rx_disable();
        sml_rx_disable();

        // Separate all numbers of the pin
        uint8_t pin_numbers[4];
        for(int8_t i = 3; i>-1; i--) {
            pin_numbers[i] = pin%10;
            pin /= 10;
        }

        // Send each number to the smart meter
        PORTC.OUTCLR = PIN2_bm;
        _delay_ms(500);
        PORTC.OUTSET = PIN2_bm;
        _delay_ms(500);
        PORTC.OUTCLR = PIN2_bm;
        _delay_ms(500);
        PORTC.OUTSET = PIN2_bm;
        _delay_ms(1000);
        for(uint8_t i = 0; i<4; i++) {
            for(uint8_t ii = 0; ii<pin_numbers[i]; ii++) {
                PORTC.OUTCLR = PIN2_bm;
                    _delay_ms(500);
                    PORTC.OUTSET = PIN2_bm;
                    _delay_ms(500);
            }
            _delay_ms(3500);
        }

        // Enable both receiver
        modbus_rx_enable();
        sml_rx_enable();

        PORTA.OUTCLR = PIN1_bm | PIN2_bm;   // Turn off both status LEDs
        
        pin = 0;
        modbus_fc16_16(MODBUS_STORAGE, MODBUS_ADDRESS_PIN, 1, &pin);  // Delete the PIN from the modbus register
    }
}



// Select which active power will write to the total active power registers
void modbus_power(void) {
    uint16_t val;
    
    modbus_fc03_16(MODBUS_STORAGE, MODBUS_ADDRESS_POWER, 1, &val);
    
    if(eeprom_read_byte(MODBUS_EEPROM_POWER) != !!val) eeprom_write_byte(MODBUS_EEPROM_POWER, (uint8_t) !!val);
}



// Ckeck if the manufacturer ID or the device ID was changed
void manufacturer_device(void) {
    uint16_t val;
    
    modbus_fc03_16(MODBUS_STORAGE, MODBUS_ADDRESS_MANUFACTURER, 1, &val);
    if(eeprom_read_word(MODBUS_EEPROM_MANUFACTURER) != val) eeprom_write_word(MODBUS_EEPROM_MANUFACTURER, val);
    
    modbus_fc03_16(MODBUS_STORAGE, MODBUS_ADDRESS_DEVICE, 1, &val);
    if(eeprom_read_word(MODBUS_EEPROM_DEVICE) != val) eeprom_write_word(MODBUS_EEPROM_DEVICE, val);
}

// ********** FUNCTION DEFINITION END ********** //





// ********** INTERRUPTS BEGIN ********** //

// Temperature conversion is completed
/*ISR(ADC0_RESRDY_vect) {
    ADC0.INTFLAGS = 0xff;           // Clear interrupt flags
    ADC0.CTRLA &= ~ADC_ENABLE_bm;   // Disable ADC
    
    int8_t sigrow_offset = SIGROW.TEMPSENSE1;       // Read signed offset from signature row
    uint8_t sigrow_gain = SIGROW.TEMPSENSE0;        // Read unsigned gain/slope from signature row
    uint16_t adc_reading = ADC0.RESULT >> 2;        // 10-bit MSb of ADC result with 1.024V internal reference
    uint32_t temp = adc_reading - sigrow_offset;
    temp *= sigrow_gain;                            // Result might overflow 16-bit variable (10-bit + 8-bit)
    temp += 0x80;                                   // Add 256/2 to get correct integer rounding on division below
    temp >>= 8;                                     // Divide result by 256 to get processed temperature in Kelvin
    uint16_t temperature_in_K = temp;
    
    modbus_fc16_16(MODBUS_STORAGE, 8196, 1, &temperature_in_K); // Write temperature to the modbus register
}*/

// ********** INTERRUPTS END ********** //