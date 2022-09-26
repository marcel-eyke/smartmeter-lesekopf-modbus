/*
 * Author:          Niklas Menke - https://www.mennik.de/ - CC BY-NC-SA (https://creativecommons.org/licenses/by-nc-sa/4.0/)
 * 
 * File:            modbus_RTU.c
 * Needed files:    modbus_RTU.h
 * 
 * MORE INFORMATION INSIDE THE HEADER FILE
 */





// ********** GLOBAL CONFIGURATION BEGIN ********** //

#include "modbus_RTU.h"

static volatile uint8_t modbus_rx_buffer[MODBUS_RX_BUFFER_SIZE];    // Buffer for received bytes
static volatile uint8_t modbus_rx_number = 0;                       // Number of received bytes
static volatile uint8_t modbus_frame_available = 0;                 // New frame enable = 1, No frame enable = 0

static volatile uint8_t modbus_wait = 0;            // Wait between two frames (3.5 symbols)

static volatile uint8_t modbus_address;             // Modbus address for this device


// Storage for tables
static uint16_t table_01[16];   // 0-7
static uint16_t table_02[8];    // 16-19
static uint16_t table_03[12];   // 24-29

static uint16_t table_04[16];   // 40-47
static uint16_t table_05[22];   // 56-66

static uint16_t table_06[16];   // 80-87
static uint16_t table_07[24];   // 96-107

static uint16_t table_08[16];   // 120-127
static uint16_t table_09[28];   // 136-149

static uint16_t table_10[48];   // 512-535

static uint16_t table_11[20] = {0, 0, 0x0220, 0x0220};    // 8192-8205

static MODBUS_REGISTER_TABLE register_tables[MODBUS_TABLE_COUNT] = {
    {0, 7, table_01, table_01+8},
    {16, 19, table_02, table_02+4},
    {24, 29, table_03, table_03+6},
    
    {40, 47, table_04, table_04+8},
    {56, 66, table_05, table_05+11},
    
    {80, 87, table_06, table_06+8},
    {96, 107, table_07, table_07+12},
    
    {120, 127, table_08, table_08+8},
    {136, 149, table_09, table_09+14},
    
    {512, 535, table_10, table_10+24},
    
    {8192, 8211, table_11, NULL},
};

// ********** GLOBAL CONFIGURATION END ********** //





// ********** FUNCTION DEFINITION BEGIN ********** //

// Configure USART interface for a RS485 one-wire communication
void modbus_init(/*const uint8_t coding*/) {
    PORTB.DIRCLR = PIN1_bm;                                 // Set communication pin as input
    PORTB.DIRSET = PIN0_bm;                                 // Set direction pin as output
    PORTB.PIN2CTRL = PORT_PULLUPEN_bm;                      // Enable pullup for communication pin
    modbus_baud_set();                                      // Set baud rate
    modbus_adr_set();                                       // Set modbus device address
    USART0.CTRLA = USART_LBME_bm | USART_RS485_bm;          // Enable one-wire and RS485 mode
    //USART0.CTRLC = USART_SBMODE_bm | USART_CHSIZE_8BIT_gc;  // Select 2 stop and 8 data bits
    //USART0.CTRLC = coding;                                  // Select communication mode, parity mode, stop bit mode and character size
    USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | eeprom_read_byte ((uint8_t*)1) | eeprom_read_byte ((uint8_t*)0) | USART_CHSIZE_8BIT_gc;
    //USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_2BIT_gc | USART_CHSIZE_8BIT_gc;
    USART0.CTRLB = USART_ODME_bm;                           // Enable open drain mode
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;               // Enable compare match interrupt for the TCA0 timer for the break between modbus frames
}



// Enable Rx mode
void modbus_rx_enable(void) {
    USART0.CTRLA |= USART_RXCIE_bm;             // Enable receive complete interrupt
    USART0.CTRLB |= USART_RXEN_bm;              // Enable receiver
}



// Disable Rx mode
void modbus_rx_disable(void) {
    USART0.CTRLA &= ~USART_RXCIE_bm;    // Disable receive complete interrupt
    USART0.CTRLB &= ~USART_RXEN_bm;     // Disable receiver 
}



// Get byte(s) from the buffer and delete the byte(s) from the buffer
void modbus_rx_get(uint8_t *storage, uint8_t number) {
    if(number > modbus_rx_number) number = modbus_rx_number;
    for(uint8_t i = 0; i < number; i++) storage[i] = modbus_rx_buffer[i];
    modbus_rx_number -= number;
    for(uint8_t i = 0; i < modbus_rx_number; i++) modbus_rx_buffer[i] = modbus_rx_buffer[i+number];
    modbus_frame_available = 0;
}



// Read byte(s) from the buffer without delete them from the buffer
void modbus_rx_read(uint8_t *storage, uint8_t number) {
    if(number > modbus_rx_number) number = modbus_rx_number;                // If the number of requested bytes are not available, then set number to the number of available bytes
    for(uint8_t i = 0; i < number; i++) storage[i] = modbus_rx_buffer[i];   // Save bytes from the buffer to the storage array
}



// Return the number of available bytes in the buffer
uint8_t modbus_rx_avail(void) {
    return modbus_rx_number;
}



// Delete alle bytes from the buffer
void modbus_rx_delete(void) {
    modbus_rx_number = 0;
}



// Transmit an array of bytes
void modbus_tx_send(const uint8_t *bytes, uint8_t number) {
    PORTA.OUTSET = PIN1_bm;                         // Enable modbus status LED
    uint8_t rx_stat = USART0.CTRLB & USART_RXEN_bm; // Save receiver mode
    if(rx_stat) modbus_rx_disable();                // Disable receiver if it is enabled
    
    // Configure TCA0 timer to wait for the begin of a frame (3.5 symbols)
    TCA0.SINGLE.CNT = 0;                                                    // Reset counter
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;   // Enable TCA0 timer with prescaler 2
    modbus_wait = 1;
    while(modbus_wait);
    
    USART0.CTRLB |= USART_TXEN_bm;                  // Enable transmitter
    USART0.STATUS |= USART_TXCIF_bm;                // Reset transmitter
    
    // Transmit byte for byte
    while(number--) {
        while(!(USART0.STATUS&USART_DREIF_bm)); // Wait until the transmitter buffer is empty
        USART0.TXDATAL = *bytes++;              // Write a byte to the transmitter buffer
    }
    
    while(!(USART0.STATUS&USART_TXCIF_bm));         // Wait until transmitting is finished
    USART0.CTRLB &= ~USART_TXEN_bm;                 // Disable transmitter
    if(rx_stat) modbus_rx_enable();                 // Enable receiver if it was enabled before
    
    // Configure TCA0 timer to wait for the end of a frame (3.5 symbols)
    TCA0.SINGLE.CNT = 0;                                                    // Reset counter
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;   // Enable TCA0 timer with prescaler 2
    modbus_wait = 1;
    while(modbus_wait);
    
    PORTA.OUTCLR = PIN1_bm;                         // Disable modbus status LED
}



// Get the current pin state of the coding switches and set the modbus device address
void modbus_adr_set(void) {
    // Set pins as input
    PORTA.DIRCLR = PIN6_bm | PIN7_bm;
    PORTB.DIRCLR = PIN1_bm | PIN3_bm | PIN4_bm | PIN5_bm;
    PORTC.DIRCLR = PIN0_bm | PIN3_bm;
    
    // Enable pullups and invert value
    PORTA.PIN6CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
    PORTA.PIN7CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm | PORT_ISC_BOTHEDGES_gc; // Enable both edge pin change interrupt
    PORTB.PIN1CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
    PORTB.PIN3CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
    PORTB.PIN4CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
    PORTB.PIN5CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
    PORTC.PIN0CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm;
    PORTC.PIN3CTRL = PORT_PULLUPEN_bm | PORT_INVEN_bm | PORT_ISC_BOTHEDGES_gc; // Enable both edge pin change interrupt
    
    // Reset pin change interrupt
    PORTA.INTFLAGS |= PIN7_bm;
    PORTC.INTFLAGS |= PIN3_bm;
    
    // Get pin level, sort pins to the correct order and save modbus device address
    modbus_address  = (PORTA.IN & PIN6_bm) >> 3
                    | (PORTA.IN & PIN7_bm) >> 7
                    | (PORTB.IN & PIN1_bm)
                    | (PORTB.IN & PIN3_bm) >> 1
                    | (PORTB.IN & PIN4_bm) << 1
                    | (PORTB.IN & PIN5_bm) << 2
                    | (PORTC.IN & PIN0_bm) << 6
                    | (PORTC.IN & PIN3_bm) << 1;
}



// Return the modbus device address
uint8_t modbus_adr_get(void) {
    return modbus_address;
}



// Get the selected baud rate from the 2x2 pin header and write it to the baud register
void modbus_baud_set(void) {
    static uint8_t config_mode_last = 0;    // Save last state of the config mode
    
    PORTA.DIRCLR = PIN4_bm | PIN5_bm;       // Set pins as input
    PORTA.INTFLAGS |= PIN4_bm | PIN5_bm;    // Reset pin change interrupt
    
    // Enable pullups and enable both edge pin change interrupt
    PORTA.PIN4CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
    PORTA.PIN5CTRL = PORT_PULLUPEN_bm | PORT_ISC_BOTHEDGES_gc;
    
    PORTA.INTFLAGS |= PIN4_bm | PIN5_bm;    // Reset pin change interrupt
    
    uint32_t baud = (uint32_t) PORTA.IN & (PIN4_bm | PIN5_bm);  // Get pin level
    
    if(!(baud&0x30) && !config_mode_last) {  // Config-mode
        config_mode_last = 1;                   // Config mode was set
        uint8_t stopbits = (modbus_address&0xf0)>>4; // Get stop bits setting
        uint8_t parity = modbus_address&0x0f;   // Get parity setting
        
        // Set stop bits and parity if values are legal
        if(((stopbits == 0x01) || (stopbits == 0x02)) && (parity < 0x03)) {
            if(stopbits == 0x01) eeprom_write_byte(MODBUS_EEPROM_STOPBITS, USART_SBMODE_1BIT_gc);  // One stop bit
            else eeprom_write_byte(MODBUS_EEPROM_STOPBITS, USART_SBMODE_2BIT_gc);                  // Two stop bits
            
            if(!parity) eeprom_write_byte(MODBUS_EEPROM_PARITY, USART_PMODE_DISABLED_gc);            // None parity
            else if(parity == 0x01) eeprom_write_byte(MODBUS_EEPROM_PARITY, USART_PMODE_EVEN_gc);    // Even parity
            else eeprom_write_byte(MODBUS_EEPROM_PARITY, USART_PMODE_ODD_gc);                        // Two stop bits
            
            modbus_init();  // Reinitialize modbus
            
            // Show that stop bits and parity were set
            PORTA.OUTCLR = PIN1_bm | PIN2_bm;
            for(uint8_t i = 0; i < 6; i++) {
                PORTA.OUTTGL = PIN1_bm | PIN2_bm;
                _delay_ms(500);
            }
        }
    }
    else {
        config_mode_last = 0;   // Reset config mode
        
        baud = (baud == 0x20) ? 19200 : (baud == 0x10) ? 38400 : 9600;  // Get baut rate out of the pin levels
        TCA0.SINGLE.CMP0 = (uint16_t) 11 * 3.5 * F_CPU / baud / 2 + 1;  // Set compare match value for TCA0 timer to detect the end of a frame
        baud = 4 * F_CPU / baud;                                        // Calculate ideal baud rate
        USART0.BAUD = (uint16_t) baud;                                  // Set baud rate to the baud register
    }
}



// CRC calculation for modbus
uint16_t modbus_crc_calc(const uint8_t *bytes, uint8_t number, const uint16_t start) {
    // CRC table for mudbus
    static const uint16_t crc_table[] = {
        0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
        0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
        0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
        0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
        0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
        0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
        0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
        0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
        0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
        0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
        0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
        0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
        0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
        0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
        0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
        0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
        0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
        0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
        0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
        0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
        0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
        0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
        0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
        0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
        0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
        0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
        0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
        0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
        0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
        0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
        0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
        0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
    };
    uint8_t pos;            // Calculated position in the CRC table
    uint16_t crc = start;   // Calculated CRC checksum
    
    // Calculate CRC checksum
    while(number--) {
        pos = (uint8_t) *bytes++ ^ crc;
        crc >>= 8;
        crc ^= crc_table[pos];
    }
    
    return crc; // Return CRC checksum with swapped bytes
}



// Return if a complete modbus frame is available in the modbus buffer
uint8_t modbus_frame_avail(void) {
    return modbus_frame_available;
}



// Analyse received modbus frame and send a response
uint8_t modbus_frame_send(const uint8_t *bytes, const uint8_t number) {
    uint8_t status = 0; // Status of this function. Will be return the transmitted function code, a 0 for a wrong modbus device address or wrong CRC checksum or a 1 for an illegal function code
    
    // Check if the modbus device address and the CRC checksum is correct
    if(bytes[0] == modbus_adr_get() && !modbus_crc_calc(bytes, number, 0xffff)) {
        if((bytes[1] == 0x03) || (bytes[1] == 0x10)) {  // Function FC03: Read Holding registers or funtion FC16: Preset Multiple Registers
            const uint16_t address_first = ((uint16_t) bytes[2]<<8) | bytes[3];                         // Get first address of the request
            const uint16_t address_last = address_first + (((uint16_t) bytes[4]<<8) | bytes[5]) - 1;    // Get last address of the request
            
            // Find legal register table
            for(uint8_t tab = 0; tab < MODBUS_TABLE_COUNT; tab++) {
                if((register_tables[tab].address_first <= address_first) && (register_tables[tab].address_last >= address_last)) {
                    
                    // Transmit a response for function FC03
                    if(bytes[1] == 0x03) {
                        const uint16_t data_byte_number = 2 * (((uint16_t) bytes[4]<<8) | bytes[5]);
                        uint8_t res[5 + data_byte_number];    // Array for the response frame (Device address, function code, number of data bytes, data bytes, crc checksum)
                        res[0] = bytes[0];
                        res[1] = 0x03;
                        res[2] = data_byte_number;
                        
                        uint8_t byte = 3;
                        for(uint8_t reg = address_first - register_tables[tab].address_first; reg <= address_last - register_tables[tab].address_first; reg++) {
                            res[byte++] = (uint8_t)(register_tables[tab].storage[reg]>>8);
                            res[byte++] = (uint8_t) register_tables[tab].storage[reg];
                        }
                        
                        const uint16_t crc = modbus_crc_calc(res, 3 + data_byte_number, 0xffff);
                        res[3 + data_byte_number] = (uint8_t) crc;
                        res[4 + data_byte_number] = (uint8_t) (crc>>8);
                        
                        modbus_tx_send(res, 5 + data_byte_number);
                        
                        status = 0x03;
                    }
                    else if((address_first <= MODBUS_ADDRESS_WRITEABLE_1) || (address_first >= MODBUS_ADDRESS_WRITEABLE_2)) { // Transmit a response for function FC16
                        uint8_t byte = 7;
                        for(uint8_t reg = address_first-register_tables[tab].address_first; reg <= address_last-register_tables[tab].address_first; reg++) {
                            register_tables[tab].storage[reg] = (uint16_t) (bytes[byte++]<<8);
                            register_tables[tab].storage[reg] |= (uint16_t) bytes[byte++];
                        }
                        
                        uint8_t res[8] = {bytes[0], 0x10, bytes[2], bytes[3], bytes[4], bytes[5]}; // Array for the response frame: Modbus device address, function code

                        // Calculate CRC checksum and add it with swapped bytes to the frame
                        uint16_t crc = modbus_crc_calc(res, 6, 0xffff); 
                        res[6] = (uint8_t) crc;
                        res[7] = (uint8_t) (crc>>8);

                        modbus_tx_send(res, 8);    // Transmit frame
                        
                        status = 0x10;
                    }
                }
            }
            
            // No legal register table found -> Exclamation 0x02: Illigal register address
            if(!status) {
                uint8_t response[5] = {bytes[0], bytes[1]|0x80, 0x02};  // Array for the exception frame: Modbus device address, function code with MSB 1 and exception code (0x02: Illegal data address)
            
                // Calculate CRC checksum and add it with swapped bytes to the exception frame
                uint16_t crc = modbus_crc_calc(response, 3, 0xffff); 
                response[3] = (uint8_t) crc;
                response[4] = (uint8_t) (crc>>8);
                
                modbus_tx_send(response, 5);    // Transmit exception frame
                
                status = bytes[1]|0x80;
            }
        }
        else if(bytes[1] == 0x08 && !bytes[2] && !bytes[3]) {   // Function FC08: Diagnostics and Subfunction 0x0000: Return query data
             modbus_tx_send(bytes, number);  // Transmit received data
             
             status = 0x08;
        }
        else {  // Exclamation 0x01: Illigal function code
            uint8_t response[5] = {bytes[0], bytes[1]|0x80, 0x01};  // Array for the exception frame: Modbus device address, function code with MSB 1 and exception code (0x01: Illegal function)
            
            // Calculate CRC checksum and add it with swapped bytes to the exception frame
            uint16_t crc = modbus_crc_calc(response, 3, 0xffff); 
            response[3] = (uint8_t) crc;
            response[4] = (uint8_t) (crc>>8);
            
            modbus_tx_send(response, 5);    // Transmit exception frame
            
            status = 0x01;
        }
    }
    
    return status;
}



uint8_t modbus_table_number(const uint16_t reg_adr, const uint8_t reg_num) {
    uint8_t ret = 0xff;
    
    for(uint8_t tab = 0; tab < MODBUS_TABLE_COUNT; tab++) {
        if((register_tables[tab].address_first <= reg_adr) && (register_tables[tab].address_last >= reg_adr + reg_num - 1)) {
            ret = tab;
        }
    }
    
    return ret;
}



// FC03: Read Holding Registers for internal use
void modbus_fc03_16(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, uint16_t *data){
    *data = 0;
    const uint8_t tab = modbus_table_number(reg_adr, reg_num);
    
    if(tab != 0xff) {
        if(reg_type) {
            for(uint8_t i = 0; i < reg_num; i++) {
                *data |= (uint16_t) register_tables[tab].storage[reg_adr-register_tables[tab].address_first+i]<<(16*(reg_num-1-i));
            }
        }
        else if(register_tables[tab].buffer != NULL) {
            for(uint8_t i = 0; i < reg_num; i++) {
                *data |= (uint16_t) register_tables[tab].buffer[reg_adr-register_tables[tab].address_first+i]<<(16*(reg_num-1-i));
            }
        }
    }
}
void modbus_fc03_32(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, uint32_t *data){
    *data = 0;
    const uint8_t tab = modbus_table_number(reg_adr, reg_num);
    
    if(tab != 0xff) {
        if(reg_type) {
            for(uint8_t i = 0; i < reg_num; i++) {
                *data |= (uint32_t) register_tables[tab].storage[reg_adr-register_tables[tab].address_first+i]<<(16*(reg_num-1-i));
            }
        }
        else if(register_tables[tab].buffer != NULL) {
            for(uint8_t i = 0; i < reg_num; i++) {
                *data |= (uint32_t) register_tables[tab].buffer[reg_adr-register_tables[tab].address_first+i]<<(16*(reg_num-1-i));
            }
        }
    }
}
void modbus_fc03_64(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, uint64_t *data){
    *data = 0;
    const uint8_t tab = modbus_table_number(reg_adr, reg_num);
    
    if(tab != 0xff) {
        if(reg_type) {
            for(uint8_t i = 0; i < reg_num; i++) {
                *data |= (uint64_t) register_tables[tab].storage[reg_adr-register_tables[tab].address_first+i]<<(16*(reg_num-1-i));
            }
        }
        else if(register_tables[tab].buffer != NULL) {
            for(uint8_t i = 0; i < reg_num; i++) {
                *data |= (uint64_t) register_tables[tab].buffer[reg_adr-register_tables[tab].address_first+i]<<(16*(reg_num-1-i));
            }
        }
    }
}



// FC16: Preset Multiple Registers for internal use
void modbus_fc16_16(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, const uint16_t *data) {
    const uint8_t tab = modbus_table_number(reg_adr, reg_num);
    
    if(tab != 0xff) {
        if(reg_type) {
            for(uint8_t i = 0; i < reg_num; i++) {
                register_tables[tab].storage[reg_adr-register_tables[tab].address_first+i] = (uint16_t) (*data>>(16*(reg_num-1-i)));
            }
        }
        else if(register_tables[tab].buffer != NULL) {
            for(uint8_t i = 0; i < reg_num; i++) {
                register_tables[tab].buffer[reg_adr-register_tables[tab].address_first+i] = (uint16_t) (*data>>(16*(reg_num-1-i)));
            }
        }
    }
}
void modbus_fc16_32(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, const uint32_t *data) {
    const uint8_t tab = modbus_table_number(reg_adr, reg_num);
    
    if(tab != 0xff) {
        if(reg_type) {
            for(uint8_t i = 0; i < reg_num; i++) {
                register_tables[tab].storage[reg_adr-register_tables[tab].address_first+i] = (uint16_t) (*data>>(16*(reg_num-1-i)));
            }
        }
        else if(register_tables[tab].buffer != NULL) {
            for(uint8_t i = 0; i < reg_num; i++) {
                register_tables[tab].buffer[reg_adr-register_tables[tab].address_first+i] = (uint16_t) (*data>>(16*(reg_num-1-i)));
            }
        }
    }
}
void modbus_fc16_64(const uint8_t reg_type, const uint16_t reg_adr, const uint8_t reg_num, const uint64_t *data) {
    const uint8_t tab = modbus_table_number(reg_adr, reg_num);
    
    if(tab != 0xff) {
        if(reg_type) {
            for(uint8_t i = 0; i < reg_num; i++) {
                register_tables[tab].storage[reg_adr-register_tables[tab].address_first+i] = (uint16_t) (*data>>(16*(reg_num-1-i)));
            }
        }
        else if(register_tables[tab].buffer != NULL) {
            for(uint8_t i = 0; i < reg_num; i++) {
                register_tables[tab].buffer[reg_adr-register_tables[tab].address_first+i] = (uint16_t) (*data>>(16*(reg_num-1-i)));
            }
        }
    }
}



// Toggle array address of buffer and storage. Clear buffer
void modbus_register_switch(const uint8_t reg_tab) {
    if(reg_tab <= MODBUS_TABLE_COUNT && (register_tables[reg_tab].buffer != NULL)) {
        uint16_t *buffer = register_tables[reg_tab].buffer;
        register_tables[reg_tab].buffer = register_tables[reg_tab].storage;
        register_tables[reg_tab].storage = buffer;
        
        for(uint16_t i = 0; i <= register_tables[reg_tab].address_last - register_tables[reg_tab].address_first; i++) {
            register_tables[reg_tab].buffer[i] = 0;
        }
    }
}



// Clear buffer of each register table
void modbus_register_clear(const uint8_t reg_tab){
    if(reg_tab <= MODBUS_TABLE_COUNT && (register_tables[reg_tab].buffer != NULL)) {
        for(uint16_t i = 0; i <= register_tables[reg_tab].address_last - register_tables[reg_tab].address_first; i++) {
            register_tables[reg_tab].buffer[i] = 0;
        }
    }
}

// ********** FUNCTION DEFINITION END ********** //





// ********** INTERRUPTS BEGIN ********** //

// Receive complete interrupt
ISR(USART0_RXC_vect) {
    PORTA.OUTSET = PIN1_bm; // Enable modbus status LED
    
    if(modbus_rx_number > MODBUS_RX_BUFFER_SIZE-1) modbus_rx_number = 0;    // Buffer overflow -> Reset buffer
    modbus_rx_buffer[modbus_rx_number++] = USART0.RXDATAL;                  // Write received byte to the buffer
    
    // Configure TCA0 timer to determine the end of a frame (3.5 symbols)
    TCA0.SINGLE.CNT = 0;                                                    // Reset counter
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV2_gc | TCA_SINGLE_ENABLE_bm;   // Enable TCA0 timer with prescaler 2
    
    PORTA.OUTCLR = PIN1_bm; // Disable modbus status LED
}



// TCA0 timer compare match interrupt --> End of frame detected
ISR(TCA0_CMP0_vect) {
    TCA0.SINGLE.INTFLAGS = 0xff;    // Delete all interrupt flags of the TCA0 timer
    TCA0.SINGLE.CTRLA = 0;          // Disable TCA0 timer
    
    modbus_wait = 0;                // Break between frames is end
    
    // Not enough bytes for a modbus frame were received
    if(modbus_rx_number < 6) {
        modbus_rx_number = 0;       // Delete all received bytes
        modbus_frame_available = 0; // There is no modbus frame available
    } else modbus_frame_available = 1;    // Enough bytes for a modbus frame were received. So a new frame is available
}



// Pin change interrupt -> modbus baud rate or modbus device address was changed
ISR(PORTA_PORT_vect) {
    PORTA.INTFLAGS = 0xff;              // Clear interrupt flags
    sml_rx_disable();                   // Disable sml receiver
    modbus_rx_disable();                // Disable modbus receiver
    RTC.CMP = RTC.CNT + 3;              // Compare match after 3 seconds
    RTC.INTCTRL |= RTC_CMP_bm;          // Enable compare match interrupt
    PORTA.OUTSET = PIN1_bm | PIN2_bm;   // Enable both LEDs
}
ISR(PORTC_PORT_vect) {
    PORTC.INTFLAGS = 0xff;              // Clear interrupt flags
    sml_rx_disable();                   // Disable sml receiver
    modbus_rx_disable();                // Disable modbus receiver
    RTC.CMP = RTC.CNT + 3;              // Compare match after 3 seconds
    RTC.INTCTRL |= RTC_CMP_bm;          // Enable compare match interrupt
    PORTA.OUTSET = PIN1_bm | PIN2_bm;   // Enable both LEDs
}



// RTC interrupt
ISR(RTC_CNT_vect) {
    // RTC overflow
    if(RTC.INTFLAGS |= RTC_OVF_bm) {
        RTC.INTFLAGS = RTC_OVF_bm;  // Reset interrupt flag
        //table_standard[0]++;       // Count up Unix time in the modbus register
        uint16_t ovf;
        modbus_fc03_16(MODBUS_STORAGE, MODBUS_ADDRESS_UNIX, 1, &ovf);
        ovf++;
        modbus_fc16_16(MODBUS_STORAGE, MODBUS_ADDRESS_UNIX, 1, &ovf);
        
        //modbus_register_write(MODBUS_STORAGE, 8196, modbus_register_read(MODBUS_STORAGE, 8196)+1);
    }
    
    // RTC compare match
    if(RTC.INTFLAGS |= RTC_CMP_bm) {
        RTC.INTFLAGS = RTC_CMP_bm;          // Reset interrupt flag
        RTC.INTCTRL &= ~RTC_CMP_bm;         // Disable compare match interrupt
        modbus_baud_set();                  // Get current modbus baud rate
        modbus_adr_set();                   // Get current modbus device address
        modbus_rx_enable();                 // Enable modbus receiver
        sml_rx_enable();                    // Enable sml receiver
        PORTA.OUTCLR = PIN1_bm | PIN2_bm;   // Turn off both LEDs
    }
}

// ********** INTERRUPTS END ********** //