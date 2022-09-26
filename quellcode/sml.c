/*
 * Author:          Niklas Menke - https://www.mennik.de/ - CC BY-NC-SA (https://creativecommons.org/licenses/by-nc-sa/4.0/)
 * 
 * File:            sml.c
 * Needed files:    sml.h, modbus_RTU.h, modbus_RTU.c
 * 
 * MORE INFORMATION INSIDE THE HEADER FILE
 */





// ********** GLOBAL CCONFIGURATION BEGIN ********** //

// Include needed files
#include "sml.h"

static volatile uint8_t sml_rx_buffer[SML_RX_BUFFER_SIZE];  // Buffer for received bytes
static volatile uint8_t sml_rx_number = 0;                  // Number of received bytes

enum STATUS {START, ESC_TAB, ESC_OBIS, STAT, VALTIME, UNIT, SCALER, VAL};   // Possible SML status
static volatile enum STATUS sml_status = START;                             // Current SML status

// ********** GLOBAL CCONFIGURATION BEGIN ********** //





// ********** FUNCTION DEFINITION BEGIN ********** //

// Configure USART interface
void sml_init(void) {
    PORTC.DIRCLR = PIN1_bm;                                 // Set communication pin as input
    PORTC.PIN1CTRL = PORT_PULLUPEN_bm;                      // Enable pullup for communication pin
    PORTC.OUTSET = PIN2_bm;                                 // Set direction pin to high level
    PORTC.DIRSET = PIN2_bm;                                 // Set direction pin as output
    PORTMUX.USARTROUTEA |= PORTMUX_USART1_ALT1_gc;          // Choose alternative pin position of the USART1
    sml_baud_set();                                         // Set baud rate
    USART1.CTRLA = 0;                                       // No settings in the control register A
    USART1.CTRLB = 0;                                       // No settings in the control register B
    USART1.CTRLC = USART_CHSIZE_8BIT_gc;                    // Select 2 stop and 8 data bits
}



// Enable Rx mode
void sml_rx_enable(void){
    USART1.CTRLA |= USART_RXCIE_bm; // Enable receive complete interrupt
    USART1.CTRLB |= USART_RXEN_bm;  // Enable receiver
    TCB0.INTFLAGS = 0xff;           // Clear all interrupt flags;
    TCB0.CNT = 0;
    TCB0.INTCTRL = TCB_OVF_bm;      // Enable overflow interrupt for TCB0 timer to find a illegal break
    
}



// Disable Rx mode
void sml_rx_disable(void){
    USART1.CTRLA &= ~USART_RXCIE_bm;    // Disable receive complete interrupt
    USART1.CTRLB &= ~USART_RXEN_bm;     // Disable receiver
    sml_status = START;
    TCB0.INTCTRL = 0;
}



// Get byte from the buffer and delete byte from the buffer
uint8_t sml_rx_get_byte(){
    if(!sml_rx_number) return 0;
    else {
        uint8_t buffer = sml_rx_buffer[0];
        sml_rx_number--;
        for(uint8_t i = 0; i < sml_rx_number; i++) sml_rx_buffer[i] = sml_rx_buffer[i+1];
        return buffer;
    }
}



// Read byte from the buffer without delete them from the buffer
uint8_t sml_rx_read_byte(){
    if(!sml_rx_number) return 0;
    else return sml_rx_buffer[0];
}



// Get bytes from the buffer and delete bytes from the buffer
void sml_rx_get_bytes(uint8_t *storage, uint8_t number){
    if(number > sml_rx_number) number = sml_rx_number;
    for(uint8_t i = 0; i < number; i++) storage[i] = sml_rx_buffer[i];
    sml_rx_number -= number;
    for(uint8_t i = 0; i < sml_rx_number; i++) sml_rx_buffer[i] = sml_rx_buffer[i+number];
}



// Read bytes from the buffer without delete them from the buffer
void sml_rx_read_bytes(uint8_t *storage, uint8_t number){
    if(number > sml_rx_number) number = sml_rx_number;                  // If the number of requested bytes are not available, then set number to the number of available bytes
    for(uint8_t i = 0; i < number; i++) storage[i] = sml_rx_buffer[i];  // Save bytes from the buffer to the storage array
}



// Return the number of available bytes in the buffer
uint8_t sml_rx_avail(void){
    return sml_rx_number;
}



// Delete all bytes from the buffer
void sml_rx_delete(void) {
    sml_rx_number = 0;
}



// Transmit an array of bytes
void sml_tx_send(const uint8_t *bytes, uint8_t number) {
    PORTA.OUTSET = PIN2_bm;                         // Enable modbus status LED
    uint8_t rx_stat = USART1.CTRLB & USART_RXEN_bm; // Save receiver mode
    if(rx_stat) sml_rx_disable();                   // Disable receiver if it is enabled
    
    USART1.CTRLB |= USART_TXEN_bm;                  // Enable transmitter
    USART1.STATUS |= USART_TXCIF_bm;                // Reset transmitter
    
    // Transmit byte for byte
    while(number--) {
        while(!(USART1.STATUS&USART_DREIF_bm)); // Wait until the transmitter buffer is empty
        USART1.TXDATAL = *bytes++;              // Write a byte to the transmitter buffer
    }
    
    while(!(USART1.STATUS&USART_TXCIF_bm)); // Wait until transmitting is finished
    USART1.CTRLB &= ~USART_TXEN_bm;         // Disable transmitter
    if(rx_stat) sml_rx_enable();            // Enable receiver if it was enabled before
    
    PORTA.OUTCLR = PIN2_bm; // Disable modbus status LED
}



// Get the baud rate from the modbus register and write it to the baud register
void sml_baud_set(void) {
    USART1.BAUD = 4 * F_CPU / 9600;
}



// CRC calculation for smartmeter (X25)
uint16_t sml_crc_calc(const uint8_t *bytes, uint8_t number, const uint16_t start) {
    // CRC table for SML (X25))
    static const uint16_t crc_table[] = {
        0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF, 0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
        0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E, 0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
        0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD, 0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
        0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C, 0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
        0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB, 0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
        0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A, 0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
        0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9, 0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
        0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738, 0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
        0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7, 0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
        0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036, 0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
        0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5, 0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
        0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134, 0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
        0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3, 0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
        0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232, 0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
        0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1, 0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
        0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330, 0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
    };
    uint8_t pos;            		// Calculated position in the CRC table
    uint16_t crc = start ^ 0xffff;	// Calculated CRC checksum
    
    // Calculate CRC checksum
    while(number--) {
        pos = (uint8_t) *bytes++ ^ crc;
        crc >>= 8;
        crc ^= crc_table[pos];
    }
    
    return crc^0xffff;  // Return CRC checksum with swapped bytes
}



// Analyse received bytes from the smartmeter
void sml_analyse(void) {
    static uint16_t sml_crc;                            // CRC checksum
    static uint8_t obis_id;                             // Storage for OBIS-Code
    static int8_t scaler;                               // Storage for scaler
    // List of supported OBIS code
    static const OBIS_CODES obis_legal[SUPPORTED_OBIS_CODES_NUMBER] = {
        {0x0e0700, 26, REG_INT32, REG_UNSIGNED, 3},     // Frequency
        {0x100700, 28, REG_INT32, REG_SIGNED, 0},     // Active power total
        
        {0x1f0700, 60, REG_INT32, REG_UNSIGNED, 3},     // Current L1
        {0x200700, 62, REG_INT32, REG_UNSIGNED, 3},     // Voltage L1-N
        {0x510704, 66, REG_INT16, REG_UNSIGNED, 0},     // Angle I-L1 - U-L1
        
        {0x330700, 100, REG_INT32, REG_UNSIGNED, 3},    // Current L2
        {0x340700, 102, REG_INT32, REG_UNSIGNED, 3},    // Voltage L2-N
        {0x51070f, 106, REG_INT16, REG_UNSIGNED, 0},    // Angle I-L2 - U-L2
        {0x510701, 107, REG_INT16, REG_UNSIGNED, 0},    // Angle U-L2 - U-L1
        
        {0x470700, 140, REG_INT32, REG_UNSIGNED, 3},    // Current L3
        {0x480700, 142, REG_INT32, REG_UNSIGNED, 3},    // Voltage L3-N
        {0x51071a, 148, REG_INT16, REG_UNSIGNED, 0},    // Angle I-L3 - U-L3
        {0x510702, 149, REG_INT16, REG_UNSIGNED, 0},    // Angle U-L3 - U-L1
        
        {0x010800, 512, REG_INT64, REG_UNSIGNED, 1},    // Active energy +, tariff 0
        {0x020800, 516, REG_INT64, REG_UNSIGNED, 1},    // Active energy -, tariff 0
        
        {0x010801, 520, REG_INT64, REG_UNSIGNED, 1},    // Active energy +, tariff 1
        {0x020801, 524, REG_INT64, REG_UNSIGNED, 1},    // Active energy -, tariff 1
        
        {0x010802, 528, REG_INT64, REG_UNSIGNED, 1},    // Active energy +, tariff 2
        {0x020802, 532, REG_INT64, REG_UNSIGNED, 1},    // Active energy -, tariff 2
        
        {0x600100, 8201, 5, REG_STRING, 0},            // Server ID (EFR)
        {0x000009, 8201, 5, REG_STRING, 0},            // Server ID (EMH)
    };
    
    switch(sml_status) {
        // Search start sequence
        case START:
            if(sml_rx_number >= 4) {
                // Get 4 bytes from the buffer
                uint8_t buffer[4];
                sml_rx_get_bytes(buffer, 4);
                
                // Check if the bytes are the start sequence (4x 0x1B)
                uint8_t cnt = 0;
                for(uint8_t i = 0; i < 4; i++) if(buffer[i] == 0x1B) cnt++;
                
                // Escape sequence...
                if(cnt != 4) sml_rx_number = 0; // ...not found: Delete all bytes from the buffer
                else {                          // ...found
                    sml_crc = 0x8c32;       // Initialize CRC checksum
                    for(uint8_t i = 0; i < MODBUS_TABLE_COUNT; i++) modbus_register_clear(i);   // Clear buffer of all register tables
                    sml_status = ESC_TAB;   // Next step
                }
            }
            break;
        
        // Search the beginning from an escape sequence (0x1B) or from a list entry (0x77)
        case ESC_TAB:
            if(sml_rx_number) {
                // Get one byte from the buffer and calculate CRC checksum
                uint8_t buffer[1];
                buffer[0] = sml_rx_get_byte();
                sml_crc = sml_crc_calc(buffer, 1, sml_crc);
                
                if(buffer[0] == 0x1b || buffer[0] == 0x77) sml_status = ESC_OBIS; // Possible escape sequence or list entry -> Next step
            }
            break;
        
        // Check if it is the escape sequence or a legal OBIS code
        case ESC_OBIS:
            if(sml_rx_number >= 7) {
                
                // Get 7 bytes from the buffer and calculate CRC checksum
                uint8_t buffer[7];
                sml_rx_get_bytes(buffer, 7);
                sml_crc = sml_crc_calc(buffer, 5, sml_crc);
                
                // Check for escape sequence (3x 0x1B)
                uint8_t cnt = 0;
                for(uint8_t i = 0; i < 3; i++) if(buffer[i] == 0x1B) cnt++;
                if(cnt == 3) {  // Escape sequence
                    if(sml_crc == (buffer[6]<<8 | buffer[5])) {  // CRC checksum is valid: Save new data to the modbus register
                        sml_values_calc(PHASE_L1);
                        sml_values_calc(PHASE_L2);
                        sml_values_calc(PHASE_L3);
                        
                        uint32_t time = 0;
                        modbus_fc03_32(MODBUS_STORAGE, MODBUS_ADDRESS_UNIX, 2, &time);
                        time += RTC.CNT;
                        RTC.CNT = 0;
                        modbus_fc16_32(MODBUS_STORAGE, MODBUS_ADDRESS_UNIX, 2, &time);
                        
                        uint16_t power_sel;
                        modbus_fc03_16(MODBUS_STORAGE, MODBUS_ADDRESS_POWER, 1, &power_sel);
                        if(power_sel) {
                            //modbus_fc16_16(MODBUS_STORAGE, MODBUS_ADDRESS_RTCCALIB, 1, &power_sel);
                            uint32_t power;
                            modbus_fc03_32(MODBUS_BUFFER, 28, 2, &power);
                            if(power&0x8000) {
                                power = (power ^ UINT32_MAX) + 1;
                                power *= 10;
                                modbus_fc16_32(MODBUS_BUFFER, 2, 2, &power);
                                power = 0;
                                modbus_fc16_32(MODBUS_BUFFER, 0, 2, &power);
                            }
                            else {
                                power *= 10;
                                modbus_fc16_32(MODBUS_BUFFER, 0, 2, &power);
                                power = 0;
                                modbus_fc16_32(MODBUS_BUFFER, 2, 2, &power);
                            }
                        }
                        
                        for(uint8_t i = 0; i < MODBUS_TABLE_COUNT-1; i++) modbus_register_switch(i);
                    }
                    else {
                        for(uint8_t i = 0; i<MODBUS_TABLE_COUNT; i++) {
                            modbus_register_clear(i);
                        }
                    }
                    sml_status = START;  // Wait for a new sequence
                }
                else {  // No escape sequence -> Check for legal OBIS code
                    sml_crc = sml_crc_calc(buffer+5, 2, sml_crc);
                    uint32_t obis_readed = (uint32_t) buffer[3]<<16 | buffer[4]<<8 | buffer[5];
                    sml_status = ESC_TAB;
                    for(uint8_t i = 0; i<SUPPORTED_OBIS_CODES_NUMBER; i++) if(obis_readed == obis_legal[i].obis_code) {    // Legal OBIS code was founded -> Next step
                        obis_id = i;
                        sml_status = STAT;
                    }
                }
            }
            break;
        
        // Read the bytes of the status field -> This field has a variable length. One byte is read to get the size of the field. Next the correct number of bytes is read. The bytes will not be used.
        case STAT:
            if(sml_rx_number) {
                uint8_t size = sml_rx_read_byte() & 0x0f;   // Get the size of this field
                
                // Get enough bytes, calculate the CRC checksum and go to the next step
                //while((sml_rx_number < size) && (sml_status == STAT));
                uint8_t buffer[size];
                if(sml_analyse_wait(size)) break;   // Wait until enough bytes are received. Break if timeout.
                sml_rx_get_bytes(buffer, size);
                sml_crc = sml_crc_calc(buffer, size, sml_crc);
                sml_status = VALTIME;
            }
            break;
        
        // Read the bytes of the time field -> This field has a variable length. One byte is read to get the size and the type of the field. Next the correct number of bytes is read. The bytes will not be used.
        case VALTIME:
            if(sml_rx_number) {
                uint8_t buffer = sml_rx_get_byte();
                sml_crc = sml_crc_calc(&buffer, 1, sml_crc);

                if(buffer == 0x72) { // SML_Time is set
                    // Get TAG
                    //while((sml_rx_number < 2) && (sml_status == VALTIME));
                    uint8_t buffer[2];
                    if(sml_analyse_wait(2)) break;   // Wait until enough bytes are received. Break if timeout.
                    sml_rx_get_bytes(buffer, 2);
                    sml_crc = sml_crc_calc(buffer, 2, sml_crc);
                    
                    if(buffer[1] == 0x03) { // SML_TimestampLocal
                        //while((sml_rx_number < 12) && (sml_status == VALTIME));
                        uint8_t buffer[12];
                        if(sml_analyse_wait(12)) break;   // Wait until enough bytes are received. Break if timeout.
                        sml_rx_get_bytes(buffer, 12);
                        sml_crc = sml_crc_calc(buffer, 12, sml_crc);
                    }
                    else {                  // secIndex or SML_Timestamp
                        //while((sml_rx_number < 5) && (sml_status == VALTIME));
                        uint8_t buffer[5];
                        if(sml_analyse_wait(5)) break;   // Wait until enough bytes are received. Break if timeout.
                        sml_rx_get_bytes(buffer, 5);
                        sml_crc = sml_crc_calc(buffer, 5, sml_crc);
                    }
                }
                
                sml_status = UNIT;
            }
            break;
        
        // Read the bytes of the unit field -> This field has a variable length. One byte is read to get the size of the field. Next the correct number of bytes is read. The bytes will not be used.
        case UNIT:
            if(sml_rx_number) {
                uint8_t size = sml_rx_read_byte() & 0x0f;   // Get the size of this field
                
                // Get enough bytes, calculate the CRC checksum and go to the next step
                //while((sml_rx_number < size) && (sml_status == UNIT));
                uint8_t buffer[size];
                if(sml_analyse_wait(size)) break;   // Wait until enough bytes are received. Break if timeout.
                sml_rx_get_bytes(buffer, size);
                sml_crc = sml_crc_calc(buffer, size, sml_crc);
                sml_status = SCALER;
            }
            break;
        
        // Read the bytes of the scaler field -> This field has a variable length. One byte is read to get the size of the field. Next the correct number of bytes is read. The scaler will be used later.
        case SCALER:
            if(sml_rx_number) {
                // Get the size of this field
                uint8_t size = sml_rx_read_byte();
                size &= 0x0f;
                
                // Get enough bytes, calculate the CRC checksum, save the scaler for further use and go to the next step
                //while((sml_rx_number < size) && (sml_status == SCALER));
                uint8_t buffer[size];
                if(sml_analyse_wait(size)) break;   // Wait until enough bytes are received. Break if timeout.
                sml_rx_get_bytes(buffer, size);
                sml_crc = sml_crc_calc(buffer, size, sml_crc);
                scaler = size > 1 ? buffer[1]+obis_legal[obis_id].scaler : 0;
                sml_status = VAL;
            }
            break;
        
        // Read the bytes of the value field -> This field has a variable length. One byte is read to get the size of the field. Next the correct number of bytes is read. The value will be write in the sml register.
        case VAL:
            if(sml_rx_number > 1) {
                uint8_t buffer[2];
                sml_rx_read_bytes(buffer, 2);
                
                if(!(buffer[0]&0x80 && buffer[1]&0x80) && !((buffer[0]&0xf0)==0x70) && !((buffer[0]&0xf0)==0xf0)) { // SML_Value is supported (Not too long or a table)
                    uint8_t size = (buffer[0]&0x80) ? (((buffer[0]&0x0f)<<4) | (buffer[1]&0x0f)) : (buffer[0]&0x0f);    // Size of this field
                    uint8_t buffer[size];
                    
                    if(size > SML_RX_BUFFER_SIZE) {
                        // Get each byte of this field
                        for(uint8_t i = 0; i < size; i++) {
                            //while(!sml_rx_number && (sml_status == VAL));
                            if(sml_analyse_wait(1)) break;   // Wait until enough bytes are received. Break if timeout.
                            buffer[i] = sml_rx_get_byte();
                        }
                    }
                    else {
                        //while((sml_rx_number < size) && (sml_status == VAL));   // Wait until all bytes are received
                        if(sml_analyse_wait(size)) break;   // Wait until enough bytes are received. Break if timeout.
                        sml_rx_get_bytes(buffer, size);
                    }
                    
                    sml_crc = sml_crc_calc(buffer, size, sml_crc);
                    
                    // (Un-)signed Value
                    if(((buffer[0]&0xf0) == 0x50) || ((buffer[0]&0xf0) == 0x60)) {
                        uint8_t val_sign = ((buffer[0]&0x10) && (buffer[1]&0x80)) ? 1 : 0;   // Save sign of the value
                        
                        // Save value in one variable
                        uint64_t val = 0;
                        //for(uint8_t s = 0, b = size-1; s<4 && b>0; s++, b--) val |= (uint64_t) buffer[b]<<(8*s);
                        for(uint8_t i = size-1; i; i--) val |= (uint64_t) buffer[i]<<(8*(size-1-i));
                        if(val_sign) val |= (UINT64_MAX<<(8*(size-1)));
                        
                        // Multiply scaler to the value if value and scaler is unequal 0
                        if(val && scaler) {
                            if(val_sign) val = (val ^ UINT64_MAX) + 1;  // Remove sign if the value is negative
                            if(scaler > 0) {
                                for(int8_t i = 0; i<scaler; i++) {
                                    val *= 10;
                                }
                            }
                            else {
                                for(int8_t i = 0; i>scaler; i--) {
                                    val /= 10;
                                    if(!val) break;
                                }
                            }
                            if(val_sign && (obis_legal[obis_id].register_type&0x10)) val = (val ^ UINT64_MAX) + 1;  // Set sign if the value is negative and the register is signed
                        }
                        
                        if(val_sign && (obis_legal[obis_id].register_type&0x20) && (!val || !scaler)) val = (val ^ UINT64_MAX) + 1;  // Remove sign if the value is negative, the register is unsigned and the value or scaler is not 0 

                        modbus_fc16_64(MODBUS_BUFFER, obis_legal[obis_id].register_start, obis_legal[obis_id].register_number, &val);
                    }
                    else if((obis_legal[obis_id].obis_code == 0x600100) || (obis_legal[obis_id].obis_code == 0x000009)) {   // Server ID
                        for(uint8_t i = 0; i < 5; i++) {
                            uint16_t string = (uint16_t) buffer[2*i+1]<<8 | buffer[2*i+2];
                            modbus_fc16_16(MODBUS_STORAGE, obis_legal[obis_id].register_start+i, 1, &string);
                        }
                    }
                }
                sml_status = ESC_TAB; // Next step
            }
            break;
        
        // Illegal step
        default:
            sml_status = START; // Start over
            break;
    }
}



// Wait until enough bytes are received. Return 1 if timeout
uint8_t sml_analyse_wait(const uint8_t count) {
    while((sml_rx_number < count) && (sml_status != START));
    if(sml_status == START) return 1;
    return 0;
}



// Calculate additional values
void sml_values_calc(const uint8_t phase) {
    uint32_t current;
    uint32_t voltage;
    modbus_fc03_32(MODBUS_BUFFER, phase + 20, 2, &current);
    modbus_fc03_32(MODBUS_BUFFER, phase + 22, 2, &voltage);
    
    if(current && voltage) {
        uint16_t angle;
        modbus_fc03_16(MODBUS_BUFFER, phase + (phase == PHASE_L3 ? 28 : 26), 1, &angle);
        
        int32_t cos_phi = (int32_t) sml_sin_cos(ANGLE_COS, angle);
        int32_t sin_phi = (int32_t) sml_sin_cos(ANGLE_SIN, angle);
        uint8_t offset = cos_phi >= 0 ? 0 : 2;
        
        // Apparent power
        uint32_t apparent_power_sum;
        modbus_fc03_32(MODBUS_BUFFER, 16 + offset, 2, &apparent_power_sum);
            
        uint32_t apparent_power = (uint32_t) (((uint64_t) current * voltage + 50000) / 100000);
        apparent_power_sum += apparent_power;
            
        modbus_fc16_32(MODBUS_BUFFER, phase + 16 + offset, 2, &apparent_power);
        modbus_fc16_32(MODBUS_BUFFER, 16 + offset, 2, &apparent_power_sum);

        
        if(cos_phi) {
            // Active power
            uint32_t active_power_sum;
            modbus_fc03_32(MODBUS_BUFFER, offset, 2, &active_power_sum);
            
            uint32_t active_power;
            if(cos_phi > 0) active_power = (uint32_t) (((uint64_t) current * voltage * cos_phi + 500000000) / 1000000000);
            else active_power = (uint32_t) (((uint64_t) current * voltage * (cos_phi * -1) + 500000000) / 1000000000);
            active_power_sum += active_power;

            modbus_fc16_32(MODBUS_BUFFER, phase + offset, 2, &active_power);
            modbus_fc16_32(MODBUS_BUFFER, offset, 2, &active_power_sum);


            // Power factor
            if(cos_phi > 0) cos_phi = (cos_phi + 5) / 10;
            else cos_phi = (cos_phi - 5) / 10;
            int32_t cos_phi_sum = (int32_t) ((uint64_t) active_power_sum * 1000 / apparent_power_sum);
            if(cos_phi < 0) cos_phi_sum *= -1;

            modbus_fc16_32(MODBUS_BUFFER, phase + 24, 2, (uint32_t*) &cos_phi);
            modbus_fc16_32(MODBUS_BUFFER, 24, 2, (uint32_t*) &cos_phi_sum);
        }
        
        
        if(sin_phi) {
            if(sin_phi > 0) offset = 0;
            else {
                sin_phi *= -1;
                offset = 2;
            }
            
            // Reactive power
            uint32_t reactive_power_sum;
            modbus_fc03_32(MODBUS_BUFFER, 4 + offset, 2, &reactive_power_sum);

            uint32_t reactive_power = (uint32_t) (((uint64_t) current * voltage * sin_phi + 500000000) / 1000000000);
            reactive_power_sum += reactive_power;

            modbus_fc16_32(MODBUS_BUFFER, 4 + offset, 2, &reactive_power_sum);
            modbus_fc16_32(MODBUS_BUFFER, phase + 4 + offset, 2, &reactive_power);
        }
    }
}



// Return value of cos(angle) with two significant 
int16_t sml_sin_cos(const uint8_t type, uint16_t angle) {
    static const uint16_t sin_val[] = {
        0x0000, 0x00AE, 0x015C, 0x020B, 0x02B9, 0x0367, 0x0415, 0x04C2, 0x056F, 0x061C,
        0x06C8, 0x0774, 0x081F, 0x08C9, 0x0973, 0x0A1C, 0x0AC4, 0x0B6B, 0x0C12, 0x0CB7,
        0x0D5C, 0x0DFF, 0x0EA2, 0x0F43, 0x0FE3, 0x1082, 0x111F, 0x11BB, 0x1256, 0x12F0,
        0x1387, 0x141E, 0x14B3, 0x1546, 0x15D7, 0x1667, 0x16F5, 0x1782, 0x180C, 0x1895,
        0x191B, 0x19A0, 0x1A23, 0x1AA3, 0x1B22, 0x1B9F, 0x1C19, 0x1C91, 0x1D07, 0x1D7B,
        0x1DEC, 0x1E5B, 0x1EC8, 0x1F32, 0x1F9A, 0x1FFF, 0x2062, 0x20C2, 0x2120, 0x217B,
        0x21D4, 0x222A, 0x227D, 0x22CE, 0x231B, 0x2367, 0x23AF, 0x23F5, 0x2437, 0x2477,
        0x24B4, 0x24EF, 0x2526, 0x255B, 0x258C, 0x25BB, 0x25E6, 0x260F, 0x2635, 0x2658,
        0x2678, 0x2694, 0x26AE, 0x26C5, 0x26D9, 0x26E9, 0x26F7, 0x2702, 0x2709, 0x270E, 0x2710
    };

    if(type == ANGLE_COS) angle += 90;
    angle %= 360;

    if (angle < 91) return sin_val[angle];              // Angle between 0 ° and 90 °
    else if (angle < 181) return sin_val[180 - angle];  // Angle between 91 ° and 180 °
    else if (angle < 271) return -sin_val[angle - 180]; // Angle between 181 ° and 270 °
    return -sin_val[360 - angle];                       // Angle bettween 271 ° and 359 °
}

// ********** FUNCTION DEFINITION END ********** //





// ********** INTERRUPTS BEGIN ********** //

// Receive complete interrupt
ISR(USART1_RXC_vect) {
    PORTA.OUTSET = PIN2_bm; // Enable modbus status LED
    
    if(sml_rx_number > SML_RX_BUFFER_SIZE-1) sml_rx_number = 0; // Buffer overflow -> Reset buffer
    sml_rx_buffer[sml_rx_number++] = USART1.RXDATAL;            // Write received byte to the buffer
    
    TCB0.CNT = 0;
    TCB0.CTRLA = TCB_CLKSEL_DIV2_gc | TCB_ENABLE_bm;
}


// TCB overflow
ISR(TCB0_INT_vect) {
    TCB0.INTFLAGS = 0xff;   // Clear all interrupt flags;
    TCB0.CTRLA = 0;         // Disable timer

    sml_rx_number = 0;
    sml_status = START;
    
    PORTA.OUTCLR = PIN2_bm; // Disable modbus status LED
}

// ********** INTERRUPTS END ********** //