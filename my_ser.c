#include "my_ser.h"

void setupSerial() {
    unsigned char dummy;     // For clearing RX buffer
    BAUDCONbits.BRG16 = 0;   // From Excel sheet (2nd choice)
    SPBRG = 25;              // From Excel sheet
    SPBRGH = 0;              // From Excel sheet
    TXSTA = 0;               // Clear all bits
    TXSTAbits.BRGH = 1;      // Baud rate high speed option
    TXSTAbits.TXEN = 1;      // Enable transmission

    RCSTA = 0;               // SERIAL RECEPTION W/ 8 Bits
    RCSTAbits.CREN = 1;      // Enable reception
    RCSTAbits.SPEN = 1;      // Enable both transmission and receiver
    
    // Enable serial port
    dummy = RCREG;           // , W    clear the receiver buffer      
    dummy = RCREG;           // , W    clear the receiver buffer
    return;
}
// This function is used to prevent busy waiting
unsigned char is_byte_available()
{
    if (RCSTAbits.FERR || RCSTAbits.OERR) // Check for error
    {
        RCSTAbits.CREN = 0;  // Turn the receiver off
        RCSTAbits.CREN = 1;  // Turn the receiver on again
    }
    // Wait until byte available
    if (PIR1bits.RCIF) return 1;
    else return 0;
}
unsigned char read_byte_no_lib() {
    unsigned char c;
    while(!PIR1bits.RCIF) {
        CLRWDT();
    }
    c = RCREG;
    return c;
}

void send_byte_no_lib(unsigned char c) {
    while (!TXSTAbits.TRMT) // If the transmission not ready
    {
        CLRWDT();
    }
    TXREG = c;    // Put the char in TX register
}

void send_string_no_lib(unsigned char *p) {
    while (*p) {
        send_byte_no_lib(*p);   // Or use the send_byte_no_lib()
        p++;
    }
}