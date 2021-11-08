#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4MH = your crystal frequency
// CONFIG1H
#include <xc.h>
#include <stdio.h>
#include <float.h>
#include "my_ser.h"
#include "PIC18F4620.h"   // Add me if errors

// Fields of structure are independent of each other
typedef struct //St1Type is Type
{
    char  chr;   // 1 byte
    short shrt;  // 2 bytes
    long  lng;   // 4 bytes
    float flt;   // 4 bytes       
} SType;
// sizeof(St1Type) = 1+2+4+8 = 11 bytes

// Fields of a union depend on each other, all in same largest field place
typedef union // union type
{
    char  chr;  // first 1 byte (lSB)
    short shrt; // first 2 bytes
    long  lng;  // All   4 bytes
    float flt;  // All   4 bytes
} UType;
// sizeof(unType) = 4 size of largest, all stored in same place


// defining general byte to be used
typedef union
{
    unsigned char byt; // One byte
    struct             // See the byte as bits
    {       
        unsigned b0 :1; // Bit 0 LSB
        unsigned b1 :1;
        unsigned b2 :1;
        unsigned b3 :1;
        unsigned b4 :1;
        unsigned b5 :1;
        unsigned b6 :1;
        unsigned b7 :1; // Bit 7 MSB
    };
    struct              // See the byte as two nibbles
    {
        unsigned lowNib  :4;    // Bits 0,1,2,3
        unsigned highNib :4;    // Bits 4,5,6,7
    };
    struct {
        unsigned leds    : 2;   // 2 LEDs; bits 0,1
        unsigned sw      : 4;   // 4 bit field, bits 2--5
        unsigned unused  : 2;   // 2 bit fields, bits 6,7 (Unused)
    };
} UByte;
// redefining PORTD
UByte port_d  __at(0xF83);      // Redefine PORTD
// There are also two variables named PORTD, PORTDbits: predefined
UByte data1, X1 ;               // Global variables of for UByte


// Redefining PORTD, TRISD to LCD;
typedef struct  {
    unsigned un1    : 1;    // Bit 0 unused 
    unsigned rs     : 1;    // Bit 1         
    unsigned rw     : 1;    // Bit 2 
    unsigned enable : 1;    // Bit 3
    unsigned data   : 4;    // bits 4,5,6,7
} lcdType; 

lcdType lcd      __at(0xF83); // Redefine PORTD
lcdType lcd_tris __at(0x95);  // Redefine TRISD

// Here we can use PORTD as PORTD, PORTDbits, port_d or lcd (Same address)

char buffer[200];  // Global variable
 
void delay_ms(unsigned int n)
{
    int i;
    for (i=0; i < n; i++){
        __delaywdt_ms(1);
    }
}
 
void setupPorts() {
    ADCON0 = 0;
    ADCON1 = 0b00001100; // 3 analog channels, change this according to your application
    
    TRISB = 0xFF; // All pushbuttons are inputs
    TRISC = 0x80; // RX input, others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}

void print_line(void)
{
    send_string_no_lib("---------------------------------------------------------\r\n");
}

void print_structure(SType* v, int n) // Use pointer better
{
   unsigned short ls, hs;
   ls = v->lng & 0x0000FFFF;    // Low  two bytes
   hs = v->lng >> 16;           // High two bytes
   print_line();
   sprintf(buffer, "Printing SType fields at (%d):\r\n", n);
   send_string_no_lib(buffer);
   sprintf(buffer, "chr = %02X, shrt = %04X, lng = %08X\r\n", 
           (unsigned char)v->chr, (unsigned short)v->shrt, (unsigned long)v->lng);
   // or
   //sprintf(buffer, "chr = %02X, shrt = %04X, lng = %04X%04X\r\n", 
   //        (unsigned char)v->chr, (unsigned short)v->shrt, hs, ls);
   send_string_no_lib(buffer);
   sprintf(buffer, "chr = %c, shrt = %d, lng = %ld, flt=%f\r\n", 
           v->chr, v->shrt, v->lng, v->flt);
   send_string_no_lib(buffer);   
}

void print_union(UType* u, int n ) // Use pointer better
{
   unsigned short ls, hs;
   ls = u->lng & 0x0000FFFF;    // Low  two bytes
   hs = (u->lng) >> 16;         // High two bytes
   print_line();
   sprintf(buffer, "Printing UType fields at (%d):\r\n", n);
   send_string_no_lib(buffer);
   sprintf(buffer, "chr = %02X, shrt = %04X, lng = %04X%04X\r\n", 
           (unsigned char)u->chr, (unsigned short)u->shrt, ls, hs);
   send_string_no_lib(buffer);
   sprintf(buffer,"chr = %c, shrt = %d, lng = %ld, flt=%f\r\n", 
           u->chr, u->shrt, u->lng, u->flt);
   send_string_no_lib(buffer);   
}

void print_byte(unsigned char val, int n)
{
   UByte ub;
   ub.byt = val;
   print_line();
   sprintf(buffer, "Byte at (%d):", n);
   send_string_no_lib(buffer);
   sprintf(buffer, "%02XH: ", ub.byt);
   send_string_no_lib(buffer);
   sprintf(buffer, "%0x %0x %0x %0x %0x %0x %0x %0x\r\n",ub.b7,ub.b6,
           ub.b5,ub.b4,ub.b3,ub.b2,ub.b1,ub.b0);
   send_string_no_lib(buffer);
}

int main(void)
{
    setupPorts();
    setupSerial();
    
    sprintf(buffer, "Enter a statement below:\n\r");
    send_string_no_lib(buffer);
    
    extern volatile char gotten __at(0x020);
    gotten = 'a';
    char * ptr = &gotten;
    
    for(short i=0; i<15; i++) {
        while(1) {
            delay_ms(10);
            if(is_byte_available()) {
                *ptr = read_byte_no_lib();
                ptr++;
                break;
            }
        }
    }
    
    for(ptr = &gotten; *ptr; ++ptr)
        if(*ptr >= 'A' && *ptr <= 'Z')
            *ptr += 32;
        else if(*ptr >= 'a' && *ptr <= 'z')
            *ptr -= 32;
    
    ptr = &gotten;
    sprintf(buffer, "\n\rReceived string: ");
    send_string_no_lib(buffer);
    send_string_no_lib(ptr);
    sprintf(buffer, "\r\n");
    send_string_no_lib(buffer);
    print_line();
    
    while(1) { CLRWDT(); }
    return 0;
}

