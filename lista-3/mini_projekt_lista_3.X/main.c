#include <xc.h>
#include <stdio.h>

// KONFIGURACJA
#pragma config FOSC = HS
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config BOREN = ON
#pragma config LVP = OFF
#pragma config CPD = OFF
#pragma config WRT = OFF
#pragma config CP = OFF

#define _XTAL_FREQ 20000000

// PRZYCISKI
#define START_BTN PORTCbits.RC0
#define STOP_BTN  PORTCbits.RC1
#define RESET_BTN PORTCbits.RC2
#define LAP_BTN   PORTCbits.RC3
#define UP_BTN    PORTCbits.RC4
#define DOWN_BTN  PORTCbits.RC5

// CZAS
volatile unsigned int cs = 0;
volatile unsigned int sec = 0;
volatile unsigned int min = 0;

volatile char running = 0;

// LAPY
#define MAX_LAPS 10

unsigned int lap_cs[MAX_LAPS];
unsigned int lap_sec[MAX_LAPS];
unsigned int lap_min[MAX_LAPS];

unsigned int lap_count = 0;
int scroll_index = 0;

// TIMER
void __interrupt() ISR() {
    if (TMR1IF) {
        TMR1IF = 0;
        TMR1 = 65536 - 50000;

        if (running) {
            cs++;
            if (cs >= 100) {
                cs = 0;
                sec++;
            }
            if (sec >= 60) {
                sec = 0;
                min++;
            }
        }
    }
}

// LAP
void save_lap() {
    if (lap_count < MAX_LAPS) {
        lap_cs[lap_count] = cs;
        lap_sec[lap_count] = sec;
        lap_min[lap_count] = min;
        lap_count++;
    }
    scroll_index = (int)lap_count - 1;
}

// RESET
void reset_all() {
    running = 0;
    cs = sec = min = 0;
    lap_count = 0;
    scroll_index = 0;
}

#define RS RD0
#define EN RD1
#define D4 RD2
#define D5 RD3
#define D6 RD4
#define D7 RD5

void lcd_pulse() {
    EN = 1;
    __delay_us(1);
    EN = 0;
    __delay_us(100);
}

void lcd_send_nibble(unsigned char nibble) {
    D4 = (nibble >> 0) & 1;
    D5 = (nibble >> 1) & 1;
    D6 = (nibble >> 2) & 1;
    D7 = (nibble >> 3) & 1;
    lcd_pulse();
}

void lcd_cmd(unsigned char cmd) {
    RS = 0;
    lcd_send_nibble(cmd >> 4);
    lcd_send_nibble(cmd & 0x0F);
    __delay_ms(2);
}

void lcd_data(unsigned char data) {
    RS = 1;
    lcd_send_nibble(data >> 4);
    lcd_send_nibble(data & 0x0F);
    __delay_ms(2);
}

void lcd_init() {
    TRISD = 0x00;

    __delay_ms(20);

    RS = 0;
    EN = 0;

    lcd_send_nibble(0x03);
    __delay_ms(5);
    lcd_send_nibble(0x03);
    __delay_us(100);
    lcd_send_nibble(0x03);
    lcd_send_nibble(0x02);

    lcd_cmd(0x28); // 4-bit, 2-line (dzia?a te? dla 4)
    lcd_cmd(0x0C); // display ON
    lcd_cmd(0x06); // increment
    lcd_cmd(0x01); // clear
}

void lcd_set_cursor(unsigned char row, unsigned char col) {
    unsigned char pos;
    switch(row) {
        case 0: pos = 0x80 + col; break;
        case 1: pos = 0xC0 + col; break;
        case 2: pos = 0x90 + col; break;
        case 3: pos = 0xD0 + col; break;
    }
    lcd_cmd(pos);
}

void lcd_print(char *str) {
    while(*str) lcd_data(*str++);
}

void lcd_print_2d(unsigned int n) {
    lcd_data((char)((n / 10) + '0')); // Dziesi?tki
    lcd_data((char)((n % 10) + '0')); // Jedno?ci
}

void display() {
    // Linia 0: Czas g?ówny
    lcd_set_cursor(0, 0);
    lcd_print("TIME ");
    lcd_print_2d(min);
    lcd_data(':');
    lcd_print_2d(sec);
    lcd_data(':');
    lcd_print_2d(cs);

    // Linie 1-3: Okr??enia
    for (int i = 0; i < 3; i++) {
        int idx = scroll_index - i;
        lcd_set_cursor((unsigned char)(i + 1), 0);

        if (idx >= 0 && idx < (int)lap_count) {
            lcd_print("L");
            lcd_print_2d(idx + 1);
            lcd_print(" ");
            lcd_print_2d(lap_min[idx]);
            lcd_data(':');
            lcd_print_2d(lap_sec[idx]);
            lcd_data(':');
            lcd_print_2d(lap_cs[idx]);
        } else {
            lcd_print(""); // Czyszczenie linii
        }
    }
}

// MAIN
void main(void) {

    TRISC = 0xFF; // przyciski
    TRISD = 0x00;

    // TIMER1
    TMR1 = 65536 - 50000;
    TMR1CS = 0;
    TMR1ON = 1;
    TMR1IE = 1;

    PEIE = 1;
    GIE = 1;

    lcd_init();
    ADCON1 = 0x06;           // cyfrowe piny
    
    while(1) {

        if (!START_BTN) {
            __delay_ms(200);
            running = 1;
        }

        if (!STOP_BTN) {
            __delay_ms(200);
            running = 0;
        }

        if (!RESET_BTN) {
            __delay_ms(200);
            reset_all();
        }

        if (!LAP_BTN) {
            __delay_ms(200);
            if (running) save_lap();
        }

        if (!UP_BTN) {
            __delay_ms(150);
            if (scroll_index < lap_count - 1)
                scroll_index++;
        }

        if (!DOWN_BTN) {
            __delay_ms(150);
            if (scroll_index > 0)
                scroll_index--;
        }
        
        display();
        __delay_ms(100);
    }
}