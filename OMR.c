/*
 * File:   OMR.c
 * Author: DeLoge & Gaudette
 *
 * Programmed in C for the dsPIC30F4011 micro-controller.
 * This program uses multiple sensors to control a motor.
 *
 * Created on May 11, 2014, 1:08 PM
 */
////////////////////////////////////////////////////////////////////////////////
//
// INCLUDES / CONFIGURATION SETTINGS / DEFINES / FUNCTIONS / INTERRUPTS / GLOBAL
//
////////////////////////////////////////////////////////////////////////////////

// INCLUDES
#include <xc.h>
#include <stdio.h>
#include <libpic30.h>
#include "OMR.h"

// CONFIGURATION SETTINGS
_FOSC(CSW_FSCM_OFF & FRC_PLL16); // Fosc=16x7.5MHz, 120MHz/4 = Fcy=30MHz = 33.333ns
_FWDT(WDT_OFF);                  // Watchdog timer off
_FBORPOR(MCLR_DIS);              // Disable reset pin

// DEFINES

// uC
#define FCY 30000000ULL
#define ON 1
#define OFF 0
#define UP 1
#define DOWN 0
#define CW 0
#define CCW 1

// LCD
#define E_PIN  _LATD1   // LCD Enable Pin
#define RS_PIN _LATD2   // LCD Register Select
#define RW_PIN _LATD3   // LCD Read/Write Pin
#define GO_LINE1 0x02
#define GO_LINE2 0xC0
#define RESET_LCD PORTBbits.RB8
#define MAX_USER_INPUTS 7

// MOTOR
#define MAX_CPR 719         // x2 Mode
#define RESOLUTION_RPM 100  // Header File Too
#define MAXSPEED (unsigned int)((unsigned long)32752) // Two Directions
#define HALFMAXSPEED (MAXSPEED>>1)

// PWM
#define FREQ_PWM 28900
#define RESOLUTION_PWM 1024 // 2^11 bits; 2048/2
#define DEAD_TIME _DTA      // Set in intialization of PWM

// ACCELEROMETER
#define ACTIVATED 20        // 20 Degrees of Bar

// Bar
#define LOWEST_POS 1        // 1" from floor
#define DESIRED_POS 17      // 17" from floor
#define KP 1                // Proportional Controller Constant
#define KI 1                // Intergral Controller Constant
#define KD 1                // Derivative Controller Constant

// STATE LEDS
#define RED _LATF4
#define GREEN _LATF0
#define YELLOW _LATF1
#define RESET _LATF5

// STRUCTURES
struct controller pid;
struct sensors sensor;
struct drive motor;
struct display lcd;
struct vars count;

// FUNCTIONS
void System_Ready(void);

void Delay_Milli_Sec(unsigned int n);
void Delay_Micro_Sec(unsigned long n);

void Send_Nibble(unsigned char nibble);
void Send_Command_Byte(unsigned char byte);
void Send_Data_Byte(unsigned char byte);
void Dispaly_LCD(void);

void InitPins(void);
void InitQEI(void);
void InitTMR1(void);
void InitTMR2(void);
void InitTMR3(void);
void InitTMR4(void);
void InitPWM(void);
void InitLCD(void);
void InitAD(void);

void Motor_Position_Calculation(void);
void Motor_RPM_Calculation(void);
void Set_Duty_Cycle(void);
void Position_PID(void);
int  Get_Digits(int n);
unsigned int Read_Analog_Channel(int v);

// INTERRUPTS
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt (void);
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt (void);
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt (void);
void __attribute__((__interrupt__, __auto_psv__)) _T4Interrupt (void);
void __attribute__((__interrupt__, __auto_psv__)) _QEIInterrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _PWMInterrupt(void);

// GLOBAL VARIABLES



////////////////////////////////////////////////////////////////////////////////
//
//                          MAIN FUNCTION
//
////////////////////////////////////////////////////////////////////////////////

int main()
{

//------------------------------------------------------------------------------
//                            INITIALIZATION
//------------------------------------------------------------------------------
    
    InitPins();
    InitLCD();
    InitAD();
    InitPWM();
    InitQEI();
    InitTMR1();
    InitTMR2();
    InitTMR3();
    InitTMR4();


//------------------------------------------------------------------------------
//                            LOOP FOREVER
//------------------------------------------------------------------------------

    while(1)
    {
        // System Ready? Loop until it is.
        if (sensor.degree_bar <= ACTIVATED) {
            T4CONbits.TON = OFF;  // Turn OFF Timer
            _PTEN = OFF;          // Turn OFF PWM
            System_Ready();
        }

//                Delay_Milli_Sec(3000);
//                motor.dutycycle_pwm = 0.25;
//                Set_Duty_Cycle();
//
//                Delay_Milli_Sec(3000);
//                motor.dutycycle_pwm = 0.50;
//                Set_Duty_Cycle();
//
//                Delay_Milli_Sec(3000);
//                motor.dutycycle_pwm = 0.75;
//                Set_Duty_Cycle();
//
//                Delay_Milli_Sec(3000);
//                motor.dutycycle_pwm = 0.50;
//                Set_Duty_Cycle();

        if (sensor.distance_bar_inch <= LOWEST_POS)
            count.direction_bar = UP; // Bar trying to go up now

        while (count.direction_bar == UP)
        {
            sensor.copy_bar_inch = sensor.distance_bar_inch;
            Delay_Milli_Sec(500);
            if (sensor.copy_bar_inch + 0.5 >= sensor.distance_bar_inch)
            {
                T4CONbits.TON = ON;  // Turn On Timer
                _PTEN = ON;          // Turn On PWM

                while (count.direction_bar == UP)
                {
                    Position_PID(); // Control PWM
                }

                T4CONbits.TON = OFF;  // Turn off Timer
                _PTEN = OFF;          // Turn off PWM
            }

        }   // End While
    }  // End While

} // End Main

////////////////////////////////////////////////////////////////////////////////
//
//                                  PID
//
////////////////////////////////////////////////////////////////////////////////

void Position_PID(void)  // Set at 100 Hz
{

    pid.error_kp = DESIRED_POS - sensor.distance_bar_inch;
    




}


////////////////////////////////////////////////////////////////////////////////
//
//                             MISCELLANEOUS
//
////////////////////////////////////////////////////////////////////////////////

void System_Ready(void)
{
    // System Deactivated
    while (sensor.degree_bar <= ACTIVATED)
    {
        RED = ON;
        GREEN = OFF;
        YELLOW = OFF;
    }

    // System Activated
    GREEN = ON;
    YELLOW = OFF;
    RED = OFF;

    T4CONbits.TON = ON;  // Turn On Timer
    _PTEN = ON;          // Turn On PWM

}

unsigned int Read_Analog_Channel(int channel)
{
	ADCHS = channel;            // Select Channel
	ADCON1bits.SAMP = ON;        // Start Sampling
	Delay_Micro_Sec(1);
	ADCON1bits.SAMP = OFF;        // Start Converting
	while (!ADCON1bits.DONE);   // 1.98us : 12*Tad

        return ADCBUF0;

}

void Delay_Milli_Sec(unsigned int n)
{
        __delay32(n*(FCY/1000ULL));  // Delay n milliseconds

}

void Delay_Micro_Sec(unsigned long n)
{
        __delay32(n*(FCY/1000000ULL)); // Delay n microseconds

}

////////////////////////////////////////////////////////////////////////////////
//
//                                  LCD
//
////////////////////////////////////////////////////////////////////////////////

void Send_Command_Byte(unsigned char byte)
{

    RS_PIN = 0;                 // 0 = Send Command Byte
    Send_Nibble(byte>>4);       // Send 4 Bits
    Send_Nibble(byte & 0xf);

}

void Send_Data_Byte(unsigned char byte)
{
    RS_PIN = 1;                 // 1 = Send Data Byte
    Send_Nibble(byte>>4);       // Send 4 Bits
    Send_Nibble(byte & 0xF);

}

void Send_Nibble(unsigned char nibble)
{
    // Shift characters to the correct pin
    LATB = 0x0000;
    LATB = nibble;

    // Send Data
    Delay_Micro_Sec(1000);
    E_PIN = 0;
    Delay_Micro_Sec(1000);
    E_PIN = 1;
    Delay_Micro_Sec(2000);   // Longest Possible Character

}

int Get_Digits(int n)
{

    // Return Number of Digits in #
    if ( n < 10 ) return 1;
    if ( n < 100 ) return 2;
    if ( n < 1000 ) return 3;
    if ( n < 10000 ) return 4;
    if ( n < 100000 ) return 5;
    if ( n < 1000000 ) return 6;
    if ( n < 10000000 ) return 7;
    if ( n < 100000000 ) return 8;

    return 8;

}

void Display_LCD (void)
{
    
        // Check user input to select which display to be shown on LCD
        if ((PORTDbits.RD0 == 0) && (count.debounce == 1))
           count.debounce = 0;

        if ((PORTDbits.RD0 == 1) && (count.debounce == 0))
        {
           if(count.input_user == MAX_USER_INPUTS)
               count.input_user = 0;
           else
               count.input_user++;

           count.copy_input_user = count.input_user;
           count.debounce = 1;
        }

        // Reset LCD Values
        if (RESET_LCD != 0)
           {
               motor.rpm = 0;
               POSCNT = 0;
               count.encoder = 0;
           }

        // Dependent on user input (button press) for LCD display
        switch (count.input_user)
        {
            case 0:
                if (count.copy_input_user != 10)
                {
                sprintf(lcd.line1, "%s", "# of CPR");
                    Send_Command_Byte(GO_LINE1);
                    for (count.i=0 ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line1[count.i]);}
                    count.copy_input_user = 10;
                }

                sprintf(lcd.line2, "%u", POSCNT);
                    count.digits = Get_Digits(POSCNT);
                    Send_Command_Byte(GO_LINE2);
                    for (count.i=0 ; count.i<count.digits ; ++count.i) {Send_Data_Byte(lcd.line2[count.i]);}
                    for (count.i=count.digits ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line2[count.i] = ' ');}
            break;

            case 1:
                if (count.copy_input_user != 10)
                {
                sprintf(lcd.line1, "%s", "# of REV");
                    Send_Command_Byte(GO_LINE1);
                    for (count.i=0 ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line1[count.i]);}
                    count.copy_input_user = 10;
                }

                sprintf(lcd.line2, "%d", count.encoder);
                    count.digits = Get_Digits(count.encoder);
                    Send_Command_Byte(GO_LINE2); //
                    for (count.i=0 ; count.i<count.digits ; ++count.i) {Send_Data_Byte(lcd.line2[count.i]);}
                    for (count.i=count.digits ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line2[count.i] = ' ');}                   
            break;

            case 2:
                if (count.copy_input_user != 10)
                {
                sprintf(lcd.line1, "%s", "# INCHES");
                    Send_Command_Byte(GO_LINE1);
                    for (count.i=0 ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line1[count.i]);}
                    count.copy_input_user = 10;
                }

                sprintf(lcd.line2, "%f", (sensor.distance_bar_inch));
                    Send_Command_Byte(GO_LINE2);
                    for (count.i=0 ; count.i<4 ; ++count.i) {Send_Data_Byte(lcd.line2[count.i]);}
            break;

            case 3:
                if (count.copy_input_user != 10)
                {
                sprintf(lcd.line1, "%s", "Bar DEG ");
                    Send_Command_Byte(GO_LINE1);
                    for (count.i=0 ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line1[count.i]);}
                    count.copy_input_user = 10;
                }

                sprintf(lcd.line2, "%f", sensor.degree_bar);
                    Send_Command_Byte(GO_LINE2);
                    for (count.i=0 ; count.i<3 ; ++count.i) {Send_Data_Byte(lcd.line2[count.i]);}
            break;

            case 4:
                if (count.copy_input_user != 10)
                {
                sprintf(lcd.line1, "%s", "Mot RPM ");
                    Send_Command_Byte(GO_LINE1);
                    for (count.i=0 ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line1[count.i]);}
                    count.copy_input_user = 10;
                }

                sprintf(lcd.line2, "%f", motor.rpm);
                    Send_Command_Byte(GO_LINE2);
                    for (count.i=0 ; count.i<5 ; ++count.i) {Send_Data_Byte(lcd.line2[count.i]);}        
            break;

            case 5:
                if (count.copy_input_user != 10)
                {
                sprintf(lcd.line1, "%s", "Mot SPD ");
                    Send_Command_Byte(GO_LINE1);
                    for (count.i=0 ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line1[count.i]);}
                    count.copy_input_user = 10;
                }

                sprintf(lcd.line2, "%d", (motor.speed));
                    count.digits = Get_Digits(motor.speed);
                    Send_Command_Byte(GO_LINE2);
                    for (count.i=0 ; count.i<count.digits ; ++count.i) {Send_Data_Byte(lcd.line2[count.i]);}
                    for (count.i=count.digits ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line2[count.i] = ' ');}
            break;


            case 6:
                if (count.copy_input_user != 10)
                {
                sprintf(lcd.line1, "%s", "Mot I -B");
                    Send_Command_Byte(GO_LINE1);
                    for (count.i=0 ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line1[count.i]);}
                    count.copy_input_user = 10;
                }

                sprintf(lcd.line2, "%f", motor.current_A);
                    Send_Command_Byte(GO_LINE2);
                    for (count.i=0 ; count.i<4 ; ++count.i) {Send_Data_Byte(lcd.line2[count.i]);}
            break;


            case 7:
                if (count.copy_input_user != 10)
                {
                sprintf(lcd.line1, "%s", "Mot I -A");
                    Send_Command_Byte(GO_LINE1);
                    for (count.i=0 ; count.i<8 ; ++count.i) {Send_Data_Byte(lcd.line1[count.i]);}
                    count.copy_input_user = 10;
                }

                sprintf(lcd.line2, "%f", motor.current_B);
                    Send_Command_Byte(GO_LINE2);
                    for (count.i=0 ; count.i<4 ; ++count.i) {Send_Data_Byte(lcd.line2[count.i]);}
            break;
      }
    
}

////////////////////////////////////////////////////////////////////////////////
//
//                                  PWM
//
////////////////////////////////////////////////////////////////////////////////

void Set_Duty_Cycle(void)
{
    
    if (motor.direction == CW)  // Clockwise
    {
        PDC2 = RESOLUTION_PWM - DEAD_TIME - (RESOLUTION_PWM * motor.dutycycle_pwm);
    }
    
    if (motor.direction == CCW)  // Counter-Clockwise
    {
        PDC2 = RESOLUTION_PWM + DEAD_TIME + (RESOLUTION_PWM * motor.dutycycle_pwm);
    }
    
}


////////////////////////////////////////////////////////////////////////////////
//
//                                  MOTOR
//
////////////////////////////////////////////////////////////////////////////////


void Motor_Position_Calculation(void)
{
    
    motor.copy_POSCNT = (int)POSCNT;  // 0 <= POSCNT <= MAX_CPR
    
    if (motor.copy_POSCNT < 0)
        motor.copy_POSCNT = -motor.copy_POSCNT;
    
    motor.angle[1] = motor.angle[0]; // 0 <= motor.angle <= 32752
    motor.angle[0] = (unsigned int)((unsigned long)motor.copy_POSCNT * 45.55); //  * (MAXSPEED / MAX_CPR)

}

void Motor_RPM_Calculation(void)
{
    
    motor.speed = motor.angle[0] - motor.angle[1];  // Get Angle Change of Motor
    
    if (motor.speed >= 0)
    {
        if (motor.speed >= (HALFMAXSPEED))
        motor.speed = motor.speed - MAXSPEED;
    }
    
    else
    {
        if (motor.speed < -(HALFMAXSPEED))
        motor.speed = motor.speed + MAXSPEED;
    }
    
    motor.speed *= 2;

    motor.rpm = motor.speed * -0.0977;  // motor.speed * ((3200 RPM)/-MAXSPEED)
    
}

////////////////////////////////////////////////////////////////////////////////
//
//                                  SENSORS
//
////////////////////////////////////////////////////////////////////////////////


void Sensor_Distance(void)
{
    
    _LATC13 = ON;                     // Send Trigger
    Delay_Micro_Sec(10);
    _LATC13 = OFF;                     // Turn-Off Trigger
    while(_RC14 == 0);               // Wait for Signal
    TMR2 = 0;               
    while(_RC14 == 1);               // Receive Signal
    sensor.data_sound = TMR2;        // Return Time Taken

    // Calculate Position from Ultra Sonic
    sensor.distance_bar_inch = sensor.data_sound/16.8;
    
    // sensor.distance_bar_inch = (((sensor.data_sound * 8.533us * 340m/s)/2)*39.3701in/m) or 0.5711 * sensor.data_sound
    // sensor.distance_bar_cm = sensor.data_sound/6.61417; // @84 : 84 = 5" = 12.7 cm
    
}

void Sensor_Current(void)
{

    // Get Current Flow to Motor
    sensor.data_current = Read_Analog_Channel(7);
    motor.current_A = (sensor.data_current * 0.22727);

    sensor.data_current = Read_Analog_Channel(8);
    motor.current_B = (sensor.data_current * 0.22727);

    // motor.current = (sensor.data_current / 20 (V/V)) / 0.22 (ohms)

}

void Sensor_Angle(void)
{

    // Get Angle of Lifting Bar
    sensor.data_accel = Read_Analog_Channel(6);
    sensor.degree_bar = (sensor.data_accel * 0.6369) - 182;

    // sensor.degree_bar = (100/157) * sensor.data_accel - 182 degrees

}


////////////////////////////////////////////////////////////////////////////////
//
//                          INITIALIZATIONS
//
////////////////////////////////////////////////////////////////////////////////

void InitPins(void)
{
    // PINS
    TRISD = 0b0000; // RD0-3: Outputs - LCD (18,22,19,23)
    TRISB = 0xFFF0; // RB0-3: Outputs - LCD (2,3,4,5)
    _TRISF0 = 0;    // RF0: Output - Green LED (30)
    _TRISF1 = 0;    // RF1: Output - Yellow LED (29)
    _TRISF4 = 0;    // RF4: Output - Red LED (28)
    _TRISF5 = 1;    // RF5: Input - Reset (27)
    _TRISC13 = 0;   // RC13: Digital Output - UltraSonic Trigger (15)
    _TRISC14 = 1;   // RC14: Input - UltraSonic Echo (16)
    TRISBbits.TRISB8 = 1;    // RB8: Input - Reset Variables on LCD (10)
    TRISE = 0b001100;        // Set PWM2 pins as Outputs

}

void InitAD(void)
{

    // Configure A/D Converter

    ADPCFGbits.PCFG4 = 1;    // QEA: Input - Encoder Channel A (6)
    ADPCFGbits.PCFG5 = 1;    // QEB: Input - Encoder Channel B (7)
    ADPCFGbits.PCFG6 = 0;    // AN6: Input - Accelermoter Z - (8)
    ADPCFGbits.PCFG7 = 0;    // AN7: Input - Current Monitor A (9)
    ADPCFGbits.PCFG8 = 0;    // AN8: Input - Current Monitor B (10)

    ADCON1 = 0;              // Enable Conversion
    ADCON2 = 0;              // AVDD and AVSS - Voltage Reference

    // DO NOT WRITE ADCS WHILE ADON = 1!
    ADCON3 = 0x0009;        // ADCS=9 -> Tad = 5*Tcy = 165ns
    ADCON1bits.ADON = 1;    // Turn ADC ON

}

void InitLCD(void)
{
    // Setup LCD
    RW_PIN = 0;
    RS_PIN = 0;
    E_PIN = 1;

    // Initialization LCD
    Delay_Milli_Sec(16);   // +15ms
    Send_Nibble(0b0011);
    Delay_Milli_Sec(5);    // +4.1ms
    Send_Nibble(0b0011);
    Delay_Micro_Sec(110);  // +100us
    Send_Nibble(0b0011);
    Delay_Milli_Sec(5);    // +4.1ms
    Send_Nibble(0b0010);   // 4-bit mode

    // Display Settings LCD
    Send_Command_Byte(0b00101000); // N=1 : 2 Lines, F=0, 5x8 Font
    Send_Command_Byte(0b00001000); // Display: Display Off, Cursor Cff, Blink Off
    Send_Command_Byte(0b00000001); // Clear Display
    Send_Command_Byte(0b00000110); // Set Entry Mode: ID=1, S=0
    Send_Command_Byte(0b00001100); // Display: Display On, Cursor On, Blink On

}

void InitTMR1(void) // Display & Accelermeter
{
    T1CONbits.TON = 0;       // Turn Timer Off
    T1CONbits.TSIDL = 0;     // Continue Operation During Sleep
    T1CONbits.TGATE = 0;     // Gated Timer Accumulation Disabled
    T1CONbits.TCS = 0;       // Tcy
    T1CONbits.TCKPS = 0b11;  // 1:256 = 8.533us
    
    PR1 = 58596;             // Interrupt Period 0.5s
    IFS0bits.T1IF = 0;       // Clear TMR1 Interrupt Flag
    IEC0bits.T1IE = 1;       // Enable TMR1 Interrupts
    T1CONbits.TON = 1;       // Turn On TMR1
    
    TMR1 = 0;                // Reset Timer Counter

}

void InitTMR2(void)  // Ultra Sonic & LCD
{
    T2CONbits.TON = 0;       // Turn Timer Off
    T2CONbits.TSIDL = 0;     // Continue Operation During Sleep
    T2CONbits.TGATE = 0;     // Gated Timer Accumulation Disabled
    T2CONbits.TCS = 0;       // Tcy
    T2CONbits.TCKPS = 0b11;  // 1:256 = 8.533us

    PR2 = 35157;             // @ 0.3s (Interrupt Period = 0.00147s -> 172: 256 prescaler)
    IFS0bits.T2IF = 0;       // Clear TMR2 Interrupt Flag
    IEC0bits.T2IE = 1;       // Enable TMR2 Interrupts
    T2CONbits.TON = 1;       // Turn On TMR2
    TMR2 = 0;                // Reset Timer Counter

}

void InitTMR3(void)     // Motor CPR
{
    T3CONbits.TON = 0;       // Turn Timer Off
    T3CONbits.TSIDL = 0;     // Continue Operation During Sleep
    T3CONbits.TGATE = 0;     // Gated Timer Accumulation Disabled
    T3CONbits.TCS = 0;       // Tcy
    T3CONbits.TCKPS = 0b01;  // 1:8 = 266.667ns

    PR3 = 33750;             // @ 0.009s Interrupt Period @ 3200 Max RPM (0.012s for 2500 RPM)
    IFS0bits.T3IF = 0;       // Clear TMR3 Flag
    IEC0bits.T3IE = 1;       // Enable TMR3
    T3CONbits.TON = 1;       // Turn TMR3 On
    TMR3 = 0;                // Reset TMR3

}

void InitTMR4(void)     // PID Controller
{
    T4CONbits.TON = 0;       // Turn Timer Off
    T4CONbits.TSIDL = 0;     // Continue Operation During Sleep
    T4CONbits.TGATE = 0;     // Gated Timer Accumulation Disabled
    T4CONbits.TCS = 0;       // Tcy
    T4CONbits.TCKPS = 0b11;  // 1:256 = 8.53us

    PR4 = 11718;             // @ 0.1s Interrupt Period
    IFS1bits.T4IF = 0;       // Clear TMR4 Interrupt Flag
    IEC1bits.T4IE = 1;       // Enable TMR4 Interrupt
    T4CONbits.TON = 0;       // Turn TMR4 On (Off)
    TMR4 = 0;                // Reset TMR4

}

void InitQEI(void)  // Encoder
{

    QEICONbits.CNTERR = 0;  // Clear any count errors
    QEICONbits.QEISIDL = 0; // Continue operation during sleep
    QEICONbits.UPDN = 1;    // Position Counter Direction is negative
    QEICONbits.QEIM = 0b101;// (7 = x4) (5 = x2) mode with position counter reset by Match
    QEICONbits.SWPAB = 0;   // QEA and QEB not swapped
    QEICONbits.PCDOUT = 0;  // Normal I/O pin operation *QEI Logic Controls
    QEICONbits.TQGATE = 0;  // Timer gated time accumulation disabled
    QEICONbits.TQCKPS = 0;  // 1:64 (2) Timer Prescale for timer mode only
    QEICONbits.TQCS = 0;    // Internal Clock
    QEICONbits.UPDN_SRC = 0;// QEB defines direction

    DFLTCONbits.CEID = 0;      // Count error interrupts disabled
    DFLTCONbits.QEOUT = 1;     // Digital filters output enabled
    DFLTCONbits.QECK = 0b101;  // 1:64 (5) Prescaler for digital filter

    POSCNT = 0;            // Reset position counter
    MAXCNT = MAX_CPR;      // Set encoder max position

//    IPC10bits.QEIIP = 3; // QEI interrupt highest priority(7)
//    IEC2bits.QEIIE = 1;  // QEI interrupt enable bit
//    IFS2bits.QEIIF = 0;  // QEI output flag

}

void InitPWM(void) // PWM
{
    PTCON = 0;          // Timer Off, No-Scaling, FreeRun Mode, Runs in Idle
    OVDCON = 0;         // Disable all PWM outputs
    PTMR = 0;           // Timer counts up, Clears timer register
    PTPER = 1023;       // PWM timer for 30MIPS @ 29.296 kHZ
    PWMCON1 = 0x0022;   // Set CH. 2 to PWM and make Complementary
    DTCON1 = 0;         // Sets Dead Time to 0 with no Pre-Scalers
    DTCON1bits.DTAPS = 0; // Sets Clock Period = TCY
    DTCON1bits.DTA = 9;   // Sets Dead Time to 300ns
    OVDCON = 0x0F00;      // Output is controlled by PWM Generator
//  PDC1 = PTPER;         // Sets PWM1 Duty Cycle to 50%
    PDC2 = 924;           // Sets PWM2 Duty Cycle to CW 10%

//    IFS2bits.PWMIF = 0;       // Clear PWM Interrupt
//    IEC2bits.PWMIE = 1;       // Enable PWM Interrupts
//    SEVTCMPbits.SEVTDIR = 0;  // Special Event Trigger Compared to PTMR

//    PTCONbits.PTEN = 1;       // Timer On

}

////////////////////////////////////////////////////////////////////////////////
//
//                             INTERRUPTS
//
////////////////////////////////////////////////////////////////////////////////

// SENSORS INTERRUPT (ACCELEROMETER / LCD DISPLAY)
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt (void)
{

    IFS0bits.T1IF = 0; // Clear TMR1 Interrupt Flag

    Sensor_Angle();
    Display_LCD();

}

// SENSORS INTERRUPT (CURRENT MONITOR / ULTRA SONIC)
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt (void)
{
    IFS0bits.T2IF = 0; // Clear TMR2 Interrupt Flag

    Sensor_Distance();
    Sensor_Current();

}

// MOTOR INTERRUPT
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt (void)
{
    
    IFS0bits.T3IF = 0;          // Clear TMR3 Interrupt Flag

    Motor_Position_Calculation();
    Motor_RPM_Calculation();

}

// PID INTERRUPT
void __attribute__((__interrupt__, __auto_psv__)) _T4Interrupt (void)
{

    IFS1bits.T4IF = 0;          // Clear TMR4 Interrupt Flag

    //Position_PID();

}

// ENCODER INTERRUPT
void __attribute__((__interrupt__, __auto_psv__)) _QEIInterrupt(void)
{
    
     IFS2bits.QEIIF = 0;    // Clear QEI Interrupt Flag
     count.encoder++;       // Count Encoder Revolutions

}

// PWM INTERRUPT
void __attribute__((__interrupt__, __auto_psv__)) _PWMInterrupt(void)
{

    IFS2bits.PWMIF = 0; // Clear PWM Interrupt Flag
    
}
