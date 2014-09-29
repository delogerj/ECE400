/*
 * File:   OMR.c
 * Author: DeLoge
 *
 * Programmed in C for the dsPIC30F4011 microcontroller.
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

// CONFIGURATION SETTINGS
_FOSC(CSW_FSCM_OFF & FRC_PLL16); // Fosc=16x7.5MHz, 120MHz/4 = Fcy=30MHz = 33.333ns
_FWDT(WDT_OFF);                  // Watchdog timer off
_FBORPOR(MCLR_DIS);              // Disable reset pin

// DEFINES

// uC
#define FCY 30000000ULL

// LCD
#define E_PIN  _LATD1
#define RS_PIN _LATD2
#define RW_PIN _LATD3
#define GO_LINE1 0x02
#define GO_LINE2 0xC0
#define RESET_LCD PORTBbits.RB8

// MOTOR
#define MAX_CPR 1068
#define RESOLUTION_RPM 100

// PWM
#define FREQ_PWM 28900
#define RESOLUTION_PWM 1024
#define DEAD_TIME _DTA

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
void InitPWM(void);
void InitLCD(void);
void InitAD(void);

void Get_Motor_Position(void);
void Get_RPM(void);
int  Get_Digits(int n);
unsigned int Read_Analog_Channel(int v);
unsigned int Read_Analog_Channel1(int v);

void Set_Duty_Cycle(void);

// INTERRUPTS
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt (void);
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt (void);
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt (void);
void __attribute__((__interrupt__, __auto_psv__)) _QEIInterrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _PWMInterrupt(void);

// GLOBAL VARIABLES
double rpm_MOTOR = 0;
double ang_MOTOR[3] = {0,0,0};
double current_MOTOR = 0;

double avg_RPM[RESOLUTION_RPM] = {0};
double dist_IN = 0;
//double dist_CM = 0;
double deg_LIFT_BAR = 0;
double dc_PWM = 0;
unsigned int data_ACCEL = 0;
unsigned int data_CURRENT = 0;

int count_ENCODER = 0;
int ang_POS[2] = {0,0};
int speed_MOTOR = 0;
int data_SOUND = 0;
int array_INDEX = 0;
int count_RPM = 0;
int loop_CYCLE = 0;
int total_DIGITS = 0;
int user_DEBOUNCE = 0;
int user_INPUT = 0;
int direction_MOTOR = 0;
long long int delay_LCD = 0;
int i;
    
char print_LINE1[8], print_LINE2[8];

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
    InitTMR1();
    InitTMR3();
    InitTMR2();
    InitLCD();
    InitAD();
    InitQEI();
    InitPWM();

//------------------------------------------------------------------------------
//                            LOOP FOREVER
//------------------------------------------------------------------------------

    while(1)
    {
        // System Ready? Loop until it is.
        if (deg_LIFT_BAR <= 20) {System_Ready();}

        PTCONbits.PTEN = 1; // Timer On

        dc_PWM = 0.25;
        Set_Duty_Cycle();
        Delay_Milli_Sec(3000);

        dc_PWM = 0.75;
        Set_Duty_Cycle();
        Delay_Milli_Sec(3000);


    }  // End While

} // End Main

////////////////////////////////////////////////////////////////////////////////
//
//                             MISCELLANEOUS
//
////////////////////////////////////////////////////////////////////////////////

void System_Ready(void)
{
    // System Deactivated
    while (deg_LIFT_BAR <= 20)
    {
        _LATF1 = 1;
        _LATF0 = 0;
    }

    // System Activated
        _LATF0 = 1;
        _LATF1 = 0;

    return;

}

unsigned int Read_Analog_Channel(int channel)
{
	ADCHS = channel;            // Select Channel
	ADCON1bits.SAMP = 1;        // Start Sampling
	Delay_Micro_Sec(1);
	ADCON1bits.SAMP = 0;        // Start Converting
	while (!ADCON1bits.DONE);   // 1.98us : 12*Tad

        return ADCBUF0;

}

void Delay_Milli_Sec(unsigned int n)
{
        __delay32(n*(FCY/1000ULL));  // Delay n milliseconds

        return;

}

void Delay_Micro_Sec(unsigned long n)
{
        __delay32(n*(FCY/1000000ULL)); // Delay n microseconds

        return;

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

    return;

}

void Send_Data_Byte(unsigned char byte)
{
    RS_PIN = 1;                 // 1 = Send Data Byte
    Send_Nibble(byte>>4);       // Send 4 Bits
    Send_Nibble(byte & 0xF);

    return;

}

void Send_Nibble(unsigned char nibble)
{
    // Shift characters to the correct pin
    LATB = 0x0000;
    LATBbits.LATB0 = (nibble & 0x1);
    LATBbits.LATB1 = (nibble & 0x2)>>1;
    LATBbits.LATB2 = (nibble & 0x4)>>2;
    LATBbits.LATB7 = (nibble & 0x8)>>3;

    // Data is latched on edge of pin E
    Delay_Micro_Sec(1000);
    E_PIN = 0;
    Delay_Micro_Sec(1000);
    E_PIN = 1;
    Delay_Micro_Sec(2000);   // Longest Possible Character

    return;

}

int Get_Digits(int n)
{

    // Return # of Place Holders in #
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
        if ((PORTDbits.RD0 == 0) && (user_DEBOUNCE == 1))
           user_DEBOUNCE = 0;

        if ((PORTDbits.RD0 == 1) && (user_DEBOUNCE == 0))
        {
           if(user_INPUT == 6)
               user_INPUT = 0;
           else
            user_INPUT++;

           user_DEBOUNCE = 1;
        }

        // Reset LCD Values
        if (RESET_LCD != 0)
           {
               rpm_MOTOR = 0;
               POSCNT = 0;
               count_ENCODER = 0;
           }

        // Dependent on user input (button press) for LCD display
        switch (user_INPUT)
        {
            case 0:
                sprintf(print_LINE1, "%s", "# of CPR");
                    Send_Command_Byte(GO_LINE1);
                    for (i=0 ; i<8 ; ++i) {Send_Data_Byte(print_LINE1[i]);}

                sprintf(print_LINE2, "%u", POSCNT);
                    total_DIGITS = Get_Digits(POSCNT);
                    Send_Command_Byte(GO_LINE2);
                    for (i=0 ; i<total_DIGITS ; ++i) {Send_Data_Byte(print_LINE2[i]);}
                    for (i=total_DIGITS ; i<8 ; ++i) {Send_Data_Byte(print_LINE2[i] = ' ');}
            break;

            case 1:
                sprintf(print_LINE1, "%s", "# of REV");
                    Send_Command_Byte(GO_LINE1);
                    for (i=0 ; i<8 ; ++i) {Send_Data_Byte(print_LINE1[i]);}

                sprintf(print_LINE2, "%d", count_ENCODER);
                    total_DIGITS = Get_Digits(count_ENCODER);
                    Send_Command_Byte(GO_LINE2); //
                    for (i=0 ; i<total_DIGITS ; ++i) {Send_Data_Byte(print_LINE2[i]);}
                    for (i=total_DIGITS ; i<8 ; ++i) {Send_Data_Byte(print_LINE2[i] = ' ');}
            break;

            case 2:
                sprintf(print_LINE1, "%s", "# Inches");
                    Send_Command_Byte(GO_LINE1);
                    for (i=0 ; i<8 ; ++i) {Send_Data_Byte(print_LINE1[i]);}

                sprintf(print_LINE2, "%f", (dist_IN));
                    total_DIGITS = Get_Digits(dist_IN);
                    Send_Command_Byte(GO_LINE2);
                    for (i=0 ; i<total_DIGITS+3 ; ++i) {Send_Data_Byte(print_LINE2[i]);}
                    for (i=total_DIGITS+3 ; i<8 ; ++i) {Send_Data_Byte(print_LINE2[i] = ' ');}
            break;

            case 3:
                sprintf(print_LINE1, "%s", "Bar Deg ");
                    Send_Command_Byte(GO_LINE1);
                    for (i=0 ; i<8 ; ++i) {Send_Data_Byte(print_LINE1[i]);}

                sprintf(print_LINE2, "%f", deg_LIFT_BAR);
                    total_DIGITS = Get_Digits(deg_LIFT_BAR);
                    Send_Command_Byte(GO_LINE2);
                    for (i=0 ; i<total_DIGITS+2 ; ++i) {Send_Data_Byte(print_LINE2[i]);}
                    for (i=total_DIGITS+2 ; i<8 ; ++i) {Send_Data_Byte(print_LINE2[i] = ' ');}
            break;

            case 4:
                sprintf(print_LINE1, "%s", "Mot RPM ");
                    Send_Command_Byte(GO_LINE1);
                    for (i=0 ; i<8 ; ++i) {Send_Data_Byte(print_LINE1[i]);}

                sprintf(print_LINE2, "%f", rpm_MOTOR);
                    total_DIGITS = Get_Digits(rpm_MOTOR);
                    Send_Command_Byte(GO_LINE2);
                    for (i=0 ; i<total_DIGITS+2 ; ++i) {Send_Data_Byte(print_LINE2[i]);}
                    for (i=total_DIGITS+2 ; i<8 ; ++i) {Send_Data_Byte(print_LINE2[i] = ' ');}
            break;

            case 5:
                sprintf(print_LINE1, "%s", "Mot SPD ");
                    Send_Command_Byte(GO_LINE1);
                    for (i=0 ; i<8 ; ++i) {Send_Data_Byte(print_LINE1[i]);}

                sprintf(print_LINE2, "%d", (speed_MOTOR));
                    total_DIGITS = Get_Digits(speed_MOTOR);
                    Send_Command_Byte(GO_LINE2);
                    for (i=0 ; i<total_DIGITS ; ++i) {Send_Data_Byte(print_LINE2[i]);}
                    for (i=total_DIGITS ; i<8 ; ++i) {Send_Data_Byte(print_LINE2[i] = ' ');}
            break;


        case 6:
            sprintf(print_LINE1, "%s", "Mot Curr");
                Send_Command_Byte(GO_LINE1);
                for (i=0 ; i<8 ; ++i) {Send_Data_Byte(print_LINE1[i]);}

            sprintf(print_LINE2, "%f", current_MOTOR);
                total_DIGITS = Get_Digits(current_MOTOR);
                Send_Command_Byte(GO_LINE2);
                for (i=0 ; i<total_DIGITS+2 ; ++i) {Send_Data_Byte(print_LINE2[i]);}
                for (i=total_DIGITS+2 ; i<8 ; ++i) {Send_Data_Byte(print_LINE2[i] = ' ');}
        break;
      }

      return;
    
}

////////////////////////////////////////////////////////////////////////////////
//
//                                  PWM
//
////////////////////////////////////////////////////////////////////////////////

void Set_Duty_Cycle(void)
{
    
    if (direction_MOTOR == 0)  // Clockwise
    {
        PDC2 = RESOLUTION_PWM - DEAD_TIME - (RESOLUTION_PWM * dc_PWM);
    }
    
    if (direction_MOTOR == 1)  // Counter-Clockwise
    {
        PDC2 = RESOLUTION_PWM + DEAD_TIME + (RESOLUTION_PWM * dc_PWM);
    }

    return;
    
}


////////////////////////////////////////////////////////////////////////////////
//
//                                  MOTOR
//
////////////////////////////////////////////////////////////////////////////////

void Get_Motor_Position(void)
{
    ang_POS[1] = ang_POS[0];            // Previous Position Counter
    ang_POS[0] = POSCNT;                // New Position Counter

    if (ang_POS[1] > ang_POS[0])        // Check 360 Degree Position
        speed_MOTOR = 1068 - ang_POS[1] + ang_POS[0];   // Circular Compensation
    else
        speed_MOTOR = ang_POS[0] - ang_POS[1];

    ang_MOTOR[1] = ang_MOTOR[0];            // Previous Motor Angle
    ang_MOTOR[0] = (speed_MOTOR * (.337));  // New Motor Angle: (360/1068)
    
//    if ((ang_MOTOR[2] + ang_MOTOR[0]) >= 360)               // Check 360 Degree Position
//        ang_MOTOR[2] = (ang_MOTOR[2] + ang_MOTOR[0]) - 360; // Circular Compensation
//    else
//        ang_MOTOR[2] = ang_MOTOR[2] + ang_MOTOR[0];

    return;
    
}


void Get_RPM(void)
{

    if ((ang_MOTOR[0] !=0 ) && (ang_MOTOR[1] != 0)) // Check Motor Running
    {
        rpm_MOTOR = (16.667 * ang_MOTOR[0]);  // (60/((1/(ang_MOTOR/360))*0.01))

        if (count_RPM != RESOLUTION_RPM) {count_RPM++;}       // Count for Average
        if (array_INDEX == RESOLUTION_RPM) {array_INDEX = 0;} // Reset Index
        else array_INDEX++;

        avg_RPM[array_INDEX] = rpm_MOTOR;     // Store Value in Array

        rpm_MOTOR = 0;

        // Sum RPMs
        for (loop_CYCLE = 0 ; loop_CYCLE < count_RPM ; loop_CYCLE++) {rpm_MOTOR += avg_RPM[loop_CYCLE];}

        rpm_MOTOR = (rpm_MOTOR/count_RPM);  // Take Average Every 1s
    }
    else
    {
        count_RPM = 0;
        loop_CYCLE = 0;
        array_INDEX = 0;
        rpm_MOTOR = 0;
    }

    return;

}

////////////////////////////////////////////////////////////////////////////////
//
//                          INITIALIZATIONS
//
////////////////////////////////////////////////////////////////////////////////

void InitPins(void)
{
    // PINS
    TRISD = 0b0001; // RD1-3: Outputs, RD0: Input (18,22,19,23) LCD / Reset
    TRISB = 0xFF78; // RB0-2,RB7: Outputs - LCD (2,3,4,9)
    _TRISF0 = 0;    // RF0: Output - Green LED (30)
    _TRISF1 = 0;    // RF1: Output - Red LED (29)
    _TRISC13 = 0;   // RC13: Digital Output - UltraSonic Trigger (15)
    _TRISC14 = 1;   // RC14: Input - UltraSonic Echo (16)
    TRISBbits.TRISB8 = 1;    // RB8: Input - Reset Variables on LCD (10)
    TRISE = 0b001100;      // Set PWM2 pins as Outputs
    
    return;

}

void InitAD(void)
{

    // Configure A/D Converter
    //ADPCFG = 0xFFBF;         // AN6: Input - Accelerometer - Z Direction (8)
    //ADPCFG |= 0x0030;        // QEA,QEB: Inputs - Encoder (6,7)
    ADPCFGbits.PCFG6 = 0;
    ADPCFGbits.PCFG4 = 1;
    ADPCFGbits.PCFG5 = 1;
    ADPCFGbits.PCFG3 = 0;    // AN3: Input - Current Monitor (5)
    ADCON1 = 0;              // Start Conversion
    ADCON2 = 0;              // AVDD and AVSS - Voltage Reference
    // DO NOT WRITE ADCS WHILE ADON = 1!
    ADCON3 = 0x0009;     // ADCS=9 -> Tad = 5*Tcy = 165ns
    ADCON1bits.ADON = 1; // Turn ADC ON

    //ADCON1bits.SSRC = 0b011; // Motor Control PWM interval stats/ends conversion

    return;

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

    return;

}

void InitTMR1(void) // Display
{
    T1CONbits.TON = 0;       // Turn Timer Off
    T1CONbits.TSIDL = 0;     // Continue Operation During Sleep
    T1CONbits.TGATE = 0;     // Gated Timer Accumulation Disabled
    T1CONbits.TCS = 0;       // Tcy
    T1CONbits.TCKPS = 0b11;  // 1:256 = 8.533us
//    T1CONbits.TSYNC = 0;   // .TCS = 0 -> Ignore
    
    PR1 = 35156;             // Interrupt Period 0.3s
    IFS0bits.T1IF = 0;       // Clear TMR1 Interrupt Flag
    IEC0bits.T1IE = 1;       // Enable TMR1 Interrupts
    T1CONbits.TON = 1;       // Turn On TMR1
    
    TMR1 = 0;                // Reset Timer Counter

    return;

}

void InitTMR2(void)  // Ultra Sonic
{
    T2CONbits.TON = 0;       // Turn Timer Off
    T2CONbits.TSIDL = 0;     // Continue Operation During Sleep
    T2CONbits.TGATE = 0;     // Gated Timer Accumulation Disabled
    T2CONbits.TCS = 0;       // Tcy
    T2CONbits.TCKPS = 0b11;  // 1:256 = 8.533us

    PR2 = 11719;             // @ 0.1s (Interrupt Period = 0.00147s -> 172: 256 prescaler)
    IFS0bits.T2IF = 0;       // Clear TMR2 Interrupt Flag
    IEC0bits.T2IE = 1;       // Enable TMR2 Interrupts
    T2CONbits.TON = 1;       // Turn On TMR2
    TMR2 = 0;                // Reset Timer Counter

    return;

}

void InitTMR3(void)     // Motor CPR
{
    T3CONbits.TON = 0;       // Turn Timer Off
    T3CONbits.TSIDL = 0;     // Continue Operation During Sleep
    T3CONbits.TGATE = 0;     // Gated Timer Accumulation Disabled
    T3CONbits.TCS = 0;       // Tcy
    T3CONbits.TCKPS = 0b01;  // 1:8 = 266.667ns

    PR3 = 37500;             // @ 0.01s Interrupt Period (3000 Max RPM: 0.012s for 2500 RPM)
    IFS0bits.T3IF = 0;       // Clear TMR3 Flag
    IEC0bits.T3IE = 1;       // Enable TMR3
    T3CONbits.TON = 1;       // Turn TMR3 On
    TMR3 = 0;                // Reset TMR3

    return;

}

void InitQEI(void)  // Encoder
{

    QEICONbits.CNTERR = 0; // Clear any count errors
    QEICONbits.QEISIDL = 0; // Continue operation during sleep
    QEICONbits.UPDN = 1; // Position Counter Direction is negative
    QEICONbits.QEIM = 0b111; // (7 = x4) (5 = x2) mode with position counter reset by Match
    QEICONbits.SWPAB = 0; // QEA and QEB not swapped
    QEICONbits.PCDOUT = 0; // Normal I/O pin operation *QEI Logic Controls
    QEICONbits.TQGATE = 0; // Timer gated time accumulation disabled
    QEICONbits.TQCKPS = 0b10; // 1:64 (2) Timer Prescale
    //QEICONbits.POSRES = 0; // Index pulse does not reset position counter
    QEICONbits.TQCS = 0; // Internal Clock
    QEICONbits.UPDN_SRC = 0; // QEB defines direction

    DFLTCONbits.CEID = 1; // Count error interrupts disabled
    DFLTCONbits.QEOUT = 1; // Digital filters output enabled
    DFLTCONbits.QECK = 0b101; // 1:64 (5) Prescaler for digital filter

    POSCNT = 0; // Reset position counter
    MAXCNT = MAX_CPR; // Set encoder max position

    IPC10bits.QEIIP = 3; // QEI interrupt highest priority(7)
    IEC2bits.QEIIE = 1; // QEI interrupt enable bit
    IFS2bits.QEIIF = 0; // QEI output flag

    return;

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
    PDC2 = 924;          // Sets PWM2 Duty Cycle to CW 10%
//    IFS2bits.PWMIF = 0; // Clear PWM Interrupt
//    IEC2bits.PWMIE = 1; // Enable PWM Interrupts
//    SEVTCMPbits.SEVTDIR = 0; // Special Event Trigger Compared to PTMR

//    PTCONbits.PTEN = 1; // Timer On

    return;

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

    // Get Angle of Lifting Bar
    data_ACCEL = Read_Analog_Channel(6);
    deg_LIFT_BAR = (data_ACCEL/1.57)-184;

    // Display Information on LCD
    Display_LCD();

    return;

}

// SENSORS INTERRUPT (CURRENT MONITOR / ULTRA SONIC)
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt (void)
{
    IFS0bits.T2IF = 0; // Clear TMR2 Interrupt Flag

    _LATC13 = 1;            // Send Trigger
    Delay_Micro_Sec(10);
    _LATC13 = 0;            // Turn-Off Trigger
    while(_RC14 == 0);      // Wait for Signal
    TMR2 = 0;               
    while(_RC14 == 1);      // Receive Signal
    data_SOUND = TMR2;      // Return Time Taken

    // Get Position from Ultra Sonic
    dist_IN = data_SOUND/16.8;
    // dist_IN = (((data_SOUND * 8.533us * 340m/s)/2)*39.3701in/m) or 0.5711 * data_SOUND
    // dist_CM = data_SOUND/6.61417; // @84 : 84 = 5" = 12.7 cm

    // Get Current Flow to Motor
    data_CURRENT = Read_Analog_Channel(3);
    current_MOTOR = (data_CURRENT * 0.22727);

    return;

}

// MOTOR INTERRUPT
void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt (void)
{
    IFS0bits.T3IF = 0;          // Clear TMR1 Interrupt Flag

    Get_Motor_Position();
    Get_RPM();

    return;

}

// ENCODER INTERRUPT
void __attribute__((__interrupt__, __auto_psv__)) _QEIInterrupt(void)
{
     //if (IFS2bits.QEIIF ==  1)  // Check O/F
     IFS2bits.QEIIF = 0;        // Clear Flag

        count_ENCODER++;

     return;

}

// PWM INTERRUPT
void __attribute__((__interrupt__, __auto_psv__)) _PWMInterrupt(void)
{

    IFS2bits.PWMIF = 0; // Clear PWM Interrupt

    return;
    
}