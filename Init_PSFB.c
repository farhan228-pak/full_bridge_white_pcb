////////////////////////////////////////////////////////////////////////////////
// © 2013 Microchip Technology Inc.
//
// MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:  You may use this software, and any
// derivatives created by any person or entity by or on your behalf, exclusively
// with Microchip?s products.  Microchip and its licensors retain all ownership
// and intellectual property rights in the accompanying software and in all
// derivatives here to.
//
// This software and any accompanying information is for suggestion only.  It
// does not modify Microchip?s standard warranty for its products.  You agree
// that you are solely responsible for testing the software and determining its
// suitability.  Microchip has no obligation to modify, test, certify, or
// support the software.
//
// THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
// EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
// WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
// PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP?S PRODUCTS,
// COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
//
// IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT
// (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT LIABILITY,
// INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
// EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF
// ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWSOEVER CAUSED, EVEN IF
// MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
// TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
// CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES,
// IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
//
// MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
// TERMS.
////////////////////////////////////////////////////////////////////////////////

#include "p33FJ16GS502.h"
#include "dsp.h"
#include "Define_PSFB.h"
#include "Variables_PSFB.h"

void init_PSFBDrive(void)
{
    PTPER = PWMPeriod;
    PDC1 = HALFPERIOD ;
    PDC2 = HALFPERIOD;
    PDC4 = HALFPERIOD;
		
/*Initialization of the PWM for the left leg*/
		
    IOCON1bits.PENH = 1;	// PWM module controls PWMxH pin
    IOCON1bits.PENL = 1;	// PWM module controls PWMxL pin
    IOCON1bits.POLH = 0;	// PWM Output Polarity is high
    IOCON1bits.POLL = 0;	// PWM Output Polarity is high
    IOCON1bits.PMOD = 0;	// PWM I/O pin pair is in the Complementary Output mode

    PWMCON1bits.ITB = 0;	// PTPER register provides timing for this PWM generator
    PWMCON1bits.MDCS = 0;	// Master time base will provide duty cycle

    DTR1    = Deadtime; 	// Dead time setting
    ALTDTR1 = ALTDeadtime; 	// Dead time setting

    PWMCON2bits.TRGIEN = 1;
    TRGCON2bits.TRGDIV=1;
    TRIG2= trigger;
		
/*Initialization of the PWM for the right leg*/
		
    IOCON2bits.PENH = 1;	// PWM module controls PWMxH pin
    IOCON2bits.PENL = 1;	// PWM module controls PWMxL pin
    IOCON2bits.POLH = 0;	// PWM Output Polarity is high
    IOCON2bits.POLL = 0;	// PWM Output Polarity is high
    IOCON2bits.PMOD = 0;	// PWM I/O pin pair is in the Complementary Output mode

    PWMCON2bits.ITB = 0;	// PTPER register provides timing for this PWM generator
    PWMCON2bits.MDCS = 0;	// Master time base will provide duty cycle

    DTR2    = Deadtime; 	// Deadtime setting
    ALTDTR2 = ALTDeadtime; 	// Deadtime setting
	   	
/* FLT1 associated with CMP2 and FLT2 associated with CMP4 */
	   	
    FCLCON1bits.FLTSRC = 1;	// Fault 2 has been selected for the Fault control signal source for PWM Generator1
    FCLCON2bits.FLTSRC = 1;	// Fault 2 has been selected for the Fault control signal source for PWM Generator2
    FCLCON3bits.FLTSRC = 1;	// Fault 2 has been selected for the Fault control signal source for PWM Generator3

    FCLCON1bits.CLSRC = 0;	// Fault 1 has been selected for current limit control signal source for PWM Generator1
    FCLCON2bits.CLSRC = 0;	// Fault 1 has been selected for current limit control signal source for PWM Generator2
    FCLCON3bits.CLSRC = 0;	// Fault 1 has been selected for current limit control signal source for PWM Generator3
				
// Over voltage
//  FCLCON1bits.CLMOD=1;	//  Cycle by cycle mode selected
//  FCLCON2bits.CLMOD=1;	//  Cycle by cycle mode selected
//  FCLCON3bits.CLMOD=1;	//  Cycle by cycle mode selected
	   		
    FCLCON1bits.FLTMOD=3;	// Latch mode is enabled
    FCLCON2bits.FLTMOD=3;	// Latch mode is enabled
    FCLCON3bits.FLTMOD=3;	// Latch mode is enabled

    IOCON1bits.OVRDAT = 0;	// State for PWMxH and PWMxL Pins is low if Override is Enabled
    IOCON2bits.OVRDAT = 0;	// State for PWMxH and PWMxL Pins is low if Override is Enabled
    IOCON3bits.OVRDAT = 0;	// State for PWMxH and PWMxL Pins is low if Override is Enabled
    IOCON4bits.OVRDAT = 0;


     
}


void init_SYNCRECTDrive(void)
{	
    IOCON3bits.PENH = 1;	// PWM module controls PWMxH pin
    IOCON3bits.PENL = 1;	// PWM module controls PWMxL pin
    IOCON3bits.POLH = 0;	// PWM Output Polarity is high
    IOCON3bits.POLL = 0;	// PWM Output Polarity is high
    IOCON3bits.PMOD = 3;	// PWM I/O pin pair is in the True Independent PWM Output mode
    IOCON3bits.SWAP = 1;	// PWMxH output signal is connected to PWMxL pins; PWMxL output signal is connected to PWMxH pin

    PWMCON3bits.ITB = 0;	// PTPER register provides timing for this PWM generator
    PWMCON3bits.MDCS =0;	// Master time base will provide duty cycle


    IOCON4bits.PENH = 1;	// PWM module controls PWMxH pin
    IOCON4bits.PENL = 1;	// PWM module controls PWMxL pin
    IOCON4bits.POLH = 0;	// PWM Output Polarity is high
    IOCON4bits.POLL = 0;	// PWM Output Polarity is high
    IOCON4bits.PMOD = 3;	// PWM I/O pin pair is in the True Independent PWM Output mode
    IOCON4bits.SWAP = 1;	// PWMxH output signal is connected to PWMxL pins; PWMxL output signal is connected to PWMxH pin

    PWMCON4bits.ITB = 0;	// PTPER register provides timing for this PWM generator
    PWMCON4bits.MDCS =0;	// Master time base will provide duty cycle
}


void init_ADC (void)   	
{
	   	
/* AN1 is for output voltage measurment and AN0 is for PSFB PRY Current */
	
    ADSTATbits.P0RDY =  0;	// Pair 0 data ready bit cleared before use
    ADSTATbits.P1RDY =  0;	// Pair 1 data ready bit cleared before use

    ADPCFGbits.PCFG0    = 0;  	// configured as analog input, AN3 for the temp measurment
    ADPCFGbits.PCFG1 = 0; 	// configured as analog input, AN1 for output voltage measurment
    ADPCFGbits.PCFG2  = 0;  	// configured as analog input, AN2 for load share
    ADPCFGbits.PCFG3    = 0;  	// configured as analog input, AN3 for the temp measurment

    ADCPC0bits.IRQEN0 = 1; 	// IRQ is  generated
    ADCPC0bits.IRQEN1 = 1; 	// IRQ is  generated

    ADCPC0bits.TRGSRC0 = 5; 	// Choose PWM2 as the trigger source
    ADCPC0bits.TRGSRC1 = 5; 	// Choose PWM2 as the trigger source

    // A/D Control Register
    ADCONbits.SLOWCLK = 1;	// ADC is clocked by the auxiliary PLL (ACLK)
    ADCONbits.FORM = 0;		// Data format is integer
    ADCONbits.EIE = 0;		// Interrupt is generated after both channels conversion is completed
    ADCONbits.ORDER = 1;	// Odd numbered analog input is converted first, followed by conversion of even numbered input
    ADCONbits.SEQSAMP = 0;	// Sequential Sample Enable
    ADCONbits.ASYNCSAMP = 0;  	/* The dedicated S&H circuit starts sampling when the trigger event is
                                   detected and completes the sampling process in two ADC clock cycles */
    ADCONbits.ADCS = 4;         // FRC = 7.37MHz; ACLK = FRC * M/N     (N = APSTSCLR) = 7.37 *16 / 1 = 117.92Mhz
				// Sample and conversion time per pair=  2TAD + 14 TAD =16TAD; TAD = ADC clock period
				//  since two SARs are used 14MSPS is; 

/* Set up the Interrupts */

    IPC3bits.ADIP = 7;		// Interrupt is priority 7 (highest priority interrupt)
    //IFS0bits.ADIF = 0;	// Clear ADC interrupt flag
    //IEC0bits.ADIE = 1;	// ADC Conversion global Interrupt request Enabled
	 
    _ADCP1IF = 0;		// ADC Pair 1 Conversion Done Interrupt Flag Status bit is cleared
    _ADCP1IE = 1;		// ADC Pair 1 Conversion Done Interrupt Enable bit is cleared
	    	
	}

void InitRemoteON_OFF(void)
{

    if (PORTBbits.RB5 == 1)		// External signal to the Opto
    {
        SystemState = System_OFF;	// Opto detected the external signal and the system will be in infinite while loop
        INTCON2bits.INT1EP = 1;		// External Interrupt 1 Edge Detect Polarity Select bit; Negative edge
    }					// Opto detected the external signal and the signal changes the state //
                                        // from high to low interrupt will be generated//

    else
    {
        SystemState = System_ON;	// there is no external signal to the opto
        INTCON2bits.INT1EP = 0;		// External Interrupt 1 Edge Detect Polarity Select bit; positive edge                                                                                     // if there will be a external signal (later stage) interrupt will be generated.
    }

    IFS1bits.INT1IF = 0;	 	// Clear interrupt flag
    IEC1bits.INT1IE = 1;		// Enable interrupt

}

void init_Timer1(void)
{
    T1CON = 0x0000;
    PR1 = 780;				//48MHZ / 780 = 51kHz is the frequency of interrupt
    _T1IP = 4;
    _T1IF = 0;
    _T1IE = 1;
    T1CONbits.TON = 1;
	
}

void Pin_Remap(void)
{
    	//*************************************************************
	// Unlock Registers
	//*************************************************************
	//__builtin_write_OSCCONL(OSCCON & ~(1<<6));


	// Assign U1Tx To Pin RP7 RP7R
	//***************************
	//RPOR3bits.RP7R = 0b101100;
       // RPOR0bits.RP0R = 0b010010;
	//*************************************************************
	// Lock Registers
	//*************************************************************
	//__builtin_write_OSCCONL(OSCCON | (1<<6));
}
