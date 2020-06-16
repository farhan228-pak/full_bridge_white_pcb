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
#include "Define_PSFB.h"
#include "Variables_PSFB.h"
#include "dsp.h"
#include "delay.h"

_FOSCSEL(FNOSC_FRC)			// Internal FRC with non PLL
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON) 	// Sw Enabled, Mon Disabled and OSCO pin has digital I/O function
_FWDT(FWDTEN_OFF)			// Watchdog Timer disabled
_FPOR(FPWRT_PWR128) 			// POR Timer Value 128mSec
_FGS(GWRP_OFF)				// General Segment may be written
_FICD(ICS_PGD2 & JTAGEN_OFF);   	// Use PGC3/EMUC3 and PGD3/EMUD3

//void Pin_Remap(void);
void init_PSFBDrive(void);		// Full bridge gate drive is initialized
void init_SYNCRECTDrive(void);		// Sync. rectifier gate drive is initialized
void init_ADC (void ) ;			// Analog ports are initialized
void init_CMP (void);			// Analog comparator is initialized
void init_Timer1(void);			// Timer 1 initialized				
void InitRemoteON_OFF(void);		// Initialization of remote ON/OFF function
void LoadshareCompensator(void);	// Load sharing 
void DigitalCompensator(void);	 	// Voltage mode PID control

int main (void)
 
{

/*Configure Oscillator to operate the device at 40Mhz*/
/* Fosc= Fin*M/(N1*N2), Fcy=Fosc/2	*/
/* Fosc= 7.37M*43(2*2)=80Mhz for 7.37M input clock*/

    PLLFBD=41;			// M=43
    CLKDIVbits.PLLPOST=0;	// N2=2
    CLKDIVbits.PLLPRE=0;	// N1=2
    OSCTUN=0;			// Tune FRC oscillator, if FRC is used

/* Disable Watch Dog Timer*/
    RCONbits.SWDTEN=0;

/* Clock switch to incorporate PLL*/
    __builtin_write_OSCCONH(0x01);	// Initiate Clock Switch to FRC with PLL (NOSC=0b001)
    __builtin_write_OSCCONL(0x01);	// Start clock switching
    while (OSCCONbits.COSC != 0b001);	// Wait for Clock switch to occur

/* Wait for PLL to lock*/
    while(OSCCONbits.LOCK!=1) {};
	   
/* Now setup the ADC and PWM clock for 120MHz; ((FRC * 16) / APSTSCLR ) = (7.37 * 16) / 1 = ~ 120MHz*/

     ACLKCONbits.FRCSEL = 1;    	//FRC provides input for Auxiliary PLL (x16)
     ACLKCONbits.SELACLK = 1;   	//Auxiliary Oscillator provides clock source for PWM & ADC
     ACLKCONbits.APSTSCLR = 7;  	//Divide Auxiliary clock by 1
     ACLKCONbits.ENAPLL = 1;    	// Enable Auxiliary PLL
     OSCTUNbits.TUN = 0x0005;
     while(ACLKCONbits.APLLCK != 1);  	// Wait for Auxiliary PLL to Lock
    __builtin_write_OSCCONL(0x00); 	// IOLOCK=0

/* Remapping of  comparator output on the virtual pin*/

   // RPINR0bits.INT1R = 0b000101;	// External interrupt2 configured to RP5; remote ON/OFF

    RPOR3bits.RP7R = 0b101101; // for pwm4
    RPOR3bits.RP6R = 0b101100; // for pwm4
    __builtin_write_OSCCONL(0x40); 	// IOLOCK=1

    TRISBbits.TRISB5 = 1;		// For Remote ON/OFF
	
    TRISBbits.TRISB8 = 1 ;               // switch for turning on and off
    //TRISBbits.TRISB8 = 0;		// I/O pin made as an output port ; used to toggle the pin
	
    InitRemoteON_OFF();
    while(SystemState == System_OFF); 	// unless the system recevies remote turnON signal , PWM O/P will not turn ON.
   // Pin_Remap();//for pwm4

    init_PSFBDrive();
    init_SYNCRECTDrive();
    init_ADC () ;
    init_Timer1();
   
    PTCONbits.PTEN = 1;			// Enable the PWM
    ADCONbits.ADON = 1;		// A/D converter module is operating
 /***********************************************************************************/
//    while(PORTBbits.RB8 == 1);
//Delay(100);
//    PTCONbits.PTEN = 1;
// Delay(1000);
//        IOCON1bits.OVRENH = 0;			// Take Override Bits to control PWM Module
//        IOCON1bits.OVRENL = 0;			// Take Override Bits to control PWM Module
//        IOCON2bits.OVRENH = 0;			// Take Override Bits to control PWM Module
//        IOCON2bits.OVRENL = 0;			// Take Override Bits to control PWM Module
//        IOCON3bits.OVRENH = 0;			// Take Override Bits to control PWM Module
//        IOCON3bits.OVRENL = 0;			// Take Override Bits to control PWM Module
//
//
//PTCONbits.PTEN = 0;
// ////////////////////////////////////////////////////////////////
 while(1)
    {
     
        if(Sleepmodeflag ==1)	// Remote ON /OFF is enabled dsPIC is forced to enter sleep mode
        {
            Sleepmodeflag = 0;		// Clear the sleap mode flag
            //Sleep();			// Entered into sleep mode
        }

        if(faultCount>=1000)		// If the fault is repeatable disbale the PWM module
        {
            PTCONbits.PTEN = 0;		// Disable the PWM module
            _ADCP1IE = 0;		//disable ADinterrupt

//            Nop();
//            Nop();
//            Nop();
        }

        if(FaultStatus == 2)
        {
            faultCount=faultCount+1;
            ADCinterruptcount = 0;
//            Nop();
//            Nop();
//            Nop();
            IOCON1bits.OVRENH = 0;	// If True, Bring PWM to normal operation
            IOCON1bits.OVRENL = 0;
            IOCON2bits.OVRENH = 0;	// If True, Bring PWM to normal operation
            IOCON2bits.OVRENL = 0;
            IOCON3bits.OVRENH = 0;	// If True, Bring PWM to normal operation
            IOCON3bits.OVRENL = 0;

            FaultStatus = 0;		// Clear the Fault Status Flag for normal operation
        }
    }
}

/* ADC Interrupt Service Routine to execute the PID compensator and faults execution */

void __attribute__((__interrupt__, no_auto_psv)) _ADCP1Interrupt()  
{
    //_LATB8 = 1;

    _ADCP1IF = 0;		// Clear the ADC pair1 Interrupt Flag
		
    VdcQ15 = (OutputVoltage << 5);	// convert the 10bit ADC valure to Q15 format
    IpsfbQ15 = (PSFBCurrent << 5);	// convert the 10bit ADC valure to Q15 format
    IshareQ15 = (LoadShare << 5);	// convert the 10bit ADC valure to Q15 format

    IpsfbQ15modifiedlong = ((__builtin_mulss(IpsfbQ15, modifier)) >> 14); // multiplies with 2 to take of the prescaler used in timer1

    if(IpsfbQ15modifiedlong  > 32767)
        IpsfbQ15modified = 32767;
    else
        IpsfbQ15modified = IpsfbQ15modifiedlong;

    ADCinterruptcount++;

    if(ADCinterruptcount >= 30000)
    {
        faultCount = 0;
        ADCinterruptcount = 30000;
    }
		
    if(OutputVoltage < OutputOverVoltageLimit ) 	// Checking for the Overvoltage fault
        OverVoltageStatus = 0;				// Output voltage is in the specified limit
    else
        OverVoltageStatus = 1;				// set the flag if output volatge limit exceeds


    IpsfbQ15modflter = (__builtin_mulss(IpsfbQ15modflter, Q15(0.9))>>15) + (__builtin_mulss(IpsfbQ15modified, Q15(0.1))>>15)  ;
//IpsfbQ15modflter = (__builtin_mulss(IpsfbQ15modflter, Q15(0.92))>>15) + (__builtin_mulss(IpsfbQ15modified, Q15(0.08))>>15)  ;
// this filter is for current limt chcecking

    if(OCLtimer >= 2000)									// After soft start it will wait for 2000 interrupts
    {
        if((IpsfbQ15modflter >>5) < OverCurrentLimittrip )	// Checking for the Overcurrent fault
            OverCurrentStatus = 0;                              // Output current is in the specified limit
        else
            OverCurrentStatus = 1;				// set the flag if output current limit exceeds
    }

    else
    {
        if((IpsfbQ15 >>5) < OverCurrentLimittrip )		// Checking for the Overcurrent fault
            OverCurrentStatus = 0;				// Output current is in the specified limit
        else
            OverCurrentStatus = 1;				// set the flag if output current limit exceeds
    }


    if((OverVoltageStatus == 0) && (OverCurrentStatus == 0) && (OverTemperatureStatus==0 ))// none of the above 3 faults occured
    {
        if(FaultStatus == 1)			// If fault occurs and then vanishes, make fault status as 2
        {
            FaultStatus = 2;
            firstPass = 1;			// Forcing the system to enter into the Soft start mode
            softstart = 1;
        }

        DigitalCompensator();			// If there is no fault execute the PID loop

        if(LoadShareCount >= 75)		// Load share routine is called with 1 Khz frequency rate
        {
            LoadshareCompensator();		// execute the load share loop
            LoadShareCount = 0;			// set the load share counter
        }

        LoadShareCount ++;			// incrementing the load share counter
    }
		
    else                                        // If fault exists, then...
     {
        FaultStatus = 1;			// Set the Fault Status Flag
        if(OverVoltageStatus == 1)
            faultCount = 2000;

        IOCON1bits.OVRENH = 1;			// Take Override Bits to control PWM Module
        IOCON1bits.OVRENL = 1;			// Take Override Bits to control PWM Module
        IOCON2bits.OVRENH = 1;			// Take Override Bits to control PWM Module
        IOCON2bits.OVRENL = 1;			// Take Override Bits to control PWM Module
        IOCON3bits.OVRENH = 1;			// Take Override Bits to control PWM Module
        IOCON3bits.OVRENL = 1;			// Take Override Bits to control PWM Module
        IOCON1bits.OVRDAT = 0;			// Force the PWM Pin to Low State or Inactive State
        IOCON2bits.OVRDAT = 0;			// Force the PWM Pin to Low State or Inactive State
        IOCON3bits.OVRDAT = 0;			// Force the PWM Pin to Low State or Inactive State
     }
//	 _LATB8 = 0;
}
  	
 void __attribute__((__interrupt__, no_auto_psv)) _INT1Interrupt()  // For remote ON/OFF functionality
{
	
    if (SystemState == System_ON)		// initially there is no external signal and external siganl chages the state
                                                // from low to high interrupt will be genearted and enters into this loop
    {

//	When remote ON/OFF is enabled and output current is zero the output should decay to 10% of rated output before 7Sec
    IOCON1bits.OVRENH = 1;			// Take Override Bits to control PWM Module
    IOCON1bits.OVRENL = 1;			// Take Override Bits to control PWM Module
    IOCON2bits.OVRENH = 1;			// Take Override Bits to control PWM Module
    IOCON2bits.OVRENL = 1;			// Take Override Bits to control PWM Module

    Delay(500);

    PTCONbits.PTEN = 0;				// Turn OFF the PWM module
    SystemState = System_OFF;
    INTCON2bits.INT1EP = 1;			// External Interrupt 1 Edge Detect Polarity Select bit; negative edge
    Sleepmodeflag = 1;				// set the flag for sleep mode
    }

    else					// Initially system is Remote OFF mode and signal changes the state from
                                                // High to low interrupt will be generated and enters into this loop
    {
        _ADCP1IE = 0;

        IOCON1bits.OVRENH = 0;
        IOCON1bits.OVRENL = 0;
        IOCON2bits.OVRENH = 0;
        IOCON2bits.OVRENL = 0;

        firstPass =1;				// Forcing the system to enter into the Soft start mode
        softstart =1;

        SystemState = System_ON;
        INTCON2bits.INT1EP = 0;			// External Interrupt 1 Edge Detect Polarity Select bit; positive edge
        VoltageIntegralOutput = 0;		// reseting the PID variables
        prevVerror = 0;
        CurrentIoutput =0;
        CurrentIntegralOutput =0;
        modifier = Q15(0.75);
        PIDOutput = 0;
        PhaseZVT  = 0;
        SaturationFlag = 0;
        LoadShareOutput = 0;
        SumPIDOutput = 0;
        AveragePIDOutput  = 0;
        Sumcount = 0;
        OCLtimer = 0;
        PHASE2 = PWMPeriod;			// During startup there is no overlap between diagonal switches, so there is no output voltage during start up.

        PTCONbits.PTEN = 1;			// Turn ON the PWM module
        _ADCP1IE = 1;
    }

	_INT1IF = 0 ;				// Clear interrupt falg
}



/* Timer1 interrupt to take care of the live volatge variations to improve the transient response and 
	also compensate the transformer current for the voltage variations */
 void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt()  // This is to get the transient response same at all the line voltages.
 {
     _T1IF = 0;

    if(softstart == 0)	 				// during the normal operation of the converter accumulate the PIDOutput
    {
        SumPIDOutput = SumPIDOutput  + PIDOutput ;	// to get the average value of the PID output
        Sumcount++;
        OCLtimer++;
    }
    else
        OverCurrentLimittrip = OverCurrentLimitSST;	// During start up higher currents are observed so limit is more than the normal


    if(OCLtimer >= 2000)				// Wait for 2000 interrupts after soft start , before reducing the limit
    {
        OverCurrentLimittrip = OverCurrentLimit;	// Trip limit is with normal over current limit
        OCLtimer = 2000;				// 2000* ~20uSec = 40msec
    }

    if(Sumcount > 512)                                  // Averaging the PIDoutput for the variation of inputs
    {
        AveragePIDOutput = (SumPIDOutput>>9);		// divide by 512 to get the average; averaging after every 10mSec
        Sumcount = 0;
        SumPIDOutput = 0;

        if(AveragePIDOutput  < Q15(0.501) )		// saturating the Average PID output
            AveragePIDOutput = Q15(0.501);

        //modifier = __builtin_divsd(((long)32767<<14),AveragePIDOutput);	// Scale factor of 0.5 is used for number limits
        modifier = __builtin_divsd(((long)32767<<14),AveragePIDOutput);														// Max value of modifier will be 2Xaverage PID output

        /* checking for over Temperature status */

        if(Temperature < OverTempLimitmax)		// Checking for the Overvoltage fault
            OverTemperatureStatus =0;			// Over temperature is in the specified limit
        else
            OverTemperatureStatus =1;			// set the flag if temperature limit exceeds
    }
} 

