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

#include "Variables_PSFB.h"
#include "Define_PSFB.h"
#include "dsp.h"

/*DMCI code*/
//	 int array1[400];	
//	 int array2[400];	 
//	 unsigned int indexno = 0, check = 4;
/*DMCI code ends*/

void DigitalCompensator(void)
{
    /*softstart code */

    if(firstPass == 1)		// Check for First Iteration of the loop
    {
        VoReference = VdcQ15;	// Assign Reference Voltage to actual measured DC Bus Voltage
        firstPass = 0;		// Clear the First Pass Flag
    }

    if(softstart == 1)                      // Check for SoftStart Flag Status
    {
        if(VoReference < VoRefQ15)          // Until Reference Voltage is Less than ~12V,
        {
            VoReference =VoReference+56;    // Increment the Reference Voltage by one count
        }

        else
        {
            VoReference = VoRefQ15;         // Else, Fix the reference voltage to 12V
            softstart = 0;
        }
    }

    /*softstart code ends*/

    Verror = ((VoReference + LoadShareOutput) - VdcQ15);    // error = reference - output voltage measured
    Verror =  ((__builtin_mulss(Q15(0.15),Verror))>>15) + ((__builtin_mulss(Q15(0.85),prevVerror))>>15) ;
    prevVerror = Verror;

    if((SaturationFlag == 0) || (Verror < 0))	// No saturation of PID output OR Verror is less than zero keep integrating
            VoltageIntegralOutput = VoltageIntegralOutput + ((__builtin_mulss((int) KiVoltage,Verror))>>15);
                    // cals for integral term  Ki*error, Integral term is long so shifted by 15

    if(VoltageIntegralOutput >Saturation)
        VoltageIntegralOutput = Saturation;
    else if(VoltageIntegralOutput < -Saturation)
        VoltageIntegralOutput = -Saturation;

    VoltagePropOutput = ((__builtin_mulss((int) KpVoltage,Verror)) >> 15);

    CurrentRef = (long)VoltagePropOutput  + VoltageIntegralOutput ;

    if(CurrentRef > Saturation)
        CurrentRef =Saturation;
    else if(CurrentRef < -Saturation)
        CurrentRef = -Saturation;

    CurrentRef = (CurrentRef <<Refshift);	// again making this back to Q15 format.
	
// multiplies with 2 to take of the prescaler used in timer1
	
    Currenterrorlong = CurrentRef - (long)IpsfbQ15modified;

    if(Currenterrorlong < -32767)
        Currenterrorlong = -32767;
    else if(Currenterrorlong > 32767)
        Currenterrorlong = 32767;

    Currenterror = Currenterrorlong;

    VL  = ((__builtin_mulss((int)Currenterror,(int) Ra)) >> 15);

    if((SaturationFlag == 0) ||(Currenterror < 0)) // No saturation of PID output OR currenterror is less than zero keep integrating
        CurrentIoutput  = CurrentIoutput + (Currenterror >> 12);

    if(CurrentIoutput > 32767)
        CurrentIoutput = 32767;
    else if(CurrentIoutput < -32767)
        CurrentIoutput = -32767;


    VoltageDecoupleTerm = ((__builtin_mulss((int) Voltagedecouple,VdcQ15)) >> 15);
    IRdropDecoupleTerm =  ((__builtin_mulss((int)Rp,(int) IpsfbQ15modified)) >> 15);

    PIDOutput  = (long)VL + (long) VoltageDecoupleTerm + (long) IRdropDecoupleTerm+ (long)CurrentIoutput ;// + (long)CurrentPIDOutput;

    SaturationFlag = 0;
    if(PIDOutput > 32767)		//saturate
    {
        PIDOutput = 32767;
        SaturationFlag = 1;
    }
    else if(PIDOutput < 0)		//put min value Q15(220/3000)
        PIDOutput = 0;

/*DMCI*/
//		if(check == 0)	
//		{
//			array1[indexno] = IpsfbQ15modified;
//			array2[indexno] = VdcQ15;
//			indexno++;
//			if(indexno>=400) 
//				indexno = 0;		
//			check = 0;	
//		}	
//	
//		else 
//			check++;
/*DMCI*/			
			
//PIDOutput = Q15(0.7);     // Open loop
	

    PhaseZVT = ((__builtin_mulss((int)PIDOutput,MaxPhase)) >> 15);   // the calculated PhaseZVT is scalled down to maximum allowed Phase shift.

    duty =  PWMPeriod  - PhaseZVT;
    PHASE2 =  duty;        
    secdutycorrection = duty - prsecdeadtime+ 400;
     
    if(secdutycorrection > PWMPeriod)
        secdutycorrection =  PWMPeriod - 1;
 		
      PDC3 =secdutycorrection ;
      SDC3 =secdutycorrection ;
       
      secondaryphase =duty - (prsecdeadtime>>1);      	      
      PHASE3 = secondaryphase;;
    
     if(secondaryphase  > HALFPERIOD)
        SPHASE3  =secondaryphase  - HALFPERIOD ;
     else
        SPHASE3  =secondaryphase  + HALFPERIOD;
}	

void LoadshareCompensator (void)
{
    Ierror = IshareQ15 - IpsfbQ15;		// Average current - individual module current

    CurrentPropOutput = ((__builtin_mulss(KpCurrent,Ierror)) >> 15);
    CurrentIntegralOutput = CurrentIntegralOutput + ((__builtin_mulss(KiCurrent,Ierror))>>15);

    if(CurrentIntegralOutput > 4800)
        CurrentIntegralOutput = 4800;
    else if(CurrentIntegralOutput < -4800)
        CurrentIntegralOutput = -4800;

    CurrentPIDOutput  = (long)CurrentPropOutput  + CurrentIntegralOutput ;

    if(CurrentPIDOutput > 2400)			// 1/16 of 32765, we will be correcting 16 % of the PWM period ( to allow 12% load correction)
        CurrentPIDOutput= 2400;
    else if(CurrentPIDOutput < -2400)
        CurrentPIDOutput = -2400;

    LoadShareOutput = 	(CurrentPIDOutput  >> 4);
}

