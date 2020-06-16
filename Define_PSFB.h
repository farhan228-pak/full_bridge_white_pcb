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

#include "dsp.h"
#include "p33FJ16GS502.h"
//#include "p33fxxxx.h"

#define PWMClockFreq		943E6
#define PWMSwitchingFreq	389E3

#define PWMPeriod		(int)((PWMClockFreq/PWMSwitchingFreq) - 8)
#define PWMDuty			(int)((PWMClockFreq/PWMSwitchingFreq)*0.8)
#define HALFPERIOD      	(int)(PWMPeriod/2)

#define MaxPhase		(int)((PWMClockFreq/PWMSwitchingFreq)* 0.57)
#define MaxDeltaPhase 		(int)((PWMClockFreq/PWMSwitchingFreq)* 0.05)
#define trigger 		(int)((PWMClockFreq/PWMSwitchingFreq)* 0.95)

#define Deadtime		90//360  700=90
#define ALTDeadtime		110//360 700=110

#define PSFBCurrent		ADCBUF0
#define OutputVoltage		ADCBUF1
#define LoadShare		ADCBUF2
#define Temperature		ADCBUF3

#define PSFBCurrentLimit 8.0
#define MAXPSFBCurrent	11.0  

#define VoRef  (VoltagreReference/MaxOutputVoltage)  	// Base output voltage is taken as 14V and reference is scaled it down to 12V
#define VoRefQ15 Q15(VoRef )                            // Scaled reference voltage is converted into q15 form (28086)

#define TRUE 1
#define FALSE 0

#define alphavalue 0.01
#define betavalue  0.99

#define KpVoltage           Q15(0.99)
#define KiVoltage           Q15(0.05)
#define Ra                  Q15(0.1215)
#define Voltagedecouple     Q15(0.4733)
#define Rp                  Q15(0.0406)

#define KpCurrent           Q15(0.0734)
#define KiCurrent           Q15(0.0092)

#define Saturation 2047 
#define Refshift 5

#define alpha Q15(alphavalue)
#define beta  Q15(betavalue)

#define VoltagreReference	11.42 		
#define MaxOutputVoltage	14.0  	// This is scaled to 3.3V
#define OutputOverVoltageLimit  930	// Over voltage limit set at 13.5V
#define OverCurrentLimit 	890 	// Consider max input current is 10A, with 1:100 CT and a gain of 2.99 will give 3.0V. 
#define OverCurrentLimitSST 	1010	 
#define OverTempLimitmax 	418	// MCP9700 Transfer function: Vout = Tc1*Ta+V0C;
                                        // (10mV/oC)*Ta + V0C  = (10mV/oC)*100oC + 500mV = 1.5V
					// 3.3V = 1023, so 1.5V equal to 465 max;
                                        // 85Deg is min choosen so 1.35V is eq to 418

#define PHASEZVT  	PHASE1
#define PHASEZVT3  	PHASE3

#define System_ON  	1		// Remote ON/OFF signal is low
#define System_OFF 	0		// Remote ON/OFF signal is high
