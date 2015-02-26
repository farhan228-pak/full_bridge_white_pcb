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
#include "dsp.h"

int faultCount = 0;
int VdcQ15 = 0;
int IpsfbQ15 = 0;
int IshareQ15 = 0;
int VoReference = 0,Verror = 0,Vpreverror = 0;
int Ierror = 0;
int VoltageDecoupleTerm = 0,IRdropDecoupleTerm = 0;
int PhaseZVT = 0,  DeltaPhaseZVT = 0;
int firstPass = 1;
int softstart = 1;
int RemoteTrip = 0,FaultStatus = 0;
int VL = 0;
int SystemState = 0;
int LoadShareCount = 0;
int prevVerror = 0;
int Sleepmodeflag = 0;
int Currenterror = 0;
int secondaryphase = 0;
int LoadShareOutput = 0;
int OverCurrentLimittrip = 1000;
int IpsfbQ15modified = 0;
int OCLtimer = 0; 
int Sumcount = 0;
int AveragePIDOutput = 0;
int SaturationFlag = 0;
int prsecdeadtime = 200;
int modifier = Q15(0.75);
int duty; 
int secdutycorrection;
int IpsfbQ15modflter = 0;
int ADCinterruptcount = 0;
char OverVoltageStatus = 0;
char OverCurrentStatus = 0;
char OverTemperatureStatus = 0;
long VoltageIntegralOutput = 0,VoltagePropOutput = 0,VoltageDiffOutput = 0,VoltagePIDOutput = 0;
long CurrentIntegralOutput = 0,CurrentPropOutput = 0,CurrentDiffOutput = 0,CurrentPIDOutput = 0;
long CurrentRef = 0;
long CurrentIoutput = 0;
long Currenterrorlong = 0;
long SumPIDOutput = 0;
long PIDOutput = 0;

long IpsfbQ15modifiedlong;

