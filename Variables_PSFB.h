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

//#include "p33FJ16GS502.h"

extern int faultCount;
extern int VdcQ15 ;
extern int IpsfbQ15;
extern int IshareQ15;
extern int VoReference,firstPass,softstart;
extern int Verror,Vpreverror,Ierror;
extern int PhaseZVT, DeltaPhaseZVT;
extern int  VoltageDecoupleTerm,IRdropDecoupleTerm;
extern int RemoteTrip,FaultSttus;
extern int VL;
extern int SystemState;
extern int LoadShareCount;
extern int prevVerror;
extern int Sleepmodeflag;
extern int OverCurrentLimittrip;
extern int Currenterror;
extern int secondaryphase;
extern int LoadShareOutput;
extern int IpsfbQ15modified;
extern int OCLtimer; 
extern int Sumcount;
extern int AveragePIDOutput;
extern int SaturationFlag;
extern int prsecdeadtime;
extern int modifier;
extern int duty; 
extern int secdutycorrection;
extern int IpsfbQ15modflter;
extern int ADCinterruptcount;
extern char OverVoltageStatus;
extern char OverCurrentStatus;
extern char OverTemperatureStatus; 
extern long VoltageIntegralOutput,VoltagePropOutput,VoltageDiffOutput,PIDOutput;
extern long CurrentIntegralOutput,CurrentPropOutput,CurrentDiffOutput,CurrentPIDOutput;
extern long CurrentRef;
extern long CurrentIoutput;
extern long Currenterrorlong;
extern long PIDOutput;
extern long SumPIDOutput;
extern int FaultStatus;
extern long IpsfbQ15modifiedlong;
