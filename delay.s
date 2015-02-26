;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; � 2013 Microchip Technology Inc.
;
; MICROCHIP SOFTWARE NOTICE AND DISCLAIMER:  You may use this software, and any
; derivatives created by any person or entity by or on your behalf, exclusively
; with Microchip?s products.  Microchip and its licensors retain all ownership
; and intellectual property rights in the accompanying software and in all
; derivatives here to.
;
; This software and any accompanying information is for suggestion only.  It
; does not modify Microchip?s standard warranty for its products.  You agree
; that you are solely responsible for testing the software and determining its
; suitability.  Microchip has no obligation to modify, test, certify, or
; support the software.
;
; THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER
; EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
; WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR
; PURPOSE APPLY TO THIS SOFTWARE, ITS INTERACTION WITH MICROCHIP?S PRODUCTS,
; COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
;
; IN NO EVENT, WILL MICROCHIP BE LIABLE, WHETHER IN CONTRACT, WARRANTY, TORT
; (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), STRICT LIABILITY,
; INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, PUNITIVE,
; EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF
; ANY KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWSOEVER CAUSED, EVEN IF
; MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
; TO THE FULLEST EXTENT ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
; CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES,
; IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
;
; MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
; TERMS.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

.set Fcy,        40000000

.set US_K,       Fcy/1000000
.set MS_K,       Fcy/10000

	.global _Delay
    .global _Delay_Us

;===============================================
; ms Delay Function
;===============================================
_Delay:

ms_oloop:
    mov #MS_K,w1
ms_iloop:
    nop
    nop
    nop
    nop
    nop

    nop
    nop


    dec     w1, w1
    bra     nz, ms_iloop    
    
    dec     w0,w0
    bra     nz,ms_oloop

    return

;===============================================
; us Delay Function
;===============================================

_Delay_Us:

us_oloop:
    
    .rept (US_K-3)
    nop
    .endr
 
    
    dec     w0,w0
    bra     nz,us_oloop

    return
