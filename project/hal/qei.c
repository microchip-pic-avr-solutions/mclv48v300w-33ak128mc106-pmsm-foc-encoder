// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file qei.c
 *
 * @brief This module configures and enables the QEI Module 
 * 
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: QEI
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* � [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#include <xc.h>

#include "userparms.h"
#include "qei.h"

// </editor-fold>


// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: InitQEI()    </B>
*
* @brief Function initializes and configures the registers  for QEI modules.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitQEI();     </CODE>
*
*/
void QEI_Initialize(void)
{
 /* Reset the control register */
  QEI1CON = 0;
 /* Position Counter Initialization Mode Select bits
  * 0b110 = Modulo Count mode for position counter */
  QEI1CONbits.PIMOD = 0b110;
 /*QEI module counter enable bit*/
  QEI1CONbits.ON = 0;
  /*QEI Status Register*/
  QEI1STAT = 0;
  /*Position Counter Register bits*/
  POS1CNT = 0;
  /*Position Counter hold register bits*/
  POS1HLD = 0;
  /* Set lower encoder boundary counts */
  QEI1LEC = -(ENCODER_COUNTS_HALF_ELEC);                 
 /* Set higher encoder boundary counts */
  QEI1GEC = (ENCODER_COUNTS_HALF_ELEC - 1);  
}
/**
* <B> Function: StartQEI()    </B>
*
* @brief Start Qei counting.
*        
* @param none.
* @return none.
* 
* @example
* <CODE> StartQEI();     </CODE>
*
*/
void StartQEI(void)
{
    QEI1CONbits.ON = 1;
}

// </editor-fold>