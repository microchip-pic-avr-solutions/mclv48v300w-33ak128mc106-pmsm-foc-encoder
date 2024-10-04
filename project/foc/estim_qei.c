// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_qei.c
 *
 * @brief This module implements  encoder speed estimation
 *
 * Component: ESTIMATOR - QEI
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* © [2024] Microchip Technology Inc. and its subsidiaries
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

#include <stdint.h>
#include <math.h>
#include <xc.h>

#include "estim_qei.h"
#include "qei.h"
#include "userparms.h"
#include "general.h"

// <editor-fold defaultstate="collapsed" desc="VARIABLES">

tEncoder              encoder;

// </editor-fold>

/**
* <B> Function: Encoder_Initialize() </B>
*
* @brief Function initializes the encoder parameters.
*        
* @param none.
* @return none.
* @example
* <CODE> Encoder_Initialize(); </CODE>
*
*/
void Encoder_Initialize(void)
{
    encoder.speed = 0.0f;
    encoder.speedKFilter = KFILTER_VELESTIM;
    encoder.sinTheta = 0;
    encoder.cosTheta = 0;
}

/**
* <B> Function: Encoder_Angle_Speed_Estimate() </B>
*
* @brief Function implements angle and speed calculate using encoder.
*        .
* @param none.
* @return none.
* @example
* <CODE> Encoder_Angle_Speed_Estimate(); </CODE>
*
*/
void Encoder_Angle_Speed_Estimate(void)
{
    /*Electrical angle of the motor*/
    float theta_electrical = 0.0f;
    /*previous values for sin and cos*/
    float sinThetaPrev = encoder.sinTheta; 
    float cosThetaPrev = encoder.cosTheta;
    /*temporary variables for speed calculations*/
    float tmp1 = 0.0f, tmp2 = 0.0f;
    
    /*Calculate electrical angle from encoder position counts*/
    encoder.positionCount = POS1CNT;
    theta_electrical = encoder.positionCount * ENCODER_COUNTS_TO_RADIANS;
    encoder.sinTheta =  sin(theta_electrical);
    encoder.cosTheta =  cos(theta_electrical);      
   
    /*Estimated Speed = ( Cos(theta)*(Sin(theta) - Previous Sin(theta))) - 
     *                  (Sin(theta)*(Cos(theta) - Previous Cos(theta)))  ) /
     *                                                      Loop Time in Sec  */
    tmp1 = encoder.cosTheta *(encoder.sinTheta - sinThetaPrev);
    tmp2 = encoder.sinTheta *(encoder.cosTheta - cosThetaPrev);
	encoder.speed = (tmp1 - tmp2) / LOOPTIME_SEC;
	encoder.Rho = theta_electrical;
    LowPassFilter(encoder.speed, encoder.speedKFilter, &encoder.VelEstim);
 }

   
