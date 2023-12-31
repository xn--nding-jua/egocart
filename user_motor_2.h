//#############################################################################
// $Copyright:
// Copyright (C) 2017-2023 Texas Instruments Incorporated - http://www.ti.com/
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

#ifndef USER_MOTOR_2_H
#define USER_MOTOR_2_H

// **************************************************************************
// the includes

// modules
#include "userParams.h"
#include "user_common.h"
//! \defgroup USER USER
//@{
#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************

// the defines

// Defines the nominal DC bus voltage, V
#define USER_M2_NOMINAL_DC_BUS_VOLTAGE_V         ((float32_t)(48.0))

// Defines the maximum voltage at the AD converter
#define USER_M2_ADC_FULL_SCALE_VOLTAGE_V         ((float32_t)(57.528))        
// Full scale voltage of AD converter, not the current voltage

// Defines the maximum current at the AD converter
#define USER_M2_ADC_FULL_SCALE_CURRENT_A         ((float32_t)(42.856))          
// High Voltage motor control kit

// Defines the analog voltage filter pole location, Hz
#define USER_M2_VOLTAGE_FILTER_POLE_Hz           ((float32_t)(338.357))

// ADC current offsets for A, B, and C phases
#define USER_M2_IA_OFFSET_A    (-21.428)              // ~=0.5*USER_M2_ADC_FULL_SCALE_CURRENT_A
#define USER_M2_IB_OFFSET_A    (-21.428)              // ~=0.5*USER_M2_ADC_FULL_SCALE_CURRENT_A
#define USER_M2_IC_OFFSET_A    (-21.428)              // ~=0.5*USER_M2_ADC_FULL_SCALE_CURRENT_A

// ADC voltage offsets for A, B, and C phases
#define USER_M2_VA_OFFSET_V    (0.990514159)          // ~=1.0
#define USER_M2_VB_OFFSET_V    (0.986255884)          // ~=1.0
#define USER_M2_VC_OFFSET_V    (0.983381569)          // ~=1.0

// Vbus used to calculate the voltage offsets A, B, and C
#define USER_M2_VBUS_OFFSET_V  (0.5*USER_M2_ADC_FULL_SCALE_VOLTAGE_V)     // =0.5*USER_M2_NOMINAL_DC_BUS_VOLTAGE_V

// Defines the maximum negative current to be applied in Id reference
#define USER_M2_MAX_NEGATIVE_ID_REF_CURRENT_A    ((float32_t)(-2.0))

// Defines the number of pwm clock ticks per isr clock tick
//        Note: Valid values are 1, 2 or 3 only
#define USER_M2_NUM_PWM_TICKS_PER_ISR_TICK        (1)

// Defines the number of ISR clock ticks per current controller clock tick
#define USER_M2_NUM_ISR_TICKS_PER_CURRENT_TICK     (1)

// Defines the number of ISR clock ticks per speed controller clock tick
#define USER_M2_NUM_ISR_TICKS_PER_SPEED_TICK     (10)

// Defines the number of current sensors
#define USER_M2_NUM_CURRENT_SENSORS               (3)

// Defines the number of voltage sensors
#define USER_M2_NUM_VOLTAGE_SENSORS               (3)

// Defines the Pulse Width Modulation (PWM) frequency, kHz
#define USER_M2_PWM_FREQ_kHz          ((float32_t)(16.0))
// Maximum PWM frequency
//#define USER_M2_PWM_FREQ_kHz          ((float32_t)(15.0))
//#define USER_M2_PWM_FREQ_kHz          ((float32_t)(10.0))

// Defines the Pulse Width Modulation (PWM) period, usec
#define USER_M2_PWM_PERIOD_usec       ((float32_t)1000.0/USER_M2_PWM_FREQ_kHz)

// Defines the Interrupt Service Routine (ISR) frequency, Hz
#define USER_M2_ISR_FREQ_Hz           (USER_M2_PWM_FREQ_kHz * (float32_t)1000.0 / (float32_t)USER_M2_NUM_PWM_TICKS_PER_ISR_TICK)

// Defines the Interrupt Service Routine (ISR) period, usec
#define USER_M2_ISR_PERIOD_usec       (USER_M2_PWM_PERIOD_usec * (float32_t)USER_M2_NUM_PWM_TICKS_PER_ISR_TICK)

// Defines the direct voltage (Vd) scale factor
#define USER_M2_VD_SF                 ((float32_t)(0.95))

// Defines the voltage scale factor for the system
#define USER_M2_VOLTAGE_SF            (USER_M2_ADC_FULL_SCALE_VOLTAGE_V / (float32_t)4096.0)           // 12 bit ADC, 2^12 = 4096

// Defines the current scale factor for the system
#define USER_M2_CURRENT_SF            (USER_M2_ADC_FULL_SCALE_CURRENT_A / (float32_t)4096.0)    // 12 bit ADC, 2^12 = 4096

// Defines the pole location for the DC bus filter, rad/sec
#define USER_M2_DCBUS_POLE_rps         ((float32_t)(100.0))

// Defines the pole location for the voltage and current offset estimation, rad/s
#define USER_M2_OFFSET_POLE_rps        ((float32_t)(20.0))

// Defines the pole location for the speed control filter, rad/sec
#define USER_M2_SPEED_POLE_rps        ((float32_t)(100.0))

// Defines the analog voltage filter pole location, rad/s
#define USER_M2_VOLTAGE_FILTER_POLE_rps  (MATH_TWO_PI * USER_M2_VOLTAGE_FILTER_POLE_Hz)

// Defines the maximum Vs magnitude in per units allowed
// This value sets the maximum magnitude for the output of the Id and
// Iq PI current controllers. The Id and Iq current controller outputs
// are Vd and Vq. The relationship between Vs, Vd, and Vq is:
// Vs = sqrt(Vd^2 + Vq^2).  In this FOC controller, the Vd value is set
// equal to USER_MAX_VS_MAG*USER_VD_MAG_FACTOR.
// so the Vq value is set equal to sqrt(USER_MAX_VS_MAG^2 - Vd^2).
// Set USER_MAX_VS_MAG = 0.5 for a pure sinewave with a peak at
// SQRT(3)/2 = 86.6% duty cycle.  No current reconstruction
// is needed for this scenario.
// Set USER_MAX_VS_MAG = 1/SQRT(3) = 0.5774 for a pure sinewave
// with a peak at 100% duty cycle.  Current reconstruction
// will be needed for this scenario (Lab08).
// Set USER_MAX_VS_MAG = 2/3 = 0.6666 to create a trapezoidal
// voltage waveform.  Current reconstruction will be needed
// for this scenario (Lab08).
// For space vector over-modulation, see lab08 for details on
// system requirements that will allow the SVM generator to
// go all the way to trapezoidal.
//#define USER_M2_MAX_VS_MAG_PU            (0.66)
//#define USER_M2_MAX_VS_MAG_PU            (0.57)
#define USER_M2_MAX_VS_MAG_PU              (0.5)

// Defines the reference Vs magnitude in per units allowed
//      Set the value equal from 0.5 to 0.95 of the maximum Vs magnitude
#define USER_M2_VS_REF_MAG_PU             (float32_t)(0.8) * USER_MAX_VS_MAG_PU)

// Defines the R/L excitation frequency, Hz
#define USER_M2_R_OVER_L_EXC_FREQ_Hz  ((float32_t)(300.0))

// Defines the R/L Kp scale factor, pu

// Kp used during R/L is USER_M2_R_OVER_L_KP_SF * USER_M2_NOMINAL_DC_BUS_VOLTAGE_V / USER_M2_MOTOR_MAX_CURRENT_A;
#define USER_M2_R_OVER_L_KP_SF        ((float32_t)(0.02))

// Defines maximum acceleration for the estimation speed profiles, Hz/sec
#define USER_M2_MAX_ACCEL_Hzps        ((float32_t)(2.0))

// Defines the controller execution period, usec
#define USER_M2_CTRL_PERIOD_usec      (USER_M2_ISR_PERIOD_usec)

// Defines the controller execution period, sec
#define USER_M2_CTRL_PERIOD_sec       ((float32_t)USER_M2_CTRL_PERIOD_usec/(float32_t)1000000.0)

// Defines the IdRated delta to use during estimation
#define USER_M2_IDRATED_DELTA_A       ((float32_t)(0.0001))

// Defines the forced angle frequency, Hz
#define USER_M2_FORCE_ANGLE_FREQ_Hz            ((float32_t)(1.0))

// Defines the fraction of IdRated to use during inductance estimation
#define USER_M2_IDRATED_FRACTION_FOR_L_IDENT    ((float32_t)(0.5))

// Defines the fraction of SpeedMax to use during inductance estimation
#define USER_M2_SPEEDMAX_FRACTION_FOR_L_IDENT  ((float32_t)(1.0))

// Defines the Power Warp gain for computing Id reference

// If motor parameters are known, set this gain to:

// USER_M2_PW_GAIN = SQRT(1.0 + USER_M2_MOTOR_Rr_Ohm / USER_M2_MOTOR_Rs_Ohm)
#define USER_M2_PW_GAIN                        ((float32_t)(1.0))

// Defines the pole location for the direction filter, rad/sec
#define USER_M2_DIRECTION_POLE_rps             (MATH_TWO_PI * (float32_t)10.0)

// Defines the pole location for the second direction filter, rad/sec
#define USER_M2_DIRECTION_POLE_2_rps           (MATH_TWO_PI * (float32_t)100.0)

// Defines the pole location for the flux estimation, rad/sec
#define USER_M2_FLUX_POLE_rps                  ((float32_t)(10.0))

// Defines the pole location for the R/L estimation, rad/sec
#define USER_M2_R_OVER_L_POLE_rps              (MATH_TWO_PI * (float32_t)3.2)

// Defines the near zero speed limit for electrical frequency estimation, Hz
//!        The flux integrator uses this limit to regulate flux integration
#define USER_M2_FREQ_NEARZEROSPEEDLIMIT_Hz     ((float32_t)(0.0))

// Defines the convergence factor for the estimator
#define USER_M2_EST_KAPPAQ                     ((float32_t)(1.5))

// Defines the scale factor for the flux estimation
//! the default value is 1.0f, change the value between 0.1f and 1.25f
#define USER_M2_EST_FLUX_HF_SF                 ((float32_t)(0.125f))

// Defines the scale factor for the frequency estimation
//! the default value is 1.0f, change the value between 0.5f and 1.5f
#define USER_M2_EST_FREQ_HF_SF                 ((float32_t)(1.0f))

// Defines the scale factor for the bemf estimation
//! the default value is 1.0f, change the value between 0.50f and 1.25f
#define USER_M2_EST_BEMF_HF_SF                 ((float32_t)(0.50f))

// A flag to bypass motor identification (1/0 : true/false)
#define USER_M2_BYPASS_MOTOR_ID            1

// Define the Kp gain for Field Weakening Control
#define USER_M2_FWC_KP                 0.05
// Define the Ki gain for Field Weakening Control
#define USER_M2_FWC_KI                 0.0002
// Define the maximum current vector angle for Field Weakening Control
#define USER_M2_FWC_MAX_ANGLE          -75.0                        // degree
#define USER_M2_FWC_MAX_ANGLE_RAD      USER_M2_FWC_MAX_ANGLE/180.0  // degree->rad
// Define the minimum current vector angle for Field Weakening Control
#define USER_M2_FWC_MIN_ANGLE          0.0                          // degree
#define USER_M2_FWC_MIN_ANGLE_RAD      USER_M2_FWC_MIN_ANGLE/180.0  // degree->rad
//============================================================================================

// Motor defines
//#define USER_M2_MOTOR Estun_EMJ_04APB22_A
//#define USER_M2_MOTOR Estun_EMJ_04APB22_B
//#define USER_M2_MOTOR Regal_Beloit_5SME39DL0756
//#define USER_M2_MOTOR Anaheim_BLY172S_24V
//#define USER_M2_MOTOR Anaheim_BLY341S_48V
//#define USER_M2_MOTOR Anaheim_BLY341S_24V
//#define USER_M2_MOTOR Traxxas_Velineon_380
//#define USER_M2_MOTOR Traxxas_Velineon_3500
//#define USER_M2_MOTOR Teknic_M2310PLN04K
//#define USER_M2_MOTOR Teknic_2310S
//#define USER_M2_MOTOR Anaheim_BLZ362S
//#define USER_M2_MOTOR Anaheim_BLWS235D
//#define USER_M2_MOTOR tekin_redline_4600KV
//#define USER_M2_MOTOR low_voltage_ceiling_fan

// ACI Motor
//#define USER_M2_MOTOR Marathon_5K33GN2A
//#define USER_M2_MOTOR Marathon_56H17T2011A
//#define USER_M2_MOTOR Dayton_3N352C
//#define USER_M2_MOTOR my_pm_motor_1
//#define USER_M2_MOTOR my_aci_motor_2

#define USER_M2_MOTOR Bosch_BLDC_F016L68035

#if (USER_M2_MOTOR == Estun_EMJ_04APB22_A)
#define USER_M2_MOTOR_TYPE                    MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (0.0)
#define USER_M2_MOTOR_Rs_Ohm                 (2.303403)
#define USER_M2_MOTOR_Ls_d_H                 (0.008464367)
#define USER_M2_MOTOR_Ls_q_H                 (0.008464367)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.3821270569)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (3.82)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (2500.0)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (400.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (20.0)          // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (200.0)         // Volt
#elif (USER_M2_MOTOR == Estun_EMJ_04APB22_B)
#define USER_M2_MOTOR_TYPE                    MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (0.0)
#ifdef _EXT_OPA_SC_EN_

// HV Kit with external OPA
#define USER_M2_MOTOR_Rs_Ohm                 (2.98774099)
#define USER_M2_MOTOR_Ls_d_H                 (0.008926632)
#define USER_M2_MOTOR_Ls_q_H                 (0.008926632)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.445965141)
#endif
#ifdef _PGA_GAIN_6_EN_

// DMPFC Board with internal PGA and gain is 6
#define USER_M2_MOTOR_Rs_Ohm                 (3.34299588)
#define USER_M2_MOTOR_Ls_d_H                 (0.0107685775)
#define USER_M2_MOTOR_Ls_q_H                 (0.0107685775)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.446204126)
#endif
#ifdef _PGA_GAIN_12_EN_

// DMPFC Board with internal PGA and gain is 12
#define USER_M2_MOTOR_Rs_Ohm                 (3.52491379)
#define USER_M2_MOTOR_Ls_d_H                 (0.0107106492)
#define USER_M2_MOTOR_Ls_q_H                 (0.0107106492)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.443505377)
#endif
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (3.82)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (2500.0)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (400.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (20.0)          // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (200.0)         // Volt
#elif (USER_M2_MOTOR == hawkins_ceiling_fan)
#define USER_M2_MOTOR_TYPE                    MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (6)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (34.98956)
#define USER_M2_MOTOR_Ls_d_H                 (0.3858752)
#define USER_M2_MOTOR_Ls_q_H                 (0.3858752)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.9784898726)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (0.1)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-0.1)
#define USER_M2_MOTOR_MAX_CURRENT_A          (1.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (10.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#elif (USER_M2_MOTOR == Teknic_M2310PLN04K)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.381334811)
#define USER_M2_MOTOR_Ls_d_H                 (0.000169791776)
#define USER_M2_MOTOR_Ls_q_H                 (0.000169791776)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.0398557819)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.5)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (6.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (40.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (1000)
#define USER_M2_MOTOR_INERTIA_Kgm2           (7.06154e-06)
#define USER_M2_MOTOR_RATED_VOLTAGE_V        (24.0)
#define USER_M2_MOTOR_RATED_SPEED_KRPM       (3.0)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (600.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (20.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (24.0)          // Volt
#elif (USER_M2_MOTOR == teknic_2310S)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.3654691)
#define USER_M2_MOTOR_Ls_d_H                 (0.0002068772)
#define USER_M2_MOTOR_Ls_q_H                 (0.0002068772)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.04052209)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (4.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#define USER_M2_MOTOR_FREQ_LOW             (10.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH            (100.0)         // Hz
#define USER_M2_MOTOR_FREQ_MAX             (120.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN             (3.0)           // Volt
#define USER_M2_MOTOR_VOLT_MAX             (18.0)          // Volt
#elif (USER_M2_MOTOR == Anaheim_BLZ362S)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.5740876)
#define USER_M2_MOTOR_Ls_d_H                 (0.001202451)
#define USER_M2_MOTOR_Ls_q_H                 (0.001202451)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.2757460695)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (10.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#elif (USER_M2_MOTOR == Anaheim_BLWS235D)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.124559097)
#define USER_M2_MOTOR_Ls_d_H                 (0.000269943761)
#define USER_M2_MOTOR_Ls_q_H                 (0.000269943761)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.0507731885)      // 4.11V/krpm
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.5)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#define USER_M2_MOTOR_INERTIA_Kgm2           (3.99683e-05)     // 0.00566 oz-in-sec2
#define USER_M2_MOTOR_RATED_VOLTAGE_V        (160.0)
#define USER_M2_MOTOR_RATED_SPEED_KRPM       (3.0)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)             // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (300.0)           // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)            // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (200.0)           // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (20.0)            // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (160.0)           // Volt
#elif (USER_M2_MOTOR == Regal_Beloit_5SME39DL0756)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (3)
#define USER_M2_MOTOR_Rr_Ohm                 (0.0)
#define USER_M2_MOTOR_Rs_Ohm                 (4.581007)
#define USER_M2_MOTOR_Ls_d_H                 (0.03727356)
#define USER_M2_MOTOR_Ls_q_H                 (0.03727356)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.6589699)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (2.6)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#elif (USER_M2_MOTOR == Teknic_M2311SLN02K)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (1.41492)
#define USER_M2_MOTOR_Ls_d_H                 (0.001190699)
#define USER_M2_MOTOR_Ls_q_H                 (0.001190699)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.1035664)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-0.5)
#define USER_M2_MOTOR_MAX_CURRENT_A          (3.8)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (1000.0)
#define USER_M2_MOTOR_RATED_VOLTAGE_V        (160.0)
#define USER_M2_MOTOR_RATED_SPEED_KRPM       (3.0)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (300.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (24.0)          // Volt
#elif (USER_M2_MOTOR == Anaheim_BLY172S_24V)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.4)
#define USER_M2_MOTOR_Ls_d_H                 (0.0007190173)
#define USER_M2_MOTOR_Ls_q_H                 (0.0007190173)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.0327013217)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.5)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-1.5)
#define USER_M2_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#define USER_M2_MOTOR_INERTIA_Kgm2           (4.80185e-06)
#define USER_M2_MOTOR_RATED_VOLTAGE_V        (24.0)
#define USER_M2_MOTOR_RATED_SPEED_KRPM       (3.0)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (300.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (24.0)          // Volt
#elif (USER_M2_MOTOR == Anaheim_BLY341S_48V)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.463800967)
#define USER_M2_MOTOR_Ls_d_H                 (0.00114538975)
#define USER_M2_MOTOR_Ls_q_H                 (0.00114538975)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.0978558362)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (2.5)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-2.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (20.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (300.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (24.0)          // Volt
#elif (USER_M2_MOTOR == Anaheim_BLY341S_24V)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.124559097)
#define USER_M2_MOTOR_Ls_d_H                 (0.000269943761)
#define USER_M2_MOTOR_Ls_q_H                 (0.000269943761)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.0507731885)      // 4.11V/krpm
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (4.5)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-4.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (12.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#define USER_M2_MOTOR_INERTIA_Kgm2           (3.99683e-05)     // 0.00566 oz-in-sec2
#define USER_M2_MOTOR_RATED_VOLTAGE_V        (24.0)
#define USER_M2_MOTOR_RATED_SPEED_KRPM       (3.0)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)             // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (300.0)           // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)            // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (200.0)           // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (4.0)             // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (24.0)            // Volt
#elif (USER_M2_MOTOR == Traxxas_Velineon_380)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (1)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (TBD)
#define USER_M2_MOTOR_Ls_d_H                 (TBD)
#define USER_M2_MOTOR_Ls_q_H                 (TBD)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (TBD)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (4.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-0.5)
#define USER_M2_MOTOR_MAX_CURRENT_A          (10.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (20.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#elif (USER_M2_MOTOR == Traxxas_Velineon_3500)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (1)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.01822988)
#define USER_M2_MOTOR_Ls_d_H                 (8.322238e-06)
#define USER_M2_MOTOR_Ls_q_H                 (8.322238e-06)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.010249538)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (2.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-2.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (40.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#elif (USER_M2_MOTOR == tekin_redline_4600KV)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (2)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.0181193)
#define USER_M2_MOTOR_Ls_d_H                 (8.180002e-06)
#define USER_M2_MOTOR_Ls_q_H                 (8.180002e-06)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.0041173688)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (3.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-3.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (60.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#elif (USER_M2_MOTOR == low_voltage_ceiling_fan)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (8)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.3974416)
#define USER_M2_MOTOR_Ls_d_H                 (0.0001718943)
#define USER_M2_MOTOR_Ls_q_H                 (0.0001718943)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.282237479574)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-1.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (3.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (10.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#elif (USER_M2_MOTOR == Marathon_5K33GN2A)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (2)
#define USER_M2_MOTOR_Rr_Ohm                 (5.508003)
#define USER_M2_MOTOR_Rs_Ohm                 (10.71121)
#define USER_M2_MOTOR_Ls_d_H                 (0.05296588)
#define USER_M2_MOTOR_Ls_q_H                 (0.05296588)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.8165*220.0/60.0)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (1.378)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (NULL)
#define USER_M2_MOTOR_MAX_CURRENT_A          (3.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (400.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (20.0)          // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (200.0)         // Volt
#elif (USER_M2_MOTOR == Marathon_56H17T2011A)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (2)
#define USER_M2_MOTOR_Rr_Ohm                 (5.159403)
#define USER_M2_MOTOR_Rs_Ohm                 (7.924815)
#define USER_M2_MOTOR_Ls_d_H                 (0.03904648)
#define USER_M2_MOTOR_Ls_q_H                 (0.03904648)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.8*0.8165*230.0/60.0)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (0.9941965)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (0.5)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (NULL)
#define USER_M2_MOTOR_MAX_CURRENT_A          (2.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (400.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (200.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (20.0)          // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (200.0)         // Volt
#elif (USER_M2_MOTOR == Dayton_3N352C)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (2)
#define USER_M2_MOTOR_Rr_Ohm                 (2.428799)
#define USER_M2_MOTOR_Rs_Ohm                 (2.863202)
#define USER_M2_MOTOR_Ls_d_H                 (0.02391323)
#define USER_M2_MOTOR_Ls_q_H                 (0.02391323)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.8165*230.0/60.0)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (1.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (NULL)
#define USER_M2_MOTOR_MAX_CURRENT_A          (3.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (5.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
//------------------------------------------------------------------
#elif (USER_M2_MOTOR == my_pm_motor_1)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (NULL)
#define USER_M2_MOTOR_Ls_d_H                 (NULL)
#define USER_M2_MOTOR_Ls_q_H                 (NULL)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (NULL)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (2.0)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-2.0)
#define USER_M2_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (40.0)
#define USER_M2_MOTOR_INERTIA_Kgm2           (7.06154e-06)
#define USER_M2_MOTOR_RATED_VOLTAGE_V        (36.0)

// Number of lines on the motor's quadrature encoder
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (1000)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (300.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (24.0)          // Volt
#elif (USER_M2_MOTOR == my_aci_motor_2)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_INDUCTION
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (2)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (NULL)
#define USER_M2_MOTOR_Ls_d_H                 (NULL)
#define USER_M2_MOTOR_Ls_q_H                 (NULL)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.8165*230.0/60.0)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (0.5)
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (NULL)
#define USER_M2_MOTOR_MAX_CURRENT_A          (5.0)
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (5.0)

// Number of lines on the motor's quadrature encoder
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (1000)
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (300.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (10.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (4.0)           // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (24.0)          // Volt

#elif (USER_M2_MOTOR == Bosch_BLDC_F016L68035)
#define USER_M2_MOTOR_TYPE                   MOTOR_TYPE_PM
#define USER_M2_MOTOR_NUM_POLE_PAIRS         (4)
#define USER_M2_MOTOR_Rr_Ohm                 (NULL)
#define USER_M2_MOTOR_Rs_Ohm                 (0.041575864)
#define USER_M2_MOTOR_Ls_d_H                 (0.000128439)
#define USER_M2_MOTOR_Ls_q_H                 (0.000128439)
#define USER_M2_MOTOR_RATED_FLUX_VpHz        (0.0863205)
#define USER_M2_MOTOR_MAGNETIZING_CURRENT_A  (NULL)
#define USER_M2_MOTOR_RES_EST_CURRENT_A      (2.5)           // 10% of I_rated. Increase, if motor is not ramping up during RampUp
#define USER_M2_MOTOR_IND_EST_CURRENT_A      (-2.5)          // 10% of I_rated. Increase, if motor is not ramping up during RampUp
#define USER_M2_MOTOR_MAX_CURRENT_A          (23.0)          // Torque-current = sqrt(2)*rated current
#define USER_M2_MOTOR_FLUX_EXC_FREQ_Hz       (60.0)
#define USER_M2_MOTOR_NUM_ENC_SLOTS          (NULL)
#define USER_M2_MOTOR_INERTIA_Kgm2           (0.0002)
#define USER_M2_MOTOR_RATED_VOLTAGE_V        (36.0)          // V
#define USER_M2_MOTOR_RATED_SPEED_KRPM       (3.4)           // krpm
#define USER_M2_MOTOR_MIN_MAX_HZ             (5.0)           // Hz
#define USER_M2_MOTOR_FREQ_MAX_HZ            (600.0)         // Hz
#define USER_M2_MOTOR_FREQ_LOW_HZ            (20.0)          // Hz
#define USER_M2_MOTOR_FREQ_HIGH_HZ           (400.0)         // Hz
#define USER_M2_MOTOR_VOLT_MIN_V             (5.0)           // Volt
#define USER_M2_MOTOR_VOLT_MAX_V             (36.0)          // Volt

#else
#error No motor type specified
#endif

// Defines the maximum current slope for Id trajectory
#define USER_M2_MAX_CURRENT_DELTA_A        (USER_M2_MOTOR_RES_EST_CURRENT_A / USER_M2_ISR_FREQ_Hz)

// Defines the maximum current slope for Id trajectory during power warp mode
#define USER_M2_MAX_CURRENT_DELTA_PW_A    (0.3 * USER_M2_MOTOR_RES_EST_CURRENT_A / USER_M2_ISR_FREQ_Hz)
#ifndef USER_M2_MOTOR
#error Motor type is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_TYPE
#error The motor type is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_NUM_POLE_PAIRS
#error Number of motor pole pairs is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_Rr_Ohm
#error The rotor resistance is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_Rs_Ohm
#error The stator resistance is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_Ls_d_H
#error The direct stator inductance is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_Ls_q_H
#error The quadrature stator inductance is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_RATED_FLUX_VpHz
#error The rated flux of motor is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_MAGNETIZING_CURRENT_A
#error The magnetizing current is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_RES_EST_CURRENT_A
#error The resistance estimation current is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_IND_EST_CURRENT_A
#error The inductance estimation current is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_MAX_CURRENT_A
#error The maximum current is not defined in user_motor1.h
#endif
#ifndef USER_M2_MOTOR_FLUX_EXC_FREQ_Hz
#error The flux excitation frequency is not defined in user_motor1.h
#endif
#if ((USER_M2_NUM_CURRENT_SENSORS < 2) || (USER_M2_NUM_CURRENT_SENSORS > 3))
#error The number of current sensors must be 2 or 3
#endif
#if (USER_M2_NUM_VOLTAGE_SENSORS != 3)
#error The number of voltage sensors must be 3
#endif

// **************************************************************************

// the typedefs

// **************************************************************************

// the globals

// **************************************************************************

// the functions
#ifdef __cplusplus
}
#endif 
// extern "C"
//@}  
// ingroup
#endif 
// end of USER_MOTOR1_H definition
