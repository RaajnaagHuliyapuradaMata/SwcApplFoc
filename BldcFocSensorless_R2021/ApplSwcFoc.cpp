/******************************************************************************/
/* File   : Template.hpp                                                      */
/*                                                                            */
/* Author : Nagaraja HULIYAPURADA MATA                                        */
/*                                                                            */
/* License / Warranty / Terms and Conditions                                  */
/*                                                                            */
/* Everyone is permitted to copy and distribute verbatim copies of this lice- */
/* nse document, but changing it is not allowed. This is a free, copyright l- */
/* icense for software and other kinds of works. By contrast, this license is */
/* intended to guarantee your freedom to share and change all versions of a   */
/* program, to make sure it remains free software for all its users. You have */
/* certain responsibilities, if you distribute copies of the software, or if  */
/* you modify it: responsibilities to respect the freedom of others.          */
/*                                                                            */
/* All rights reserved. Copyright © 1982 Nagaraja HULIYAPURADA MATA           */
/*                                                                            */
/* Always refer latest software version from:                                 */
/* https://github.com/NagarajaHuliyapuradaMata?tab=repositories               */
/*                                                                            */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "types.hpp"

#include "cmsis_gcc.hpp"
#include "Mat.hpp"
#include "ApplSwcFoc.hpp"

#include "bdrv.hpp"
#include "adc1.hpp"
#include "ccu6.hpp"
#include "csa.hpp"
#include "gpt12e.hpp"

#include "ccu6_defines.hpp"
#include "gpt12e_defines.hpp"
#include "scu_defines.hpp"

#include "ApplSwcFoc_Pbcfg.hpp"

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/
#define CCU6_MASK_MCMOUTS_ENABLE_MCMOUTS                               (0x00BFu)
#define EMO_IMESS                                                              1

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* TYPEDEFS                                                                   */
/******************************************************************************/

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
uint32      Emo_AdcResult[4u];
TEmo_Status Emo_Status;
TEmo_Ctrl   Emo_Ctrl;
TEmo_Foc    Emo_Foc;
TEmo_Svm    Emo_Svm;
uint16      CSA_Offset;

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
static void Emo_lInitFocPar(void);

static void Emo_lInitFocPar(void){
   float KU = 15.0;
   float KI;
   float KPSIE = 0.1;
   float KUZ;
   static float x;
   float CoAFlux = 200.0;
   float OpGain;
   uint16 i;
   Emo_Status.MotorStartError = 0;
   Emo_Svm.MaxAmp = (uint16)(CCU6_T12PR / EMO_CFG_FOC_TABLE_SCALE);
   Emo_Svm.MaxAmp9091pr = (29789 * Emo_Svm.MaxAmp) >> MAT_FIX_SHIFT;
   Emo_Svm.MaxAmp4164pr = (13643 * Emo_Svm.MaxAmp) >> MAT_FIX_SHIFT;
   Emo_Svm.MaxAmpQuadrat = (uint32)Emo_Svm.MaxAmp * Emo_Svm.MaxAmp;
   Emo_Svm.Kfact256 = 8388608 / Emo_Svm.MaxAmp; //2 exp23/MaxAmp

   OpGain = 10.0;
   CSA_Set_Gain(0u);
   x = Emo_Focpar_Cfg.NominalCurrent * Emo_Focpar_Cfg.Rshunt * 10.0;
   if(x < 1.25){
      OpGain = 10.0;
      CSA_Set_Gain(0u);
   }
   x = Emo_Focpar_Cfg.NominalCurrent * Emo_Focpar_Cfg.Rshunt * 20.0;
   if(x < 1.25){
      OpGain = 20.0;
      CSA_Set_Gain(1u);
   }
   x = Emo_Focpar_Cfg.NominalCurrent * Emo_Focpar_Cfg.Rshunt * 40.0;
   if(x < 1.25){
      OpGain = 40.0;
      CSA_Set_Gain(2u);
   }
   x = Emo_Focpar_Cfg.NominalCurrent * Emo_Focpar_Cfg.Rshunt * 60.0;
   if(x < 1.25){
      OpGain = 60.0;
      CSA_Set_Gain(3u);
   }

   KI = 5.0 * 2.0 / (Emo_Focpar_Cfg.Rshunt * OpGain);
   KUZ = 32768.0 * 12.0 / 1612.0;
   CSA_ClearVZERO();  // CSA_Power_Off ?
   ADC1_SetMode(SW_MODE);
   while(true == ADC1_Busy()){}
   ADC1_SetSocSwMode(ADC1_CH1);
   while(false == ADC1_GetEocSwMode()){}
   while(0 == ADC1_GetVF1()){}
   CSA_Offset = ADC1_GetOUT_CH1();
   ADC1_SetMode(SEQ_MODE);
   while(false == ADC1_Busy()){}
   i = CSA_Offset;
   if(i > 1800){
      i = 1800;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_CSAOFFSET;
   }
   if(i < 1500){
      i = 1500;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_CSAOFFSET;
   }
   Emo_Svm.CsaOffset = i;

   x = 32768.0 * KI * Emo_Focpar_Cfg.PhaseRes / KU;
   if(x > 32767.0){
      x = 32767.0;
   }
   Emo_Foc.PhaseRes = (uint16)x;

   x = 32768.0 * KI * Emo_Focpar_Cfg.PhaseInd / KPSIE;
   if(x > 32767.0){
      x = 32767.0;
   }
   Emo_Foc.PhaseInd = (uint16)x;

   Emo_Foc.Kdcdivident1 = (uint32)(KU * 1.7320508 * Emo_Svm.MaxAmp / KUZ * 32768 / 2.0);
   Emo_Foc.Kdcfactor2 = (uint16)(KUZ * 32768.0 * 32768.0 / (KU * 1.7320508 * Emo_Svm.MaxAmp * 64));
   Emo_Foc.Kdcfactoriqc = (uint16)(32768.0 * KUZ / (1.7320508 * KU * 32.0));
   Emo_Foc.PolePair = (uint16)Emo_Focpar_Cfg.PolePair;

   x = Emo_Focpar_Cfg.StartCurrent / KI * 32768.0;
   if(x > 32767){
      x = 32767;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_STARTCURRENT;
   }
   if(x < 1){
      x = 1;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_STARTCURRENT;
   }
   Emo_Foc.StartCurrent = (sint16)x;

   x = Emo_Focpar_Cfg.TimeSpeedzero * SCU_FSYS / ((GPT12E_T2) * 4.0);
   if(x > 32767){
      x = 32767;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_STARTTIME;
   }
   if(x < 1){
      x = 1;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_STARTTIME;
   }
   Emo_Foc.TimeSpeedzero = (uint16)x;

   Emo_Foc.StartEndSpeed = (sint16)Emo_Focpar_Cfg.StartSpeedEnd;

   x = ((GPT12E_T2) * 4.0) / SCU_FSYS * Emo_Focpar_Cfg.StartSpeedSlewRate * 65536.0;
   if(x > 2147483647){
      x = 2147483647;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_SPEEDSLEWRATE;
   }
   if(x < 1){
      x = 1;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_SPEEDSLEWRATE;
   }
   Emo_Foc.StartSpeedSlewRate = (sint32)x;

   Emo_Foc.SpeedtoFrequency = (sint16)((32768.0 * (1.0 / CCU6_T12_FREQ) * 32768.0 / 30.0) * FOC_POLE_PAIRS); // TBD: Read from Pbcfg - FOC_POLE_PAIRS

   CoAFlux = 32768.0 * KU / (KPSIE * CCU6_T12_FREQ);
   Emo_Foc.RealFluxLp.CoefA = (sint16)CoAFlux;
   Emo_Foc.ImagFluxLp.CoefA = (sint16)CoAFlux;
   Emo_Foc.LpCoefb1 = (uint16)(32768.0 / (Emo_Focpar_Cfg.TimeConstantEstFluxFilter * CCU6_T12_FREQ)); //Time const = 0.10s
   Emo_Foc.LpCoefb2 = (uint16)(32768.0 / (0.01 * CCU6_T12_FREQ)); //Time const = 0.010s
   Emo_Foc.RealFluxLp.CoefB = Emo_Foc.LpCoefb1;
   Emo_Foc.ImagFluxLp.CoefB = Emo_Foc.LpCoefb1;
   Emo_Ctrl.SpeedPi.Kp = Emo_Focpar_Cfg.SpeedPi_Kp;
   Emo_Ctrl.SpeedPi.Ki = Emo_Focpar_Cfg.SpeedPi_Ki;
   Emo_Ctrl.SpeedPi.PiMin = (sint16)(32767.0 * Emo_Focpar_Cfg.MinRefStartCurr / KI);
   Emo_Ctrl.SpeedPi.PiMax = (sint16)(32767.0 * Emo_Focpar_Cfg.MaxRefStartCurr / KI);

   x = 32767.0 * Emo_Focpar_Cfg.MaxRefCurr / KI;
   if(x > 32767){
      x = 32767;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_REFCURRENT;
   }
   Emo_Ctrl.MaxRefCurrent = (sint16)x;

   x = 32767.0 * Emo_Focpar_Cfg.MinRefCurr / KI;
   if(x < -32767){
      x = -32767;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_REFCURRENT;
   }
   Emo_Ctrl.MinRefCurrent = (sint16)x;

   x = 32767.0 * Emo_Focpar_Cfg.MaxRefStartCurr / KI;
   if(x > 32767){
      x = 32767;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_REFCURRENT;
   }
   Emo_Ctrl.MaxRefStartCurrent = (sint16)x;

   x = 32767.0 * Emo_Focpar_Cfg.MinRefStartCurr / KI;
   if(x < -32767){
      x = -32767;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_REFCURRENT;
   }
   Emo_Ctrl.MinRefStartCurrent = (sint16)x;

   Emo_Ctrl.Speedlevelmaxstart = (sint16)Emo_Focpar_Cfg.SpeedLevelPos;
   Emo_Ctrl.Speedlevelminstart = (sint16)Emo_Focpar_Cfg.SpeedLevelNeg;
   Emo_Ctrl.SpeedLevelSwitchOn = (sint16)Emo_Focpar_Cfg.SpeedLevelSwitchOn;

   if(Emo_Ctrl.Speedlevelmaxstart < Emo_Ctrl.Speedlevelminstart){
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_SPEED_POINTS;
   }
   if(Emo_Ctrl.Speedlevelmaxstart < Emo_Ctrl.SpeedLevelSwitchOn){
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_SPEED_POINTS;
   }
   if((Emo_Ctrl.Speedlevelminstart + Emo_Ctrl.SpeedLevelSwitchOn) > 0){
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_SPEED_POINTS;
   }
   if(Emo_Foc.StartEndSpeed < Emo_Ctrl.SpeedLevelSwitchOn){
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_SPEED_POINTS;
   }
   if(Emo_Ctrl.MaxRefCurrent < Emo_Ctrl.MaxRefStartCurrent){
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_LIMITS_REFCURRENT;
   }
   if(Emo_Ctrl.MinRefCurrent > Emo_Ctrl.MinRefStartCurrent){
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_LIMITS_REFCURRENT;
   }

   x = Emo_Focpar_Cfg.AdjustmCurrentControl;
   if((x > 1) || (x < 0.01)){
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_VALUE_CU_ADCC;
   }

   x = Emo_Focpar_Cfg.AdjustmCurrentControl * KI * Emo_Focpar_Cfg.PhaseInd / (256.0 * (1.0 / CCU6_T12_FREQ) * KU) * 32767.0;
   if(x > 32767){
      x = 32767;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_VALUE_CU_KP;
   }
   if(x < 1){
      x = 1;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_VALUE_CU_KP;
   }
   Emo_Ctrl.RealCurrPi.Kp = (sint16)x;

   x = Emo_Focpar_Cfg.AdjustmCurrentControl * KI * Emo_Focpar_Cfg.PhaseRes / (4.0 * KU) * 32767.0;
   if(x > 32767){
      x = 32767;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_VALUE_CU_KI;
   }
   if(x < 1){
      x = 1;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_VALUE_CU_KI;
   }
   Emo_Ctrl.RealCurrPi.Ki = (sint16)x;

   Emo_Ctrl.RealCurrPi.IMin = -28272;
   Emo_Ctrl.RealCurrPi.IMax = 28272;
   Emo_Ctrl.RealCurrPi.PiMin = -28272;
   Emo_Ctrl.RealCurrPi.PiMax = 28272;
   Emo_Ctrl.ImagCurrPi.Kp =  Emo_Ctrl.RealCurrPi.Kp;
   Emo_Ctrl.ImagCurrPi.Ki =  Emo_Ctrl.RealCurrPi.Ki;
   Emo_Ctrl.ImagCurrPi.IMin = -16580;
   Emo_Ctrl.ImagCurrPi.IMax = 16580;
   Emo_Ctrl.ImagCurrPi.PiMin = -16580;
   Emo_Ctrl.ImagCurrPi.PiMax = 16580;

   x = (1.0 / CCU6_T12_FREQ) / (Emo_Focpar_Cfg.TimeConstantSpeedFilter) * 32768.0;
   if(x > 32767){
      x = 32767;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_T_SPEED_LP;
   }
   if(x < 1){
      x = 1;
      Emo_Status.MotorStartError = Emo_Status.MotorStartError | EMO_ERROR_T_SPEED_LP;
   }
   Emo_Ctrl.RotCurrImagLpdisplay.CoefA = 10;
   Emo_Ctrl.RotCurrImagLpdisplay.CoefB = 10;
   Emo_Ctrl.SpeedLp.CoefA = (sint16)x;
   Emo_Ctrl.SpeedLp.CoefB = (sint16)x;
   Emo_Ctrl.FluxbtrLp.CoefA = 1000;
   Emo_Ctrl.FluxbtrLp.CoefB = 1000;
   Emo_Ctrl.SpeedLpdisplay.CoefA = 1000;
   Emo_Ctrl.SpeedLpdisplay.CoefB = 1000;
   Emo_Ctrl.Pllkp = 100;

   x = 60.0 * Emo_Focpar_Cfg.PWM_Frequency / 64.0;
   if(x > 32767.0){
      x = 32767;
   }
   Emo_Ctrl.Factorspeed = (uint16)x;

   x = 60.0 * Emo_Focpar_Cfg.PWM_Frequency / (Emo_Focpar_Cfg.MaxSpeed * Emo_Focpar_Cfg.PolePair * 4.0);
   if(x >= 32.0){
      Emo_Ctrl.Exppllhigh = 5;
      Emo_Ctrl.Anglersptr = 32;
      Emo_Ctrl.Expspeedhigh = 0;
   }
   else{
      if(x >= 16.0){
         Emo_Ctrl.Exppllhigh = 4;
         Emo_Ctrl.Anglersptr = 16;
         Emo_Ctrl.Expspeedhigh = 1;
      }
      else{
         if(x >= 8.0){
            Emo_Ctrl.Exppllhigh = 3;
            Emo_Ctrl.Anglersptr = 8;
            Emo_Ctrl.Expspeedhigh = 2;
         }
         else{
            if(x >= 4.0){
               Emo_Ctrl.Exppllhigh = 2;
               Emo_Ctrl.Anglersptr = 4;
               Emo_Ctrl.Expspeedhigh = 3;
            }
            else{
               Emo_Ctrl.Exppllhigh = 1;
               Emo_Ctrl.Anglersptr = 2;
               Emo_Ctrl.Expspeedhigh = 4;
            }
         }
      }
   }
}

uint32 Emo_Init(void){
   if(
         EMO_MOTOR_STATE_UNINIT
      != Emo_Status.MotorState
   ){
     return EMO_ERROR_MOTOR_INIT;
   }
   GPT12E_T2_Start();
   Emo_lInitFocPar();
   Emo_Status.MotorState = EMO_MOTOR_STATE_STOP;
   return EMO_ERROR_NONE;
}

/*
void Emo_SetRefSpeed(sint16 RefSpeed){
   UNUSED(RefSpeed);
}

uint32 Emo_GetSpeed(void){
   uint32 Speed;
   if(Emo_Ctrl.ActSpeed >= 0){
      Speed = (uint16)Emo_Ctrl.ActSpeed;
   }
   else{
      Speed = (uint16)(-Emo_Ctrl.ActSpeed);
   }
   return Speed;
}
*/
uint32 Emo_StartMotor(uint32 EnableBridge){
   UNUSED(EnableBridge);
   if(
         EMO_MOTOR_STATE_STOP
      != Emo_Status.MotorState
   ){
      return EMO_ERROR_MOTOR_NOT_STOPPED;
   }
   CCU6_StartTmr_T12();
   CCU6_Multi_Ch_PWM_Shadow_Reg_Load(0x0000);
   CCU6_MCM_PWM_Str_SW_En();
   BDRV_Clr_Sts(0x1F1FFC00);
   BDRV_Set_Bridge(Ch_PWM, Ch_PWM, Ch_PWM, Ch_PWM, Ch_PWM, Ch_PWM);
   Emo_lInitFocVar();
   Emo_Status.MotorState = EMO_MOTOR_STATE_START;
   return EMO_ERROR_NONE;
}

uint32 Emo_StopMotor(void){
   if(
               (EMO_MOTOR_STATE_RUN   != Emo_Status.MotorState)
      && (
               (EMO_MOTOR_STATE_START != Emo_Status.MotorState)
            && (EMO_MOTOR_STATE_FAULT != Emo_Status.MotorState)
         )
   ){
      return EMO_ERROR_MOTOR_NOT_STARTED;
   }
   BDRV_Set_Bridge(Ch_Off, Ch_Off, Ch_Off, Ch_Off, Ch_Off, Ch_Off);
   CCU6_StopTmr_T12();
   Emo_Status.MotorState = EMO_MOTOR_STATE_STOP;
   return EMO_ERROR_NONE;
}

void Emo_lInitFocVar(void){
   uint16 i;
   Emo_Svm.Angle = 0u;
   Emo_Svm.Amp = 0u;
   Emo_Svm.Sector = 0u;
   Emo_Svm.T1 = 0u;
   Emo_Svm.T2 = 0u;
   Emo_Svm.PhaseCurr.A = 0;
   Emo_Svm.PhaseCurr.B = 0;
   Emo_Svm.comp60up = 0;
   Emo_Svm.comp61up = 0;
   Emo_Svm.comp62up = 0;
   Emo_Svm.comp60up = 0;
   Emo_Svm.comp61down = 0;
   Emo_Svm.comp62down = 0;
   Emo_Foc.StatCurr.Real = 0;
   Emo_Foc.StatCurr.Imag = 0;
   Emo_Foc.RotCurr.Real = 0;
   Emo_Foc.RotCurr.Imag = 0;
   Emo_Foc.StatVolt.Real = 0;
   Emo_Foc.StatVolt.Imag = 0;
   Emo_Foc.RotVolt.Real = 0;
   Emo_Foc.RotVolt.Imag = 0;
   Emo_Foc.Angle = 0u;
   Emo_Foc.FluxAngle = 0u;
   Emo_Foc.StoredAngle = 0u;
   Emo_Foc.RealFluxLp.Out = 0;
   Emo_Foc.ImagFluxLp.Out = 0;
   Emo_Ctrl.ActSpeed = 0;
   Emo_Ctrl.RefCurr = 0;
   Emo_Ctrl.RealCurrPi.IOut = 0;
   Emo_Ctrl.ImagCurrPi.IOut = 0;
   Emo_Ctrl.SpeedLp.Out = 0;
   Emo_Foc.StartSpeedSlopeMem = 0;
   Emo_Foc.CountStart = Emo_Foc.TimeSpeedzero;
   Emo_Foc.StartAngle = (uint16)(30.0 / 360.0 * 65536.0);

   for(
      i = 0;
      i < 31;
      i ++
   ){
      Emo_Ctrl.AngleBuffer[i] = 0;
   }

   Emo_Ctrl.SpeedLpdisplay.Out = 0;
   Emo_Ctrl.SpeedLp.Out = 0;
   Emo_Svm.CounterOffsetAdw = 0;
   Emo_Svm.CsaOffsetAdwSumme = 0;
}

void Emo_HandleT2Overflow(void){
   if(
         EMO_MOTOR_STATE_START
      == Emo_Status.MotorState
   ){
      if(
            0
         == Emo_Foc.CountStart
      ){
         if(
               0
            <  Emo_Ctrl.RefSpeed
         ){
            Emo_Foc.StartSpeedSlope = Mat_Ramp(
                  Emo_Foc.StartEndSpeed
               ,  Emo_Foc.StartSpeedSlewRate
               , &Emo_Foc.StartSpeedSlopeMem
            );

            Emo_Foc.StartFrequencySlope = __SSAT(
                  Mat_FixMulScale(
                        Emo_Foc.StartSpeedSlope
                     ,  Emo_Foc.SpeedtoFrequency
                     ,  0
                  )
               ,  MAT_FIX_SAT
            );

            if(
                  Emo_Foc.StartSpeedSlope
               == Emo_Foc.StartEndSpeed
            ){
               Emo_Status.MotorState = EMO_MOTOR_STATE_RUN;
               Emo_Ctrl.SpeedPi.IOut = Emo_Ctrl.RefCurr << 14;
            }
         }
         else{
            Emo_Foc.StartSpeedSlope = Mat_Ramp(
                 -Emo_Foc.StartEndSpeed
               ,  Emo_Foc.StartSpeedSlewRate
               , &Emo_Foc.StartSpeedSlopeMem
            );

            Emo_Foc.StartFrequencySlope = __SSAT(
                  Mat_FixMulScale(
                        Emo_Foc.StartSpeedSlope
                     ,  Emo_Foc.SpeedtoFrequency
                     ,  0
                  )
               ,  MAT_FIX_SAT
            );

            if(
                   Emo_Foc.StartSpeedSlope
               == -Emo_Foc.StartEndSpeed
            ){
               Emo_Status.MotorState = EMO_MOTOR_STATE_RUN;
               Emo_Ctrl.SpeedPi.IOut = Emo_Ctrl.RefCurr << 14;
            }
         }
      }
      else{
         if(
               1
            == Emo_Focpar_Cfg.EnableFrZero
         ){
            Emo_Foc.CountStart--;
         }
         else{
            Emo_Foc.CountStart = 0;
         }
         Emo_Foc.StartFrequencySlope = 0;
      }

      Emo_Ctrl.ActSpeeddisplay = Mat_ExeLp_without_min_max(
           &Emo_Ctrl.SpeedLpdisplay
         ,  Emo_Ctrl.ActSpeed
      );

      Emo_Ctrl.EnableStartVoltage = 1;
   }
   else if(
         EMO_MOTOR_STATE_RUN
      == Emo_Status.MotorState
   ){
      if(
            Emo_Ctrl.ActSpeed
         >  Emo_Ctrl.Speedlevelmaxstart
      ){
         Emo_Ctrl.SpeedPi.PiMax = Emo_Ctrl.MaxRefCurrent;
         Emo_Ctrl.SpeedPi.IMax  = Emo_Ctrl.MaxRefCurrent;
      }
      else{
         if(
               Emo_Ctrl.ActSpeed
            <  Emo_Ctrl.Speedlevelminstart
         ){
            Emo_Ctrl.SpeedPi.PiMin = Emo_Ctrl.MinRefCurrent;
            Emo_Ctrl.SpeedPi.IMin  = Emo_Ctrl.MinRefCurrent;
         }
         else{
            Emo_Ctrl.SpeedPi.PiMax = Emo_Ctrl.MaxRefStartCurrent;
            Emo_Ctrl.SpeedPi.IMax  = Emo_Ctrl.MaxRefStartCurrent;
            Emo_Ctrl.SpeedPi.PiMin = Emo_Ctrl.MinRefStartCurrent;
            Emo_Ctrl.SpeedPi.IMin  = Emo_Ctrl.MinRefStartCurrent;
         }
      }
      Emo_Ctrl.RefCurr = Mat_ExePi(
           &Emo_Ctrl.SpeedPi
         ,  Emo_Ctrl.RefSpeed - Emo_Ctrl.ActSpeed
      );

      Emo_Ctrl.ActSpeeddisplay = Mat_ExeLp_without_min_max(
           &Emo_Ctrl.SpeedLpdisplay
         ,  Emo_Ctrl.ActSpeed
      );
   }
   else{
      Emo_StopMotor();
   }

   if(
         EMO_MOTOR_STATE_STOP
      == Emo_Status.MotorState
   ){
      Emo_Ctrl.ActSpeed = Mat_ExeLp_without_min_max(
           &Emo_Ctrl.SpeedLp
         ,  0
      );

      Emo_Ctrl.ActSpeeddisplay = Mat_ExeLp_without_min_max(
           &Emo_Ctrl.SpeedLpdisplay
         ,  Emo_Ctrl.ActSpeed
      );
   }

   Emo_Foc.DcLinkVoltage     = ADC1_GetRES_OUT6();
   Emo_Foc.Dcfactor1         =  Emo_Foc.Kdcdivident1 / Emo_Foc.DcLinkVoltage;
   Emo_Foc.Dcfactor2         = __SSAT(Mat_FixMulScale(Emo_Foc.DcLinkVoltage, Emo_Foc.Kdcfactor2,   3), MAT_FIX_SAT);
   Emo_Ctrl.ImagCurrPi.IMax  = __SSAT(Mat_FixMulScale(Emo_Foc.DcLinkVoltage, Emo_Foc.Kdcfactoriqc, 5), MAT_FIX_SAT);
   Emo_Ctrl.ImagCurrPi.PiMax =  Emo_Ctrl.ImagCurrPi.IMax;
   Emo_Ctrl.ImagCurrPi.IMin  = -Emo_Ctrl.ImagCurrPi.IMax;
   Emo_Ctrl.ImagCurrPi.PiMin =  Emo_Ctrl.ImagCurrPi.IMin;

   if(
         1
      == Emo_Ctrl.EnableStartVoltage
   ){
      Emo_Foc.StartVoltAmpDivUz = __SSAT(
            Mat_FixMulScale(
                  Emo_Foc.StartVoltAmp
               ,  Emo_Foc.Dcfactor1
               ,  1
            )
         ,  MAT_FIX_SAT
      );
   }

   Emo_Ctrl.SpeedPi.IMin = Emo_Ctrl.SpeedPi.PiMin;
   Emo_Ctrl.SpeedPi.IMax = Emo_Ctrl.SpeedPi.PiMax;

   if(
         Emo_Svm.Amp
      <  Emo_Svm.MaxAmp9091pr
   ){
      Emo_Foc.RealFluxLp.CoefB = Emo_Foc.LpCoefb1;
      Emo_Foc.ImagFluxLp.CoefB = Emo_Foc.LpCoefb1;
   }
   else{
      Emo_Foc.RealFluxLp.CoefB = Emo_Foc.LpCoefb2;
      Emo_Foc.ImagFluxLp.CoefB = Emo_Foc.LpCoefb2;
   }
}

/*
uint32 Emo_GetMotorState(void){
   return(uint32)Emo_Status.MotorState;
}
*/

/*
void Emo_HandleAdc1(void){
  CCU6_SetT13Compare(Emo_Svm.CompT13ValueDown);
  CCU6_SetT13Trigger(0x76);
  Emo_AdcResult[0u] = ADC1.RES_OUT1.reg;
  ADC1.IE.bit.ESM_IE = 0;
}

void Emo_CurrAdc1(void){
  sint16 AdcResult0;
  sint16 AdcResult1;
  uint16 sector;
  sint16 R0mioffs;
  sint16 OffsmiR1;
  sint16 R1miR0;
  AdcResult0 = Emo_AdcResult[2u] & 0x0FFFu;
  AdcResult1 = Emo_AdcResult[1u] & 0x0FFFu;
  Emo_AdcResult[3u] = AdcResult0 + AdcResult1;
  R0mioffs = AdcResult0 - Emo_Svm.CsaOffsetAdw;
  OffsmiR1 = Emo_Svm.CsaOffsetAdw - AdcResult1;
  R1miR0 = AdcResult1 - AdcResult0;
  sector = Emo_Svm.StoredSector1;
  switch (sector){
    case 0:{
      Emo_Svm.PhaseCurr.A = R0mioffs;
      Emo_Svm.PhaseCurr.B = R1miR0;
    }
    break;

    case 1:{
      Emo_Svm.PhaseCurr.A = R1miR0;
      Emo_Svm.PhaseCurr.B = R0mioffs;
    }
    break;

    case 2:{
      Emo_Svm.PhaseCurr.A = OffsmiR1;
      Emo_Svm.PhaseCurr.B = R0mioffs;
    }
    break;

    case 3:{
      Emo_Svm.PhaseCurr.A = OffsmiR1;
      Emo_Svm.PhaseCurr.B = R1miR0;
    }
    break;

    case 4:{
      Emo_Svm.PhaseCurr.A = R1miR0;
      Emo_Svm.PhaseCurr.B = OffsmiR1;
    }
    break;

    case 5:{
      Emo_Svm.PhaseCurr.A = R0mioffs;
      Emo_Svm.PhaseCurr.B = OffsmiR1;
    }
    break;

    default:{
      Emo_StopMotor();
    }
    break;
  }
  Emo_Svm.StoredSector1 = Emo_Svm.Sector;
}

void Emo_HandleCCU6ShadowTrans(void){
  CCU6_LoadShadowRegister_CC60(Emo_Svm.comp60up);
  CCU6_LoadShadowRegister_CC61(Emo_Svm.comp61up);
  CCU6_LoadShadowRegister_CC62(Emo_Svm.comp62up);
  CCU6_EnableST_T12();
  CCU6_SetT13Trigger(0x7a);
  CCU6_SetT13Compare(Emo_Svm.CompT13ValueUp);
  CCU6.IEN.bit.ENT12PM = 0;
}

void Emo_HandleFoc(void){
  uint16 angle;
  uint16 ampl;
  uint16 i;
  TComplex Vect1 = {0, 0};
  TComplex Vect2;
  sint16 Speed;
  sint32 jj;
  Emo_AdcResult[2u] = Emo_AdcResult[0u];
  ADC1.ICLR.bit.ESM_ICLR = 1;
  ADC1.IE.bit.ESM_IE = 1;
  Emo_AdcResult[1u] = ADC1.RES_OUT1.reg;
  CCU6_LoadShadowRegister_CC60(Emo_Svm.comp60down);
  CCU6_LoadShadowRegister_CC61(Emo_Svm.comp61down);
  CCU6_LoadShadowRegister_CC62(Emo_Svm.comp62down);
  CCU6_EnableST_T12();
  Emo_CurrAdc1();
  Emo_Foc.StatCurr = Mat_Clarke(Emo_Svm.PhaseCurr);
  Emo_Foc.RotCurr = Mat_Park(Emo_Foc.StatCurr, Emo_Foc.Angle);
  Emo_lEstFlux();
  if(Emo_Status.MotorState == EMO_MOTOR_STATE_START){
    Emo_Foc.StartAngle += Emo_Foc.StartFrequencySlope;
    Emo_Foc.Angle = Emo_Foc.StartAngle;
    Emo_Ctrl.PtrAngle = (Emo_Ctrl.PtrAngle + 1) & 0x1f;
    if(Emo_Ctrl.Anglersptr == 32){
      angle = Emo_Ctrl.AngleBuffer[Emo_Ctrl.PtrAngle];
      Emo_Ctrl.AngleBuffer[Emo_Ctrl.PtrAngle] = Emo_Foc.FluxAngle;
      Emo_Ctrl.Speedest = Emo_Foc.FluxAngle - angle;
    }
    else{
      Emo_Ctrl.AngleBuffer[Emo_Ctrl.PtrAngle] = Emo_Foc.FluxAngle;
      Emo_Ctrl.Speedest = Emo_Foc.FluxAngle - Emo_Ctrl.AngleBuffer[(Emo_Ctrl.PtrAngle - Emo_Ctrl.Anglersptr) & 0x1f];
    }
    Emo_Ctrl.Speedpll = Emo_Ctrl.Speedest >> Emo_Ctrl.Exppllhigh;
    jj = Mat_FixMulScale(Emo_Ctrl.Speedest, Emo_Ctrl.Factorspeed, Emo_Ctrl.Expspeedhigh);
    Speed = jj / Emo_Foc.PolePair;
    Emo_Ctrl.ActSpeed = Mat_ExeLp_without_min_max(&Emo_Ctrl.SpeedLp, Speed);
    Emo_FluxAnglePll();
    if(Emo_Ctrl.RefSpeed > 0){
      Emo_Ctrl.RefCurr = Emo_Foc.StartCurrent;
    }
    else{
      Emo_Ctrl.RefCurr = -Emo_Foc.StartCurrent;
    }
    Emo_Foc.RotVolt.Real = Mat_ExePi(&Emo_Ctrl.RealCurrPi, Emo_Ctrl.RefCurr - Emo_Foc.RotCurr.Real);
    Emo_Foc.RotVolt.Imag = Mat_ExePi(&Emo_Ctrl.ImagCurrPi, 0 - Emo_Foc.RotCurr.Imag);
  }
  else{
    Emo_Ctrl.PtrAngle = (Emo_Ctrl.PtrAngle + 1) & 0x1f;
    if(Emo_Ctrl.Anglersptr == 32){
      angle = Emo_Ctrl.AngleBuffer[Emo_Ctrl.PtrAngle];
      Emo_Ctrl.AngleBuffer[Emo_Ctrl.PtrAngle] = Emo_Foc.FluxAngle;
      Emo_Ctrl.Speedest = Emo_Foc.FluxAngle - angle;
    }
    else{
      Emo_Ctrl.AngleBuffer[Emo_Ctrl.PtrAngle] = Emo_Foc.FluxAngle;
      Emo_Ctrl.Speedest = Emo_Foc.FluxAngle - Emo_Ctrl.AngleBuffer[(Emo_Ctrl.PtrAngle - Emo_Ctrl.Anglersptr) & 0x1f];
    }
    Emo_Ctrl.Speedpll = Emo_Ctrl.Speedest >> Emo_Ctrl.Exppllhigh;
    jj = Mat_FixMulScale(Emo_Ctrl.Speedest, Emo_Ctrl.Factorspeed, Emo_Ctrl.Expspeedhigh);
    Speed = jj / Emo_Foc.PolePair;
    Emo_Ctrl.ActSpeed = Mat_ExeLp_without_min_max(&Emo_Ctrl.SpeedLp, Speed);
    Emo_FluxAnglePll();
    Emo_Foc.Angle = Emo_Ctrl.FluxAnglePll;
#if(EMO_DECOUPLING==0)
    Emo_Foc.RotVolt.Real = Mat_ExePi(&Emo_Ctrl.RealCurrPi, 0 - Emo_Foc.RotCurr.Real);
    Emo_Foc.RotVolt.Imag = Mat_ExePi(&Emo_Ctrl.ImagCurrPi, Emo_Ctrl.RefCurr - Emo_Foc.RotCurr.Imag);
#else
    Emo_Foc.RotVoltCurrentcontrol.Real = Mat_ExePi(&Emo_Ctrl.RealCurrPi, 0 - Emo_Foc.RotCurr.Real);
    Emo_Foc.RotVoltCurrentcontrol.Imag = Mat_ExePi(&Emo_Ctrl.ImagCurrPi, Emo_Ctrl.RefCurr - Emo_Foc.RotCurr.Imag);
    Emo_Foc.RotVolt = Emo_CurrentDecoupling();
#endif
  }
  Vect1.Real = __SSAT(Mat_FixMulScale(Emo_Foc.RotVolt.Real, Emo_Foc.Dcfactor1, 1), MAT_FIX_SAT);
  Vect1.Imag = __SSAT(Mat_FixMulScale(Emo_Foc.RotVolt.Imag, Emo_Foc.Dcfactor1, 1), MAT_FIX_SAT);
  Vect2 = Limitsvektor(&Vect1, &Emo_Svm);
  angle = Mat_CalcAngleAmp(Vect2, &ampl);
  if(ampl > Emo_Svm.MaxAmp){
    ampl = Emo_Svm.MaxAmp;
  }
  if(Emo_Svm.CounterOffsetAdw > 127){
    Emo_Svm.Amp = ampl;
  }
  else{
    Emo_Svm.Amp    = 0;
    Emo_Svm.CsaOffsetAdwSumme = Emo_Svm.CsaOffsetAdwSumme + Emo_AdcResult[3u];
    Emo_Svm.CounterOffsetAdw++;
    if(Emo_Svm.CounterOffsetAdw == 128){
      i = Emo_Svm.CsaOffsetAdwSumme >> 8;
      if(i < Emo_Svm.CsaOffset){
        i = Emo_Svm.CsaOffset;
      }
      else{
        if(i > (Emo_Svm.CsaOffset + 100)){
          i = Emo_Svm.CsaOffset + 100;
        }
      }
      Emo_Svm.CsaOffsetAdw = i;
    }
  }
  Emo_Svm.Angle = angle + Emo_Foc.Angle;
  Emo_Foc.StatVoltAmpM = __SSAT(Mat_FixMulScale(Emo_Svm.Amp, Emo_Foc.Dcfactor2, 3), MAT_FIX_SAT);
  Emo_Foc.StatVolt = Mat_PolarKartesisch(Emo_Foc.StatVoltAmpM, Emo_Svm.Angle);
  Emo_lExeSvm(&Emo_Svm);
  Emo_Ctrl.RotCurrImagdisplay = Mat_ExeLp_without_min_max(&Emo_Ctrl.RotCurrImagLpdisplay, Emo_Foc.RotCurr.Imag);
}

void Emo_lExeSvm(TEmo_Svm *pSvm){
  sint32 T1;
  sint32 T2;
  uint32 Sector;
  uint32 Angle;
  uint32 Index;
  uint32 Compare0up;
  uint32 Compare1up;
  uint32 Compare2up;
  uint32 Compare0down;
  uint32 Compare1down;
  uint32 Compare2down;
  uint16 T13ValueUp;
  uint16 T13ValueDown;
  uint16 i;
  uint16 per;
  sint32 ci;
  Angle = ((uint32)pSvm->Angle) * 6u;
  Sector = (Angle >> 16u) & 7;
  pSvm->Sector = (uint16)Sector;
  Index = (Angle >> 8u) & 0xFFu;
  T1 = (((uint32)pSvm->Amp) * Table_Sin60[255u - Index]) >> (MAT_FIX_SHIFT + 1);
  pSvm->T1 = (sint16)T1;
  T2 = (((uint32)pSvm->Amp) * Table_Sin60[Index]) >> (MAT_FIX_SHIFT + 1);
  pSvm->T2 = (sint16)T2;
  per = CCU6_T12PR;
  switch (Sector){
    case 0u:{
      ci = ((sint32)CCU6_T12PR / 2u) - T1 - T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      Compare0up = ci;
      ci = ((sint32)CCU6_T12PR / 2u) + T1 - T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare1up = ci;
      ci = (CCU6_T12PR / 2u) + T1 + T2;
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare2up = ci;
      Compare0down = Compare0up;
      Compare1down = Compare1up;
      Compare2down = Compare2up;
#if(EMO_IMESS==1)
      if(T1 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T1 - T1;
        Compare1up = Compare1up + i;
        if(Compare1up > per){
          Compare1up = per;
        }
        if(i < Compare1down){
          Compare1down = Compare1down - i;
        }
        else{
          Compare1down = 1;
        }
      }
      if(T2 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T2 - T2;
        Compare1up = Compare1up + i;
        if(Compare1up > per){
          Compare1up = per;
        }
        if(i < Compare1down){
          Compare1down = Compare1down - i;
        }
        else{
          Compare1down = 1;
        }
      }
#endif
      T13ValueUp = (Compare1up + Compare0up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare1down + Compare2down) / 2;
      break;
    }
    case 1u:{
      ci = ((sint32)CCU6_T12PR / 2u) - T1 - T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      Compare1up = ci;
      ci = ((sint32)CCU6_T12PR / 2u) - T1 + T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare0up = ci;
      ci = (CCU6_T12PR / 2u) + T1 + T2;
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare2up = ci;
      Compare0down = Compare0up;
      Compare1down = Compare1up;
      Compare2down = Compare2up;
#if(EMO_IMESS==1)
      i = T1;
      T1 = T2;
      T2 = i;
      if(T1 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T1 - T1;
        Compare0up = Compare0up + i;
        if(Compare0up > per){
          Compare0up = per;
        }
        if(i < Compare0down){
          Compare0down = Compare0down - i;
        }
        else{
          Compare0down = 1;
        }
      }
      if(T2 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T2 - T2;
        Compare0up = Compare0up + i;
        if(Compare0up > per){
          Compare0up = per;
        }
        if(i < Compare0down){
          Compare0down = Compare0down - i;
        }
        else{
          Compare0down = 1;
        }
      }
#endif
      T13ValueUp = (Compare0up + Compare1up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare0down + Compare2down) / 2;
      break;
    }
    case 2u:{
      ci = ((sint32)CCU6_T12PR / 2u) - T1 - T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      Compare1up = ci;
      ci = ((sint32)CCU6_T12PR / 2u) + T1 - T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare2up = ci;
      ci = (CCU6_T12PR / 2u) + T1 + T2;
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare0up = ci;
      Compare0down = Compare0up;
      Compare1down = Compare1up;
      Compare2down = Compare2up;
#if(EMO_IMESS==1)
      if(T1 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T1 - T1;
        Compare2up = Compare2up + i;
        if(Compare2up > per){
          Compare2up = per;
        }
        if(i < Compare2down){
          Compare2down = Compare2down - i;
        }
        else{
          Compare2down = 1;
        }
      }
      if(T2 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T2 - T2;
        Compare2up = Compare2up + i;
        if(Compare2up > per){
          Compare2up = per;
        }
        if(i < Compare2down){
          Compare2down = Compare2down - i;
        }
        else{
          Compare2down = 1;
        }
      }
#endif
      T13ValueUp = (Compare2up + Compare1up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare2down + Compare0down) / 2;
      break;
    }
    case 3u:{
      ci = ((sint32)CCU6_T12PR / 2u) - T1 - T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      Compare2up = ci;
      ci = ((sint32)CCU6_T12PR / 2u) - T1 + T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare1up = ci;
      ci = (CCU6_T12PR / 2u) + T1 + T2;
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare0up = ci;
      Compare0down = Compare0up;
      Compare1down = Compare1up;
      Compare2down = Compare2up;
#if(EMO_IMESS==1)
      i = T1;
      T1 = T2;
      T2 = i;
      if(T1 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T1 - T1;
        Compare1up = Compare1up + i;
        if(Compare1up > per){
          Compare1up = per;
        }
        if(i < Compare1down){
          Compare1down = Compare1down - i;
        }
        else{
          Compare1down = 1;
        }
      }
      if(T2 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T2 - T2;
        Compare1up = Compare1up + i;
        if(Compare1up > per){
          Compare1up = per;
        }
        if(i < Compare1down){
          Compare1down = Compare1down - i;
        }
        else{
          Compare1down = 1;
        }
      }
#endif
      T13ValueUp = (Compare1up + Compare2up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare1down + Compare0down) / 2;
      break;
    }
    case 4u:{
      ci = ((sint32)CCU6_T12PR / 2u) - T1 - T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      Compare2up = ci;
      ci = ((sint32)CCU6_T12PR / 2u) + T1 - T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare0up = ci;
      ci = (CCU6_T12PR / 2u) + T1 + T2;
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare1up = ci;
      Compare0down = Compare0up;
      Compare1down = Compare1up;
      Compare2down = Compare2up;
#if(EMO_IMESS==1)
      if(T1 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T1 - T1;
        Compare0up = Compare0up + i;
        if(Compare0up > per){
          Compare0up = per;
        }
        if(i < Compare0down){
          Compare0down = Compare0down - i;
        }
        else{
          Compare0down = 1;
        }
      }
      if(T2 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T2 - T2;
        Compare0up = Compare0up + i;
        if(Compare0up > per){
          Compare0up = per;
        }
        if(i < Compare0down){
          Compare0down = Compare0down - i;
        }
        else{
          Compare0down = 1;
        }
      }
#endif
      T13ValueUp = (Compare0up + Compare2up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare0down + Compare1down) / 2;
      break;
    }
    default:{
      ci = ((sint32)CCU6_T12PR / 2u) - T1 - T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      Compare0up = ci;
      ci = ((sint32)CCU6_T12PR / 2u) - T1 + T2;
      if(ci < EMO_SVM_DEADTIME){
        ci = 0;
      };
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare2up = ci;
      ci = (CCU6_T12PR / 2u) + T1 + T2;
      if(ci > (CCU6_T12PR - EMO_SVM_DEADTIME)){
        ci = CCU6_T12PR + 1;
      };
      Compare1up = ci;
      Compare0down = Compare0up;
      Compare1down = Compare1up;
      Compare2down = Compare2up;
#if(EMO_IMESS==1)
      i = T1;
      T1 = T2;
      T2 = i;
      if(T1 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T1 - T1;
        Compare2up = Compare2up + i;
        if(Compare2up > per){
          Compare2up = per;
        }
        if(i < Compare2down){
          Compare2down = Compare2down - i;
        }
        else{
          Compare2down = 1;
        }
      }
      if(T2 < (EMO_SVM_MINTIME / 2)){
        i = EMO_SVM_MINTIME - T2 - T2;
        Compare2up = Compare2up + i;
        if(Compare2up > per){
          Compare2up = per;
        }
        if(i < Compare2down){
          Compare2down = Compare2down - i;
        }
        else{
          Compare2down = 1;
        }
      }
#endif
      T13ValueUp = (Compare2up + Compare0up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare2down + Compare1down) / 2;
      break;
    }
  }
  if(Emo_Svm.CounterOffsetAdw > 127){
    pSvm->CompT13ValueUp = T13ValueUp;
    pSvm->CompT13ValueDown = T13ValueDown;
  }
  else{
    pSvm->CompT13ValueUp = CCU6_T12PR / 3 ;
    pSvm->CompT13ValueDown = CCU6_T12PR * 2 / 3;
  }
  pSvm->comp60up = Compare0up;
  pSvm->comp61up = Compare1up;
  pSvm->comp62up = Compare2up;
  pSvm->comp60down = Compare0down;
  pSvm->comp61down = Compare1down;
  pSvm->comp62down = Compare2down;
  if(CCU6.TCTR0.bit.CDIR == 0){
    CCU6.IEN.bit.ENT12PM = 1;
  }
  else{
    CCU6_LoadShadowRegister_CC60(Emo_Svm.comp60up);
    CCU6_LoadShadowRegister_CC61(Emo_Svm.comp61up);
    CCU6_LoadShadowRegister_CC62(Emo_Svm.comp62up);
    CCU6_EnableST_T12();
    CCU6_SetT13Trigger(0x7a);
    CCU6_SetT13Compare(Emo_Svm.CompT13ValueUp);
  }
}
void Emo_lEstFlux(void){
  sint16 Temp;
  uint16 Tempu;
  uint16 FluxAbsValue;
  TComplex fluxh = {0, 0};
  static TComplex Fluxrf;
  static TComplex Flux;
  Temp = __SSAT(Fluxrf.Real + Emo_Foc.StatVolt.Real - Mat_FixMul(Emo_Foc.StatCurr.Real, Emo_Foc.PhaseRes), MAT_FIX_SAT);
  fluxh.Real = Mat_ExeLp_without_min_max(&Emo_Foc.RealFluxLp, Temp);
  Flux.Real = __SSAT(fluxh.Real - Mat_FixMulScale(Emo_Foc.StatCurr.Real, Emo_Foc.PhaseInd, 0), MAT_FIX_SAT);
  Temp = __SSAT(Fluxrf.Imag + Emo_Foc.StatVolt.Imag - Mat_FixMul(Emo_Foc.StatCurr.Imag, Emo_Foc.PhaseRes), MAT_FIX_SAT);
  fluxh.Imag = Mat_ExeLp_without_min_max(&Emo_Foc.ImagFluxLp, Temp);
  Flux.Imag = __SSAT(fluxh.Imag - Mat_FixMulScale(Emo_Foc.StatCurr.Imag, Emo_Foc.PhaseInd, 0), MAT_FIX_SAT);
  Emo_Foc.FluxAngle =  Mat_CalcAngleAmp(Flux, &Tempu);
  FluxAbsValue = __SSAT(Mat_FixMul(Tempu, 32000), MAT_FIX_SAT + 1);
  Temp = Mat_ExeLp_without_min_max(&Emo_Ctrl.FluxbtrLp, FluxAbsValue);
  if(Flux.Real > Temp){
    Fluxrf.Real = -400;
  }
  else{
    if(Flux.Real < -Temp){
      Fluxrf.Real = 400;
    }
    else{
      Fluxrf.Real = 0;
    }
  }
  if  (Flux.Imag > Temp){
    Fluxrf.Imag = -400;
  }
  else{
    if(Flux.Imag < -Temp){
      Fluxrf.Imag = 400;
    }
    else{
      Fluxrf.Imag = 0;
    }
  }
}

void Emo_FluxAnglePll(void){
  sint16 deltaphi;
  sint16 domega;
  deltaphi = Emo_Foc.FluxAngle - Emo_Ctrl.FluxAnglePll;
  domega = __SSAT(Mat_FixMulScale(deltaphi, Emo_Ctrl.Pllkp, 0) + Emo_Ctrl.Speedpll, MAT_FIX_SAT);
  Emo_Ctrl.FluxAnglePll = Emo_Ctrl.FluxAnglePll + domega;
}

#if(EMO_DECOUPLING==1)
TComplex Emo_CurrentDecoupling(void){
  TComplex StatOut = {0, 0};
  sint16 Kentk = 10547;
  sint16 Oml;
  Oml = Mat_FixMulScale(Kentk, Emo_Ctrl.SpeedLpdisplay.Out, 1);
  StatOut.Real = __SSAT(Emo_Foc.RotVoltCurrentcontrol.Real - Mat_FixMulScale(Emo_Foc.RotCurr.Imag, Oml, 1), MAT_FIX_SAT);
  StatOut.Imag = __SSAT(Emo_Foc.RotVoltCurrentcontrol.Imag + Mat_FixMulScale(Emo_Foc.RotCurr.Real, Oml, 1), MAT_FIX_SAT);
   return StatOut;
}
#endif

TComplex Limitsvektor(
      TComplex* inp
   ,  TEmo_Svm* par
){
   uint32 btrqu;
   TComplex outp = {0, 0};
   uint16 inpa;
   btrqu = inp->Real * inp->Real + inp->Imag * inp->Imag;
   if(btrqu > par->MaxAmpQuadrat){
      if((abs(inp->Real)) > par->MaxAmp9091pr){
         if(inp->Real < 0){
            outp.Real = -par->MaxAmp9091pr;
         }
         else{
            outp.Real = par->MaxAmp9091pr;
         }
         if(inp->Imag < 0){
            outp.Imag = -par->MaxAmp4164pr;
         }
         else{
            outp.Imag = par->MaxAmp4164pr;
         }
      }
      else{
         inpa = abs(inp->Real);
         inpa = (inpa * par->Kfact256) >> MAT_FIX_SHIFT;
         if(inpa > 256){
            inpa = 256;
         }
         inpa = Table_sqrtmqu[inpa];
         inpa = (inpa * par->MaxAmp) >> 8;
         outp.Real = inp->Real;
         if(inp->Imag < 0){
            outp.Imag = -inpa;
         }
         else{
            outp.Imag = inpa;
         }
      }
   }
   else{
      outp.Real = inp->Real;
      outp.Imag = inp->Imag;
   }
   return outp;
}

TComplex Limitsvektorphase(
      TComplex* inp
   ,  TEmo_Svm* par
){
   uint32 btrqu;
   uint32 btrfactor;
   TComplex outp = {0, 0};
   btrqu = inp->Real * inp->Real + inp->Imag * inp->Imag;
   if(btrqu > par->MaxAmpQuadrat){
      btrfactor = par->MaxAmpQuadrat / (btrqu >> 15);
      btrfactor = 32500 - ((32767 - btrfactor) >> 1);
      outp.Real = (sint16)(__SSAT(Mat_FixMulScale(inp->Real, btrfactor, 0), MAT_FIX_SAT));
      outp.Imag = (sint16)(__SSAT(Mat_FixMulScale(inp->Imag, btrfactor, 0), MAT_FIX_SAT));
   }
   else{
      outp.Real = inp->Real;
      outp.Imag = inp->Imag;
   }
   return outp;
}
*/

void Emo_setspeedreferenz(uint16 speedreferenz){
  Emo_Ctrl.RefSpeed = speedreferenz;
}

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

