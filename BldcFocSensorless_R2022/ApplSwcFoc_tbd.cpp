/******************************************************************************/
/* File   : ApplSwcFoc_tbd.cpp                                                */
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
#include "ApplSwcFoc_tbd.hpp"

#include "bdrv.hpp"
#include "adc1.hpp"

#include "ccu6.hpp"
#include "ccu6_defines.hpp"

#include "csa.hpp"
#include "gpt12e.hpp"

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

void Emo_lInitFocVar(void){
   uint16 i;

   Emo_Svm.Angle              = 0u;
   Emo_Svm.Amp                = 0u;
   Emo_Svm.Sector             = 0u;
   Emo_Svm.T1                 = 0u;
   Emo_Svm.T2                 = 0u;
   Emo_Svm.comp60up           = 0;
   Emo_Svm.comp61up           = 0;
   Emo_Svm.comp62up           = 0;
   Emo_Svm.comp60up           = 0;
   Emo_Svm.comp61down         = 0;
   Emo_Svm.comp62down         = 0;
   Emo_Foc.StatCurr.Real      = 0;
   Emo_Foc.StatCurr.Imag      = 0;
   Emo_Foc.RotCurr.Real       = 0;
   Emo_Foc.RotCurr.Imag       = 0;
   Emo_Foc.StatVolt.Real      = 0;
   Emo_Foc.StatVolt.Imag      = 0;
   Emo_Foc.RotVolt.Real       = 0;
   Emo_Foc.RotVolt.Imag       = 0;
   Emo_Foc.Angle              = 0u;
   Emo_Foc.FluxAngle          = 0u;
   Emo_Foc.StoredAngle        = 0u;
   Emo_Foc.RealFluxLp.Out     = 0;
   Emo_Foc.ImagFluxLp.Out     = 0;
   Emo_Ctrl.ActSpeed          = 0;
   Emo_Ctrl.RefCurr           = 0;
   Emo_Ctrl.RealCurrPi.IOut   = 0;
   Emo_Ctrl.ImagCurrPi.IOut   = 0;
   Emo_Ctrl.SpeedLp.Out       = 0;
   Emo_Foc.StartSpeedSlopeMem = 0;
   Emo_Foc.CountStart         = Emo_Foc.TimeSpeedzero;
   Emo_Foc.StartAngle         = (uint16)(30.0 / 360.0 * 65536.0);

   for(
      i = 0;
      i < 31;
      i ++
   ){
      Emo_Ctrl.AngleBuffer[i] = 0;
   }

   Emo_Ctrl.SpeedLpdisplay.Out = 0;
   Emo_Ctrl.SpeedLp.Out        = 0;
   Emo_Svm.CounterOffsetAdw    = 0;
   Emo_Svm.CsaOffsetAdwSumme   = 0;
}

extern void RteRead_SpeedReference (uint16* lptrOutput); //TBD: Move to header
void Emo_HandleT2Overflow_StateStart(void){
   if(
         0
      == Emo_Foc.CountStart
   ){
      uint16 lu16SpeedReference;
      RteRead_SpeedReference(&lu16SpeedReference);
      if(0 < lu16SpeedReference){Emo_Foc.StartSpeedSlope = Mat_Ramp( Emo_Foc.StartEndSpeed, Emo_Foc.StartSpeedSlewRate, &Emo_Foc.StartSpeedSlopeMem); Emo_Foc.StartFrequencySlope = __SSAT(Mat_FixMulScale(Emo_Foc.StartSpeedSlope, Emo_Foc.SpeedtoFrequency, 0), MAT_FIX_SAT); if(Emo_Foc.StartSpeedSlope ==  Emo_Foc.StartEndSpeed){Emo_Status.MotorState = EMO_MOTOR_STATE_RUN; Emo_Ctrl.SpeedPi.IOut = Emo_Ctrl.RefCurr << 14;}}
      else                      {Emo_Foc.StartSpeedSlope = Mat_Ramp(-Emo_Foc.StartEndSpeed, Emo_Foc.StartSpeedSlewRate, &Emo_Foc.StartSpeedSlopeMem); Emo_Foc.StartFrequencySlope = __SSAT(Mat_FixMulScale(Emo_Foc.StartSpeedSlope, Emo_Foc.SpeedtoFrequency, 0), MAT_FIX_SAT); if(Emo_Foc.StartSpeedSlope == -Emo_Foc.StartEndSpeed){Emo_Status.MotorState = EMO_MOTOR_STATE_RUN; Emo_Ctrl.SpeedPi.IOut = Emo_Ctrl.RefCurr << 14;}}
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

   Emo_Ctrl.EnableStartVoltage = 1;
}

extern void RteRead_SpeedReference (uint16* lptrOutput); //TBD: Move to header
void Emo_HandleT2Overflow_StateRun(void){
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

   uint16 lu16SpeedReference;
   RteRead_SpeedReference(&lu16SpeedReference);
   Emo_Ctrl.RefCurr = Mat_ExePi(
        &Emo_Ctrl.SpeedPi
      ,  lu16SpeedReference - Emo_Ctrl.ActSpeed
   );
}

void Emo_HandleT2Overflow_StateStop(void){
   Emo_Ctrl.ActSpeed = Mat_ExeLp_without_min_max(
        &Emo_Ctrl.SpeedLp
      ,  0
   );
}

uint32 Emo_StopMotor(void){
   if(
         (EMO_MOTOR_STATE_RUN   != Emo_Status.MotorState)
      && (EMO_MOTOR_STATE_START != Emo_Status.MotorState)
      && (EMO_MOTOR_STATE_FAULT != Emo_Status.MotorState)
   ){
      return EMO_ERROR_MOTOR_NOT_STARTED;
   }
   else{
      BDRV_Set_Bridge(Ch_Off, Ch_Off, Ch_Off, Ch_Off, Ch_Off, Ch_Off);
      CCU6_StopTmr_T12();
      Emo_Status.MotorState = EMO_MOTOR_STATE_STOP;
      return EMO_ERROR_NONE;
   }
}

void Emo_HandleT2Overflow(void){
   switch(Emo_Status.MotorState){
      case EMO_MOTOR_STATE_START :                  Emo_HandleT2Overflow_StateStart(); break;
      case EMO_MOTOR_STATE_RUN   :                  Emo_HandleT2Overflow_StateRun();   break;
      case EMO_MOTOR_STATE_STOP  : Emo_StopMotor(); Emo_HandleT2Overflow_StateStop();  break;
      default                    : Emo_StopMotor();                                    break;
   }

   Emo_Ctrl.ActSpeeddisplay = Mat_ExeLp_without_min_max(
        &Emo_Ctrl.SpeedLpdisplay
      ,  Emo_Ctrl.ActSpeed
   );

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
void Emo_HandleCCU6ShadowTrans(void){
  CCU6_LoadShadowRegister_CC60(Emo_Svm.comp60up);
  CCU6_LoadShadowRegister_CC61(Emo_Svm.comp61up);
  CCU6_LoadShadowRegister_CC62(Emo_Svm.comp62up);
  CCU6_EnableST_T12();
  CCU6_SetT13Trigger(0x7a);
  CCU6_SetT13Compare(Emo_Svm.CompT13ValueUp);
  CCU6.IEN.bit.ENT12PM = 0;
}
*/

static sint16 Emo_lEstFlux_Optimize(
      sint16 Input
   ,  sint16 Limit
){
   sint16 Output;

   if(Input > Limit){
      Output = -400;
   }
   else{
      if(Input < -Limit){
         Output = 400;
      }
      else{
         Output = 0;
      }
   }

   return Output;
}

void Emo_lEstFlux(void){
   static TComplex Flux;
   static TComplex Fluxrf;

   sint16 Temp = __SSAT(Fluxrf.Real + Emo_Foc.StatVolt.Real - Mat_FixMul(Emo_Foc.StatCurr.Real, Emo_Foc.PhaseRes), MAT_FIX_SAT);
   sint16 fluxh_Real = Mat_ExeLp_without_min_max(&Emo_Foc.RealFluxLp, Temp);
   Flux.Real = __SSAT(fluxh_Real - Mat_FixMulScale(Emo_Foc.StatCurr.Real, Emo_Foc.PhaseInd, 0), MAT_FIX_SAT);

   Temp = __SSAT(Fluxrf.Imag + Emo_Foc.StatVolt.Imag - Mat_FixMul(Emo_Foc.StatCurr.Imag, Emo_Foc.PhaseRes), MAT_FIX_SAT);
   sint16 fluxh_Imag = Mat_ExeLp_without_min_max(&Emo_Foc.ImagFluxLp, Temp);
   Flux.Imag = __SSAT(fluxh_Imag - Mat_FixMulScale(Emo_Foc.StatCurr.Imag, Emo_Foc.PhaseInd, 0), MAT_FIX_SAT);

   uint16 Tempu;
   Emo_Foc.FluxAngle = Mat_CalcAngleAmp(Flux, &Tempu);

   uint16 FluxAbsValue = __SSAT(Mat_FixMul(Tempu, 32000), MAT_FIX_SAT + 1);
   Temp = Mat_ExeLp_without_min_max(&Emo_Ctrl.FluxbtrLp, FluxAbsValue);

   Fluxrf.Real = Emo_lEstFlux_Optimize(Flux.Real, Temp);
   Fluxrf.Imag = Emo_lEstFlux_Optimize(Flux.Imag, Temp);
}

#include "Table.hpp"
extern bool Emo_lExeSvm_Ccu6_1(void);
extern void Emo_lExeSvm_Ccu6_2(void);
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
   switch(Sector){
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

   if(0 == Emo_lExeSvm_Ccu6_1()){
      Emo_lExeSvm_Ccu6_2();
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

extern uint32 Emo_StopMotor(void);
extern void RteRead_AdcResult(  //TBD: move to destination module specific Rte interface
   sint16* lptrOutput
);
#include "infProjectARA_Exp.hpp" //TBD: move to destination module specific Rte interface
extern void RteWrite_PhaseCurr(  //TBD: move to destination module specific Rte interface
   TPhaseCurr* lptrInput
);
void Emo_HandleIoHwAb(void){
   static sint16 ls16AdcResult[3u];
   RteRead_AdcResult(&ls16AdcResult[0u]);

   sint16 R1miR0    = ls16AdcResult[1u]    - ls16AdcResult[0u];
   sint16 R0mioffs  = ls16AdcResult[0u]    - Emo_Svm.CsaOffsetAdw;
   sint16 OffsmiR1  = Emo_Svm.CsaOffsetAdw - ls16AdcResult[1u];
   static TPhaseCurr lstPhaseCurr;
   switch(Emo_Svm.StoredSector1){
      case 0: lstPhaseCurr.A = R0mioffs; lstPhaseCurr.B = R1miR0;   break;
      case 1: lstPhaseCurr.A = R1miR0;   lstPhaseCurr.B = R0mioffs; break;
      case 2: lstPhaseCurr.A = OffsmiR1; lstPhaseCurr.B = R0mioffs; break;
      case 3: lstPhaseCurr.A = OffsmiR1; lstPhaseCurr.B = R1miR0;   break;
      case 4: lstPhaseCurr.A = R1miR0;   lstPhaseCurr.B = OffsmiR1; break;
      case 5: lstPhaseCurr.A = R0mioffs; lstPhaseCurr.B = OffsmiR1; break;
      default: Emo_StopMotor(); break;
   }
   Emo_Svm.StoredSector1 = Emo_Svm.Sector;
   RteWrite_PhaseCurr(&lstPhaseCurr);
}

#include "Mat.hpp"
TComplex Mat_Clarke(
   TPhaseCurr* lptrInput
){
   TComplex StatCurr = {0, 0};
   StatCurr.Real     = __SSAT(
         4 * lptrInput->A
      ,  MAT_FIX_SAT
   );
   StatCurr.Imag = (sint16)__SSAT(
         Mat_FixMulScale(
               MAT_ONE_OVER_SQRT_3
            ,          ((sint32)lptrInput->A)
               +  (2 * ((sint32)lptrInput->B))
            ,  2
         )
      ,  MAT_FIX_SAT
   );
   return StatCurr;
}
extern void Emo_HandleCcu6(
      uint16 lu16CompT13ValueDown
   ,  uint16 lu16Comp60down
   ,  uint16 lu16Comp61down
   ,  uint16 lu16Comp62down
);
void RteRead_PhaseCurr(  //TBD: move to destination module specific Rte interface
   TPhaseCurr* lptrOutput
);
extern void   RteRead_SpeedReference (uint16* lptrOutput); //TBD: Move to header
extern uint32 RteRead_Adc2(void); //TBD: Move to header
void Emo_HandleFoc(void){
   Emo_HandleCcu6(
         Emo_Svm.CompT13ValueDown
      ,  Emo_Svm.comp60down
      ,  Emo_Svm.comp61down
      ,  Emo_Svm.comp62down
   );

   static TPhaseCurr lstPhaseCurr;
   RteRead_PhaseCurr(&lstPhaseCurr);

   Emo_Foc.StatCurr = Mat_Clarke(&lstPhaseCurr);
   Emo_Foc.RotCurr  = Mat_Park(Emo_Foc.StatCurr, Emo_Foc.Angle);
   Emo_lEstFlux();
   Emo_Ctrl.PtrAngle = (Emo_Ctrl.PtrAngle+1)&0x1f;

   uint16 lu16EmoCtrlAnglersptr_Sat;
   if(32 == Emo_Ctrl.Anglersptr){lu16EmoCtrlAnglersptr_Sat = 0;}
   else                         {lu16EmoCtrlAnglersptr_Sat = Emo_Ctrl.Anglersptr;}

   uint16 lu16Angle = Emo_Ctrl.AngleBuffer[(Emo_Ctrl.PtrAngle-lu16EmoCtrlAnglersptr_Sat)&0x1f];
   Emo_Ctrl.AngleBuffer[Emo_Ctrl.PtrAngle] = Emo_Foc.FluxAngle;
   Emo_Ctrl.Speedest = Emo_Foc.FluxAngle - lu16Angle;
   Emo_Ctrl.Speedpll = Emo_Ctrl.Speedest >> Emo_Ctrl.Exppllhigh;
   sint32 jj = Mat_FixMulScale(Emo_Ctrl.Speedest, Emo_Ctrl.Factorspeed, Emo_Ctrl.Expspeedhigh);
   sint16 Speed = jj / Emo_Foc.PolePair;
   Emo_Ctrl.ActSpeed = Mat_ExeLp_without_min_max(&Emo_Ctrl.SpeedLp, Speed);
   sint16 deltaphi = Emo_Foc.FluxAngle - Emo_Ctrl.FluxAnglePll;
   sint16 domega = __SSAT(Mat_FixMulScale(deltaphi, Emo_Ctrl.Pllkp, 0) + Emo_Ctrl.Speedpll, MAT_FIX_SAT);
   Emo_Ctrl.FluxAnglePll = Emo_Ctrl.FluxAnglePll + domega;

   sint16 ls16EmoCtrlRefCurr_Real;
   sint16 ls16EmoCtrlRefCurr_Imag;
   if(
         EMO_MOTOR_STATE_START
      == Emo_Status.MotorState
   ){
      Emo_Foc.StartAngle += Emo_Foc.StartFrequencySlope;

      uint16 lu16SpeedReference;
      RteRead_SpeedReference(&lu16SpeedReference);
      if(0 < lu16SpeedReference){Emo_Ctrl.RefCurr =  Emo_Foc.StartCurrent;}
      else                      {Emo_Ctrl.RefCurr = -Emo_Foc.StartCurrent;}

      Emo_Foc.Angle           = Emo_Foc.StartAngle;
      ls16EmoCtrlRefCurr_Real = Emo_Ctrl.RefCurr;
      ls16EmoCtrlRefCurr_Imag = 0;
   }
   else{
      Emo_Foc.Angle           = Emo_Ctrl.FluxAnglePll;
      ls16EmoCtrlRefCurr_Real = 0;
      ls16EmoCtrlRefCurr_Imag = Emo_Ctrl.RefCurr;
   }
   Emo_Foc.RotVolt.Real = Mat_ExePi(&Emo_Ctrl.RealCurrPi, ls16EmoCtrlRefCurr_Real - Emo_Foc.RotCurr.Real);
   Emo_Foc.RotVolt.Imag = Mat_ExePi(&Emo_Ctrl.ImagCurrPi, ls16EmoCtrlRefCurr_Imag - Emo_Foc.RotCurr.Imag);

   TComplex Vect1 = {0, 0};
   Vect1.Real = __SSAT(Mat_FixMulScale(Emo_Foc.RotVolt.Real, Emo_Foc.Dcfactor1, 1), MAT_FIX_SAT);
   Vect1.Imag = __SSAT(Mat_FixMulScale(Emo_Foc.RotVolt.Imag, Emo_Foc.Dcfactor1, 1), MAT_FIX_SAT);

   TComplex Vect2 = Limitsvektor(&Vect1, &Emo_Svm);

   uint16 ampl;
   lu16Angle = Mat_CalcAngleAmp(Vect2, &ampl);
   if(ampl > Emo_Svm.MaxAmp){ampl = Emo_Svm.MaxAmp;}

   if(127 < Emo_Svm.CounterOffsetAdw){
      Emo_Svm.Amp = ampl;
   }
   else{
      Emo_Svm.Amp = 0;
      Emo_Svm.CsaOffsetAdwSumme += RteRead_Adc2();
      Emo_Svm.CounterOffsetAdw++;
      if(128 == Emo_Svm.CounterOffsetAdw){
         uint16 i = Emo_Svm.CsaOffsetAdwSumme >> 8;
         if(i < Emo_Svm.CsaOffset){
            i = Emo_Svm.CsaOffset;
         }
         else{
            if((Emo_Svm.CsaOffset + 100) < i){
               i = Emo_Svm.CsaOffset + 100;
            }
         }
         Emo_Svm.CsaOffsetAdw = i;
      }
   }
   Emo_Svm.Angle = lu16Angle + Emo_Foc.Angle;
   Emo_Foc.StatVoltAmpM = __SSAT(Mat_FixMulScale(Emo_Svm.Amp, Emo_Foc.Dcfactor2, 3), MAT_FIX_SAT);
   Emo_Foc.StatVolt = Mat_PolarKartesisch(Emo_Foc.StatVoltAmpM, Emo_Svm.Angle);
   Emo_lExeSvm(&Emo_Svm);
   Emo_Ctrl.RotCurrImagdisplay = Mat_ExeLp_without_min_max(&Emo_Ctrl.RotCurrImagLpdisplay, Emo_Foc.RotCurr.Imag);
}
/*
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
*/

TComplex Limitsvektor(
      TComplex* inp
   ,  TEmo_Svm* par
){
   TComplex outp = {0, 0};
   uint32 btrqu = inp->Real * inp->Real + inp->Imag * inp->Imag;

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
         uint16 inpa = abs(inp->Real);
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

/*
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

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

