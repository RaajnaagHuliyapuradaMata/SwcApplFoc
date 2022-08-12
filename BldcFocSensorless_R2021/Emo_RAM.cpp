/******************************************************************************/
/* File   : Template.hpp                                                      */
/* Author : NAGARAJA HM (c) since 1982. All rights reserved.                  */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "types.hpp"

#include "Emo_RAM.hpp"

#include "tle987x.hpp"

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/
#define EMO_IMESS  1

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
//extern TEmo_Status Emo_Status;
uint32 Emo_AdcResult[4u];
TEmo_Ctrl Emo_Ctrl;
TEmo_Foc Emo_Foc;
TEmo_Svm Emo_Svm;

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
/*
void Emo_HandleAdc1(void){
  CCU6_SetT13Compare(Emo_Svm.CompT13ValueDown);
  CCU6_SetT13Trigger(0x76);
  Emo_AdcResult[0u] = ADC1->RES_OUT1.reg;
  ADC1->IE.bit.ESM_IE = 0;
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
    case 0:
    {
      Emo_Svm.PhaseCurr.A = R0mioffs;
      Emo_Svm.PhaseCurr.B = R1miR0;
    }
    break;

    case 1:
    {
      Emo_Svm.PhaseCurr.A = R1miR0;
      Emo_Svm.PhaseCurr.B = R0mioffs;
    }
    break;

    case 2:
    {
      Emo_Svm.PhaseCurr.A = OffsmiR1;
      Emo_Svm.PhaseCurr.B = R0mioffs;
    }
    break;

    case 3:
    {
      Emo_Svm.PhaseCurr.A = OffsmiR1;
      Emo_Svm.PhaseCurr.B = R1miR0;
    }
    break;

    case 4:
    {
      Emo_Svm.PhaseCurr.A = R1miR0;
      Emo_Svm.PhaseCurr.B = OffsmiR1;
    }
    break;

    case 5:
    {
      Emo_Svm.PhaseCurr.A = R0mioffs;
      Emo_Svm.PhaseCurr.B = OffsmiR1;
    }
    break;

    default:
    {

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
  CCU6->IEN.bit.ENT12PM = 0;
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
  ADC1->ICLR.bit.ESM_ICLR = 1;
  ADC1->IE.bit.ESM_IE = 1;
  Emo_AdcResult[1u] = ADC1->RES_OUT1.reg;
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
      else
      {
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
    case 0u:
    {
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
        else
        {
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
        else
        {
          Compare1down = 1;
        }
      }
#endif
      T13ValueUp = (Compare1up + Compare0up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare1down + Compare2down) / 2;
      break;
    }
    case 1u:
    {
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
        else
        {
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
        else
        {
          Compare0down = 1;
        }
      }
#endif
      T13ValueUp = (Compare0up + Compare1up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare0down + Compare2down) / 2;
      break;
    }
    case 2u:
    {
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
        else
        {
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
        else
        {
          Compare2down = 1;
        }
      }
#endif
      T13ValueUp = (Compare2up + Compare1up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare2down + Compare0down) / 2;
      break;
    }
    case 3u:
    {
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
        else
        {
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
        else
        {
          Compare1down = 1;
        }
      }
#endif
      T13ValueUp = (Compare1up + Compare2up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare1down + Compare0down) / 2;
      break;
    }
    case 4u:
    {
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
        else
        {
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
        else
        {
          Compare0down = 1;
        }
      }
#endif
      T13ValueUp = (Compare0up + Compare2up) / 2;
      T13ValueDown = CCU6_T12PR - (Compare0down + Compare1down) / 2;
      break;
    }
    default:
    {
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
        else
        {
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
        else
        {
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
  if(CCU6->TCTR0.bit.CDIR == 0){
    CCU6->IEN.bit.ENT12PM = 1;
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
/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

