#pragma once
/******************************************************************************/
/* File   : Template.hpp                                                      */
/* Author : NAGARAJA HM (c) since 1982. All rights reserved.                  */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/
#define EMO_CFG_CTRAP_ENABLED                                                (0)
#define EMO_CFG_FOC_TABLE_SCALE                                    (0.117553711)
#define EMO_CFG_MON16_ENABLED                                                (1)
#define EMO_E_DCLINK_CURRENT_FAILURE                                        (0u)
#define EMO_E_OVER_TEMP_FAILURE                                             (2u)
#define EMO_E_MOTOR_CURRENT_FAILURE                                         (3u)
#define EMO_MOTOR_STATE_UNINIT                                              (0u)
#define EMO_MOTOR_STATE_STOP                                                (1u)
#define EMO_MOTOR_STATE_START                                               (2u)
#define EMO_MOTOR_STATE_RUN                                                 (3u)
#define EMO_MOTOR_STATE_FAULT                                               (4u)
#define EMO_ERROR_NONE                                                      (0u)
#define EMO_ERROR_MOTOR_INIT                                                (1u)
#define EMO_ERROR_MOTOR_NOT_STOPPED                                         (2u)
#define EMO_ERROR_MOTOR_NOT_STARTED                                         (3u)
#define EMO_ERROR_VALUE_CU_KI                                           (0x0001)
#define EMO_ERROR_VALUE_CU_KP                                           (0x0002)
#define EMO_ERROR_VALUE_CU_ADCC                                         (0x0004)
#define EMO_ERROR_LIMITS_REFCURRENT                                     (0x0008)
#define EMO_ERROR_REFCURRENT                                            (0x0010)
#define EMO_ERROR_SPEED_POINTS                                          (0x0020)
#define EMO_ERROR_T_SPEED_LP                                            (0x0040)
#define EMO_ERROR_STARTCURRENT                                          (0x0080)
#define EMO_ERROR_STARTTIME                                             (0x0100)
#define EMO_ERROR_SPEEDSLEWRATE                                         (0x0200)
#define EMO_ERROR_MINTIME                                               (0x0400)
#define EMO_ERROR_POLPAIR                                               (0x0800)
#define EMO_ERROR_CSAOFFSET                                             (0x1000)
#define EMO_SVM_MINTIME                                                     (80)
#define EMO_SVM_DEADTIME                                                    (30)
#define EMO_DECOUPLING                                                      (0u)
#define EMO_RUN                                                              (1)

#define FOC_POLE_PAIRS                                                     (0x4)

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* TYPEDEFS                                                                   */
/******************************************************************************/
typedef struct{
   uint8  MotorState;
   uint16 MotorStartError;
}TEmo_Status;

typedef struct{
   sint16 Kp;
   sint16 Ki;
   sint16 IMin;
   sint16 IMax;
   sint16 PiMin;
   sint16 PiMax;
}TEmo_Pi_Cfg;

typedef struct{
   sint16 CoefA;
   sint16 CoefB;
   sint16 Min;
   sint16 Max;
}TEmo_Lp_Cfg;

typedef struct{
   TComplex StatCurr;
   TComplex RotCurr;
   TComplex StatVolt;
   TComplex RotVolt;
   uint16 Angle;
   uint16 FluxAngle;
   uint16 StoredAngle;
   uint16 PhaseRes;
   uint16 PhaseInd;
   sint16 StartEndSpeed;
   uint16 StartVoltAmp;
   uint16 dummy;
   TMat_Lp_Simple RealFluxLp;
   TMat_Lp_Simple ImagFluxLp;
   uint16 CountStart;
   uint16 PolePair;
   sint16 StartCurrent;
   uint16 TimeSpeedzero;
   sint16 SpeedtoFrequency;
   sint32 StartSpeedSlewRate;
   uint16 CsaGain;
   uint16 DcLinkVoltage;
   uint16 Dcfactor1;
   sint16 StartSpeedSlope;
   sint16 StartFrequencySlope;
   sint32 StartSpeedSlopeMem;
   TComplex StartVoltage;
   uint16 StartAngle;
   TComplex RotVoltCurrentcontrol;
   uint16 StartVoltAmpDivUz;
   uint16 Dcfactor2;
   uint32 Kdcdivident1;
   uint16 Kdcfactor2;
   uint16 Kdcfactoriqc;
   uint16 StatVoltAmpM;
   uint16 LpCoefb1;
   uint16 LpCoefb2;
}TEmo_Foc;

typedef struct{
   sint16 RefSpeed;
   sint16 ActSpeed;
   sint16 RefCurr;
   sint16 ActSpeeddisplay;
   uint16 SpeedPiInit;
   TMat_Pi SpeedPi;
   TMat_Pi RealCurrPi;
   TMat_Pi ImagCurrPi;
   TMat_Lp_Simple SpeedLp;
   uint16 AngleBuffer[32];
   uint16 PtrAngle;
   sint16 MaxRefCurrent;
   sint16 MinRefCurrent;
   sint16 MaxRefStartCurrent;
   sint16 MinRefStartCurrent;
   sint16 Speedlevelmaxstart;
   sint16 Speedlevelminstart;
   sint16 SpeedLevelSwitchOn;
   TMat_Lp_Simple SpeedLpdisplay;
   TMat_Lp_Simple FluxbtrLp;
   sint16 Speedest;
   sint16 Speedpll;
   uint16 FluxAnglePll;
   uint16 Pllkp;
   uint16 Anglersptr;
   uint16 Factorspeed;
   uint16 Expspeedhigh;
   uint16 Exppllhigh;
   uint16 EnableStartVoltage;
   TMat_Lp_Simple RotCurrImagLpdisplay;
   sint16 RotCurrImagdisplay;
}TEmo_Ctrl;

typedef struct{
   uint16 Angle;
   uint16 Amp;
   uint16 Sector;
   uint16 comp60up;
   uint16 T1;
   uint16 comp61up;
   uint16 T2;
   uint16 comp62up;
   TPhaseCurr PhaseCurr;
   uint16 CsaOffset;
   uint16 MaxAmp;
   uint16 MaxAmp9091pr;
   uint16 MaxAmp4164pr;
   uint16 Kfact256;
   uint32 MaxAmpQuadrat;
   uint16 CompT13ValueUp;
   uint16 CompT13ValueDown;
   uint16 T13Trigger;
   uint16 comp60down;
   uint16 comp61down;
   uint16 comp62down;
   uint16 StoredSector1;
   uint16 CounterOffsetAdw;
   uint32 CsaOffsetAdwSumme;
   uint16 CsaOffsetAdw;
}TEmo_Svm;

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
extern uint32                                                 Emo_AdcResult[4u];
extern TEmo_Status                                                   Emo_Status;
extern TEmo_Ctrl                                                       Emo_Ctrl;
extern TEmo_Foc                                                         Emo_Foc;
extern TEmo_Svm                                                         Emo_Svm;
extern uint16                                                 speeduserreferenz;

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
extern uint32   Emo_Init                                                 (void);
extern void     Emo_SetRefSpeed                               (sint16 RefSpeed);
extern uint32   Emo_GetSpeed                                             (void);
extern uint32   Emo_StartMotor                            (uint32 EnableBridge);
extern uint32   Emo_StopMotor                                            (void);
extern void     Emo_lInitFocVar                                          (void);
extern void     Emo_HandleT2Overflow                                     (void);
extern uint32   Emo_GetMotorState                                        (void);
extern void     Emo_HandleAdc1                                           (void);
extern void     Emo_HandleFoc                                            (void);
extern void     Emo_InitFoc                                              (void);
extern void     Emo_ExeSvmTest                                 (TEmo_Svm *pSvm);
extern void     Emo_EstFluxTest                                          (void);
extern uint16   Emo_CalcAngleAmpTest              (TComplex Stat, uint16 *pAmp);
extern void     Emo_CalcAngleAmpSvmTest                                  (void);
extern void     Emo_setspeedreferenz                     (uint16 speedreferenz);
extern TComplex Limitsvektor                     (TComplex *inp, TEmo_Svm *par);
extern TComplex Limitsvektorphase                (TComplex *inp, TEmo_Svm *par);
extern sint16   abs                                                (sint16 inp);

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

