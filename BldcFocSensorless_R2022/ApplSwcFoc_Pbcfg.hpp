/******************************************************************************/
/* File   : ApplSwcFoc_Pbcfg.hpp                                              */
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

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* TYPEDEFS                                                                   */
/******************************************************************************/
typedef struct{
   float  Rshunt;
   float  NominalCurrent;
   float  PWM_Frequency;
   float  PhaseRes;
   float  PhaseInd;
   uint16 SpeedPi_Kp;
   uint16 SpeedPi_Ki;
   float  MaxRefCurr;
   float  MinRefCurr;
   float  MaxRefStartCurr;
   float  MinRefStartCurr;
   float  SpeedLevelPos;
   float  SpeedLevelNeg;
   float  TimeConstantSpeedFilter;
   float  TimeConstantEstFluxFilter;
   uint16 CsaOffset;
   uint16 PolePair;
   float  StartCurrent;
   float  TimeSpeedzero;
   float  StartSpeedEnd;
   float  StartSpeedSlewRate;
   uint16 EnableFrZero;
   float  SpeedLevelSwitchOn;
   float  AdjustmCurrentControl;
   float  MaxSpeed;
}TEmo_Focpar_Cfg;

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/
extern const TEmo_Focpar_Cfg                                     Emo_Focpar_Cfg;

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

