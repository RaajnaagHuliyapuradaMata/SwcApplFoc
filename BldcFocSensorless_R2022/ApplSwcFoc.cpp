/******************************************************************************/
/* File   : ApplSwcFoc.cpp                                                    */
/* Author : NAGARAJA HM (c) since 1982. All rights reserved.                  */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "Module.hpp"
#include "ApplSwcFoc.hpp"
#include "infApplSwcFoc_Imp.hpp"

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/
#define APPLSWCFOC_AR_RELEASE_VERSION_MAJOR                                    4
#define APPLSWCFOC_AR_RELEASE_VERSION_MINOR                                    3

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/
#if(APPLSWCFOC_AR_RELEASE_VERSION_MAJOR != STD_AR_RELEASE_VERSION_MAJOR)
   #error "Incompatible APPLSWCFOC_AR_RELEASE_VERSION_MAJOR!"
#endif

#if(APPLSWCFOC_AR_RELEASE_VERSION_MINOR != STD_AR_RELEASE_VERSION_MINOR)
   #error "Incompatible APPLSWCFOC_AR_RELEASE_VERSION_MINOR!"
#endif

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
VAR(module_ApplSwcFoc, APPLSWCFOC_VAR) ApplSwcFoc;

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
extern void   BDRV_Init (void); //TBD: use interface headers as per architecture
extern uint32 Emo_Init  (void); //TBD: use interface headers as per architecture

FUNC(void, APPLSWCFOC_CODE) module_ApplSwcFoc::InitFunction(
      CONSTP2CONST(ConstModule_TypeAbstract, APPLSWCFOC_CONST,       APPLSWCFOC_APPL_CONST) lptrConstModule
   ,  CONSTP2CONST(CfgModule_TypeAbstract,   APPLSWCFOC_CONFIG_DATA, APPLSWCFOC_APPL_CONST) lptrCfgModule
){
#if(STD_ON == ApplSwcFoc_InitCheck)
   if(
         E_OK
      != IsInitDone
   ){
#endif
      if(
            (NULL_PTR != lptrConstModule)
         && (NULL_PTR != lptrCfgModule)
      ){
         lptrConst = (const ConstApplSwcFoc_Type*)lptrConstModule;
         lptrCfg   = lptrCfgModule;
      }
      else{
#if(STD_ON == ApplSwcFoc_DevErrorDetect)
         ServiceDet_ReportError(
               0 //TBD: IdModule
            ,  0 //TBD: IdInstance
            ,  0 //TBD: IdApi
            ,  0 //TBD: IdError
         );
#endif
      }
      BDRV_Init();
      Emo_Init();
#if(STD_ON == ApplSwcFoc_InitCheck)
      IsInitDone = E_OK;
   }
   else{
#if(STD_ON == ApplSwcFoc_DevErrorDetect)
      ServiceDet_ReportError(
            0 //TBD: IdModule
         ,  0 //TBD: IdInstance
         ,  0 //TBD: IdApi
         ,  APPLSWCFOC_E_UNINIT
      );
#endif
   }
#endif
}

FUNC(void, APPLSWCFOC_CODE) module_ApplSwcFoc::DeInitFunction(
   void
){
#if(STD_ON == ApplSwcFoc_InitCheck)
   if(
         E_OK
      == IsInitDone
   ){
#endif
#if(STD_ON == ApplSwcFoc_InitCheck)
      IsInitDone = E_NOT_OK;
   }
   else{
#if(STD_ON == ApplSwcFoc_DevErrorDetect)
      ServiceDet_ReportError(
            0 //TBD: IdModule
         ,  0 //TBD: IdInstance
         ,  0 //TBD: IdApi
         ,  APPLSWCFOC_E_UNINIT
      );
#endif
   }
#endif
}

extern void Emo_HandleFoc  (void);
extern void Emo_HandleAdc1 (void);
#define RteRead_SpeedReference() false //TBD: Implement RTE Read for this port
FUNC(void, APPLSWCFOC_CODE) module_ApplSwcFoc::MainFunction(
   void
){
#if(STD_ON == ApplSwcFoc_InitCheck)
   if(
         E_OK
      == IsInitDone
   ){
#endif

// uint16 mV;
   if(
         true
      == RteRead_SpeedReference() // ADC1_GetChResult_mV(&mV, ADC1_CH4)
   ){
/*
      Emo_setspeedreferenz(mV / 2);
      if(
            1000
         <  mV
      ){
         Main_lStartMotor();
      }
      else if(
            800
         >  mV
      ){
         Main_lStopMotor();
      }
      else{}
*/
   }
   else{}

   Emo_HandleAdc1();
   Emo_HandleFoc();

#if(STD_ON == ApplSwcFoc_InitCheck)
   }
   else{
#if(STD_ON == ApplSwcFoc_DevErrorDetect)
      ServiceDet_ReportError(
            0 //TBD: IdModule
         ,  0 //TBD: IdInstance
         ,  0 //TBD: IdApi
         ,  APPLSWCFOC_E_UNINIT
      );
#endif
   }
#endif
}

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

