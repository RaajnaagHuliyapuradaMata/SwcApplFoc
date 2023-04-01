/******************************************************************************/
/* File   : ApplSwcFoc.cpp                                                    */
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

FUNC(void, APPLSWCFOC_CODE) module_ApplSwcFoc::MainFunction(
   void
){
#if(STD_ON == ApplSwcFoc_InitCheck)
   if(
         E_OK
      == IsInitDone
   ){
#endif
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

