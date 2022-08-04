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
#define SWCAPPLFOC_AR_RELEASE_VERSION_MAJOR                                    4
#define SWCAPPLFOC_AR_RELEASE_VERSION_MINOR                                    3

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/
#if(SWCAPPLFOC_AR_RELEASE_VERSION_MAJOR != STD_AR_RELEASE_VERSION_MAJOR)
   #error "Incompatible SWCAPPLFOC_AR_RELEASE_VERSION_MAJOR!"
#endif

#if(SWCAPPLFOC_AR_RELEASE_VERSION_MINOR != STD_AR_RELEASE_VERSION_MINOR)
   #error "Incompatible SWCAPPLFOC_AR_RELEASE_VERSION_MINOR!"
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
VAR(module_ApplSwcFoc, SWCAPPLFOC_VAR) ApplSwcFoc;

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
FUNC(void, SWCAPPLFOC_CODE) module_ApplSwcFoc::InitFunction(
      CONSTP2CONST(ConstModule_TypeAbstract, SWCAPPLFOC_CONST,       SWCAPPLFOC_APPL_CONST) lptrConstModule
   ,  CONSTP2CONST(CfgModule_TypeAbstract,   SWCAPPLFOC_CONFIG_DATA, SWCAPPLFOC_APPL_CONST) lptrCfgModule
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
         ,  SWCAPPLFOC_E_UNINIT
      );
#endif
   }
#endif
}

FUNC(void, SWCAPPLFOC_CODE) module_ApplSwcFoc::DeInitFunction(
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
         ,  SWCAPPLFOC_E_UNINIT
      );
#endif
   }
#endif
}

FUNC(void, SWCAPPLFOC_CODE) module_ApplSwcFoc::MainFunction(
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
         ,  SWCAPPLFOC_E_UNINIT
      );
#endif
   }
#endif
}

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

