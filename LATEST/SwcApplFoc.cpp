/******************************************************************************/
/* File   : SwcApplFoc.cpp                                                    */
/* Author : NAGARAJA HM (c) since 1982. All rights reserved.                  */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "Module.hpp"
#include "CfgSwcApplFoc.hpp"
#include "SwcApplFoc_core.hpp"
#include "infSwcApplFoc_Exp.hpp"
#include "infSwcApplFoc_Imp.hpp"

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
class module_SwcApplFoc:
      INTERFACES_EXPORTED_SWCAPPLFOC
      public abstract_module
   ,  public class_SwcApplFoc_Functionality
{
   private:
/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
            Std_TypeReturn          IsInitDone{E_NOT_OK};
      const CfgModule_TypeAbstract* lptrCfg{(CfgModule_TypeAbstract*)NULL_PTR};

   public:
      module_SwcApplFoc(Std_TypeVersionInfo lVersionInfo) : abstract_module(lVersionInfo){
      }
      FUNC(void, SWCAPPLFOC_CODE) InitFunction(
         CONSTP2CONST(CfgModule_TypeAbstract, SWCAPPLFOC_CONFIG_DATA, SWCAPPLFOC_APPL_CONST) lptrCfgModule
      );
      FUNC(void, SWCAPPLFOC_CODE) DeInitFunction (void);
      FUNC(void, SWCAPPLFOC_CODE) MainFunction   (void);
      SWCAPPLFOC_CORE_FUNCTIONALITIES
};

extern VAR(module_SwcApplFoc, SWCAPPLFOC_VAR) SwcApplFoc;

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/
CONSTP2VAR(infEcuMClient, SWCAPPLFOC_VAR, SWCAPPLFOC_CONST) gptrinfEcuMClient_SwcApplFoc = &SwcApplFoc;
CONSTP2VAR(infDcmClient,  SWCAPPLFOC_VAR, SWCAPPLFOC_CONST) gptrinfDcmClient_SwcApplFoc  = &SwcApplFoc;
CONSTP2VAR(infSchMClient, SWCAPPLFOC_VAR, SWCAPPLFOC_CONST) gptrinfSchMClient_SwcApplFoc = &SwcApplFoc;

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
VAR(module_SwcApplFoc, SWCAPPLFOC_VAR) SwcApplFoc(
   {
#if(STD_ON == _ReSIM)
         "SwcApplFoc"
#else
#endif
      ,  SWCAPPLFOC_AR_RELEASE_VERSION_MAJOR
      ,  SWCAPPLFOC_AR_RELEASE_VERSION_MINOR
      ,  0x00
      ,  0xFF
      ,  0x01
      ,  '0'
      ,  '1'
      ,  '0'
   }
);

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::InitFunction(
   CONSTP2CONST(CfgModule_TypeAbstract, SWCAPPLFOC_CONFIG_DATA, SWCAPPLFOC_APPL_CONST) lptrCfgModule
){
#if(STD_ON == SwcApplFoc_InitCheck)
   if(
         E_OK
      != IsInitDone
   ){
#endif
      if(NULL_PTR != lptrCfgModule){
         if(STD_HIGH){
            lptrCfg = lptrCfgModule;
         }
         else{
            lptrCfg = &PBcfgSwcApplFoc;
         }
      }
      else{
#if(STD_ON == SwcApplFoc_DevErrorDetect)
         Det_ReportError(
               0 //TBD: IdModule
            ,  0 //TBD: IdInstance
            ,  0 //TBD: IdApi
            ,  0 //TBD: IdError
         );
#endif
      }
#if(STD_ON == SwcApplFoc_InitCheck)
      IsInitDone = E_OK;
   }
   else{
#if(STD_ON == SwcApplFoc_DevErrorDetect)
      Det_ReportError(
            0 //TBD: IdModule
         ,  0 //TBD: IdInstance
         ,  0 //TBD: IdApi
         ,  SWCAPPLFOC_E_UNINIT
      );
#endif
   }
#endif
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::DeInitFunction(void){
#if(STD_ON == SwcApplFoc_InitCheck)
   if(
         E_OK
      == IsInitDone
   ){
#endif
#if(STD_ON == SwcApplFoc_InitCheck)
      IsInitDone = E_NOT_OK;
   }
   else{
#if(STD_ON == SwcApplFoc_DevErrorDetect)
      Det_ReportError(
            0 //TBD: IdModule
         ,  0 //TBD: IdInstance
         ,  0 //TBD: IdApi
         ,  SWCAPPLFOC_E_UNINIT
      );
#endif
   }
#endif
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::MainFunction(void){
#if(STD_ON == SwcApplFoc_InitCheck)
   if(
         E_OK
      == IsInitDone
   ){
#endif
#if(STD_ON == SwcApplFoc_InitCheck)
   }
   else{
#if(STD_ON == SwcApplFoc_DevErrorDetect)
      Det_ReportError(
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

