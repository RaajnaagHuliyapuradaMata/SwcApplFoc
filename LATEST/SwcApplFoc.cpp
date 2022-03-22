/******************************************************************************/
/* File   : SwcApplFoc.cpp                                                    */
/* Author : NAGARAJA HM (c) since 1982. All rights reserved.                  */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "module.h"
//#include "SwcApplFoc_Cfg.h"
#include "infSwcApplFoc_EcuM.h"
#include "infSwcApplFoc_Dcm.h"
#include "infSwcApplFoc_SchM.h"

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/
#define SWCAPPLFOC_AR_RELEASE_MAJOR_VERSION                                    4
#define SWCAPPLFOC_AR_RELEASE_MINOR_VERSION                                    3

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/
#if(SWCAPPLFOC_AR_RELEASE_MAJOR_VERSION != STD_AR_RELEASE_MAJOR_VERSION)
   #error "Incompatible SWCAPPLFOC_AR_RELEASE_MAJOR_VERSION!"
#endif

#if(SWCAPPLFOC_AR_RELEASE_MINOR_VERSION != STD_AR_RELEASE_MINOR_VERSION)
   #error "Incompatible SWCAPPLFOC_AR_RELEASE_MINOR_VERSION!"
#endif

/******************************************************************************/
/* TYPEDEFS                                                                   */
/******************************************************************************/
class module_SwcApplFoc:
      public abstract_module
{
   public:
      FUNC(void, SWCAPPLFOC_CODE) InitFunction   (void);
      FUNC(void, SWCAPPLFOC_CODE) DeInitFunction (void);
      FUNC(void, SWCAPPLFOC_CODE) GetVersionInfo (void);
      FUNC(void, SWCAPPLFOC_CODE) MainFunction   (void);
};

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
VAR(module_SwcApplFoc, SWCAPPLFOC_VAR) SwcApplFoc;
CONSTP2VAR(infEcuMClient, SWCAPPLFOC_VAR, SWCAPPLFOC_CONST) gptrinfEcuMClient_SwcApplFoc = &SwcApplFoc;
CONSTP2VAR(infDcmClient,  SWCAPPLFOC_VAR, SWCAPPLFOC_CONST) gptrinfDcmClient_SwcApplFoc  = &SwcApplFoc;
CONSTP2VAR(infSchMClient, SWCAPPLFOC_VAR, SWCAPPLFOC_CONST) gptrinfSchMClient_SwcApplFoc = &SwcApplFoc;

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::InitFunction(void){
   SwcApplFoc.IsInitDone = E_OK;
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::DeInitFunction(void){
   SwcApplFoc.IsInitDone = E_NOT_OK;
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::GetVersionInfo(void){
#if(STD_ON == SwcApplFoc_DevErrorDetect)
//TBD: API parameter check
   Det_ReportError(
   );
#endif
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::MainFunction(void){
}

#include "SwcApplFoc_Unused.h"

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

