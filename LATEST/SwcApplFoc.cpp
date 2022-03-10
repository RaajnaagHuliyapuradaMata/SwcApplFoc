/*****************************************************/
/* File   : SwcApplFoc.cpp                           */
/* Author : Naagraaj HM                              */
/*****************************************************/

/*****************************************************/
/* #INCLUDES                                         */
/*****************************************************/
#include "module.h"
#include "infSwcApplFoc_EcuM.h"
#include "infSwcApplFoc_SchM.h"
#include "SwcApplFoc_Unused.h"

/*****************************************************/
/* #DEFINES                                          */
/*****************************************************/

/*****************************************************/
/* MACROS                                            */
/*****************************************************/

/*****************************************************/
/* TYPEDEFS                                          */
/*****************************************************/
class module_SwcApplFoc:
      public abstract_module
{
   public:
      FUNC(void, SWCAPPLFOC_CODE) InitFunction   (void);
      FUNC(void, SWCAPPLFOC_CODE) DeInitFunction (void);
      FUNC(void, SWCAPPLFOC_CODE) GetVersionInfo (void);
      FUNC(void, SWCAPPLFOC_CODE) MainFunction   (void);
};

/*****************************************************/
/* CONSTS                                            */
/*****************************************************/

/*****************************************************/
/* PARAMS                                            */
/*****************************************************/

/*****************************************************/
/* OBJECTS                                           */
/*****************************************************/
module_SwcApplFoc SwcApplFoc;
infEcuMClient*    gptrinfEcuMClient_SwcApplFoc = &SwcApplFoc;
infDcmClient*     gptrinfDcmClient_SwcApplFoc  = &SwcApplFoc;
infSchMClient*    gptrinfSchMClient_SwcApplFoc = &SwcApplFoc;

/*****************************************************/
/* FUNCTIONS                                         */
/*****************************************************/
FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::InitFunction(void){
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::DeInitFunction(void){
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::GetVersionInfo(void){
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::MainFunction(void){
}

/*****************************************************/
/* EOF                                               */
/*****************************************************/

