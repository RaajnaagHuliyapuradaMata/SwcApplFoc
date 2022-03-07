/*****************************************************/
/* File   : SwcApplFoc.cpp                           */
/* Author : Naagraaj HM                              */
/*****************************************************/

/*****************************************************/
/* #INCLUDES                                         */
/*****************************************************/
#include "module.h"
#include "SwcApplFoc_EcuM.h"
#include "SwcApplFoc_SchM.h"
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
infSchMClient*    gptrinfSchMClient_SwcApplFoc = &SwcApplFoc;

/*****************************************************/
/* FUNCTIONS                                         */
/*****************************************************/
FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::InitFunction(void){
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::DeInitFunction(void){
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::MainFunction(void){
}

/*****************************************************/
/* EOF                                               */
/*****************************************************/

