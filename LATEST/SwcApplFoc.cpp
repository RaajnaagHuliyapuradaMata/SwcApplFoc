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
   ,  public interface_SwcApplFoc_EcuM
   ,  public interface_SwcApplFoc_SchM
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

interface_SwcApplFoc_EcuM *EcuM_Client_ptr_SwcApplFoc = &SwcApplFoc;
interface_SwcApplFoc_SchM *SchM_Client_ptr_SwcApplFoc = &SwcApplFoc;

/*****************************************************/
/* FUNCTIONS                                         */
/*****************************************************/
FUNC(void, SWCAPPLFOC_CODE) InitFunction(void){
}

FUNC(void, SWCAPPLFOC_CODE) DeInitFunction(void){
}

FUNC(void, SWCAPPLFOC_CODE) MainFunction(void){
}

/*****************************************************/
/* EOF                                               */
/*****************************************************/

