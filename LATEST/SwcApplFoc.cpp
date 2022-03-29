/******************************************************************************/
/* File   : SwcApplFoc.cpp                                                    */
/* Author : NAGARAJA HM (c) since 1982. All rights reserved.                  */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "module.hpp"
#include "infSwcApplFoc_EcuM.hpp"
#include "infSwcApplFoc_Dcm.hpp"
#include "infSwcApplFoc_SchM.hpp"

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
      module_SwcApplFoc(Std_TypeVersionInfo lVersionInfo) : abstract_module(lVersionInfo){
      }
      FUNC(void, SWCAPPLFOC_CODE) InitFunction   (void);
      FUNC(void, SWCAPPLFOC_CODE) DeInitFunction (void);
      FUNC(void, SWCAPPLFOC_CODE) MainFunction   (void);
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
//#include "CfgSwcApplFoc.hpp"

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
VAR(module_SwcApplFoc, SWCAPPLFOC_VAR) SwcApplFoc(
   {
         0x0000
      ,  0xFFFF
      ,  0x01
      ,  '0'
      ,  '1'
      ,  '0'
   }
);

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::InitFunction(void){
   SwcApplFoc.IsInitDone = E_OK;
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::DeInitFunction(void){
   SwcApplFoc.IsInitDone = E_NOT_OK;
}

FUNC(void, SWCAPPLFOC_CODE) module_SwcApplFoc::MainFunction(void){
}

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

