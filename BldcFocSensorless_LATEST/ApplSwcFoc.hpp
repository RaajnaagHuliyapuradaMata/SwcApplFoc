#pragma once
/******************************************************************************/
/* File   : ApplSwcFoc.hpp                                                    */
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
#include "ConstApplSwcFoc.hpp"
#include "CfgApplSwcFoc.hpp"
#include "ApplSwcFoc_core.hpp"
#include "infApplSwcFoc_Exp.hpp"

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* TYPEDEFS                                                                   */
/******************************************************************************/
class module_ApplSwcFoc:
      INTERFACES_EXPORTED_APPLSWCFOC
      public abstract_module
   ,  public class_ApplSwcFoc_Functionality
{
   private:
/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
      const ConstApplSwcFoc_Type* lptrConst = (ConstApplSwcFoc_Type*)NULL_PTR;

   public:
/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
      FUNC(void, APPLSWCFOC_CODE) InitFunction(
            CONSTP2CONST(ConstModule_TypeAbstract, APPLSWCFOC_CONST,       APPLSWCFOC_APPL_CONST) lptrConstModule
         ,  CONSTP2CONST(CfgModule_TypeAbstract,   APPLSWCFOC_CONFIG_DATA, APPLSWCFOC_APPL_CONST) lptrCfgModule
      );
      FUNC(void, APPLSWCFOC_CODE) DeInitFunction (void);
      FUNC(void, APPLSWCFOC_CODE) MainFunction   (void);
      APPLSWCFOC_CORE_FUNCTIONALITIES
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
extern VAR(module_ApplSwcFoc, APPLSWCFOC_VAR) ApplSwcFoc;

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

