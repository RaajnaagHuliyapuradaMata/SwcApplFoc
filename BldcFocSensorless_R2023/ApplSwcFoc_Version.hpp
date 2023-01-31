#pragma once
/******************************************************************************/
/* File   : ApplSwcFoc_Version.hpp                                            */
/* Author : NAGARAJA HM (c) since 1982. All rights reserved.                  */
/******************************************************************************/

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
/* EOF                                                                        */
/******************************************************************************/

