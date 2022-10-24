/******************************************************************************/
/* File   : Template.hpp                                                      */
/* Author : NAGARAJA HM (c) since 1982. All rights reserved.                  */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "types.hpp"

#include "ApplSwcFoc_Pbcfg.hpp"

#include "Mat.hpp"
#include "ApplSwcFoc.hpp"

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/
#define FOC_R_SHUNT                                                    (0.00500)
#define FOC_NOM_CUR                                                    (5.00000)
#define FOC_PWM_FREQ                                                    (0x4E20)
#define FOC_R_PHASE                                                       (0.36)
#define FOC_L_PHASE                                                    (0.00020)
#define FOC_SPEED_KP                                                     (0x5DC)
#define FOC_SPEED_KI                                                     (0x258)
#define FOC_MAX_POS_REF_CUR                                            (4.00000)
#define FOC_MAX_NEG_REF_CUR                                           (-4.00000)
#define FOC_MIN_POS_REF_CUR                                            (3.00000)
#define FOC_MIN_NEG_REF_CUR                                           (-3.00000)
#define FOC_MAX_CUR_SPEED                                           (1000.00000)
#define FOC_MIN_CUR_SPEED                                          (-1000.00000)
#define FOC_SPEED_FILT_TIME                                               (0.01)
#define FOC_FLUX_ADJUST                                                   (0.02)
#define FOC_ESTFLUX_FILT_TIME                                    FOC_FLUX_ADJUST
#define FOC_START_CUR                                                      (0x2)
#define FOC_ZERO_VEC_TIME                                                 (0.10)
#define FOC_END_START_SPEED                                              (0x320)
#define FOC_START_ACCEL                                             (1000.00000)
#define FOC_START_FREQ_ZERO                                                (0x1)
#define FOC_SWITCH_ON_SPEED                                               (0x64)
#define FOC_CUR_ADJUST                                                    (0.50)
#define FOC_MAX_SPEED                                                    (0x7D0)

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* TYPEDEFS                                                                   */
/******************************************************************************/

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/
const TEmo_Focpar_Cfg Emo_Focpar_Cfg = {
      (float)  FOC_R_SHUNT
   ,  (float)  FOC_NOM_CUR
   ,  (float)  FOC_PWM_FREQ
   ,  (float)  FOC_R_PHASE
   ,  (float)  FOC_L_PHASE
   ,  (uint16) FOC_SPEED_KP
   ,  (uint16) FOC_SPEED_KI
   ,  (float)  FOC_MAX_POS_REF_CUR
   ,  (float)  FOC_MAX_NEG_REF_CUR
   ,  (float)  FOC_MIN_POS_REF_CUR
   ,  (float)  FOC_MIN_NEG_REF_CUR
   ,  (float)  FOC_MAX_CUR_SPEED
   ,  (float)  FOC_MIN_CUR_SPEED
   ,  (float)  FOC_SPEED_FILT_TIME
   ,  (float)  FOC_ESTFLUX_FILT_TIME
   ,  (uint16) 0
   ,  (uint16) FOC_POLE_PAIRS
   ,  (float)  FOC_START_CUR
   ,  (float)  FOC_ZERO_VEC_TIME
   ,  (float)  FOC_END_START_SPEED
   ,  (float)  FOC_START_ACCEL
   ,  (uint16) FOC_START_FREQ_ZERO
   ,  (float)  FOC_SWITCH_ON_SPEED
   ,  (float)  FOC_CUR_ADJUST
   ,  (float)  FOC_MAX_SPEED
};

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

