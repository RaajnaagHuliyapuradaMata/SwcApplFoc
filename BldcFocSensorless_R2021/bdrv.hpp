#pragma once
/******************************************************************************/
/* File   : Template.hpp                                                      */
/* Author : NAGARAJA HM (c) since 1982. All rights reserved.                  */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
//#include "tle_variants.hpp"
//#include "sfr_access.hpp"
//#include "scu.hpp"
//#include "int.hpp"
//#include "wdt1.hpp"

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/
#define TBDRV_Curr_RANGE_POS 5u
#define TBDRV_Curr_CURRENT_MASK 0x3Fu

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* TYPEDEFS                                                                   */
/******************************************************************************/
typedef enum _TBdrv_Ch_Cfg
{
  Ch_Off = 0u,
  Ch_En  = 1u,
  Ch_PWM = 3u,
  Ch_On  = 5u,
  Ch_DCS = 9u
}TBdrv_Ch_Cfg;

typedef enum _TBdrv_Ch
{
  LS1 = 0u,
  LS2 = 1u,
  HS1 = 2u,
  HS2 = 3u
#if(UC_SERIES == TLE987)
  ,
  LS3 = 4u,
  HS3 = 5u
#endif
}TBdrv_Ch;

#define LS1_DS SCUPM_BDRV_ISCLR_LS1_DS_ICLR_Msk
#define LS2_DS SCUPM_BDRV_ISCLR_LS2_DS_ICLR_Msk
#define HS1_DS SCUPM_BDRV_ISCLR_HS1_DS_ICLR_Msk
#define HS2_DS SCUPM_BDRV_ISCLR_HS2_DS_ICLR_Msk
#define LS1_OC SCUPM_BDRV_ISCLR_LS1_OC_ICLR_Msk
#define LS2_OC SCUPM_BDRV_ISCLR_LS2_OC_ICLR_Msk
#define HS1_OC SCUPM_BDRV_ISCLR_HS1_OC_ICLR_Msk
#define HS2_OC SCUPM_BDRV_ISCLR_HS2_OC_ICLR_Msk
#if(UC_SERIES == TLE987)
  #define LS3_DS SCUPM_BDRV_ISCLR_LS3_DS_ICLR_Msk
  #define HS3_DS SCUPM_BDRV_ISCLR_HS3_DS_ICLR_Msk
  #define LS3_OC SCUPM_BDRV_ISCLR_LS3_OC_ICLR_Msk
  #define HS3_OC SCUPM_BDRV_ISCLR_HS3_OC_ICLR_Msk
#endif

typedef enum _TBDRV_Off_Diag_Sts
{
  Ch_Ok = 0u,
  Ch_Short_to_Gnd = 1u,
  Ch_Short_to_VBat = 2u
}TBDRV_Off_Diag_Sts;

typedef struct{
  bool GlobFailSts;
  TBDRV_Off_Diag_Sts Phase1;
  TBDRV_Off_Diag_Sts Phase2;
#if(UC_SERIES == TLE987)
  TBDRV_Off_Diag_Sts Phase3;
#endif
}TBDRV_Off_Diag;

typedef enum _TBdrv_Ch_Int
{
  Int_Off = 0U,
  Int_DS = 1U,
  Int_OC = 2U,
  Int_DS_OC = 3U
}TBdrv_Ch_Int;

typedef enum _TBdrv_DSM_Threshold
{
  Threshold_0_25_V = 0U,
  Threshold_0_50_V = 1U,
  Threshold_0_75_V = 2U,
  Threshold_1_00_V = 3U,
  Threshold_1_25_V = 4U,
  Threshold_1_50_V = 5U,
  Threshold_1_75_V = 6U,
  Threshold_2_00_V = 7U
}TBdrv_DSM_Threshold;

typedef enum _TBDRV_Curr
{

  BDRV_Curr_HR_Disabled = 0U,
  BDRV_Curr_HR_Min = 1U,
  BDRV_Curr_HR_10mA = 2U,
  BDRV_Curr_HR_15mA = 3U,
  BDRV_Curr_HR_20mA = 4U,
  BDRV_Curr_HR_25mA = 5U,
  BDRV_Curr_HR_30mA = 6U,
  BDRV_Curr_HR_35mA = 7U,
  BDRV_Curr_HR_40mA = 8U,
  BDRV_Curr_HR_45mA = 9U,
  BDRV_Curr_HR_50mA = 10U,
  BDRV_Curr_HR_55mA = 11U,
  BDRV_Curr_HR_60mA = 12U,
  BDRV_Curr_HR_65mA = 13U,
  BDRV_Curr_HR_70mA = 14U,
  BDRV_Curr_HR_75mA = 15U,
  BDRV_Curr_HR_80mA = 16U,
  BDRV_Curr_HR_85mA = 17U,
  BDRV_Curr_HR_90mA = 18U,
  BDRV_Curr_HR_95mA = 19U,
  BDRV_Curr_HR_100mA = 20U,
  BDRV_Curr_HR_105mA = 21U,
  BDRV_Curr_HR_110mA = 22U,
  BDRV_Curr_HR_115mA = 23U,
  BDRV_Curr_HR_120mA = 24U,
  BDRV_Curr_HR_125mA = 25U,
  BDRV_Curr_HR_130mA = 26U,
  BDRV_Curr_HR_135mA = 27U,
  BDRV_Curr_HR_140mA = 28U,
  BDRV_Curr_HR_145mA = 29U,
  BDRV_Curr_HR_150mA = 30U,
  BDRV_Curr_HR_155mA = 31U,

  BDRV_Curr_FR_Disabled = 32U,
  BDRV_Curr_FR_Min =  33U,
  BDRV_Curr_FR_20mA = 43U,
  BDRV_Curr_FR_30mA = 35U,
  BDRV_Curr_FR_40mA = 36U,
  BDRV_Curr_FR_50mA = 37U,
  BDRV_Curr_FR_60mA = 38U,
  BDRV_Curr_FR_70mA = 39U,
  BDRV_Curr_FR_80mA = 40U,
  BDRV_Curr_FR_90mA = 41U,
  BDRV_Curr_FR_100mA = 42U,
  BDRV_Curr_FR_110mA = 43U,
  BDRV_Curr_FR_120mA = 44U,
  BDRV_Curr_FR_130mA = 45U,
  BDRV_Curr_FR_140mA = 46U,
  BDRV_Curr_FR_150mA = 47U,
  BDRV_Curr_FR_160mA = 48U,
  BDRV_Curr_FR_170mA = 49U,
  BDRV_Curr_FR_180mA = 50U,
  BDRV_Curr_FR_190mA = 51U,
  BDRV_Curr_FR_200mA = 52U,
  BDRV_Curr_FR_210mA = 53U,
  BDRV_Curr_FR_220mA = 54U,
  BDRV_Curr_FR_230mA = 55U,
  BDRV_Curr_FR_240mA = 56U,
  BDRV_Curr_FR_250mA = 57U,
  BDRV_Curr_FR_260mA = 58U,
  BDRV_Curr_FR_270mA = 59U,
  BDRV_Curr_FR_280mA = 60U,
  BDRV_Curr_FR_290mA = 61U,
  BDRV_Curr_FR_300mA = 62U,
  BDRV_Curr_FR_310mA = 63U
}TBDRV_Curr;

#if(UC_SERIES == TLE987)
  #define BDRV_ISCLR_OC    (LS1_OC | HS1_OC | LS2_OC | HS2_OC | LS3_OC | HS3_OC)
  #define BDRV_ISCLR_DS    (LS1_DS | HS1_DS | LS2_DS | HS2_DS | LS3_DS | HS3_DS)
#else
  #define BDRV_ISCLR_OC    (LS1_OC | HS1_OC | LS2_OC | HS2_OC)
  #define BDRV_ISCLR_DS    (LS1_DS | HS1_DS | LS2_DS | HS2_DS)
#endif
#define BDRV_IRQ_BITS     (BDRV_ISCLR_OC | BDRV_ISCLR_DS)
#define BDRV_DS_STS_BITS   BDRV_ISCLR_DS

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
extern void           BDRV_Init                       (void);
extern void           BDRV_Set_Bridge                 (TBdrv_Ch_Cfg LS1_Cfg, TBdrv_Ch_Cfg HS1_Cfg,TBdrv_Ch_Cfg LS2_Cfg, TBdrv_Ch_Cfg HS2_Cfg, TBdrv_Ch_Cfg LS3_Cfg,TBdrv_Ch_Cfg HS3_Cfg);
extern void           BDRV_Set_Bridge                 (TBdrv_Ch_Cfg LS1_Cfg, TBdrv_Ch_Cfg HS1_Cfg,TBdrv_Ch_Cfg LS2_Cfg, TBdrv_Ch_Cfg HS2_Cfg);
extern void           BDRV_Clr_Sts                    (uint32 Sts_Bit);
extern void           BDRV_Set_Channel                (TBdrv_Ch BDRV_Ch, TBdrv_Ch_Cfg Ch_Cfg);
extern void           BDRV_Set_Int_Channel            (TBdrv_Ch BDRV_Ch, TBdrv_Ch_Int Ch_Int);
extern void           BDRV_Set_DSM_Threshold          (TBdrv_DSM_Threshold BDRV_Threshold);
extern TBDRV_Curr     BDRV_getChrgCurrent_mA          (void);
extern void           BDRV_setChrgCurrent_mA          (TBDRV_Curr BDRV_Current);
extern TBDRV_Curr     BDRV_getDischrgCurrent_mA       (void);
extern void           BDRV_setDischrgCurrent_mA       (TBDRV_Curr BDRV_Current);
extern bool           BDRV_Diag_OpenLoad              (void);
extern bool           BDRV_Diag_OpenLoad              (void);
extern TBDRV_Off_Diag BDRV_Off_Diagnosis              (void);
extern TBDRV_Off_Diag BDRV_Off_Diagnosis              (void);
extern void           BDRV_HS1_OC_Int_Clr             (void);
extern void           BDRV_LS1_OC_Int_Clr             (void);
extern void           BDRV_HS2_OC_Int_Clr             (void);
extern void           BDRV_LS2_OC_Int_Clr             (void);
extern void           BDRV_HS3_OC_Int_Clr             (void);
extern void           BDRV_LS3_OC_Int_Clr             (void);
extern void           BDRV_HS1_DS_Int_Clr             (void);
extern void           BDRV_LS1_DS_Int_Clr             (void);
extern void           BDRV_HS2_DS_Int_Clr             (void);
extern void           BDRV_LS2_DS_Int_Clr             (void);
extern void           BDRV_HS3_DS_Int_Clr             (void);
extern void           BDRV_LS3_DS_Int_Clr             (void);
extern void           BDRV_VCP_LO_Int_Clr             (void);
extern void           BDRV_HS1_OC_Int_En              (void);
extern void           BDRV_HS1_OC_Int_Dis             (void);
extern void           BDRV_LS1_OC_Int_En              (void);
extern void           BDRV_LS1_OC_Int_Dis             (void);
extern void           BDRV_HS2_OC_Int_En              (void);
extern void           BDRV_HS2_OC_Int_Dis             (void);
extern void           BDRV_LS2_OC_Int_En              (void);
extern void           BDRV_LS2_OC_Int_Dis             (void);
extern void           BDRV_HS3_OC_Int_En              (void);
extern void           BDRV_HS3_OC_Int_Dis             (void);
extern void           BDRV_LS3_OC_Int_En              (void);
extern void           BDRV_LS3_OC_Int_Dis             (void);
extern void           BDRV_HS1_DS_Int_En              (void);
extern void           BDRV_HS1_DS_Int_Dis             (void);
extern void           BDRV_LS1_DS_Int_En              (void);
extern void           BDRV_LS1_DS_Int_Dis             (void);
extern void           BDRV_HS2_DS_Int_En              (void);
extern void           BDRV_HS2_DS_Int_Dis             (void);
extern void           BDRV_LS2_DS_Int_En              (void);
extern void           BDRV_LS2_DS_Int_Dis             (void);
extern void           BDRV_HS3_DS_Int_En              (void);
extern void           BDRV_HS3_DS_Int_Dis             (void);
extern void           BDRV_LS3_DS_Int_En              (void);
extern void           BDRV_LS3_DS_Int_Dis             (void);
extern void           BDRV_VCP_LO_Int_En              (void);
extern void           BDRV_VCP_LO_Int_Dis             (void);
extern uint8          BDRV_HS1_OC_Int_Sts             (void);
extern uint8          BDRV_LS1_OC_Int_Sts             (void);
extern uint8          BDRV_HS2_OC_Int_Sts             (void);
extern uint8          BDRV_LS2_OC_Int_Sts             (void);
extern uint8          BDRV_HS3_OC_Int_Sts             (void);
extern uint8          BDRV_LS3_OC_Int_Sts             (void);
extern uint8          BDRV_HS1_DS_Int_Sts             (void);
extern uint8          BDRV_LS1_DS_Int_Sts             (void);
extern uint8          BDRV_HS2_DS_Int_Sts             (void);
extern uint8          BDRV_LS2_DS_Int_Sts             (void);
extern uint8          BDRV_HS3_DS_Int_Sts             (void);
extern uint8          BDRV_LS3_DS_Int_Sts             (void);
extern uint8          BDRV_VCP_LO_Int_Sts             (void);
extern uint8          BDRV_getChrgCurrent_dig         (void);
extern void           BDRV_setChrgCurrent_dig         (uint8 u8_cur_dig);
extern uint8          BDRV_getChrgCurrentRange_dig    (void);
extern void           BDRV_setChrgCurrentRange_dig    (uint8 u8_cur_range_dig);
extern uint8          BDRV_getDischrgCurrent_dig      (void);
extern void           BDRV_setDischrgCurrent_dig      (uint8 u8_cur_dig);
extern uint8          BDRV_getDischrgCurrentRange_dig (void);
extern void           BDRV_setDischrgCurrentRange_dig (uint8 u8_cur_range_dig);

/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/
