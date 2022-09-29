/******************************************************************************/
/* File   : Template.hpp                                                      */
/* Author : NAGARAJA HM (c) since 1982. All rights reserved.                  */
/******************************************************************************/

/******************************************************************************/
/* #INCLUDES                                                                  */
/******************************************************************************/
#include "types.hpp"

#include "bdrv.hpp"

#include "tle987x.hpp"
#include "sfr_access.hpp"
#include "cmsis_misra.hpp"
#include "RTE_Components.hpp"
#include "scu.hpp"

#include "bdrv_defines.hpp"

/******************************************************************************/
/* #DEFINES                                                                   */
/******************************************************************************/
#define SCUPM_BDRV_ISCLR_Pos (0UL)
#define SCUPM_BDRV_ISCLR_Msk (0xFFFFFFFFUL)
#define BDRV_CTRL1_LS1_Cfg_Pos BDRV_CTRL1_LS1_EN_Pos
#define BDRV_CTRL1_LS2_Cfg_Pos BDRV_CTRL1_LS2_EN_Pos
#define BDRV_CTRL1_HS1_Cfg_Pos BDRV_CTRL1_HS1_EN_Pos
#define BDRV_CTRL1_HS2_Cfg_Pos BDRV_CTRL1_HS2_EN_Pos
#if(UC_SERIES == TLE987)
  #define BDRV_CTRL2_LS3_Cfg_Pos BDRV_CTRL2_LS3_EN_Pos
  #define BDRV_CTRL2_HS3_Cfg_Pos BDRV_CTRL2_HS3_EN_Pos
#endif

#define BDRV_CTRL1_LS1_Cfg_Msk (BDRV_CTRL1_LS1_EN_Msk | BDRV_CTRL1_LS1_PWM_Msk | BDRV_CTRL1_LS1_ON_Msk | BDRV_CTRL1_LS1_DCS_EN_Msk)
#define BDRV_CTRL1_LS2_Cfg_Msk (BDRV_CTRL1_LS2_EN_Msk | BDRV_CTRL1_LS2_PWM_Msk | BDRV_CTRL1_LS2_ON_Msk | BDRV_CTRL1_LS2_DCS_EN_Msk)
#define BDRV_CTRL1_HS1_Cfg_Msk (BDRV_CTRL1_HS1_EN_Msk | BDRV_CTRL1_HS1_PWM_Msk | BDRV_CTRL1_HS1_ON_Msk | BDRV_CTRL1_HS1_DCS_EN_Msk)
#define BDRV_CTRL1_HS2_Cfg_Msk (BDRV_CTRL1_HS2_EN_Msk | BDRV_CTRL1_HS2_PWM_Msk | BDRV_CTRL1_HS2_ON_Msk | BDRV_CTRL1_HS2_DCS_EN_Msk)
#if(UC_SERIES == TLE987)
  #define BDRV_CTRL2_LS3_Cfg_Msk (BDRV_CTRL2_LS3_EN_Msk | BDRV_CTRL2_LS3_PWM_Msk | BDRV_CTRL2_LS3_ON_Msk | BDRV_CTRL2_LS3_DCS_EN_Msk)
  #define BDRV_CTRL2_HS3_Cfg_Msk (BDRV_CTRL2_HS3_EN_Msk | BDRV_CTRL2_HS3_PWM_Msk | BDRV_CTRL2_HS3_ON_Msk | BDRV_CTRL2_HS3_DCS_EN_Msk)
#endif

/******************************************************************************/
/* MACROS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* TYPEDEFS                                                                   */
/******************************************************************************/

/******************************************************************************/
/* CONSTS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* PARAMS                                                                     */
/******************************************************************************/

/******************************************************************************/
/* OBJECTS                                                                    */
/******************************************************************************/
BDRV_Type BDRV = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/******************************************************************************/
/* FUNCTIONS                                                                  */
/******************************************************************************/
void BDRV_Init(void){
   BDRV.CP_CLK_CTRL.reg = (uint32) BDRV_CP_CLK_CTRL;
   BDRV.CP_CTRL_STS.reg = (uint32) BDRV_CP_CTRL_STS;
   CMSIS_Irq_Dis();
   SCU_OpenPASSWD();
   BDRV.TRIM_DRVx.reg = (uint32) BDRV_TRIM_DRVx;
   SCU_ClosePASSWD();
   CMSIS_Irq_En();
   BDRV.CTRL1.reg = (uint32) BDRV_CTRL1;
   BDRV.CTRL2.reg = (uint32) BDRV_CTRL2;
   BDRV.CTRL3.reg = (uint32) BDRV_CTRL3;
#if(CONFIGWIZARD == 1)
   BDRV.OFF_SEQ_CTRL.reg = (uint32) BDRV_OFF_SEQ;
   BDRV.ON_SEQ_CTRL.reg = (uint32) BDRV_ON_SEQ;
#else
   BDRV.OFF_SEQ_CTRL.reg = (uint32) BDRV_OFF_SEQ_CTRL;
   BDRV.ON_SEQ_CTRL.reg = (uint32) BDRV_ON_SEQ_CTRL;
#endif
#if(UC_SERIES == TLE987)
   MF.BEMFC_CTRL_STS.reg = (uint32) MF_BEMFC_CTRL_STS;
   MF.TRIM_BEMFx.reg = (uint32) MF_TRIM_BEMFx;
#endif
}

#if(UC_SERIES == TLE987)
void BDRV_Set_Bridge(
      TBdrv_Ch_Cfg LS1_Cfg
   ,  TBdrv_Ch_Cfg HS1_Cfg
   ,  TBdrv_Ch_Cfg LS2_Cfg
   ,  TBdrv_Ch_Cfg HS2_Cfg
   ,  TBdrv_Ch_Cfg LS3_Cfg
   ,  TBdrv_Ch_Cfg HS3_Cfg
){
  Field_Mod32(&BDRV.CTRL1.reg, BDRV_CTRL1_LS1_Cfg_Pos, BDRV_CTRL1_LS1_Cfg_Msk, (uint32)LS1_Cfg);
  Field_Mod32(&BDRV.CTRL1.reg, BDRV_CTRL1_HS1_Cfg_Pos, BDRV_CTRL1_HS1_Cfg_Msk, (uint32)HS1_Cfg);
  Field_Mod32(&BDRV.CTRL1.reg, BDRV_CTRL1_LS2_Cfg_Pos, BDRV_CTRL1_LS2_Cfg_Msk, (uint32)LS2_Cfg);
  Field_Mod32(&BDRV.CTRL1.reg, BDRV_CTRL1_HS2_Cfg_Pos, BDRV_CTRL1_HS2_Cfg_Msk, (uint32)HS2_Cfg);
  Field_Mod32(&BDRV.CTRL2.reg, BDRV_CTRL2_LS3_Cfg_Pos, BDRV_CTRL2_LS3_Cfg_Msk, (uint32)LS3_Cfg);
  Field_Mod32(&BDRV.CTRL2.reg, BDRV_CTRL2_HS3_Cfg_Pos, BDRV_CTRL2_HS3_Cfg_Msk, (uint32)HS3_Cfg);
}
#else
#endif

void BDRV_Clr_Sts(uint32 Sts_Bit){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_Pos, SCUPM_BDRV_ISCLR_Msk, Sts_Bit);
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_Pos, SCUPM_BDRV_ISCLR_Msk, Sts_Bit);
}
/*
void BDRV_Set_Channel(TBdrv_Ch BDRV_Ch, TBdrv_Ch_Cfg Ch_Cfg){
#if(UC_SERIES == TLE987)
  if(BDRV_Ch < LS3){
    Field_Mod32(&BDRV.CTRL1.reg, ((uint32)BDRV_Ch << 3u), (uint32)0x0F << ((uint32)BDRV_Ch << 3u), (uint32)Ch_Cfg);
  }
  else{
    Field_Mod32(&BDRV.CTRL2.reg, (((uint32)BDRV_Ch - 4u) << 3u), (uint32)0x0F << (((uint32)BDRV_Ch - 4u) << 3u), (uint32)Ch_Cfg);
  }

#else
  Field_Mod32(&BDRV.CTRL1.reg, ((uint32)BDRV_Ch << 3u), (uint32)0x0F << ((uint32)BDRV_Ch << 3u), (uint32)Ch_Cfg);
#endif
}

void BDRV_Set_Int_Channel(TBdrv_Ch BDRV_Ch, TBdrv_Ch_Int Ch_Int){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, (uint32)BDRV_Ch, ((uint32)SCUPM_BDRV_IRQ_CTRL_LS1_DS_IE_Msk << (uint32)BDRV_Ch), (uint32)Ch_Int);
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, (uint32)BDRV_Ch + SCUPM_BDRV_IRQ_CTRL_LS1_OC_IE_Pos, (SCUPM_BDRV_IRQ_CTRL_LS1_OC_IE_Msk << (uint32)BDRV_Ch), (uint32)Ch_Int >> 1u);
  if((SCUPM.BDRV_IRQ_CTRL.reg & (uint32)BDRV_IRQ_BITS) != 0u){
    NVIC_Node14_En();
  }
}

void BDRV_Set_DSM_Threshold(TBdrv_DSM_Threshold BDRV_Threshold){
  Field_Mod32(&BDRV.CTRL3.reg, BDRV_CTRL3_DSMONVTH_Pos, BDRV_CTRL3_DSMONVTH_Msk, (uint32)BDRV_Threshold);
}

TBDRV_Curr BDRV_getChrgCurrent_mA(void){
  uint8 u8_range;
  uint8 u8_current;
  u8_range = BDRV.CTRL3.bit.ICHARGEDIV2_N;
  u8_current = BDRV.CTRL3.bit.ICHARGE_TRIM;
   return(TBDRV_Curr)((u8_range << TBDRV_Curr_RANGE_POS) | u8_current);
}

void BDRV_setChrgCurrent_mA(TBDRV_Curr BDRV_Current){
  uint8 u8_range;
  uint8 u8_current;
  u8_range = (uint8)BDRV_Current >> TBDRV_Curr_RANGE_POS;
  u8_current = (uint8)BDRV_Current & TBDRV_Curr_CURRENT_MASK;
  BDRV.CTRL3.bit.ICHARGEDIV2_N = u8_range;
  BDRV.CTRL3.bit.ICHARGE_TRIM = u8_current;
}

TBDRV_Curr BDRV_getDischrgCurrent_mA(void){
  uint8 u8_range;
  uint8 u8_current;
  u8_range = BDRV.CTRL3.bit.IDISCHARGEDIV2_N;
  u8_current = BDRV.CTRL3.bit.IDISCHARGE_TRIM;
   return(TBDRV_Curr)((u8_range << TBDRV_Curr_RANGE_POS) | u8_current);
}

void BDRV_setDischrgCurrent_mA(TBDRV_Curr BDRV_Current){
  uint8 u8_range;
  uint8 u8_current;
  u8_range = (uint8)BDRV_Current >> TBDRV_Curr_RANGE_POS;
  u8_current = (uint8)BDRV_Current & TBDRV_Curr_CURRENT_MASK;
  BDRV.CTRL3.bit.IDISCHARGEDIV2_N = u8_range;
  BDRV.CTRL3.bit.IDISCHARGE_TRIM = u8_current;
}

#if(UC_SERIES == TLE986)
bool BDRV_Diag_OpenLoad(void){
  uint8 iBDRV_Int_En;
  bool bOpenLoad;
  uint32 lCTRL3;
  bOpenLoad = false;
  iBDRV_Int_En = CPU.NVIC_ISER0.bit.Int_BDRV;
  NVIC_Node14_Dis();
  lCTRL3 = BDRV.CTRL3.reg;
  BDRV_Set_DSM_Threshold(Threshold_2_00_V);
  BDRV.CTRL3.bit.IDISCHARGEDIV2_N = 0;
  BDRV_Set_Discharge_Current(Current_Min);
  BDRV_Set_Channel(LS1, Ch_En);
  BDRV_Set_Channel(LS2, Ch_En);
  BDRV_Set_Channel(HS1, Ch_En);
  BDRV_Set_Channel(HS2, Ch_En);
  BDRV_Set_Channel(HS1, Ch_DCS);
  Delay_us(800u);
  BDRV_Clr_Sts(BDRV_ISCLR_DS);
  if((SCUPM.BDRV_IS.reg & (uint32)BDRV_IRQ_BITS) != 0u){
    bOpenLoad = true;
  }
  BDRV.CTRL1.reg = (uint32) 0;
  BDRV.CTRL3.reg = lCTRL3;
  if(iBDRV_Int_En == (uint8)1){
    NVIC_Node14_En();
  }
   return(bOpenLoad);
}
#endif

#if(UC_SERIES == TLE987)
bool BDRV_Diag_OpenLoad(void){
  uint8 bBDRV_Int_En;
  bool bOpenLoad;
  uint32 lCTRL3;
  bOpenLoad = false;
  bBDRV_Int_En = CPU.NVIC_ISER0.bit.Int_BDRV;
  NVIC_Node14_Dis();
  lCTRL3 = BDRV.CTRL3.reg;
  BDRV_Set_DSM_Threshold(Threshold_2_00_V);
  BDRV_setDischrgCurrent_mA(BDRV_Curr_HR_Min);
  BDRV_Set_Bridge(Ch_En, Ch_En, Ch_En, Ch_DCS, Ch_En, Ch_DCS);
  Delay_us(800u);
  Delay_us(800u);
  BDRV_Clr_Sts(BDRV_ISCLR_DS);
  if(BDRV_HS1_DS_Int_Sts() == 1u){
    bOpenLoad = true;
  }
  BDRV_Set_Bridge(Ch_En, Ch_DCS, Ch_En, Ch_En, Ch_En, Ch_DCS);
  Delay_us(800u);
  Delay_us(800u);
  BDRV_Clr_Sts(BDRV_ISCLR_DS);
  if(BDRV_HS2_DS_Int_Sts() == 1u){
    bOpenLoad = true;
  }
  BDRV_Set_Bridge(Ch_En, Ch_DCS, Ch_En, Ch_DCS, Ch_En, Ch_En);
  Delay_us(800u);
  Delay_us(800u);
  BDRV_Clr_Sts(BDRV_ISCLR_DS);
  if(BDRV_HS3_DS_Int_Sts() == 1u){
    bOpenLoad = true;
  }
  BDRV.CTRL1.reg = (uint32)0;
  BDRV.CTRL2.reg = (uint32)0;
  BDRV.CTRL3.reg = lCTRL3;
  if(bBDRV_Int_En == (uint8)1){
    NVIC_Node14_En();
  }
   return(bOpenLoad);
}
#endif

#if(UC_SERIES == TLE986)
TBDRV_Off_Diag BDRV_Off_Diagnosis(void){
  TBDRV_Off_Diag res;
  uint8 iBDRV_Int_En;
  uint32 lCTRL3;
  res.GlobFailSts = false;
  res.Phase1 = Ch_Ok;
  res.Phase2 = Ch_Ok;
  iBDRV_Int_En = CPU.NVIC_ISER0.bit.Int_BDRV;
  NVIC_Node14_Dis();
  lCTRL3 = BDRV.CTRL3.reg;
  BDRV_Set_Bridge(Ch_En, Ch_En, Ch_En, Ch_En);
  BDRV_Set_DSM_Threshold(Threshold_0_25_V);
  BDRV.CTRL3.bit.IDISCHARGEDIV2_N = 0;
  BDRV_Set_Discharge_Current(Current_Min);
  BDRV_Set_Bridge(Ch_DCS, Ch_DCS, Ch_DCS, Ch_DCS);
  Delay_us(800u);
  BDRV_Clr_Sts(BDRV_IRQ_BITS);
  if((SCUPM.BDRV_IS.reg & (uint32)BDRV_DS_STS_BITS) != 0u){
    res.GlobFailSts = true;
    if(BDRV_HS1_DS_Int_Sts() == 1u){
      res.Phase1 = Ch_Short_to_VBat;
    }
    if(BDRV_LS1_DS_Int_Sts() == 1u){
      res.Phase1 = Ch_Short_to_Gnd;
    }
    if(BDRV_HS2_DS_Int_Sts() == 1u){
      res.Phase2 = Ch_Short_to_VBat;
    }
    if(BDRV_LS2_DS_Int_Sts() == 1u){
      res.Phase2 = Ch_Short_to_Gnd;
    }
  }
  BDRV.CTRL1.reg = (uint32) 0;
  BDRV.CTRL3.reg = lCTRL3;
  if(iBDRV_Int_En == (uint8)1){
    NVIC_Node14_En();
  }
   return(res);
}
#endif

#if(UC_SERIES == TLE987)
TBDRV_Off_Diag BDRV_Off_Diagnosis(void){
  TBDRV_Off_Diag res;
  uint8 iBDRV_Int_En;
  uint32 lCTRL3;
  res.GlobFailSts = false;
  res.Phase1 = Ch_Ok;
  res.Phase2 = Ch_Ok;
  res.Phase3 = Ch_Ok;
  iBDRV_Int_En = CPU.NVIC_ISER0.bit.Int_BDRV;
  NVIC_Node14_Dis();
  lCTRL3 = BDRV.CTRL3.reg;
  BDRV_Set_Bridge(Ch_En, Ch_En, Ch_En, Ch_En, Ch_En, Ch_En);
  BDRV_Set_DSM_Threshold(Threshold_0_25_V);
  BDRV_setDischrgCurrent_mA(BDRV_Curr_HR_Min);
  BDRV_Set_Bridge(Ch_DCS, Ch_DCS, Ch_DCS, Ch_DCS, Ch_DCS, Ch_DCS);
  Delay_us(800u);
  BDRV_Clr_Sts(BDRV_DS_STS_BITS);
  if((SCUPM.BDRV_IS.reg & (uint32)BDRV_DS_STS_BITS) != 0u){
    res.GlobFailSts = true;
    if(BDRV_HS1_DS_Int_Sts() == 1u){
      res.Phase1 = Ch_Short_to_VBat;
    }
    if(BDRV_LS1_DS_Int_Sts() == 1u){
      res.Phase1 = Ch_Short_to_Gnd;
    }
    if(BDRV_HS2_DS_Int_Sts() == 1u){
      res.Phase2 = Ch_Short_to_VBat;
    }
    if(BDRV_LS2_DS_Int_Sts() == 1u){
      res.Phase2 = Ch_Short_to_Gnd;
    }
    if(BDRV_HS3_DS_Int_Sts() == 1u){
      res.Phase3 = Ch_Short_to_VBat;
    }
    if(BDRV_LS3_DS_Int_Sts() == 1u){
      res.Phase3 = Ch_Short_to_Gnd;
    }
  }
  BDRV.CTRL1.reg = (uint32) 0;
  BDRV.CTRL3.reg = lCTRL3;
  if(iBDRV_Int_En == (uint8)1){
    NVIC_Node14_En();
  }
   return(res);
}
#endif

void BDRV_HS1_OC_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_HS1_OC_ICLR_Pos, SCUPM_BDRV_ISCLR_HS1_OC_ICLR_Msk, 1u);
}

void BDRV_LS1_OC_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_LS1_OC_ICLR_Pos, SCUPM_BDRV_ISCLR_LS1_OC_ICLR_Msk, 1u);
}

void BDRV_HS2_OC_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_HS2_OC_ICLR_Pos, SCUPM_BDRV_ISCLR_HS2_OC_ICLR_Msk, 1u);
}

void BDRV_LS2_OC_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_LS2_OC_ICLR_Pos, SCUPM_BDRV_ISCLR_LS2_OC_ICLR_Msk, 1u);
}

#if(UC_SERIES == TLE987)
void BDRV_HS3_OC_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_HS3_OC_ICLR_Pos, SCUPM_BDRV_ISCLR_HS3_OC_ICLR_Msk, 1u);
}
#endif

#if(UC_SERIES == TLE987)
void BDRV_LS3_OC_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_LS3_OC_ICLR_Pos, SCUPM_BDRV_ISCLR_LS3_OC_ICLR_Msk, 1u);
}
#endif

void BDRV_HS1_DS_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_HS1_DS_ICLR_Pos, SCUPM_BDRV_ISCLR_HS1_DS_ICLR_Msk, 1u);
}

void BDRV_LS1_DS_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_LS1_DS_ICLR_Pos, SCUPM_BDRV_ISCLR_LS1_DS_ICLR_Msk, 1u);
}

void BDRV_HS2_DS_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_HS2_DS_ICLR_Pos, SCUPM_BDRV_ISCLR_HS2_DS_ICLR_Msk, 1u);
}

void BDRV_LS2_DS_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_LS2_DS_ICLR_Pos, SCUPM_BDRV_ISCLR_LS2_DS_ICLR_Msk, 1u);
}

#if(UC_SERIES == TLE987)
void BDRV_HS3_DS_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_HS3_DS_ICLR_Pos, SCUPM_BDRV_ISCLR_HS3_DS_ICLR_Msk, 1u);
}
#endif

#if(UC_SERIES == TLE987)
void BDRV_LS3_DS_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_LS3_DS_ICLR_Pos, SCUPM_BDRV_ISCLR_LS3_DS_ICLR_Msk, 1u);
}
#endif

void BDRV_VCP_LO_Int_Clr(void){
  Field_Wrt32(&SCUPM.BDRV_ISCLR.reg, SCUPM_BDRV_ISCLR_VCP_LOWTH2_ICLR_Pos, SCUPM_BDRV_ISCLR_VCP_LOWTH2_ICLR_Msk, 1u);
}

void BDRV_HS1_OC_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS1_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS1_OC_IE_Msk, 1u);
}

void BDRV_HS1_OC_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS1_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS1_OC_IE_Msk, 0u);
}

void BDRV_LS1_OC_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS1_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS1_OC_IE_Msk, 1u);
}

void BDRV_LS1_OC_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS1_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS1_OC_IE_Msk, 0u);
}

void BDRV_HS2_OC_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS2_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS2_OC_IE_Msk, 1u);
}

void BDRV_HS2_OC_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS2_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS2_OC_IE_Msk, 0u);
}

void BDRV_LS2_OC_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS2_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS2_OC_IE_Msk, 1u);
}

void BDRV_LS2_OC_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS2_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS2_OC_IE_Msk, 0u);
}

#if(UC_SERIES == TLE987)
void BDRV_HS3_OC_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS3_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS3_OC_IE_Msk, 1u);
}
#endif

#if(UC_SERIES == TLE987)
void BDRV_HS3_OC_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS3_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS3_OC_IE_Msk, 0u);
}
#endif

#if(UC_SERIES == TLE987)
void BDRV_LS3_OC_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS3_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS3_OC_IE_Msk, 1u);
}
#endif

#if(UC_SERIES == TLE987)
void BDRV_LS3_OC_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS3_OC_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS3_OC_IE_Msk, 0u);
}
#endif

void BDRV_HS1_DS_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS1_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS1_DS_IE_Msk, 1u);
}

void BDRV_HS1_DS_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS1_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS1_DS_IE_Msk, 0u);
}

void BDRV_LS1_DS_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS1_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS1_DS_IE_Msk, 1u);
}

void BDRV_LS1_DS_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS1_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS1_DS_IE_Msk, 0u);
}

void BDRV_HS2_DS_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS2_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS2_DS_IE_Msk, 1u);
}

void BDRV_HS2_DS_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS2_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS2_DS_IE_Msk, 0u);
}

void BDRV_LS2_DS_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS2_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS2_DS_IE_Msk, 1u);
}

void BDRV_LS2_DS_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS2_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS2_DS_IE_Msk, 0u);
}

#if(UC_SERIES == TLE987)
void BDRV_HS3_DS_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS3_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS3_DS_IE_Msk, 1u);
}
#endif

#if(UC_SERIES == TLE987)
void BDRV_HS3_DS_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_HS3_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_HS3_DS_IE_Msk, 0u);
}
#endif

#if(UC_SERIES == TLE987)
void BDRV_LS3_DS_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS3_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS3_DS_IE_Msk, 1u);
}
#endif

#if(UC_SERIES == TLE987)
void BDRV_LS3_DS_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_LS3_DS_IE_Pos, SCUPM_BDRV_IRQ_CTRL_LS3_DS_IE_Msk, 0u);
}
#endif

void BDRV_VCP_LO_Int_En(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_VCP_LOWTH2_IE_Pos, SCUPM_BDRV_IRQ_CTRL_VCP_LOWTH2_IE_Msk, 1u);
}

void BDRV_VCP_LO_Int_Dis(void){
  Field_Mod32(&SCUPM.BDRV_IRQ_CTRL.reg, SCUPM_BDRV_IRQ_CTRL_VCP_LOWTH2_IE_Pos, SCUPM_BDRV_IRQ_CTRL_VCP_LOWTH2_IE_Msk, 0u);
}

uint8 BDRV_HS1_OC_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_HS1_OC_IS_Pos, SCUPM_BDRV_IS_HS1_OC_IS_Msk);
}

uint8 BDRV_LS1_OC_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_LS1_OC_IS_Pos, SCUPM_BDRV_IS_LS1_OC_IS_Msk);
}

uint8 BDRV_HS2_OC_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_HS2_OC_IS_Pos, SCUPM_BDRV_IS_HS2_OC_IS_Msk);
}

uint8 BDRV_LS2_OC_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_LS2_OC_IS_Pos, SCUPM_BDRV_IS_LS2_OC_IS_Msk);
}

#if(UC_SERIES == TLE987)
uint8 BDRV_HS3_OC_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_HS3_OC_IS_Pos, SCUPM_BDRV_IS_HS3_OC_IS_Msk);
}
#endif

#if(UC_SERIES == TLE987)
uint8 BDRV_LS3_OC_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_LS3_OC_IS_Pos, SCUPM_BDRV_IS_LS3_OC_IS_Msk);
}
#endif

uint8 BDRV_HS1_DS_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_HS1_DS_IS_Pos, SCUPM_BDRV_IS_HS1_DS_IS_Msk);
}

uint8 BDRV_LS1_DS_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_LS1_DS_IS_Pos, SCUPM_BDRV_IS_LS1_DS_IS_Msk);
}

uint8 BDRV_HS2_DS_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_HS2_DS_IS_Pos, SCUPM_BDRV_IS_HS2_DS_IS_Msk);
}

uint8 BDRV_LS2_DS_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_LS2_DS_IS_Pos, SCUPM_BDRV_IS_LS2_DS_IS_Msk);
}

#if(UC_SERIES == TLE987)
uint8 BDRV_HS3_DS_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_HS3_DS_IS_Pos, SCUPM_BDRV_IS_HS3_DS_IS_Msk);
}
#endif

#if(UC_SERIES == TLE987)
uint8 BDRV_LS3_DS_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_LS3_DS_IS_Pos, SCUPM_BDRV_IS_LS3_DS_IS_Msk);
}
#endif

uint8 BDRV_VCP_LO_Int_Sts(void){
   return u1_Field_Rd32(&SCUPM.BDRV_IS.reg, SCUPM_BDRV_IS_VCP_LOWTH2_IS_Pos, SCUPM_BDRV_IS_VCP_LOWTH2_IS_Msk);
}

uint8 BDRV_getChrgCurrent_dig(void){
   return BDRV.CTRL3.bit.ICHARGE_TRIM;
}

void BDRV_setChrgCurrent_dig(uint8 u8_cur_dig){
  BDRV.CTRL3.bit.ICHARGE_TRIM = u8_cur_dig;
}

uint8 BDRV_getChrgCurrentRange_dig(void){
   return BDRV.CTRL3.bit.ICHARGEDIV2_N;
}

void BDRV_setChrgCurrentRange_dig(uint8 u8_cur_range_dig){
  BDRV.CTRL3.bit.ICHARGEDIV2_N = u8_cur_range_dig;
}

uint8 BDRV_getDischrgCurrent_dig(void){
   return BDRV.CTRL3.bit.IDISCHARGE_TRIM;
}

void BDRV_setDischrgCurrent_dig(uint8 u8_cur_dig){
  BDRV.CTRL3.bit.IDISCHARGE_TRIM = u8_cur_dig;
}

uint8 BDRV_getDischrgCurrentRange_dig(void){
   return BDRV.CTRL3.bit.IDISCHARGEDIV2_N;
}

void BDRV_setDischrgCurrentRange_dig(uint8 u8_cur_range_dig){
  BDRV.CTRL3.bit.IDISCHARGEDIV2_N = u8_cur_range_dig;
}
*/
/******************************************************************************/
/* EOF                                                                        */
/******************************************************************************/

