#include "WiFi.h"

extern "C" {
#include "inc/hw_types.h"
#include "utility/cc_io_park.h"
}

#define LSI_SLEEP_DURATION_IN_MSEC (2000)

struct soc_io_park cc32xx_io_park[] = {
        {PIN_01, "GPIO_10", WEAK_PULL_DOWN_STD},
        {PIN_02, "GPIO_11", WEAK_PULL_DOWN_STD},
        {PIN_03, "GPIO_12", WEAK_PULL_DOWN_STD},
        {PIN_04, "GPIO_13", WEAK_PULL_DOWN_STD},
        {PIN_05, "GPIO_14", WEAK_PULL_DOWN_STD},
        {PIN_06, "GPIO_15", WEAK_PULL_DOWN_STD},
        {PIN_07, "GPIO_16", WEAK_PULL_DOWN_STD},
        {PIN_08, "GPIO_17", WEAK_PULL_DOWN_STD},
        {PIN_15, "GPIO_22", WEAK_PULL_DOWN_STD},
        {PIN_16, "GPIO_23/JTAG_TDI", WEAK_PULL_DOWN_STD},
        {PIN_17, "GPIO_24/JTAG_TDO", WEAK_PULL_DOWN_STD},
        {PIN_18, "GPIO_28", WEAK_PULL_DOWN_STD},
        {PIN_19, "GPIO_28//JTAG_TCK", WEAK_PULL_DOWN_STD},
        {PIN_20, "GPIO_29/JTAG_TMS", WEAK_PULL_DOWN_STD},
        {PIN_21, "GPIO_25/SOP2", WEAK_PULL_DOWN_STD},
        {PIN_45, "DCDC_FLASH_SW_P", WEAK_PULL_DOWN_STD},
        {PIN_50, "GPIO_00", WEAK_PULL_DOWN_STD},
        {PIN_52, "GPIO_32", WEAK_PULL_DOWN_STD},
        {PIN_53, "GPIO_30", WEAK_PULL_DOWN_STD},
        {PIN_55, "GPIO_01", WEAK_PULL_DOWN_STD},
        {PIN_57, "GPIO_02", WEAK_PULL_DOWN_STD},
        {PIN_58, "GPIO_03", WEAK_PULL_DOWN_STD},
        {PIN_59, "GPIO_04", WEAK_PULL_DOWN_STD},
        {PIN_60, "GPIO_05", WEAK_PULL_DOWN_STD},
        {PIN_61, "GPIO_06", WEAK_PULL_DOWN_STD},
        {PIN_62, "GPIO_07", WEAK_PULL_DOWN_STD},
        {PIN_63, "GPIO_08", WEAK_PULL_DOWN_STD},
        {PIN_64, "GPIO_09", WEAK_PULL_DOWN_STD}
};

//****************************************************************************
//
//! \brief  This function backs up necessary registers and data(before S3)
//!
//! \param  none
//!
//! \return none
//
//****************************************************************************
void lp3p0_back_up_soc_data(void)
{
        /* Park the IO PINs safely to avoid any leakage */
        cc_io_park_safe(cc32xx_io_park, 
                sizeof(cc32xx_io_park)/sizeof(struct soc_io_park));

        /* Park the Antennas selection pins */
        HWREG(0x4402E108) = 0x00000E61;
        HWREG(0x4402E10C) = 0x00000E61;

        return;
}

void lpds_init(void (*restore)(void)) {
  uint8_t ucConfigOpt = 0;

  /* Set power mgmnt policy of NWP */  
  sl_WlanPolicySet(SL_POLICY_PM , SL_LOW_POWER_POLICY, NULL, 0);

  /* disable scan */
  ucConfigOpt = SL_SCAN_POLICY(0);
  sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);

  _u16 PolicyBuff[4] = {0,0,LSI_SLEEP_DURATION_IN_MSEC,0};
  sl_WlanPolicySet(SL_POLICY_PM , SL_LONG_SLEEP_INTERVAL_POLICY, (_u8*)PolicyBuff,sizeof(PolicyBuff));
}
