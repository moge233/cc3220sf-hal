/*
 *
 * Power, Reset, and Clock Management for the CC3220SF MCU
 *
 */

use core::ptr::read_volatile;

use cc3220sf::{COMMON_REG, GPRCM, OCP_SHARED, HIB1P2, HIB3P3};

use crate::sysctl::{control_power, reset_power, Domain, RunMode, PowerState};


#[derive(Clone, Copy, Debug)]
pub enum ResetCause {
    PowerOn = 0,
    LpdsExit = 1,
    CoreReset = 3,
    McuReset = 4,
    WdtReset = 5,
    SocReset = 6,
    HibExit = 7,
}
impl ResetCause {
    pub fn from_u32(value: u32) -> ResetCause {
        match value {
            0 => ResetCause::PowerOn,
            1 => ResetCause::LpdsExit,
            3 => ResetCause::CoreReset,
            4 => ResetCause::McuReset,
            5 => ResetCause::WdtReset,
            6 => ResetCause::SocReset,
            7 => ResetCause::HibExit,
            _ => unreachable!(),
        }
    }
}


macro_rules! delay {
    // Small, rudimentary delay
    ($n:expr) => {
        for _ in 0..$n {
            $n; // Just put a statement here so this does not get optimized away
        }
    }
}


/*****************************************************************
 * PRCM API as defined by the CC3220SF Technical Reference Manual
 *****************************************************************/

pub fn mcu_init() {
    /*
     * Implementation of PRCMC3200MCUInit() from the 
     * CC32xx Simplelink SDK (prcm.c)
     *
     * This is the initialization function ran by the TI CC3220SF 
     * Launchpad when building a C application using the Simplelink SDK.
     *
     */

    let reset_cause = get_reset_cause();

    unsafe {
        let rom_version;
            rom_version = read_volatile(0x00000400 as *mut u32);

        if reset_cause as u32 != ResetCause::LpdsExit as u32 {
            // We are not coming out of low power deep sleep
            if rom_version == 0x00010001 {
                // DIG DCDC NFET COT and SEL mode disable
                (*HIB1P2::ptr()).dig_dcdc_parameters2.write(
                    |w| w.bits(0x30031820)
                );
                (*HIB1P2::ptr()).dig_dcdc_parameters1.write(
                    |w| w.bits(0x04000000)
                );

                // At 80MHz --> roughly 2 instructions per loop? --> 125uS
                delay!(5000);

                // ANA DCDC clock config
                (*HIB1P2::ptr()).mem_ana_dcdc_clk_config.write(
                    |w| w.bits(0x099)
                );
                (*HIB1P2::ptr()).mem_ana_dcdc_clk_config.write(
                    |w| w.bits(0x0AA)
                );
                (*HIB1P2::ptr()).mem_ana_dcdc_clk_config.write(
                    |w| w.bits(0x1AA)
                );

                // PA DCDC clock config
                (*HIB1P2::ptr()).mem_pa_dcdc_clk_config.write(
                    |w| w.bits(0x099)
                );
                (*HIB1P2::ptr()).mem_pa_dcdc_clk_config.write(
                    |w| w.bits(0x0AA)
                );
                (*HIB1P2::ptr()).mem_pa_dcdc_clk_config.write(
                    |w| w.bits(0x1AA)
                );

                // TD Flash timing considerations in case of MCU WDT reset
                if (*GPRCM::ptr()).apps_reset_cause.read().bits() == 0x00000005 {
                    (*COMMON_REG::ptr()).idmem_tim_update.write(
                        |w| w.bits(0x01840082)
                    );
                    (*COMMON_REG::ptr()).idmem_tim_updated.write(
                        |w| w.bits(0x00000001)
                    );
                    (*COMMON_REG::ptr()).idmem_tim_updated.write(
                        |w| w.bits(0x00000000)
                    );
                }

                // Take I2C Semaphore
                (*COMMON_REG::ptr()).i2c_properties_register.modify(
                    |r, w| w.bits((r.bits() & !0x3) | 0x1)
                );
                
                // Take GPIO Semaphore
                (*COMMON_REG::ptr()).gpio_properties_register.modify(
                    |r, w| w.bits((r.bits() & !0x3FF) | 0x155)
                );

                // Enable 32kHz internal RC oscillator
                (*HIB3P3::ptr()).mem_int_osc_conf.write(
                    |w| w.bits(0x00000101)
                );

                delay!(10000); // 250uS delay

                // Enable 16MHz clock
                (*HIB1P2::ptr()).cm_osc_16m_config.write(
                    |w| w.bits(0x00010008)
                );

                delay!(10000); // 250uS delay
            }
            else {
                // DIG DCDC LPDS ECO enable
                (*HIB1P2::ptr()).ana_dcdc_parameters16.modify(
                    |r, w| w.bits(r.bits() | 0x800000)
                );

                // Enable hibernate ECO for PG 1.32 devices only. With this ECO enabled, any hibernate
                // wakeup source will be kept masked until the device enters hibernate completely
                // (analog + digital)
                (*HIB3P3::ptr()).mem_hib_reg0.modify(
                    |r, w| w.bits(r.bits() | (1 << 4))
                );
                delay!(10000); // 250uS delay

                // Handling the clock switching (for 1.32 only)
                (*OCP_SHARED::ptr()).spare_reg_5.modify(
                    |r, w| w.bits(r.bits() | 0x3C)
                );
            }

            // Enable uDMA
            control_power(Domain::MicroDma, RunMode::Run, PowerState::On);
            // Reset uDMA
            reset_power(Domain::MicroDma);
            // Disable uDMA
            control_power(Domain::MicroDma, RunMode::Run, PowerState::Off);

            // Enable RTC
            if get_reset_cause() as u32 == ResetCause::PowerOn as u32 {
                (*HIB3P3::ptr()).mem_hib_rtc_timer_enable.write(
                    |w| w.bits(0x1)
                );
            }

            // SWD Mode
            if ((*HIB1P2::ptr()).sop_sense_value.read().bits() & 0xFF) == 0x2 {
                (*OCP_SHARED::ptr()).gpio_pad_config_28.modify(
                    |r, w| w.bits((r.bits() & !0xC0F) | 0x2)
                );
                (*OCP_SHARED::ptr()).gpio_pad_config_29.modify(
                    |r, w| w.bits((r.bits() & !0xC0F) | 0x2)
                );
            }

            // Override JTAG Mux
            (*OCP_SHARED::ptr()).cc3xx_dev_padconf.modify(
                |r, w| w.bits(r.bits() | 0x2)
            );

            // Change UART pins (55, 57) mode to PIN_MODE_0 if they are in PIN_MODE_3
            if ((*OCP_SHARED::ptr()).gpio_pad_config_1.read().bits() & 0xF) == 0x3 {
                (*OCP_SHARED::ptr()).gpio_pad_config_1.write(
                    |w| w.bits(0x0)
                )
            }
            if ((*OCP_SHARED::ptr()).gpio_pad_config_2.read().bits() & 0xF) == 0x3 {
                (*OCP_SHARED::ptr()).gpio_pad_config_2.write(
                    |w| w.bits(0x0)
                )
            }

            // Change I2C pins (1, 2) mode to PIN_MODE_0 if they are in PIN_MODE_1
            if ((*OCP_SHARED::ptr()).gpio_pad_config_10.read().bits() & 0xF) == 0x1 {
                (*OCP_SHARED::ptr()).gpio_pad_config_10.write(
                    |w| w.bits(0x0)
                )
            }
            if ((*OCP_SHARED::ptr()).gpio_pad_config_11.read().bits() & 0xF) == 0x1 {
                (*OCP_SHARED::ptr()).gpio_pad_config_11.write(
                    |w| w.bits(0x0)
                )
            }

            // DIG DCDC VOUT trim settings based on process indicator
            if (((*GPRCM::ptr()).efuse_read_reg0.read().bits() >> 22) & 0xF) == 0xE {
                (*HIB1P2::ptr()).dig_dcdc_vtrim_cfg.modify(
                    |r, w| w.bits((r.bits() & !0x00FC0000) | (0x32 << 18))
                )
            }
            else {
                (*HIB1P2::ptr()).dig_dcdc_vtrim_cfg.modify(
                    |r, w| w.bits((r.bits() & !0x00FC0000) | (0x29 << 18))
                )
            }

            // Enable SOFT RESTART in case of DIG DCDC collapse
            (*HIB3P3::ptr()).hibana_spare_lowv.modify(
                |r, w| w.bits(r.bits() & !0x10000000)
            );

            // Required only if ROM version is lower than 2.x.x
            if (rom_version & 0xFFFF) < 2 {
                // Disable the sleep for ANA DCDC
                (*HIB1P2::ptr()).ana_dcdc_parameters_override.modify(
                    |r, w| w.bits(r.bits() | 0x00000004)
                );
            }
            else if (rom_version >> 16) >= 1 {
                // Enable NWP force reset and HIB on WDT reset
                // Enable direct boot path for flash
                (*OCP_SHARED::ptr()).spare_reg_8.modify(
                    |r, w| w.bits(r.bits() | ((7 << 5) | 1))
                );
                if ((*HIB3P3::ptr()).mem_hib_reg2.read().bits() & 0x1) == 0x1 {
                    (*HIB3P3::ptr()).mem_hib_reg2.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                    (*OCP_SHARED::ptr()).spare_reg_8.modify(
                        |r, w| w.bits(r.bits() | (1 << 9))
                    );

                    // Clear the RTC hib wake up source
                    (*HIB3P3::ptr()).mem_hib_rtc_wake_en.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );

                    // Reset RTC match value
                    (*HIB3P3::ptr()).mem_hib_rtc_wake_lsw_conf.write(
                        |w| w.bits(0)
                    );
                    (*HIB3P3::ptr()).mem_hib_rtc_wake_msw_conf.write(
                        |w| w.bits(0)
                    );
                }
            }

            // Read the device identification register
            let efuse_reg2 = (*GPRCM::ptr()).efuse_read_reg2.read().bits();

            // Read ROM major and minor version
            let major_ver = (efuse_reg2 >> 28) & 0xF;
            let minor_ver = (efuse_reg2 >> 24) & 0xF;

            if ((major_ver == 3) && (minor_ver == 0)) || (major_ver < 3) {
                let pre_regulated_mode = (((*HIB3P3::ptr()).mem_hib_detection_status.read().bits() >> 6) & 1) != 0;
                if pre_regulated_mode {
                    // TODO: make these read-modify-write's instead of using scratch
                    let mut scratch = (*HIB1P2::ptr()).ana_dcdc_parameters1.read().bits();
                    scratch = scratch & 0xFFFFFF7F;
                    (*HIB1P2::ptr()).ana_dcdc_parameters1.write(
                        |w| w.bits(scratch)
                    );

                    scratch = (*HIB1P2::ptr()).dig_dcdc_parameters2.read().bits();
                    scratch = scratch & 0x0FFFFFFF;
                    scratch = scratch | 0x10000000;
                    (*HIB1P2::ptr()).dig_dcdc_parameters2.write(
                        |w| w.bits(scratch)
                    );
                }
                else {
                    (*HIB1P2::ptr()).ana_dcdc_parameters0.modify(
                        |r, w| w.bits(
                            (((((r.bits() & 0xFFFFFFF0) |
                                            0x00000001) &
                                            0xFFFFF0FF) |
                                            0x00000500) &
                                            0xFFFE7FFF) |
                                            0x00010000
                         )
                    );
                    (*HIB1P2::ptr()).ana_dcdc_parameters1.modify(
                        |r, w| w.bits(
                            (((((r.bits() & 0xFFFFFF7F) &
                                            0x0FFFFFFF) &
                                            0xFF0FFFFF) |
                                            0x00300000) &
                                            0xFFF0FFFF) |
                                            0x00030000
                        )
                    );
                    (*HIB1P2::ptr()).dig_dcdc_parameters2.modify(
                         |r, w| w.bits(r.bits() & 0x0FFFFFFF)
                    );
                }
            } // endif major/minor version check
            else {
                let pre_regulated_mode = (((*HIB3P3::ptr()).mem_hib_detection_status.read().bits() >> 6) & 1) != 0;
               
                (*HIB1P2::ptr()).ana_dcdc_parameters1.modify(
                    |r, w| w.bits(r.bits() & 0xFFFFFF7F)
                );
               
                (*HIB1P2::ptr()).dig_dcdc_parameters2.modify(
                    |r, w| w.bits(r.bits() & 0x0FFFFFFF)
                );
               
                if pre_regulated_mode {
                    (*HIB1P2::ptr()).dig_dcdc_parameters2.modify(
                        |r, w| w.bits(r.bits() & 0x10000000)
                    );
                }
            }
        } // endif reset cause
        else {
            // I2C Configuration
            (*COMMON_REG::ptr()).i2c_properties_register.modify(
                |r, w| w.bits((r.bits() & !0x3) | 0x1)
            );
           
            // GPIO Configuration
            (*COMMON_REG::ptr()).gpio_properties_register.modify(
                |r, w| w.bits((r.bits() & !0x3FF) | 0x155)
            );
        } // endelse reset cause
    }
}


pub fn get_reset_cause() -> ResetCause {
    /*
     * Implementation of PRCMSysResetCauseGet() from the 
     * CC32xx Simplelink SDK (prcm.c)
     */

    // Read reset status.  We need the last 8 bits of the status.
    let mut reset_status;
    unsafe {
        reset_status = (*GPRCM::ptr()).apps_reset_cause
                        .read()
                        .bits();
    }
    reset_status = reset_status & 0xFF;

    // Convert the reset cause u32 to a ResetCause enum
    let mut reset_cause = ResetCause::from_u32(reset_status);

    // Do additional checks
    match reset_cause {
        ResetCause::PowerOn => {
        
            // Read HIB3P3.HIB_WAKE_STATUS
            let mut hib_wake_status;
            unsafe {
                hib_wake_status = (*HIB3P3::ptr()).mem_hib_wake_status
                                    .read()
                                    .bits();
            }
            hib_wake_status = hib_wake_status & 0x01;
            if hib_wake_status == 0x01 {
                reset_cause = ResetCause::HibExit;

                let mut spare_reg_status;
                unsafe {
                    spare_reg_status = (*OCP_SHARED::ptr()).spare_reg_8
                                        .read()
                                        .bits();
                }
                spare_reg_status = spare_reg_status & 0x280;
                if spare_reg_status == 0x280 {
                    reset_cause = ResetCause::WdtReset;
                }
            }
        }

        ResetCause::LpdsExit => {
            let mut efuse_read_status;
            unsafe {
                efuse_read_status = (*GPRCM::ptr()).efuse_read_reg1
                                    .read()
                                    .bits();
            }
            efuse_read_status = efuse_read_status & (0x01 << 2);
            if efuse_read_status == (0x01 << 2) {
                let mut spare_reg_status;
                unsafe {
                    spare_reg_status = (*OCP_SHARED::ptr()).spare_reg_8
                                        .read()
                                        .bits();
                }
                
                spare_reg_status = spare_reg_status & (0x01 << 8);
                if spare_reg_status == (0x01 << 8) {
                    reset_cause = ResetCause::PowerOn;
                }
            }
        }
        _ => { }
    }
    reset_cause
}
