/*
 *
 * Power, Reset, and Clock Management for the CC3220SF MCU
 *
 */

use core::ptr::read_volatile;

use cc3220sf::{ARCM, COMMON_REG, GPRCM, OCP_SHARED, HIB1P2, HIB3P3};

use crate::sysctl::{control_power, Domain, RunMode, PowerState};


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


/*****************************
 * PRCM API as defined by the CC3220SF Technical Reference Manual
 ****************************/

pub fn mcu_init() {
    /*
     * Implementation of PRCMC3200MCUInit() from the 
     * CC32xx Simplelink SDK (prcm.c)
     *
     * This is the initialization function ran by the TI CC3220SF 
     * Launchpad when building a C application using the Simplelink SDK.
     *
     */

    let mut reg_value: u32; // use this as a temporary register value

    let reset_cause = get_reset_cause();

    if (reset_cause as u32) != (ResetCause::LpdsExit as u32) {
        // We are not coming out of LPDS
        
        // DIG DCDC LPDS ECO Enable
        unsafe {
            (*HIB1P2::ptr()).ana_dcdc_parameters16.modify(
                |r, w| w.bits(r.bits() | 0x00800000)
            );
        }

        // Enable hibernate ECO for PG 1.32 devices
        unsafe {
            (*HIB3P3::ptr()).mem_hib_reg0.modify(
                |r, w| w.bits(r.bits() | 0x10)
            );
        }

        // Write 0x3C to OCP_SHARED.SPARE_REG_5 to handle clock switching
        // for PG 1.32 devices
        unsafe {
            (*OCP_SHARED::ptr()).spare_reg_5.modify(
                |r, w| w.bits(r.bits() | 0x3C)
            );
        }
        

        // Enable, reset, and disable the uDMA peripheral clock
        control_power(Domain::MicroDma, RunMode::Run, PowerState::Off);
        // control_run_power(Domain::MicroDma, true);
        // TODO: write reset_peripheral() function
        // For now, write 1 then 0 to ARCM.DMASWRST
        unsafe {
            (*ARCM::ptr()).dmaswrst.modify(
                |r, w|  w.bits(r.bits() | 0x1) // Set 0-bit to 1
            );
            (*ARCM::ptr()).dmaswrst.modify(
                |r, w| w.bits(r.bits() & (!0x1)) // Set 0-bit to 0
            );
        }
        // Turn off uDMA
        control_power(Domain::MicroDma, RunMode::Run, PowerState::Off);
        // control_run_power(Domain::MicroDma, false);

        // Enable RTC
        if (reset_cause as u32) == (ResetCause::PowerOn as u32) {
            unsafe {
            // HIB3P3.MEM_HIB_RTC_TIMER_ENABLE
                (*HIB3P3::ptr()).mem_hib_rtc_timer_enable.write(
                    |w| w.bits(0x1)
                );
            }
        }

        // SWD mode
        // Read HIB1P2.SOP_SENSE_VALUE and check if first 8 bits are
        // equal to 0x02
        unsafe {
            reg_value = (*HIB1P2::ptr()).sop_sense_value.read().bits();
        }
        reg_value = reg_value & 0xFF;
        if reg_value == 0x2 {
            unsafe {
                (*OCP_SHARED::ptr()).gpio_pad_config_28.modify(
                    |r, w| w.bits((r.bits() & (!0xC0F)) | 0x2)
                );

                (*OCP_SHARED::ptr()).gpio_pad_config_29.modify(
                    |r, w| w.bits((r.bits() & (!0xC0F)) | 0x2)
                );
            }
        }

        // Override JTAG mux
        unsafe {
            (*OCP_SHARED::ptr()).cc3xx_dev_padconf.modify(
                |r, w| w.bits(r.bits() | 0x2)
            );
        }

        // Change UART pins (55, 57) pinmode to pin mode 0 if they are in
        // pin mode 3
        unsafe {
            let mut pin_mode: u32;

            // Check pin 55
            pin_mode = (*OCP_SHARED::ptr()).gpio_pad_config_1.read().bits();
            if (pin_mode & 0xF) == 0x3 {
                pin_mode = pin_mode & (!0xF); // Set bits [3:0] to 0
                (*OCP_SHARED::ptr()).gpio_pad_config_1.write(
                    |w| w.bits(pin_mode)
                );
            }

            // Check pin 57
            pin_mode = (*OCP_SHARED::ptr()).gpio_pad_config_2.read().bits();
            if (pin_mode & 0xF) == 0x3 {
                pin_mode = pin_mode & (!0xF); // Set bits [3:0] to 0
                (*OCP_SHARED::ptr()).gpio_pad_config_2.write(
                    |w| w.bits(pin_mode)
                );
            }
        }

        // Change I2C pins (1, 2) pinmode to pin mode 0 if they are in
        // pin mode 1
        unsafe {
            let mut pin_mode: u32;

            // Check pin 1
            pin_mode = (*OCP_SHARED::ptr()).gpio_pad_config_10
                        .read().bits();
            if (pin_mode & 0xF) == 0x1 {
                pin_mode = pin_mode & (!0xF); // Set bits [3:0] to 0
                (*OCP_SHARED::ptr()).gpio_pad_config_10.write(
                    |w| w.bits(pin_mode)
                );
            }

            // Check pin 2
            pin_mode = (*OCP_SHARED::ptr()).gpio_pad_config_11
                        .read().bits();
            if (pin_mode & 0xF) == 0x1 {
                pin_mode = pin_mode & (!0xF); // Set bits [3:0] to 0
                (*OCP_SHARED::ptr()).gpio_pad_config_11.write(
                    |w| w.bits(pin_mode)
                );
            }
        }


        // DIG DCDC VOUT trim settings based on PROCESS INDICATOR
        unsafe {
            reg_value = (*GPRCM::ptr()).efuse_read_reg0.read().bits();
            reg_value = (reg_value >> 22) & 0xF;

            if reg_value == 0xE {
                (*HIB1P2::ptr()).dig_dcdc_vtrim_cfg.modify(
                    |r, w|
                    w.bits(
                        (r.bits() & (!0x00FC0000)) | (0x32 << 18)
                    )
                );
                    
            }
            else {
                (*HIB1P2::ptr()).dig_dcdc_vtrim_cfg.modify(
                    |r, w|
                    w.bits(
                        (r.bits() & (!0x00FC0000)) | (0x29 << 18)
                    )
                )
            }
        }

        // Enable SOFT RESTART in case of DIG DCDC collapse
        unsafe {
            (*HIB3P3::ptr()).hibana_spare_lowv.modify(
                |r, w| w.bits(r.bits() & (!0x10000000))
            );
        }

        let rom_version;
        unsafe {
            rom_version = read_volatile(0x400 as *mut u32);
        }

        if (rom_version >> 16) >= 1 { // Check if rom version >= 1

            // Enable NWP force reset and HIB on WDT reset
            // Enable direct boot path for flash
            unsafe {
                (*OCP_SHARED::ptr()).spare_reg_8.modify(
                    |r, w| w.bits(r.bits() | ((7 << 5) | 1))
                );


                reg_value = (*HIB3P3::ptr()).mem_hib_reg2.read().bits();
                reg_value = reg_value & 0x1;
                if reg_value == 0x1 {
                    (*HIB3P3::ptr()).mem_hib_reg2.modify(
                        |r, w| w.bits(r.bits() & (!0x1))
                    );
                    
                    (*OCP_SHARED::ptr()).spare_reg_8.modify(
                        |r, w| w.bits(r.bits() | (1 << 9))
                    );
                }

                // Clear RTC HIB wakeup source
                (*HIB3P3::ptr()).mem_hib_rtc_wake_en.modify(
                    |r, w| w.bits(r.bits() & (!0x1))
                );

                // Reset RTC match value
                (*HIB3P3::ptr()).mem_hib_rtc_wake_lsw_conf.write(
                    |w| w.bits(0x0)
                );
                (*HIB3P3::ptr()).mem_hib_rtc_wake_msw_conf.write(
                    |w| w.bits(0x0)
                );
            } // end unsafe
        } // end if rom_version >= 1

        let efuse_reg2;
        let dev_major_ver;
        let dev_minor_ver;

        // Read the device ID register
        unsafe {
            efuse_reg2 = (*GPRCM::ptr()).efuse_read_reg2.read().bits();
        }

        dev_major_ver = (efuse_reg2 >> 28) & 0xF;
        dev_minor_ver = (efuse_reg2 >> 24) & 0xF;

        if ((dev_major_ver == 3) && (dev_minor_ver == 0)) ||
            dev_major_ver < 3 {

            // let mut scratch: u32;
            let mut pre_regulated_mode: u32;

            unsafe {
                // HIB3P3.mem_hib_detection_status
                pre_regulated_mode = (*HIB3P3::ptr())
                                        .mem_hib_detection_status
                                        .read().bits();
            }
            pre_regulated_mode = (pre_regulated_mode >> 6) & 0x1;

            if pre_regulated_mode == 1 {
                unsafe {
                    (*HIB1P2::ptr()).ana_dcdc_parameters1.modify(
                        |r, w| w.bits(r.bits() & 0xFFFFFF7F)
                    );

                    (*HIB1P2::ptr()).hib_timer_rtc_wup_timestamp_msw.modify(
                        |r, w| w.bits((r.bits() & 0x0FFFFFFF) | 0x10000000)
                    );
                }
            } // end if pre_regulated_mode

            else {
                unsafe {
                    (*HIB1P2::ptr()).ana_dcdc_parameters0.modify(
                        |r, w| w.bits(
                            r.bits() & 0xFFFFFFF0 |
                                       0x00000001 &
                                       0xFFFFF0FF |
                                       0x00000500 &
                                       0xFFFE7FFF |
                                       0x00010000
                        )
                    );

                    (*HIB1P2::ptr()).ana_dcdc_parameters1.modify(
                        |r, w| w.bits(
                            r.bits() & 0xFFFFFF7F &
                                       0x0FFFFFFF &
                                       0xFF0FFFFF |
                                       0x00300000 &
                                       0xFFF0FFFF |
                                       0x00030000
                        )
                    );

                    (*HIB1P2::ptr()).dig_dcdc_parameters2.modify(
                        |r, w| w.bits(
                            r.bits() & 0x0FFFFFFF
                        )
                    );
                }
            }
        } // end if dev_major == 3 && dev_minor == 0 || dev_major < 3

        else {
            // let mut scratch: u32;
            let mut pre_regulated_mode: u32;

            // HIB3P3.mem_hib_detection_status
            unsafe {
                pre_regulated_mode = (*HIB3P3::ptr())
                                        .mem_hib_detection_status
                                        .read().bits();

                pre_regulated_mode = (pre_regulated_mode >> 6) & 0x1;

                (*HIB1P2::ptr()).ana_dcdc_parameters1.modify(
                    |r, w| w.bits(r.bits() & 0xFFFFFF7F)
                );

                (*HIB1P2::ptr()).dig_dcdc_parameters2.modify(
                    |r, w| w.bits(r.bits() & 0x0FFFFFFF)
                );

                if pre_regulated_mode == 1 {
                    (*HIB1P2::ptr()).dig_dcdc_parameters2.modify(
                        |r, w| w.bits(r.bits() | 0x10000000)
                    );
                }
            }
        }
    } // not waking up from LPDS


    else {
    // we are coming out of LPDS
        unsafe {
            // I2C Configuration
            (*COMMON_REG::ptr()).i2c_properties_register.modify(
                |r, w| w.bits(
                    (r.bits() & (!0x3)) | 0x1
                )
            );

            // GPIO Configuration
            (*COMMON_REG::ptr()).gpio_properties_register.modify(
                |r, w| w.bits(
                    (r.bits() & (!0x3FF)) | 0x155
                )
            );
        }
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
