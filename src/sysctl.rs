/*
 *
 * CC3220SF System Control 
 *
 */


use cortex_m::asm::nop;

use cc3220sf::ARCM;


#[derive(Clone, Copy)]
pub enum RunMode {
    Run,
    Sleep,
    DeepSleep,
}


#[derive(Clone, Copy)]
pub enum PowerState {
    Off,
    On,
}


pub enum Domain {
    Camera,     // Camera
    Mcasp,      // MCASP
    Sdio,       // SDIO
    Gspi,       // GSPI
    MicroDma,   // uDMA
    Gpio0,      // GPIOA0
    Gpio1,      // GPIOA1
    Gpio2,      // GPIOA2
    Gpio3,      // GPIOA3
    Gpio4,      // GPIOA4
    Watchdog,   // WDT
    Uart0,      // UARTA0
    Uart1,      // UARTA1
    Timer0,     // Timer 0
    Timer1,     // Timer 1
    Timer2,     // Timer 2
    Timer3,     // Timer 3
    I2c,        // I2C
}



/****************************
 * Reset a peripheral
 ***************************/

pub fn reset_power(pd: Domain) {
    match pd {
        Domain::Camera => {
            unsafe {
                (*ARCM::ptr()).camswrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).camswrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Mcasp => {
            unsafe {
                (*ARCM::ptr()).mcaspswrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).mcaspswrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        }
        Domain::Sdio => {
            unsafe {
                (*ARCM::ptr()).sdiomswrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).sdiomswrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        }
        Domain::Gspi => {
            unsafe {
                (*ARCM::ptr()).apspiswrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).apspiswrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        }
        Domain::MicroDma => {
            unsafe {
                (*ARCM::ptr()).dmaswrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).dmaswrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Gpio0 => {
            unsafe {
                (*ARCM::ptr()).gpio0swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).gpio0swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Gpio1 => {
            unsafe {
                (*ARCM::ptr()).gpio1swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).gpio1swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Gpio2 => {
            unsafe {
                (*ARCM::ptr()).gpio2swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).gpio2swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Gpio3 => {
            unsafe {
                (*ARCM::ptr()).gpio3swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).gpio3swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Gpio4 => {
            unsafe {
                (*ARCM::ptr()).gpio4swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).gpio4swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Watchdog => {
            unsafe {
                (*ARCM::ptr()).wdtswrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).wdtswrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Uart0 => {
            unsafe {
                (*ARCM::ptr()).uart0swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                ); 
                (*ARCM::ptr()).uart0swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                ); 
            }
        },
        Domain::Uart1 => {
            unsafe {
                (*ARCM::ptr()).uart1swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                ); 
                (*ARCM::ptr()).uart1swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                ); 
            }
        },
        Domain::Timer0 => {
            unsafe {
                (*ARCM::ptr()).gpt0swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).gpt0swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Timer1 => {
            unsafe {
                (*ARCM::ptr()).gpt1swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).gpt1swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Timer2 => {
            unsafe {
                (*ARCM::ptr()).gpt2swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).gpt2swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::Timer3 => { 
            unsafe {
                (*ARCM::ptr()).gpt3swrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).gpt3swrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
        Domain::I2c => {
            unsafe {
                (*ARCM::ptr()).i2cswrst.modify(
                    |r, w| w.bits(r.bits() | 0x1)
                );
                (*ARCM::ptr()).i2cswrst.modify(
                    |r, w| w.bits(r.bits() & !0x1)
                );
            }
        },
    }
}


/****************************
 * Enable or disable a peripheral clock
 ***************************/

pub fn control_power(pd: Domain, run_mode: RunMode,
                     state: PowerState) {
    let on = match state {
        PowerState::On => true,
        PowerState::Off => false,
    };

    match run_mode {
        RunMode::Run => control_run_power(pd, on),
        RunMode::Sleep => control_sleep_power(pd, on),
        RunMode::DeepSleep => control_deep_sleep_power(pd, on),
    }

    nop();
    nop();
    nop();
}


fn control_run_power(pd: Domain, on: bool) {
    match pd {
        Domain::Camera => {
            unsafe {
                if on {
                    (*ARCM::ptr()).camclken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).camclken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );

                }
            }
        },
        Domain::Mcasp => {
            unsafe {
                if on {
                    (*ARCM::ptr()).mcaspclken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).mcaspclken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        }
        Domain::Sdio => {
            unsafe {
                if on {
                    (*ARCM::ptr()).sdiomclken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).sdiomclken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        }
        Domain::Gspi => {
            unsafe {
                if on {
                    (*ARCM::ptr()).apspiclken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).apspiclken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        }
        Domain::MicroDma => {
            unsafe {
                if on {
                    (*ARCM::ptr()).dmaclken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).dmaclken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );

                }
            }
        },
        Domain::Gpio0 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio0clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio0clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
        Domain::Gpio1 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio1clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio1clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
        Domain::Gpio2 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio2clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio2clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
        Domain::Gpio3 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio3clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio3clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
        Domain::Gpio4 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio4clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio4clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
        Domain::Watchdog => {
            unsafe {
                if on {
                    (*ARCM::ptr()).wdtclken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).wdtclken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
        Domain::Uart0 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).uart0clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    ); 
                }
                else {
                    (*ARCM::ptr()).uart0clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    ); 
                }
            }
        },
        Domain::Uart1 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).uart1clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    ); 
                }
                else {
                    (*ARCM::ptr()).uart1clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    ); 
                }
            }
        },
        Domain::Timer0 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt0clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt0clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
        Domain::Timer1 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt1clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt1clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
        Domain::Timer2 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt2clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt2clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
        Domain::Timer3 => { 
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt3clken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt3clken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
        Domain::I2c => {
            unsafe {
                if on {
                    (*ARCM::ptr()).i2cclken.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
                else {
                    (*ARCM::ptr()).i2cclken.modify(
                        |r, w| w.bits(r.bits() & !0x1)
                    );
                }
            }
        },
    }
}


fn control_sleep_power(pd: Domain, on: bool) {
    match pd {
        Domain::Camera => {
            unsafe {
                if on {
                    (*ARCM::ptr()).camclken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).camclken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );

                }
            }
        },
        Domain::Mcasp => {
            unsafe {
                if on {
                    (*ARCM::ptr()).mcaspclken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).mcaspclken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        }
        Domain::Sdio => {
            unsafe {
                if on {
                    (*ARCM::ptr()).sdiomclken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).sdiomclken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        }
        Domain::Gspi => {
            unsafe {
                if on {
                    (*ARCM::ptr()).apspiclken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).apspiclken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        }
        Domain::MicroDma => {
            unsafe {
                if on {
                    (*ARCM::ptr()).dmaclken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).dmaclken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );

                }
            }
        },
        Domain::Gpio0 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio0clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio0clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
        Domain::Gpio1 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio1clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio1clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
        Domain::Gpio2 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio2clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio2clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
        Domain::Gpio3 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio3clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio3clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
        Domain::Gpio4 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio4clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio4clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
        Domain::Watchdog => {
            unsafe {
                if on {
                    (*ARCM::ptr()).wdtclken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).wdtclken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
        Domain::Uart0 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).uart0clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    ); 
                }
                else {
                    (*ARCM::ptr()).uart0clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    ); 
                }
            }
        },
        Domain::Uart1 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).uart1clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    ); 
                }
                else {
                    (*ARCM::ptr()).uart1clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    ); 
                }
            }
        },
        Domain::Timer0 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt0clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt0clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
        Domain::Timer1 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt1clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt1clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
        Domain::Timer2 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt2clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt2clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
        Domain::Timer3 => { 
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt3clken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt3clken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
        Domain::I2c => {
            unsafe {
                if on {
                    (*ARCM::ptr()).i2cclken.modify(
                        |r, w| w.bits(r.bits() | 0x100)
                    );
                }
                else {
                    (*ARCM::ptr()).i2cclken.modify(
                        |r, w| w.bits(r.bits() & !0x100)
                    );
                }
            }
        },
    }
}


fn control_deep_sleep_power(pd: Domain, on: bool) {
    match pd {
        Domain::Camera => {
            unsafe {
                if on {
                    (*ARCM::ptr()).camclken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).camclken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );

                }
            }
        },
        Domain::Mcasp => {
            unsafe {
                if on {
                    (*ARCM::ptr()).mcaspclken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).mcaspclken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        }
        Domain::Sdio => {
            unsafe {
                if on {
                    (*ARCM::ptr()).sdiomclken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).sdiomclken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        }
        Domain::Gspi => {
            unsafe {
                if on {
                    (*ARCM::ptr()).apspiclken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).apspiclken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        }
        Domain::MicroDma => {
            unsafe {
                if on {
                    (*ARCM::ptr()).dmaclken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).dmaclken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );

                }
            }
        },
        Domain::Gpio0 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio0clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio0clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
        Domain::Gpio1 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio1clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio1clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
        Domain::Gpio2 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio2clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio2clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
        Domain::Gpio3 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio3clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio3clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
        Domain::Gpio4 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpio4clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).gpio4clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
        Domain::Watchdog => {
            unsafe {
                if on {
                    (*ARCM::ptr()).wdtclken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).wdtclken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
        Domain::Uart0 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).uart0clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    ); 
                }
                else {
                    (*ARCM::ptr()).uart0clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    ); 
                }
            }
        },
        Domain::Uart1 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).uart1clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    ); 
                }
                else {
                    (*ARCM::ptr()).uart1clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    ); 
                }
            }
        },
        Domain::Timer0 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt0clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt0clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
        Domain::Timer1 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt1clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt1clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
        Domain::Timer2 => {
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt2clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt2clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
        Domain::Timer3 => { 
            unsafe {
                if on {
                    (*ARCM::ptr()).gpt3clken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).gpt3clken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
        Domain::I2c => {
            unsafe {
                if on {
                    (*ARCM::ptr()).i2cclken.modify(
                        |r, w| w.bits(r.bits() | 0x10000)
                    );
                }
                else {
                    (*ARCM::ptr()).i2cclken.modify(
                        |r, w| w.bits(r.bits() & !0x10000)
                    );
                }
            }
        },
    }
}
