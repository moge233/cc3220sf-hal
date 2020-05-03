/*
 *
 * General Purpose Input/Output
 *
 */


use core::marker::PhantomData;
use core::ptr::{read_volatile, write_volatile};

use hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin,
                       ToggleableOutputPin};

use crate::sysctl::*;

/**************************************
 * Public trait definitions
 *************************************/
pub trait OutputMode { }
pub trait InputMode { }
pub trait IsUnlocked { }
pub trait GpioExtension {
    type Parts;
    fn split(self) -> Self::Parts;
}


/**************************************
 * Public struct definitions
 *************************************/
#[derive(Debug)]
pub struct PinError;
pub struct Locked;
pub struct Tristate;
pub struct Output<MODE> where MODE: OutputMode {
    _mode: PhantomData<MODE>,
}
pub struct Input<MODE> where MODE: InputMode {
    _mode: PhantomData<MODE>,
}
pub struct PullDown;
pub struct PullUp;
pub struct PushPull;
pub struct OpenDrain;
pub struct AlternateFunction;


/**************************************
 * Implementations
 *************************************/
impl IsUnlocked for Tristate { }
impl<MODE> IsUnlocked for Output<MODE> where MODE: OutputMode { }
impl<MODE> IsUnlocked for Input<MODE> where MODE: InputMode { }

impl OutputMode for PushPull { }
impl OutputMode for PullDown { }
impl OutputMode for PullUp { }
impl OutputMode for OpenDrain { }

impl InputMode for PullDown { }
impl InputMode for PullUp { }


/**************************************
 * Public enums
 *************************************/
pub enum InterruptMode {
    LevelLow,
    LevelHigh,
    EdgeRising,
    EdgeFalling,
    EdgeBoth,
    Disabled,
}

pub enum DriveStrength {
    Low = 0b001,        // 2 mA
    Medium = 0b010,     // 4 mA
    High = 0b011,       // 6 mA
}


/**************************************
 * GPIO macro definition
 *************************************/
macro_rules! gpio_macro {
    ($GPIOX:ident, $gpiox:ident, $GpioX:ident, $PXx:ident,
     [$($PXi:ident: ($pxi:ident, $padcfg:ident,  $i:expr, $pini: expr,
                     $MODE:ty),)+]
    ) => {
        pub mod $gpiox {

            use cc3220sf::{$GPIOX, OCP_SHARED};
            use super::*;

            pub struct Parts {
                $(
                    // Pin
                    pub $pxi: $PXi<$MODE>,
                 )+
            }


            pub struct PadConfig {
                _0: (),
            }



            impl GpioExtension for $GPIOX {
                type Parts = Parts;
                
                fn split(self) -> Parts {
                    control_power(Domain::$GpioX,
                                  RunMode::Run,
                                  PowerState::On);
                    Parts {
                        $(
                            $pxi: $PXi {
                                _mode: PhantomData,
                            },
                         )+
                    }
                }
            }

            struct $PXx<MODE> {
                i: u8,
                _mode: PhantomData<MODE>,
            }

            impl<MODE> InputPin for $PXx<Input<MODE>>
            where MODE: InputMode {
                
                type Error = PinError;

                fn is_high(&self) -> Result<bool, Self::Error> {
                    let bit = self.i;
                    let data_addr: u32 = $GPIOX::ptr() as u32;
                    let offset = 2u32.pow(bit as u32) << 2;
                    let mut data;
                    unsafe {
                        data = read_volatile(
                                    (data_addr + offset) as *mut u32);
                    }
                    data = (data & (offset >> 2)) >> bit;
                    if data == 1 {
                        return Result::Ok(true);
                    }
                    Result::Ok(false)
                }

                fn is_low(&self) -> Result<bool, Self::Error> {
                    Result::Ok(!self.is_high().unwrap())
                }
            }

            impl<MODE> OutputPin for $PXx<Output<MODE>>
            where MODE: OutputMode {
                
                type Error = PinError;

                fn set_high(&mut self) -> Result<(), Self::Error> {
                    let bit = self.i;
                    let data_addr: u32 = $GPIOX::ptr() as u32;
                    let offset = 2u32.pow(bit as u32) << 2;
                    unsafe {
                        write_volatile((data_addr + offset) as *mut u32,
                                       offset >> 2);
                    }
                    Result::Ok(())
                }

                fn set_low(&mut self) -> Result<(), Self::Error> {
                    let bit = self.i;
                    let data_addr: u32 = $GPIOX::ptr() as u32;
                    let offset = 2u32.pow(bit as u32) << 2;
                    unsafe {
                        write_volatile((data_addr + offset) as *mut u32, 0);
                    }
                    Result::Ok(())
                }
            }
            impl<MODE> StatefulOutputPin for $PXx<Output<MODE>>
            where MODE: OutputMode {
                
                fn is_set_high(&self) -> Result<bool, Self::Error> {
                    let bit = self.i;
                    let data_addr: u32 = $GPIOX::ptr() as u32;
                    let offset = 2u32.pow(bit as u32) << 2;
                    let mut data;
                    unsafe {
                        data = read_volatile(
                                    (data_addr + offset) as *mut u32);
                    }
                    data = (data & (offset >> 2)) >> bit;
                    if data == 1 {
                        return Result::Ok(true);
                    }
                    Result::Ok(false)
                }

                fn is_set_low(&self) -> Result<bool, Self::Error> {
                    Result::Ok(!self.is_set_high().unwrap())
                }
            }

            impl<MODE> ToggleableOutputPin for $PXx<Output<MODE>>
            where MODE: OutputMode {
                
                type Error = PinError;

                fn toggle(&mut self) -> Result<(), Self::Error> {
                    if self.is_set_low().unwrap() == true {
                        self.set_high().unwrap();
                    }
                    else {
                        self.set_low().unwrap();
                    }
                    Result::Ok(())
                }
            }

            $(
                pub struct $PXi<MODE> {
                    _mode: PhantomData<MODE>,
                }

                impl<MODE> $PXi<MODE> {

                    pub fn into_alternate_function(&self)
                    -> $PXi<AlternateFunction> {
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_pull_up_input(&self)
                    -> $PXi<Input<PullUp>> {
                        unsafe {
                            // Clear data direction bit to 0
                            (*$GPIOX::ptr()).dir.modify(
                                |r, w| w.bits(r.bits() & !(1 << $i))
                            );

                            // Clear current pad config
                            (*OCP_SHARED::ptr()).$padcfg.write(
                                |w| w.bits(0)
                            );

                            // Set internal weak pullup
                            (*OCP_SHARED::ptr()).$padcfg.modify(
                                |r, w| w.bits(
                                    r.bits() | (1 << 8)
                                )
                            );
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_pull_down_input(&self)
                    -> $PXi<Input<PullDown>> {
                        unsafe {
                            // Clear the data direction bit to 0
                            (*$GPIOX::ptr()).dir.modify(
                                |r, w| w.bits(r.bits() & !(1 << $i))
                            );

                            // Clear current pad config
                            (*OCP_SHARED::ptr()).$padcfg.write(
                                |w| w.bits(0)
                            );

                            // Set internal weak pulldown and clear
                            // internal weak pullup
                            (*OCP_SHARED::ptr()).$padcfg.modify(
                                |r, w| w.bits(
                                    r.bits() | (1 << 9)
                                )
                            );
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_open_drain_output(&self)
                    -> $PXi<Output<OpenDrain>> {
                        unsafe {
                            // Set the data direction bit to 1 (output)
                            (*$GPIOX::ptr()).dir.modify(
                                |r, w| w.bits(r.bits() | (1 << $i))
                            );

                            // Clear current pad config
                            (*OCP_SHARED::ptr()).$padcfg.write(
                                |w| w.bits(0)
                            );

                            // Set the open drain bit in the pad config,
                            // clear internal pulldown, and set internal
                            // pull up
                            (*OCP_SHARED::ptr()).$padcfg.modify(
                                |r, w| w.bits(
                                    r.bits() | (1 << 4)
                                )
                            );
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn into_push_pull_output(&self)
                    -> $PXi<Output<PushPull>> {
                        unsafe {
                            // Set the data direction bit to 1 (output)
                            (*$GPIOX::ptr()).dir.modify(
                                |r, w| w.bits(
                                    r.bits() | (1 << $i)
                                )
                            );

                            // Clear current pad config
                            (*OCP_SHARED::ptr()).$padcfg.write(
                                |w| w.bits(0)
                            );
                            
                            // Set bits 8 and 9 in the pad config
                            // and clear open drain [bit 4]
                            (*OCP_SHARED::ptr()).$padcfg.modify(
                                |r, w| w.bits(
                                    r.bits() | (1 << 8) | (1 << 9) &
                                    !(1 << 4)
                                )
                            );
                        }
                        $PXi { _mode: PhantomData }
                    }

                    pub fn set_drive_strength(
                        &self,
                        strength: DriveStrength
                    )
                    -> $PXi<MODE> {
                        unsafe {
                            // Clear curent drive strength
                            (*OCP_SHARED::ptr()).$padcfg.modify(
                                |r, w| w.bits(
                                    r.bits() & (!(0b111 << 5))
                                )
                            );
                            
                            // Set new drive strength
                            (*OCP_SHARED::ptr()).$padcfg.modify(
                                |r, w| w.bits(
                                    r.bits() | ((strength as u32) << 5)
                                )
                            );

                            $PXi { _mode: PhantomData }
                        }
                    }

                    pub fn clear_pad_config(&self) -> $PXi<MODE> {
                        unsafe {
                            (*OCP_SHARED::ptr()).$padcfg.write(
                                |w| w.bits(0)
                            );
                        }
                        
                        $PXi { _mode: PhantomData }
                    }
                }

                impl $PXi<Output<OpenDrain>> {
                    pub fn internal_pull_up(&mut self, on: bool) {
                        // Enables/disables the internal pull up
                        // in open drain mode
                        unsafe {
                            if on {
                                (*OCP_SHARED::ptr()).$padcfg.modify(
                                    |r, w| w.bits(
                                        r.bits() | (1 << 8)
                                    )
                                );
                            }
                            else {
                                (*OCP_SHARED::ptr()).$padcfg.modify(
                                    |r, w| w.bits(
                                        r.bits() & !(1 << 8)
                                    )
                                );
                            }
                        }
                    }
                }

                impl<MODE> InputPin for $PXi<Input<MODE>>
                where MODE: InputMode {

                    type Error = PinError;

                    fn is_high(&self) -> Result<bool, Self::Error> {
                        let bit = $i;
                        let data_addr: u32 = $GPIOX::ptr() as u32;
                        let offset = 2u32.pow(bit as u32) << 2;
                        let mut data;
                        unsafe {
                            data = read_volatile(
                                        (data_addr + offset) as *mut u32);
                        }
                        data = (data & (offset >> 2)) >> bit;
                        if data == 1 {
                            return Result::Ok(true);
                        }
                        Result::Ok(false)
                    }
                    fn is_low(&self) -> Result<bool, Self::Error> {
                        Result::Ok(!self.is_high().unwrap())
                    }
                }

                impl<MODE> OutputPin for $PXi<Output<MODE>>
                where MODE: OutputMode {

                    type Error = PinError;

                    fn set_high(&mut self) -> Result<(), Self::Error> {
                        let bit = $i;
                        let data_addr: u32 = $GPIOX::ptr() as u32;
                        let offset = 2u32.pow(bit as u32) << 2;
                        unsafe {
                            write_volatile((data_addr + offset) as *mut u32,
                                           offset >> 2);
                        }
                        Result::Ok(())
                    }

                    fn set_low(&mut self) -> Result<(), Self::Error> {
                        let bit = $i;
                        let data_addr: u32 = $GPIOX::ptr() as u32;
                        let offset = 2u32.pow(bit as u32) << 2;
                        unsafe {
                            write_volatile((data_addr + offset) as *mut u32,
                                           0);
                        }
                        Result::Ok(())
                    }
                }
                
                impl<MODE> StatefulOutputPin for $PXi<Output<MODE>>
                where MODE: OutputMode {
                    
                    fn is_set_high(&self) -> Result<bool, Self::Error> {
                        let bit = $i;
                        let data_addr: u32 = $GPIOX::ptr() as u32;
                        let offset = 2u32.pow(bit as u32) << 2;
                        let mut data;
                        unsafe {
                            data = read_volatile(
                                        (data_addr + offset) as *mut u32);
                        }
                        data = (data & (offset >> 2)) >> bit;
                        if data == 1 {
                            return Result::Ok(true);
                        }
                        Result::Ok(false)
                    }

                    fn is_set_low(&self) -> Result<bool, Self::Error> {
                        Result::Ok(!self.is_set_high().unwrap())
                    }
                }

                impl<MODE> ToggleableOutputPin for $PXi<Output<MODE>>
                where MODE: OutputMode {
                    
                    type Error = PinError;

                    fn toggle(&mut self) -> Result<(), Self::Error> {
                        if self.is_set_low().unwrap() == true {
                            self.set_high().unwrap();
                        }
                        else {
                            self.set_low().unwrap();
                        }
                        Result::Ok(())
                    }
                }

            )+
        }
    }
}


/**************************************
 * GPIO initializations
 *************************************/
gpio_macro!(GPIOA0, gpioa0, Gpio0, PA0x, 
            [
            PA00: (gpio_00, gpio_pad_config_0, 0, 50, Tristate),
            PA01: (gpio_01, gpio_pad_config_1, 1, 55, Tristate),
            PA02: (gpio_02, gpio_pad_config_2, 2, 57, Tristate),
            PA03: (gpio_03, gpio_pad_config_3, 3, 58, Tristate),
            PA04: (gpio_04, gpio_pad_config_4, 4, 59, Tristate),
            PA05: (gpio_05, gpio_pad_config_5, 5, 60, Tristate),
            PA06: (gpio_06, gpio_pad_config_6, 6, 61, Tristate),
            PA07: (gpio_07, gpio_pad_config_7, 7, 62, Tristate),
            ]);


gpio_macro!(GPIOA1, gpioa1, Gpio1, PA1x,
            [
            PA10: (gpio_08, gpio_pad_config_8, 0, 63, Tristate),
            PA11: (gpio_09, gpio_pad_config_9, 1, 64, Tristate),
            PA12: (gpio_10, gpio_pad_config_10, 2, 1, Tristate),
            PA13: (gpio_11, gpio_pad_config_11, 3, 2, Tristate),
            PA14: (gpio_12, gpio_pad_config_12, 4, 3, Tristate),
            PA15: (gpio_13, gpio_pad_config_13, 5, 4, Tristate),
            PA16: (gpio_14, gpio_pad_config_14, 6, 5, Tristate),
            PA17: (gpio_15, gpio_pad_config_15, 7, 6, Tristate),
            ]);


gpio_macro!(GPIOA2, gpioa2, Gpio2, PA2x,
            [
            PA20: (gpio_16, gpio_pad_config_16, 0, 7, Tristate),
            PA21: (gpio_17, gpio_pad_config_17, 1, 8, Tristate),
            PA22: (gpio_18, gpio_pad_config_18, 2, 99, Locked),
            PA23: (gpio_19, gpio_pad_config_19, 3, 99, Locked),
            PA24: (gpio_20, gpio_pad_config_20, 4, 99, Locked),
            PA25: (gpio_21, gpio_pad_config_21, 5, 99, Locked),
            PA26: (gpio_22, gpio_pad_config_22, 6, 15, Tristate),
            PA27: (gpio_23, gpio_pad_config_23, 7, 16, Tristate),
            ]);


gpio_macro!(GPIOA3, gpioa3, Gpio3, PA3x,
            [
            PA30: (gpio_24, gpio_pad_config_24, 0, 17, Tristate),
            PA31: (gpio_25, gpio_pad_config_25, 1, 21, Tristate),
            PA32: (gpio_26, gpio_pad_config_26, 2, 29, Tristate),
            PA33: (gpio_27, gpio_pad_config_27, 3, 30, Tristate),
            PA34: (gpio_28, gpio_pad_config_28, 4, 18, Tristate),
            PA35: (gpio_29, gpio_pad_config_29, 5, 20, Tristate),
            PA36: (gpio_30, gpio_pad_config_30, 6, 53, Tristate),
            PA37: (gpio_31, gpio_pad_config_31, 7, 45, Tristate),
            ]);
