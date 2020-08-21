/*
 * 
 * HAL for the TI CC3220SF Microcontroller
 *
 */


#![deny(warnings)]
#![no_std]


extern crate cortex_m;
extern crate embedded_hal as hal;
extern crate nb;
extern crate void;

pub extern crate cc3220sf as pac;


pub mod adc;
pub mod gpio;
pub mod i2c;
pub mod prcm;
pub mod prelude;
// pub mod spi;
pub mod sysctl;
pub mod timer;
pub mod uart;
