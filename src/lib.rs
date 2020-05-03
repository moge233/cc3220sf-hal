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
pub extern crate cc3220sf;
extern crate void;


pub mod gpio;
pub mod i2c;
pub mod prcm;
pub mod prelude;
pub mod spi;
pub mod sysctl;
pub mod timer;
pub mod uart;
