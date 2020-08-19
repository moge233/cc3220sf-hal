/*
 *
 * I2C Communication Module
 *
 */


use core::marker::PhantomData;

use hal::blocking::i2c::{Read, Write, WriteRead};

use crate::sysctl::*;
use crate::gpio::{AlternateFunction, DriveStrength};


/*********************************************
 * Available GPIO for I2C
 * SDA Pins         SCL Pins
 * GPIO11 (PA13)     GPIO10 (PA12)
 * GPIO13 (PA15)     GPIO12 (PA14)
 * GPIO15 (PA17)     GPIO14 (PA16)
 * GPIO24 (PA30)     GPIO23 (PA27)
 ********************************************/
use crate::gpio::gpioa1::{PA12, PA13, PA14, PA15, PA16, PA17};
use crate::gpio::gpioa2::PA27;
use crate::gpio::gpioa3::PA30;

use cc3220sf::OCP_SHARED;


/**************************************
* Public trait definitions
**************************************/
pub trait SclPin<I2c> {
    fn to_i2c_pin(&self) {
        unimplemented!()
    }
}
pub trait SdaPin<I2c> {
    fn to_i2c_pin(&self) {
        unimplemented!()
    }
}


/**************************************
* Public struct definitions
**************************************/
pub struct I2c<I2C, PINS> {
    #[allow(dead_code)]
    i2c: I2C,
    #[allow(dead_code)]
    pins: PINS,
}

pub struct Sda<I2C> {
    _i2c: PhantomData<I2C>,
}

pub struct Scl<I2C> {
    _i2c: PhantomData<I2C>,
}

#[derive(Debug)]
pub struct I2cError { }


/**************************************
* Implementations
**************************************/
impl SclPin<I2CA0> for PA12<AlternateFunction> {
    fn to_i2c_pin(&self) {
        unsafe {
            // Clear the current drive strength and CONF mode as
            // well as bits 10 and 11 and then set the
            // new drive strength to Low (2mA), set the appropriate CONF
            // mode, enable open drain mode, and enable the internal
            // weak pullup
            (*OCP_SHARED::ptr()).gpio_pad_config_10.modify(
                |r, w| w.bits(
                    (r.bits() & (!0xCEF)) |
                    0x1 | // CONF mode
                    ((DriveStrength::Low as u32) << 5) |
                    (1 << 4) | // Open drain mode
                    (1 << 8)   // Weak pullup
                )
            );
        }
    }
}

impl SclPin<I2CA0> for PA14<AlternateFunction> {
    fn to_i2c_pin(&self) {
        unsafe {
            // Clear the current drive strength and CONF mode as
            // well as bits 10 and 11 and then set the
            // new drive strength to Low (2mA), set the appropriate CONF
            // mode, enable open drain mode, and enable the internal
            // weak pullup
            (*OCP_SHARED::ptr()).gpio_pad_config_12.modify(
                |r, w| w.bits(
                    (r.bits() & (!0xCEF)) |
                    0x5 | // CONF mode
                    ((DriveStrength::Low as u32) << 5) |
                    (1 << 4) | // Open drain mode
                    (1 << 8)   // Weak pullup
                )
            );
        }
    }
}

impl SclPin<I2CA0> for PA16<AlternateFunction> {
    fn to_i2c_pin(&self) {
        unsafe {
            // Clear the current drive strength and CONF mode as
            // well as bits 10 and 11 and then set the
            // new drive strength to Low (2mA), set the appropriate CONF
            // mode, enable open drain mode, and enable the internal
            // weak pullup
            (*OCP_SHARED::ptr()).gpio_pad_config_14.modify(
                |r, w| w.bits(
                    (r.bits() & (!0xCEF)) |
                    0x5 | // CONF mode
                    ((DriveStrength::Low as u32) << 5) |
                    (1 << 4) | // Open drain mode
                    (1 << 8)   // Weak pullup
                )
            );
        }
    }
}

impl SclPin<I2CA0> for PA27<AlternateFunction> {
    fn to_i2c_pin(&self) {
        unsafe {
            // Clear the current drive strength and CONF mode as
            // well as bits 10 and 11 and then set the
            // new drive strength to Low (2mA), set the appropriate CONF
            // mode, enable open drain mode, and enable the internal
            // weak pullup
            (*OCP_SHARED::ptr()).gpio_pad_config_23.modify(
                |r, w| w.bits(
                    (r.bits() & (!0xCEF)) |
                    0x9 | // CONF mode
                    ((DriveStrength::Low as u32) << 5) |
                    (1 << 4) | // Open drain mode
                    (1 << 8)   // Weak pullup
                )
            );
        }
    }
}


impl SdaPin<I2CA0> for PA13<AlternateFunction> {
    fn to_i2c_pin(&self) {
        unsafe {
            // Clear the current drive strength and CONF mode as
            // well as bits 10 and 11 and then set the
            // new drive strength to Low (2mA), set the appropriate CONF
            // mode, enable open drain mode, and enable the internal
            // weak pullup
            (*OCP_SHARED::ptr()).gpio_pad_config_11.modify(
                |r, w| w.bits(
                    (r.bits() & (!0xCEF)) |
                    0x1 | // CONF mode
                    ((DriveStrength::Low as u32) << 5) |
                    (1 << 4) | // Open drain mode
                    (1 << 8)   // Weak pullup
                )
            );
        }
    }
}

impl SdaPin<I2CA0> for PA15<AlternateFunction> {
    fn to_i2c_pin(&self) {
        unsafe {
            // Clear the current drive strength and CONF mode as
            // well as bits 10 and 11 and then set the
            // new drive strength to Low (2mA), set the appropriate CONF
            // mode, enable open drain mode, and enable the internal
            // weak pullup
            (*OCP_SHARED::ptr()).gpio_pad_config_13.modify(
                |r, w| w.bits(
                    (r.bits() & (!0xCEF)) |
                    0x5 | // CONF mode
                    ((DriveStrength::Low as u32) << 5) |
                    (1 << 4) | // Open drain mode
                    (1 << 8)   // Weak pullup
                )
            );
        }
    }
}

impl SdaPin<I2CA0> for PA17<AlternateFunction> {
    fn to_i2c_pin(&self) {
        unsafe {
            // Clear the current drive strength and CONF mode as
            // well as bits 10 and 11 and then set the
            // new drive strength to Low (2mA), set the appropriate CONF
            // mode, enable open drain mode, and enable the internal
            // weak pullup
            (*OCP_SHARED::ptr()).gpio_pad_config_15.modify(
                |r, w| w.bits(
                    (r.bits() & (!0xCEF)) |
                    0x5 | // CONF mode
                    ((DriveStrength::Low as u32) << 5) |
                    (1 << 4) | // Open drain mode
                    (1 << 8)   // Weak pullup
                )
            );
        }
    }
}

impl SdaPin<I2CA0> for PA30<AlternateFunction> {
    fn to_i2c_pin(&self) {
        unsafe {
            // Clear the current drive strength and CONF mode as
            // well as bits 10 and 11 and then set the
            // new drive strength to Low (2mA), set the appropriate CONF
            // mode, enable open drain mode, and enable the internal
            // weak pullup
            (*OCP_SHARED::ptr()).gpio_pad_config_24.modify(
                |r, w| w.bits(
                    (r.bits() & (!0xCEF)) |
                    0x9 | // CONF mode
                    ((DriveStrength::Low as u32) << 5) |
                    (1 << 4) | // Open drain mode
                    (1 << 8)   // Weak pullup
                )
            );
        }
    }
}


/**************************************
* Public enums
**************************************/
pub enum I2cSpeed {
    Standard = 100000,   // 100 Kbps
    Fast     = 400000,   // 400 Kbps
}


macro_rules! busy_wait {
    ($I2CX:ident) => {
        while (*$I2CX::ptr()).i2cmcs.read().busy_or_run().bits() { }
    }
}

macro_rules! bus_busy_wait {
    ($I2CX:ident) => {
        while (*$I2CX::ptr()).i2cmcs.read().busbusy_or_burst().bits() { }
    }
}


macro_rules! i2c_macro {
    ($I2CX:ident, $i2cX:ident) => {

        use cc3220sf::$I2CX;

        impl <SCL, SDA> I2c<$I2CX, (SCL, SDA)> {
            pub fn $i2cX(
                i2c: $I2CX,
                pins: (SCL, SDA),
                speed: I2cSpeed,
            ) -> Self where SCL: SclPin<$I2CX>, SDA: SdaPin<$I2CX>, {

                // Turn on power to the peripheral
                control_power(Domain::I2c, RunMode::Run,
                              PowerState::On);
                reset_power(Domain::I2c);

                pins.0.to_i2c_pin();
                pins.1.to_i2c_pin();

                unsafe {
                    // Enable I2C for master operation
                    (*$I2CX::ptr()).i2cmcr.modify(
                        |_, w| w.mfe().set_bit()
                    );

                    // Configure I2C clock period
                    // The system clock is 80MHz
                    // Standard Mode (100Kbps) => TPR = 0x27
                    // Fast Mode (400Kbps)     => TPR = 0x09
                    // See page 185 of CC3220SF Technical Reference Manual
                    // for details.
                    match speed {
                        I2cSpeed::Standard => {
                            (*$I2CX::ptr()).i2cmtpr.modify(
                                |_, w| w.tpr().bits(0x27)
                            );
                        },
                        I2cSpeed::Fast => {
                            (*$I2CX::ptr()).i2cmtpr.modify(
                                |_, w| w.tpr().bits(0x09)
                            );
                        },
                    }
                }
                
                I2c { 
                    i2c,
                    pins,
                }
            }
        }

        impl<PINS> Write for I2c<$I2CX, PINS> {
            type Error = I2cError;

            fn write(&mut self, addr: u8, bytes: &[u8])
            -> Result<(), Self::Error> {
                // Number of bytes to send
                let size: usize = bytes.len();
                let mut index: usize = 0;

                assert!(size > 0, "no data to send (bytes.len() <= 0)");

                unsafe {
                    // Specify the slave address and set the next
                    // transmission as a write
                    (*$I2CX::ptr()).i2cmsa.modify(
                        |_, w| w.sa().bits(addr)
                                .r_s().clear_bit()
                    );

                    // Put the first byte in the data register
                    (*$I2CX::ptr()).i2cmdr.write(
                        |w| w.data().bits(bytes[index])
                    );
                    index += 1;

                    bus_busy_wait!($I2CX);

                    // Send START + RUN
                    // Send STOP if there is only one byte to transfer
                    if size == 1 {
                        (*$I2CX::ptr()).i2cmcs.write(
                            |w| w.adrack_or_stop().set_bit()
                                 .error_or_start().set_bit()
                                 .busy_or_run().set_bit()
                        );
                    }
                    else {
                        (*$I2CX::ptr()).i2cmcs.write(
                            |w| w.error_or_start().set_bit()
                                 .busy_or_run().set_bit()
                        );
                    }

                    while index < size {
                        let c = bytes[index];
                        index += 1;

                        // Put byte in the data register
                        (*$I2CX::ptr()).i2cmdr.write(
                            |w| w.data().bits(c)
                        );

                        // Send RUN command (Burst continue)
                        // Set STOP if we are on the last byte
                        if index == (size - 1) {
                            (*$I2CX::ptr()).i2cmcs.write(
                                |w| w.adrack_or_stop().set_bit()
                                     .busy_or_run().set_bit()
                            );
                        }
                        else {
                            (*$I2CX::ptr()).i2cmcs.write(
                                |w| w.busy_or_run().set_bit()
                            );
                        }
                    }

                    busy_wait!($I2CX);
                }

                Ok(())
            }
        }

        impl<PINS> Read for I2c<$I2CX, PINS> {
            type Error = I2cError;

            fn read(&mut self, addr: u8, buffer: &mut [u8])
            -> Result<(), Self::Error> {
                let size: usize = buffer.len();
                let mut index: usize = 0;

                assert!(size > 0, "no data to send (bytes.len() <= 0)");

                unsafe {
                    // Specify the slave address and set the read bit
                    (*$I2CX::ptr()).i2cmsa.write(
                        |w| w.sa().bits(addr)
                    );
                    (*$I2CX::ptr()).i2cmsa.write(
                        |w| w.r_s().set_bit()
                    );

                    bus_busy_wait!($I2CX);

                    // If there is only one byte to receive, send
                    // START, RUN, STOP
                    if size == 1 {
                        (*$I2CX::ptr()).i2cmcs.write(
                            |w| w.adrack_or_stop().set_bit()
                                 .error_or_start().set_bit()
                                 .busy_or_run().set_bit()
                        );
                    }
                    // Send START + RUN + ACK
                    else {
                        (*$I2CX::ptr()).i2cmcs.write(
                            |w| w.datack_or_ack().set_bit()
                                 .error_or_start().set_bit()
                                 .busy_or_run().set_bit()
                        );

                        busy_wait!($I2CX);

                        // Read first data byte
                        buffer[index] = (*$I2CX::ptr()).i2cmdr.read().data().bits();
                        index += 1;

                        while index < (size - 1) {
                            // Send RUN + ACK
                            (*$I2CX::ptr()).i2cmcs.write(
                                |w| w.datack_or_ack().set_bit()
                                     .busy_or_run().set_bit()
                            );
                            busy_wait!($I2CX);
                            buffer[index] = (*$I2CX::ptr()).i2cmdr.read().data().bits();
                            index += 1;
                        }

                        // Send RUN + STOP
                        (*$I2CX::ptr()).i2cmcs.write(
                            |w| w.adrack_or_stop().set_bit()
                                 .busy_or_run().set_bit()
                        );
                    }
                    
                    busy_wait!($I2CX);

                    // Read the last byte
                    buffer[index] = (*$I2CX::ptr()).i2cmdr.read().data().bits();
                }

                Ok(())
            }
        }

        impl<PINS> WriteRead for I2c<$I2CX, PINS> {
            type Error = I2cError;

            fn write_read(&mut self, addr: u8, bytes: &[u8],
                          buffer: &mut [u8])
            -> Result<(), Self::Error> {
                let write_len: usize = bytes.len();
                let read_len: usize = buffer.len();
                let mut write_index: usize = 0;
                let mut read_index: usize = 0;

                assert!((read_len != 0) || (write_len != 0),
                        "nothing to write or read");

                if read_len == 0 {
                    // We are only writing
                    return self.write(addr, bytes);
                }
                else if write_len == 0 {
                    // We are only reading
                    return self.read(addr, buffer);
                }
                
                unsafe {
                    // Write the slave address and clear the receive bit
                    (*$I2CX::ptr()).i2cmsa.write(
                        |w| w.sa().bits(addr)
                    );
                    (*$I2CX::ptr()).i2cmsa.modify(
                        |r, w| w.bits(
                            r.bits() & (!0x01)
                        )
                    );

                    // Send the first byte
                    (*$I2CX::ptr()).i2cmdr.write(
                        |w| w.data().bits(bytes[write_index])
                    );
                    write_index += 1;

                    // Wait for the bus to clear
                    bus_busy_wait!($I2CX);

                    // Send START + RUN
                    (*$I2CX::ptr()).i2cmcs.write(
                        |w| w.bits(0x03)
                    );

                    // Wait for I2C
                    busy_wait!($I2CX);

                    while write_index < write_len {
                        // Send next byte of data
                        (*$I2CX::ptr()).i2cmdr.write(
                            |w| w.data().bits(bytes[write_index])
                        );
                        write_index += 1;

                        // Send RUN
                        (*$I2CX::ptr()).i2cmcs.write(
                            |w| w.bits(0x01)
                        );

                        busy_wait!($I2CX);
                    }

                    // Setup for read
                    // Write slave address and set read bit
                    (*$I2CX::ptr()).i2cmsa.write(
                        |w| w.sa().bits(addr)
                    );
                    (*$I2CX::ptr()).i2cmsa.modify(
                        |r, w| w.bits(
                            r.bits() | 0x01
                        )
                    );

                    if read_len == 1 {
                        // Send START + STOP
                        (*$I2CX::ptr()).i2cmcs.write(
                            |w| w.bits(0x06)
                        );
                        busy_wait!($I2CX);
                        buffer[read_index] = (*$I2CX::ptr()).i2cmdr
                                                            .read()
                                                            .data()
                                                            .bits();
                    }
                    else {
                        // Send START + ACK
                        (*$I2CX::ptr()).i2cmcs.write(
                            |w| w.bits(0x0A)
                        );

                        busy_wait!($I2CX);

                        buffer[read_index] = (*$I2CX::ptr()).i2cmdr
                                                            .read()
                                                            .data()
                                                            .bits();
                        read_index += 1;

                        while read_index < (read_len-1) {
                            // Send RUN + ACK
                            (*$I2CX::ptr()).i2cmcs.write(
                                |w| w.bits(0x09)
                            );

                            busy_wait!($I2CX);

                            buffer[read_index] = (*$I2CX::ptr()).i2cmdr
                                                                .read()
                                                                .data()
                                                                .bits();
                            read_index += 1;
                        }

                        // Send RUN + STOP
                        (*$I2CX::ptr()).i2cmcs.write(
                            |w| w.bits(0x05)
                        );

                        busy_wait!($I2CX);

                        buffer[read_index] = (*$I2CX::ptr()).i2cmdr
                                                            .read()
                                                            .data()
                                                            .bits();
                    }
                }

                Ok(())
            }
        }
    }
}


i2c_macro!(I2CA0, i2ca0);
