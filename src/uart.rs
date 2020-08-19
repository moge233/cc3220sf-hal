/*
 *
 * Serial Communication Module
 *
 */


use core::marker::PhantomData;

use hal::serial::{Read, Write};

use crate::sysctl::*;
use crate::gpio::AlternateFunction;
use crate::gpio::DriveStrength;

/******************************************
 * Available GPIO for UART0
 * TX Pins          RX Pins
 * GPIO1  (PA1)     GPIO2  (PA2)
 * GPIO7  (PA7)
 * GPIO12 (PB4)     GPIO13 (PB5)
 * GPIO30 (PD6)     GPIO31 (PD7)
 *
 *
 * Available GPIO for UART1
 * TX Pins          RX Pins
 * GPIO1  (PA01)    GPIO2  (PA02)
 * GPIO3  (PA03)    GPIO4  (PA04)
 * GPIO10 (PA12)    GPIO11 (PA13)
 * GPIO16 (PA20)    GPIO17 (PA21)
 * GPIO23 (PA27)    GPIO24 (PA30)
 *                  GPIO31 (PA37)
******************************************/
use crate::gpio::gpioa0::{PA01, PA02, PA03, PA04, PA07};
use crate::gpio::gpioa1::{PA12, PA13, PA14, PA15,};
use crate::gpio::gpioa2::{PA20, PA21, PA27,};
use crate::gpio::gpioa3::{PA30, PA36, PA37,};

use cc3220sf::OCP_SHARED;


/**************************************
* Public trait definitions
**************************************/
pub trait TxPin<UART> {
    fn to_uart_pin(&self) {
        unimplemented!()
    }
}
pub trait RxPin<UART> { 
    fn to_uart_pin(&self) {
        unimplemented!()
    }
}


/**************************************
* Public struct definitions
**************************************/
pub struct Serial<UART, PINS> {
    #[allow(dead_code)]
    uart: UART,

    #[allow(dead_code)]
    pins: PINS,
}
struct BaudRateDivisor {
    integer: u32,
    fraction: u32,
}
pub struct Tx<UART> {
    _uart: PhantomData<UART>,
}
pub struct Rx<UART> {
    _uart: PhantomData<UART>,
}
pub struct BaudRate(pub u32);

#[derive(Debug)]
pub struct UartError(pub u32);


/**************************************
* Implementations
**************************************/
impl BaudRateDivisor {
    pub fn new(baud: u32) -> BaudRateDivisor {
        let mut b = BaudRateDivisor{ integer: 0, fraction: 0 };
        let div = baud_rate_divisor_integer(80000000, baud);
        let i = div as u32;
        let f = (div % 1.00) as f32;
        let f = baud_rate_divisor_fraction(f);
        b.integer = i;
        b.fraction = f;
        b
    }
}


// UART 0 implementations
impl TxPin<UARTA0> for PA01<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe {
            (*OCP_SHARED::ptr()).gpio_pad_config_1.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x3 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl TxPin<UARTA0> for PA14<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe {
            (*OCP_SHARED::ptr()).gpio_pad_config_12.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x7 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl TxPin<UARTA0> for PA36<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_30.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x9 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}

impl RxPin<UARTA0> for PA02<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_2.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x3 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl RxPin<UARTA0> for PA07<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_7.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0xB |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl RxPin<UARTA0> for PA15<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_13.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x7 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl RxPin<UARTA0> for PA37<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_31.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x9 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}

// UART 1 implementations
impl TxPin<UARTA1> for PA01<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_1.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x6 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl TxPin<UARTA1> for PA03<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_3.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x6 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl TxPin<UARTA1> for PA12<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_10.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x7 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl TxPin<UARTA1> for PA20<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_16.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x5 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl TxPin<UARTA1> for PA27<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_23.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x2 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}

impl RxPin<UARTA1> for PA02<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_2.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x6 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl RxPin<UARTA1> for PA04<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_4.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x6 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl RxPin<UARTA1> for PA13<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_11.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x7 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl RxPin<UARTA1> for PA21<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_17.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x5 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl RxPin<UARTA1> for PA30<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_24.modify(
                // Clear current drive strength and CONF mode then set
                // new drive strength to Low (2mA) and new CONF mode
                // to appropriate config mode
                |r, w| w.bits((r.bits() & !0xEF) | 0x2 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}
impl RxPin<UARTA1> for PA37<AlternateFunction> {
    fn to_uart_pin(&self) {
        unsafe{
            (*OCP_SHARED::ptr()).gpio_pad_config_31.modify(
                |r, w| w.bits((r.bits() & !0xEF) | 0x2 |
                              ((DriveStrength::Low as u32) << 5))
            );
        }
    }
}


/**************************************
* Public enums
**************************************/
pub enum Parity {
    Even = (3 << 1),
    Odd = (2 << 1),
    None = (0 << 1),
}

pub enum StopBits {
    One = (0 << 3),
    Two = (1 << 3),
}

pub enum DataLength {
    Eight = (3 << 5),
    Seven = (2 << 5),
    Six = (1 << 5),
    Five = (0 << 5),
}


/**************************************
* Utility functions
**************************************/
fn baud_rate_divisor_integer(clock: u32, baud: u32) -> f32 {
    // Calculate the BRD integer value using the provided clock rate
    // (80 MHz for the CC3220SF) and the desired BAUD rate
    // 
    // Returns an f32 that can be passed to the
    // baud_rate_divisor_fraction() function after taking its modulo
    // with 64 (% 64).
    (clock as f32 / (16.00 * baud as f32)) as f32
}

fn baud_rate_divisor_fraction(i: f32) -> u32 {
    // Calculate the BRD fraction value from the BRD integer value
    // calculated used the baud_rate_divisor_integer() function
    (i * 64.00 + 0.5) as u32
}


/**************************************
* Serial macro definition
**************************************/
macro_rules! serial_macro {
    ($UARTAX:ident, $uartax:ident, $Uartx:ident) => {

        use cc3220sf::$UARTAX;

        impl<TX, RX> Serial<$UARTAX, (TX, RX)> {
            pub fn $uartax(
                uart: $UARTAX,
                pins: (TX, RX),
                mut baud: BaudRate,
                data_length: DataLength,
                par: Parity,
                stop_bits: StopBits,
            ) -> Self where TX: TxPin<$UARTAX>, RX: RxPin<$UARTAX> {

                // Turn on power to the peripheral and then reset it
                control_power(Domain::$Uartx,
                              RunMode::Run,
                              PowerState::On);
                reset_power(Domain::$Uartx);

                pins.0.to_uart_pin();
                pins.1.to_uart_pin();

                unsafe {

                    // Enable the UART to start (UARTEN = UARTAx.CTL[0])
                    (*$UARTAX::ptr()).uartctl.modify(
                        |_, w| w.uarten().set_bit()
                    );

                    // Enable the UART FIFO (FIFOEN = UARTAx.LCRH[4] = 1)
                    (*$UARTAX::ptr()).uartlcrh.modify(
                        |_, w| w.fen().set_bit()
                    );


                    // Enable the RX and TX of the UART
                    // TXEN = UARTAx.CTL[8]
                    // RXEN = UARTAx.CTL[9]
                    // UARTEN = UARTAx.CTL[0]
                    (*$UARTAX::ptr()).uartctl.modify(
                        |_, w| w.txe().set_bit()
                                .rxe().set_bit()
                                .uarten().set_bit()
                    );

                    // Set the FIFO levels
                    (*$UARTAX::ptr()).uartifls.modify(
                        |_, w| w.rxiflsel().bits(0x4)
                                .txiflsel().bits(0x1)
                    );

                    // Disable UART for the remaining steps
                    (*$UARTAX::ptr()).uartctl.modify(
                        |_, w| w.uarten().clear_bit()
                    );

                    // Get the desired Baud Rate Divisor values
                    if (baud.0 * 16) > 80_000_000 { // 80MHz clock
                        baud = BaudRate(baud.0 / 2);
                        // Turn on high speed mode
                        // UARTAx.CTL[5] = 1
                        (*$UARTAX::ptr()).uartctl.modify(
                            |_, w| w.hse().set_bit()
                        );
                    }
                    else {
                        // Turn off high speed mode
                        (*$UARTAX::ptr()).uartctl.modify(
                            |_, w| w.hse().clear_bit()
                        );
                    }

                    let brd = BaudRateDivisor::new(baud.0);
                    let par_u32: u32 = par as u32;
                    let stop_u32: u32 = stop_bits as u32;
                    let data_length_u32: u32 = data_length as u32;

                    // Set baud rate divisor integer and fraction values
                    // Integer: UARTAx.IBRD[15:0]
                    // Fraction: UARTAx.FBRD[5:0]
                    (*$UARTAX::ptr()).uartibrd.write(
                        |w| w.divint().bits((brd.integer & 0xFFFF) as u16)
                    );
                    (*$UARTAX::ptr()).uartfbrd.write(
                        |w| w.divfrac().bits((brd.fraction & 0x3F) as u8)
                    );

                    // Set parity, data length, and stop bits
                    // PEN (parity enable) UARTAx.LCRH[1]
                    // WLEN (data length) UARTAx.LCRH[6:5]
                    // STP2 (stop bits) UARTAx.LCRH[3]
                    (*$UARTAX::ptr()).uartlcrh.modify(
                        |r, w| w.bits(
                            r.bits() | par_u32 | data_length_u32 | stop_u32
                        )
                    );

                    // Clear the flags register UARTAx.FR
                    (*$UARTAX::ptr()).uartfr.write(
                        |w| w.bits(0)
                    );

                    // Re-enable the UART
                    (*$UARTAX::ptr()).uartctl.modify(
                        |_, w| w.uarten().set_bit()
                    );
                }

                Serial {
                    uart,
                    pins,
                }

            }

            pub fn split(self) -> (Tx<$UARTAX>, Rx<$UARTAX>) {
                (
                    Tx { _uart: PhantomData, },
                    Rx { _uart: PhantomData, },
                )
            }
        }

        impl Read<u8> for Rx<$UARTAX> {
            type Error = UartError;

            fn read(&mut self) -> nb::Result<u8, Self::Error> {
                let fxe = unsafe {
                    (*$UARTAX::ptr()).uartfr.read().rxfe().bits()
                };

                if fxe {
                    // There is nothing to read (FR.RXFE == 1)
                    return Err(nb::Error::WouldBlock);
                }

                let result = unsafe {
                    (*$UARTAX::ptr()).uartdr.read().data().bits()
                };
                Ok(result)
            }
        }

        impl Write<u8> for Tx<$UARTAX> {
            type Error = UartError;

            fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
                let txff = unsafe {
                    (*$UARTAX::ptr()).uartfr.read().txff().bits()
                };

                // See if there is space in the FIFO register
                // If TXFF is set, that means the FIFO is full and we
                // can not send the byte
                if txff {
                    return Err(nb::Error::WouldBlock);
                }

                // Otherwise the FIFO is not full and we can transmit the
                // data
                unsafe {
                    (*$UARTAX::ptr()).uartdr.write(
                        |w| w.data().bits(word)
                    );
                }
                Ok(())
            }

            fn flush(&mut self) -> nb::Result<(), Self::Error> {
                let txfe = unsafe {
                    (*$UARTAX::ptr()).uartfr.read().busy().bits()
                };

                // If BUSY is set, that means the UART is busy transmitting
                // data
                if txfe {
                    return Err(nb::Error::WouldBlock);
                }

                Ok(())
            }
        }
    } // end serial_macro expansion block
} // end serial_macro


serial_macro!(UARTA0, uarta0, Uart0);
serial_macro!(UARTA1, uarta1, Uart1);
