/*
 *
 * General Purpose Timers
 *
 */


use hal::timer::{Cancel, CountDown, Periodic};

use void::Void;

use crate::sysctl;


/**************************************
 * Public struct definitions
 *************************************/
pub struct Timer<TIM> {
    #[allow(dead_code)]
    tim: TIM,
    #[allow(dead_code)]
    timeout: Ticks,
    #[allow(dead_code)]
    mode: TimerMode,
}
#[derive(Debug)]
pub struct TimerError { }
pub struct Ticks(pub u32);


/**************************************
 * Public enums
 *************************************/
pub enum TimerMode {
    OneShot,
    Periodic,
}


/**************************************
 * Timer macro definition
 *************************************/
macro_rules! timer_macro {
    ($TIMAX:ident, $timax:ident, $Timx:ident, $mode:expr)
    => {

        use cc3220sf::$TIMAX;

        impl Cancel for Timer<$TIMAX> {
            type Error = TimerError;
            
            fn cancel(&mut self) -> Result<(), Self::Error> {

                self.timeout = Ticks(0);

                unsafe {
                    // Disable the timer
                    (*$TIMAX::ptr()).ctl.modify(
                        |r, w| w.bits(r.bits() & !0x1) // write 0 to bit 0
                    );

                    // Clear the timer mode register
                    (*$TIMAX::ptr()).tamr.write(
                        |w| w.bits(0x00000000)
                    );

                    // Clear timer interval load value
                    (*$TIMAX::ptr()).tailr.write(
                        |w| w.bits(self.timeout.0) 
                    );

                    // Set the timer value register to its reset value
                    (*$TIMAX::ptr()).tav.write(
                        |w| w.bits(0xFFFFFFFF)
                    );
                }
                Result::Ok(()) 
            }
        }

        impl CountDown for Timer<$TIMAX> {
            type Time = Ticks;
            
            fn start<T>(&mut self, count: T)
            where T: Into<Ticks> {

                sysctl::control_power(sysctl::Domain::$Timx,
                                      sysctl::RunMode::Run,
                                      sysctl::PowerState::On);

                self.timeout = count.into();

                unsafe {
                    // Ensure the timer is disabled before making
                    // any changes
                    (*$TIMAX::ptr()).ctl.modify(
                        |r, w| w.bits(r.bits() & !0x1) // write 0 to bit 0
                    );

                    // Write a value of 0 to the CFG register
                    (*$TIMAX::ptr()).cfg.write(
                        |w| w.bits(0x00000000)
                    );
                  
                    match self.mode {
                        TimerMode::OneShot => {
                            // Configure timer as a one shot timer
                            (*$TIMAX::ptr()).tamr.write(
                                |w| w.tamr().bits(1)
                            );
                        }
                        _ => {
                            // Configure timer as a periodic timer
                            (*$TIMAX::ptr()).tamr.write(
                                |w| w.tamr().bits(2)
                            );
                        }
                    }

                    // Set timer interval load value
                    (*$TIMAX::ptr()).tailr.write(
                        |w| w.bits(self.timeout.0) 
                    );

                    // Enable interrupts??
                    (*$TIMAX::ptr()).imr.write(
                        |w| w.bits(1)
                    );

                    // Enable the timer to start counting
                    (*$TIMAX::ptr()).ctl.modify(
                        |r, w| w.bits(r.bits() | 0x3) // write 1 to bits 0,1
                    );
                }
            }

            fn wait(&mut self) -> Result<(), nb::Error<Void>> {
                let int_status;
                unsafe {
                    int_status = (*$TIMAX::ptr()).ris.read().bits() & 0x1;
                }
                if int_status == 1 {
                    // Poll the raw interrupt status bit until it is 1
                    // Write a 1 to the interrupt clear register
                    unsafe {
                        (*$TIMAX::ptr()).icr.modify(
                            |r, w| w.bits(r.bits() | 0x1)
                        );
                    }
                    Ok(())
                }
                else {
                    Err(nb::Error::WouldBlock)
                }
            }
        }

        impl Periodic for Timer<$TIMAX> { }

        impl Timer<$TIMAX> {
            
            pub fn $timax<T>(tim: $TIMAX, timeout: T) -> Self
            where T: Into<Ticks> {
                let mut timer = Timer {
                    tim,
                    timeout: Ticks(0),
                    mode: $mode,
                };

                sysctl::control_power(sysctl::Domain::$Timx,
                                      sysctl::RunMode::Run,
                                      sysctl::PowerState::On);

                timer.start(timeout);
                timer
            }

            pub fn to_one_shot(mut self) -> Self {
                self.cancel().unwrap();
                let mut timer = Timer {
                    tim: self.tim,
                    timeout: Ticks(0),
                    mode: TimerMode::OneShot,
                };

                timer.start(self.timeout);
                timer
            }

            pub fn to_periodic(mut self) -> Self {
                self.cancel().unwrap();
                let mut timer = Timer {
                    tim: self.tim,
                    timeout: Ticks(0),
                    mode: TimerMode::Periodic,
                };

                timer.start(self.timeout);
                timer
            }

            pub fn clear_interrupts(self) {
                unsafe {
                    (*$TIMAX::ptr()).icr.modify(
                        |r, w| w.bits(r.bits() | 0x1)
                    );
                }
            }
        }

    }
}


timer_macro!(TIMERA0, timera0, Timer0, TimerMode::Periodic);
timer_macro!(TIMERA1, timera1, Timer1, TimerMode::Periodic);
timer_macro!(TIMERA2, timera2, Timer2, TimerMode::Periodic);
timer_macro!(TIMERA3, timera3, Timer3, TimerMode::Periodic);
