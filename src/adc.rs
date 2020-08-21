/*
 *
 * ADC Module
 *
 */
 

use hal::adc::{Channel, OneShot};

use cc3220sf::OCP_SHARED;

use crate::gpio::AlternateFunction;
/*********************************************
 * Available GPIO for ADC
 * GPIO02 (ADC_CH0)
 * GPIO03 (ADC_CH1)
 * GPIO04 (ADC_CH2)
 * GPIO05 (ADC_CH3)
 ********************************************/
use crate::gpio::gpioa0::{PA02, PA03, PA04, PA05};


/**************************************
* Public trait definitions
**************************************/
pub trait AdcPin<Adc> {
    fn to_adc_pin(&mut self) {
        unimplemented!()
    }
    
    fn enable_channel(&mut self) {
        unimplemented!()
    }
    
    fn fifo_read(&mut self) -> u32 {
        unimplemented!()
    }
    
    fn get_fifo_lvl(&mut self) -> u8 {
        unimplemented!()
    }
}


/**************************************
* Public struct definitions
**************************************/
pub struct Adc<ADC> {
    #[allow(dead_code)]
    adc: ADC,
}


/**************************************
* Implementations
**************************************/
impl AdcPin<ADC> for PA02<AlternateFunction> {
    fn to_adc_pin(&mut self) {
        unsafe {
            // Isolate the input
            (*OCP_SHARED::ptr()).gpio_pad_cmn_config.modify(
                |r, w| w.bits(
                    r.bits() | 0x200 // Channel 0
                )
            );
            
            // Isolate the output
            (*OCP_SHARED::ptr()).gpio_pad_config_2.write(
                |w| w.bits(0xC00)
            );
        }
    }
    
    fn enable_channel(&mut self) {
        unsafe {
            (*ADC::ptr()).ch_enable.modify(
                |_, w| w.adc_channel0_en().set_bit()
            )
        }
    }
    
    fn fifo_read(&mut self) -> u32 {
        let mut data: u32;
        unsafe {
            data = (*ADC::ptr()).channel0fifodata.read().bits();
            data = (data >> 2) & 0xFFF;
        }
        data
    }
    
    fn get_fifo_lvl(&mut self) -> u8 {
        let lvl: u32;
        unsafe {
            lvl = (*ADC::ptr()).ch0_fifo_lvl.read().bits() & 0x7;
        }
        lvl as u8
    }
}

impl AdcPin<ADC> for PA03<AlternateFunction> {
    fn to_adc_pin(&mut self) {
        unsafe {
            // Isolate the input
            (*OCP_SHARED::ptr()).gpio_pad_cmn_config.modify(
                |r, w| w.bits(
                    r.bits() | 0x400 // Channel 0
                )
            );
            
            // Isolate the output
            (*OCP_SHARED::ptr()).gpio_pad_config_3.write(
                |w| w.bits(0xC00)
            );
        }
    }
    
    fn enable_channel(&mut self) {
        unsafe {
            (*ADC::ptr()).ch_enable.modify(
                |_, w| w.adc_channel2_en().set_bit()
            )
        }
    }
    
    fn fifo_read(&mut self) -> u32 {
        let mut data: u32;
        unsafe {
            data = (*ADC::ptr()).channel2fifodata.read().bits();
            data = (data >> 2) & 0xFFF;
        }
        data
    }
    
    fn get_fifo_lvl(&mut self) -> u8 {
        let lvl: u32;
        unsafe {
            lvl = (*ADC::ptr()).ch2_fifo_lvl.read().bits() & 0x7;
        }
        lvl as u8
    }
}

impl AdcPin<ADC> for PA04<AlternateFunction> {
    fn to_adc_pin(&mut self) {
        unsafe {
            // Isolate the input
            (*OCP_SHARED::ptr()).gpio_pad_cmn_config.modify(
                |r, w| w.bits(
                    r.bits() | 0x800 // Channel 0
                )
            );
            
            // Isolate the output
            (*OCP_SHARED::ptr()).gpio_pad_config_4.write(
                |w| w.bits(0xC00)
            );
        }
    }
    
    fn enable_channel(&mut self) {
        unsafe {
            (*ADC::ptr()).ch_enable.modify(
                |_, w| w.adc_channel4_en().set_bit()
            )
        }
    }
    
    fn fifo_read(&mut self) -> u32 {
        let mut data: u32;
        unsafe {
            data = (*ADC::ptr()).channel4fifodata.read().bits();
            data = (data >> 2) & 0xFFF;
        }
        data
    }
    
    fn get_fifo_lvl(&mut self) -> u8 {
        let lvl: u32;
        unsafe {
            lvl = (*ADC::ptr()).ch4_fifo_lvl.read().bits() & 0x7;
        }
        lvl as u8
    }
}

impl AdcPin<ADC> for PA05<AlternateFunction> {
    fn to_adc_pin(&mut self) {
        // Isolate the input
        unsafe {
            (*OCP_SHARED::ptr()).gpio_pad_cmn_config.modify(
                |r, w| w.bits(
                    r.bits() | 0x1000 // Channel 0
                )
            );
            
            // Isolate the output
            (*OCP_SHARED::ptr()).gpio_pad_config_5.write(
                |w| w.bits(0xC00)
            );
        }
    }
    
    fn enable_channel(&mut self) {
        unsafe {
            (*ADC::ptr()).ch_enable.modify(
                |_, w| w.adc_channel6_en().set_bit()
            )
        }
    }
    
    fn fifo_read(&mut self) -> u32 {
        let mut data: u32;
        unsafe {
            data = (*ADC::ptr()).channel6fifodata.read().bits();
            data = (data >> 2) & 0xFFF;
        }
        data
    }
    
    fn get_fifo_lvl(&mut self) -> u8 {
        let lvl: u32;
        unsafe {
            lvl = (*ADC::ptr()).ch6_fifo_lvl.read().bits() & 0x7;
        }
        lvl as u8
    }
}

impl Channel<ADC> for PA02<AlternateFunction> {
    type ID = u8;
    
    fn channel() -> u8 { 0u8 }
}

impl Channel<ADC> for PA03<AlternateFunction> {
    type ID = u8;
    
    fn channel() -> u8 { 2u8 }
}

impl Channel<ADC> for PA04<AlternateFunction> {
    type ID = u8;
    
    fn channel() -> u8 { 4u8 }
}

impl Channel<ADC> for PA05<AlternateFunction> {
    type ID = u8;
    
    fn channel() -> u8 { 6u8 }
}


macro_rules! adc_macro {
    ($ADCX:ident, $adcx:ident) => {
        
        use cc3220sf::$ADCX;
        
        impl Adc<$ADCX> {
            pub fn $adcx<ADCP>(
                adc: $ADCX,
                pin: &mut ADCP
            ) -> Self 
            where
                ADCP: AdcPin<$ADCX>
            {
                /*
                 * Initialize the ADC module for application use
                 */
                
                let mut s = Self {
                    adc
                };
                
                // Turn the pin into an analog pin
                pin.to_adc_pin();
                
                // Enable the ADC for application use
                s.enable_adc();
                
                // Enable the channel
                pin.enable_channel();
                
                // Empty 5 samples from the ADC
                for _ in (0..5) {
                    while(pin.get_fifo_lvl() == 0) { }
                    let _discard_sample = pin.fifo_read();
                }
                
                // Return the ADC for use
                s
            }
            
            pub fn enable_adc(&mut self) {
                // Enables the ADC module
                unsafe {
                    (*$ADCX::ptr()).ctrl.modify(
                        |r, w| w.bits(
                            r.bits() | 0x01
                        )
                    )
                }
            }
            
            pub fn disable_adc(&mut self) {
                // Disables the ADC module
                unsafe {
                    (*$ADCX::ptr()).ctrl.modify(
                        |r, w| w.bits(
                            r.bits() & (!0x01)
                        )
                    )
                }
            }
            
            pub fn convert<ADCP>(&mut self, pin: &mut ADCP) -> u32 
            where
                ADCP: AdcPin<$ADCX>
            {
                // Performs an ADC conversion on the channel of the given pin
                while pin.get_fifo_lvl() == 0 {
                }
                let data: u32 = pin.fifo_read();
                data
            }
        }
        
        impl<WORD, PIN> OneShot<$ADCX, WORD, PIN> for Adc<$ADCX> 
        where
            WORD: From<u32>,
            PIN: Channel<$ADCX, ID=u8> + AdcPin<$ADCX>
        {
            type Error = ();
            
            fn read(&mut self, _pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
                let result = self.convert(_pin);
                Ok(result.into())
            }
        }
    }
}


adc_macro!(ADC, adc);
