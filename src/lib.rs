#![no_std]

extern crate silica_stm32f2xx;

pub use silica_stm32f2xx::*;
pub use silica_stm32f2xx::rcc::*;

pub const PERIPH_BASE: usize = 0x40000000;
pub const APB2PERIPH_BASE: usize = PERIPH_BASE + 0x00010000;
pub const AHB1PERIPH_BASE: usize = PERIPH_BASE + 0x00020000;

pub const GPIOC_BASE: usize = AHB1PERIPH_BASE + 0x0800;
pub const GPIOG_BASE: usize = AHB1PERIPH_BASE + 0x1800;
pub const USART6_BASE: usize = APB2PERIPH_BASE + 0x1400;
pub const RCC_BASE: usize = AHB1PERIPH_BASE + 0x3800;

pub const USART6: *mut usart::USARTRegisters = USART6_BASE as *mut usart::USARTRegisters;
pub const GPIOC: *mut gpio::PortRegisters = GPIOC_BASE as *mut gpio::PortRegisters;
pub const GPIOG: *mut gpio::PortRegisters = GPIOG_BASE as *mut gpio::PortRegisters;
pub const RCC: *mut rcc::RCCRegisters = RCC_BASE as *mut rcc::RCCRegisters;

pub const GPIOPORTC: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIOC,
    clock: RCCPeripheral { rcc: RCC, clock: Clock::GPIOC }
};
pub const GPIOPORTG: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIOG,
    clock: RCCPeripheral { rcc: RCC, clock: Clock::GPIOG }
};
