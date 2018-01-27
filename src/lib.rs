#![no_std]
#![feature(drop_types_in_const)]

extern crate silica_stm32f2xx;

pub use silica_stm32f2xx::*;
use silica_stm32f2xx::rcc::*;

pub const PERIPH_BASE: usize = 0x40000000;
pub const APB1PERIPH_BASE: usize = PERIPH_BASE;
pub const APB2PERIPH_BASE: usize = PERIPH_BASE + 0x00010000;
pub const AHB1PERIPH_BASE: usize = PERIPH_BASE + 0x00020000;
pub const AHB2PERIPH_BASE: usize = PERIPH_BASE + 0x10000000;
pub const AHB3PERIPH_BASE: usize = PERIPH_BASE + 0x60000000;

pub const TIM2_BASE: usize = APB1PERIPH_BASE;
pub const TIM3_BASE: usize = APB1PERIPH_BASE + 0x0400;
pub const TIM4_BASE: usize = APB1PERIPH_BASE + 0x0800;
pub const TIM5_BASE: usize = APB1PERIPH_BASE + 0x0C00;
pub const TIM6_BASE: usize = APB1PERIPH_BASE + 0x1000;
pub const TIM7_BASE: usize = APB1PERIPH_BASE + 0x1400;
pub const TIM12_BASE: usize = APB1PERIPH_BASE + 0x1800;
pub const TIM13_BASE: usize = APB1PERIPH_BASE + 0x1C00;
pub const TIM14_BASE: usize = APB1PERIPH_BASE + 0x2000;
pub const RTC_BKP_BASE: usize = APB1PERIPH_BASE + 0x2800;
pub const WWDG_BASE: usize = APB1PERIPH_BASE + 0x2C00;
pub const IWDG_BASE: usize = APB1PERIPH_BASE + 0x3000;
pub const SPI2_BASE: usize = APB1PERIPH_BASE + 0x3800;
pub const SPI3_BASE: usize = APB1PERIPH_BASE + 0x3C00;
pub const USART2_BASE: usize = APB1PERIPH_BASE + 0x4400;
pub const USART3_BASE: usize = APB1PERIPH_BASE + 0x4800;
pub const UART4_BASE: usize = APB1PERIPH_BASE + 0x4C00;
pub const UART5_BASE: usize = APB1PERIPH_BASE + 0x5000;
pub const I2C1_BASE: usize = APB1PERIPH_BASE + 0x5400;
pub const I2C2_BASE: usize = APB1PERIPH_BASE + 0x5800;
pub const I2C3_BASE: usize = APB1PERIPH_BASE + 0x5C00;
pub const CAN1_BASE: usize = APB1PERIPH_BASE + 0x6400;
pub const CAN2_BASE: usize = APB1PERIPH_BASE + 0x6800;
pub const PWR_BASE: usize = APB1PERIPH_BASE + 0x7000;
pub const DAC_BASE: usize = APB1PERIPH_BASE + 0x7400;

pub const TIM1_BASE: usize = APB2PERIPH_BASE;
pub const TIM8_BASE: usize = APB2PERIPH_BASE + 0x0400;
pub const USART1_BASE: usize = APB2PERIPH_BASE + 0x1000;
pub const USART6_BASE: usize = APB2PERIPH_BASE + 0x1400;
pub const ADC1_BASE: usize = APB2PERIPH_BASE + 0x2000;
pub const ADC2_BASE: usize = APB2PERIPH_BASE + 0x2100;
pub const ADC3_BASE: usize = APB2PERIPH_BASE + 0x2200;
pub const ADC_COMMON_BASE: usize = APB2PERIPH_BASE + 0x2300;
pub const SDIO_BASE: usize = APB2PERIPH_BASE + 0x2C00;
pub const SPI1_BASE: usize = APB2PERIPH_BASE + 0x3000;
pub const SYSCFG_BASE: usize = APB2PERIPH_BASE + 0x3800;
pub const EXTI_BASE: usize = APB2PERIPH_BASE + 0x3C00;
pub const TIM9_BASE: usize = APB2PERIPH_BASE + 0x4000;
pub const TIM10_BASE: usize = APB2PERIPH_BASE + 0x4400;
pub const TIM11_BASE: usize = APB2PERIPH_BASE + 0x4800;

pub const GPIOA_BASE: usize = AHB1PERIPH_BASE + 0x0000;
pub const GPIOB_BASE: usize = AHB1PERIPH_BASE + 0x0400;
pub const GPIOC_BASE: usize = AHB1PERIPH_BASE + 0x0800;
pub const GPIOD_BASE: usize = AHB1PERIPH_BASE + 0x0C00;
pub const GPIOE_BASE: usize = AHB1PERIPH_BASE + 0x1000;
pub const GPIOF_BASE: usize = AHB1PERIPH_BASE + 0x1400;
pub const GPIOG_BASE: usize = AHB1PERIPH_BASE + 0x1800;
pub const GPIOH_BASE: usize = AHB1PERIPH_BASE + 0x1C00;
pub const GPIOI_BASE: usize = AHB1PERIPH_BASE + 0x2000;
pub const CRC_BASE: usize = AHB1PERIPH_BASE + 0x3000;
pub const RCC_BASE: usize = AHB1PERIPH_BASE + 0x3800;
pub const FLASH_BASE: usize = AHB1PERIPH_BASE + 0x3C00;
pub const BKPSRAM_BASE: usize = AHB1PERIPH_BASE + 0x4000;
pub const DMA1_BASE: usize = AHB1PERIPH_BASE + 0x6000;
pub const DMA2_BASE: usize = AHB1PERIPH_BASE + 0x6400;
pub const ETHMAC_BASE: usize = AHB1PERIPH_BASE + 0x8000;
pub const USBOTGHS_BASE: usize = AHB1PERIPH_BASE + 0x20000;

pub const USBOTGFS_BASE: usize = AHB2PERIPH_BASE;
pub const DCMI_BASE: usize = AHB2PERIPH_BASE + 0x50000;
pub const CRYP_BASE: usize = AHB2PERIPH_BASE + 0x60000;
pub const HASH_BASE: usize = AHB2PERIPH_BASE + 0x60400;
pub const RNG_BASE: usize = AHB2PERIPH_BASE + 0x60800;

pub const FSMC_BASE: usize = AHB3PERIPH_BASE;

pub const RCC_REGISTERS: *mut rcc::RCCRegisters = RCC_BASE as *mut rcc::RCCRegisters;
pub const FLASH_REGISTERS: *mut flash::FlashRegisters = FLASH_BASE as *mut flash::FlashRegisters;

pub const GPIO_A_REGISTERS: *mut gpio::PortRegisters = GPIOA_BASE as *mut gpio::PortRegisters;
pub const GPIO_B_REGISTERS: *mut gpio::PortRegisters = GPIOB_BASE as *mut gpio::PortRegisters;
pub const GPIO_C_REGISTERS: *mut gpio::PortRegisters = GPIOC_BASE as *mut gpio::PortRegisters;
pub const GPIO_D_REGISTERS: *mut gpio::PortRegisters = GPIOD_BASE as *mut gpio::PortRegisters;
pub const GPIO_E_REGISTERS: *mut gpio::PortRegisters = GPIOE_BASE as *mut gpio::PortRegisters;
pub const GPIO_F_REGISTERS: *mut gpio::PortRegisters = GPIOF_BASE as *mut gpio::PortRegisters;
pub const GPIO_G_REGISTERS: *mut gpio::PortRegisters = GPIOG_BASE as *mut gpio::PortRegisters;
pub const GPIO_H_REGISTERS: *mut gpio::PortRegisters = GPIOH_BASE as *mut gpio::PortRegisters;
pub const GPIO_I_REGISTERS: *mut gpio::PortRegisters = GPIOI_BASE as *mut gpio::PortRegisters;

pub const USART_1_REGISTERS: *mut usart::USARTRegisters = USART1_BASE as *mut usart::USARTRegisters;
pub const USART_2_REGISTERS: *mut usart::USARTRegisters = USART2_BASE as *mut usart::USARTRegisters;
pub const USART_3_REGISTERS: *mut usart::USARTRegisters = USART3_BASE as *mut usart::USARTRegisters;
pub const UART_4_REGISTERS: *mut usart::USARTRegisters = UART4_BASE as *mut usart::USARTRegisters;
pub const UART_5_REGISTERS: *mut usart::USARTRegisters = UART5_BASE as *mut usart::USARTRegisters;
pub const USART_6_REGISTERS: *mut usart::USARTRegisters = USART6_BASE as *mut usart::USARTRegisters;

pub const DMA_1_REGISTERS: *mut dma::DMARegisters = DMA1_BASE as *mut dma::DMARegisters;
pub const DMA_2_REGISTERS: *mut dma::DMARegisters = DMA2_BASE as *mut dma::DMARegisters;

pub static GPIO_A: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIO_A_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::GPIOA }
};
pub static GPIO_B: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIO_B_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::GPIOB }
};
pub static GPIO_C: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIO_C_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::GPIOC }
};
pub static GPIO_D: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIO_D_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::GPIOD }
};
pub static GPIO_E: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIO_E_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::GPIOE }
};
pub static GPIO_F: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIO_F_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::GPIOF }
};
pub static GPIO_G: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIO_G_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::GPIOG }
};
pub static GPIO_H: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIO_H_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::GPIOH }
};
pub static GPIO_I: gpio::PortPeripheral = gpio::PortPeripheral {
    base_address: GPIO_I_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::GPIOI }
};

pub static DMA_1: dma::DMAPeripheral = dma::DMAPeripheral {
    base_address: DMA_1_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::DMA1 }
};
pub static DMA_2: dma::DMAPeripheral = dma::DMAPeripheral {
    base_address: DMA_2_REGISTERS,
    clock: RCCPeripheral { rcc: RCC_REGISTERS, clock: Clock::DMA2 }
};

#[no_mangle]
pub fn rcc_get() -> &'static mut RCCRegisters {
    unsafe { &mut *RCC_REGISTERS }
}

#[no_mangle]
pub fn flash_get() -> &'static mut flash::FlashRegisters {
    unsafe { &mut *FLASH_REGISTERS }
}
