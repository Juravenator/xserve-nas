#![no_std]
#![no_main]
#![feature(
    type_alias_impl_trait,
    generic_const_exprs,
    iter_array_chunks,
    generic_arg_infer
)]

use cross::led::{Led, LedColour, LedIntensity};
use cross::led_driver::LedDriver;
use cross::shift_register::DMAShiftRegister;
use defmt::*;
use dma::DMAStream;
use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Pin};
use embassy_stm32::interrupt::{InterruptExt, Priority};
// use embassy_stm32::peripherals;
use embassy_stm32::rcc::{AHBPrescaler, APBPrescaler, Hse, Pll, PllPreDiv, PllSource, Sysclk};
use embassy_stm32::time::Hertz;
// use embassy_stm32::usb_otg::{self, Driver, Instance};
use embassy_stm32::{bind_interrupts, interrupt, pac, Config as STM32Config};
use embassy_stm32::{
    gpio::{Level, Output, Pull, Speed},
    peripherals::*,
};
use embassy_time::Timer;
// use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
// use embassy_usb::driver::EndpointError;
// use embassy_usb::Builder;
// use futures::future::join;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

mod dma;

static EXECUTOR_BLINK: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LS: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn USART2() {
    EXECUTOR_BLINK.on_interrupt()
}

// bind_interrupts!(struct Irqs {
//     OTG_FS => usb_otg::InterruptHandler<peripherals::USB_OTG_FS>;
// });

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = {
        let mut config: STM32Config = Default::default();

        config.enable_debug_during_sleep = false;

        config.rcc.hsi = false;
        config.rcc.hse = Some(Hse {
            freq: Hertz::mhz(25),
            mode: embassy_stm32::rcc::HseMode::Oscillator,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV25,
            mul: embassy_stm32::rcc::PllMul::MUL336,
            divp: Some(embassy_stm32::rcc::PllPDiv::DIV4),
            divq: Some(embassy_stm32::rcc::PllQDiv::DIV7),
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;

        embassy_stm32::init(config)
    };

    let led = Output::new(p.PC13, Level::High, Speed::Low);

    // // SATA status LED control
    // let sck_rg = Output::new(p.PA5, Level::Low, Speed::VeryHigh);
    // let sin_rg = Output::new(p.PA6, Level::Low, Speed::VeryHigh);
    // let lat_rg = Output::new(p.PA7, Level::Low, Speed::VeryHigh);

    // // SATA sensors
    // let sck_sens = Output::new(p.PA2, Level::Low, Speed::VeryHigh);
    // let sin_sens = Output::new(p.PA3, Level::Low, Speed::VeryHigh);
    // let nload_sens = Output::new(p.PA4, Level::Low, Speed::VeryHigh);

    // Center daughterboard LED control
    let sck = DMAShiftRegister::init_dma_pin(p.PB0);
    let driver_center = {
        let sin = p.PB1;
        let lat = p.PB2;
        let nen1 = p.PB3;
        let nen2 = p.PB4;
        let nen3 = p.PB5;
        let nen4 = p.PB6;
        let nen5 = p.PB7;
        LedDriver::<5, 2, 5>::new(
            sck,
            sin,
            lat,
            [
                nen1.degrade(),
                nen2.degrade(),
                nen3.degrade(),
                nen4.degrade(),
                nen5.degrade(),
            ],
        )
    };

    // Left-side daughterboard LED control
    let driver_l = {
        let sin = p.PB8;
        let lat = p.PB10;
        let nen1 = p.PB9;
        let nen2 = p.PB12;
        LedDriver::<2, 1, 3>::new(sck, sin, lat, [nen1.degrade(), nen2.degrade()])
    };
    // Left-side daughterboard sensors
    let _btn_warn = Input::new(p.PA0, Pull::Down);
    let _btn_volume = Input::new(p.PA1, Pull::Down);

    // let btn_warn = ExtiInput::new(btn_warn, p.EXTI0);
    // let btn_volume = ExtiInput::new(btn_volume, p.EXTI1);
    // spawner.spawn(run_btn(btn_volume, led)).unwrap();

    // Right-side daughterboard LED control
    let driver_r = {
        let sin = p.PB13;
        let lat = p.PB14;
        let nen = p.PB15;
        LedDriver::<1, 1, 4>::new(sck, sin, lat, [nen.degrade()])
    };
    // Right-side daughterboard sensors
    // let sens_lock = Input::new(p.PA10, Pull::Down);

    // // Hello world of GPIOB
    // pac::GPIOB.bsrr().write_value(Bsrr(0x0000FFFF));
    // // pac::GPIOB.bsrr().write(|w| w.0 = 0x0000FFFF);
    // // pac::GPIOB.bsrr().write(|w| {
    // //     for i in 0..16 {
    // //         w.set_bs(i, true);
    // //     }
    // //     for i in 0..16 {
    // //         w.set_br(i, false);
    // //     }
    // // });
    // Timer::after_secs(1).await;
    // pac::GPIOB.bsrr().write_value(Bsrr(0xFFFF0000));
    // // pac::GPIOB.bsrr().write(|w| w.0 = 0xFFFF0000);
    // // pac::GPIOB.bsrr().write(|w| {
    // //     for i in 0..16 {
    // //         w.set_bs(i, false);
    // //     }
    // //     for i in 0..16 {
    // //         w.set_br(i, true);
    // //     }
    // // });
    // Timer::after_millis(100).await;

    let dmas = dma::init(p.TIM1);

    interrupt::USART2.set_priority(Priority::P6);
    let spawner = EXECUTOR_BLINK.start(interrupt::USART2);
    spawner.spawn(run_blink(led)).unwrap();
    // spawner.spawn(run_usb(p.USB_OTG_FS, p.PA12, p.PA11)).unwrap();

    let executor = EXECUTOR_LS.init(Executor::new());
    executor.run(|s| {
        s.spawn(all_on(dmas[1], driver_l, driver_center, driver_r))
            .unwrap()
    });
}

#[embassy_executor::task]
async fn all_on(
    dmas: DMAStream,
    mut driver_l: LedDriver<2, 1, 3>,
    mut driver_center: LedDriver<5, 2, 5>,
    mut driver_r: LedDriver<1, 1, 4>,
) {
    // A buffer that can hold the biggest driver on GPIOB
    let mut odr_buffer = [0u16; LedDriver::<5, 2, 0>::buffer_size()];

    driver_center.init(&mut odr_buffer);

    dmas.set_source(&odr_buffer);
    dmas.set_destination(pac::GPIOB.odr().as_ptr() as u32);
    dmas.enable();

    let mut left_intensity = LedIntensity::MAX;
    let mut right_intensity = LedIntensity::MIN;
    loop {
        fn do_leds<const BANKS: usize, const CHIPS: usize, const LEDS_PER_CHIP: usize>(
            driver: &mut LedDriver<BANKS, CHIPS, LEDS_PER_CHIP>,
            left_intensity: LedIntensity,
            right_intensity: LedIntensity,
        ) where
            [Led; CHIPS * LEDS_PER_CHIP]: Sized,
        {
            for bank in 0..driver.bank_count() {
                let led_count = driver.leds_count();
                driver
                    .get_bank_leds(bank)
                    .into_iter()
                    .enumerate()
                    .for_each(|(i, led)| {
                        let intensity = if i < led_count / 2 {
                            left_intensity
                        } else {
                            right_intensity
                        };
                        led.r = intensity;
                        led.g = intensity;
                        led.b = intensity;
                    });
            }
        }
        do_leds(&mut driver_center, left_intensity, right_intensity);
        do_leds(&mut driver_l, left_intensity, right_intensity);
        do_leds(&mut driver_r, left_intensity, right_intensity);

        driver_center.write_serial(&mut odr_buffer);
        driver_l.write_serial(&mut odr_buffer);
        driver_r.write_serial(&mut odr_buffer);

        dmas.disable(); // stop current DMA
        dmas.enable(); // start a new DMA

        left_intensity = if left_intensity == LedIntensity::MIN {
            1.into()
        } else {
            LedIntensity::MIN
        };
        right_intensity = if right_intensity == LedIntensity::MIN {
            1.into()
        } else {
            LedIntensity::MIN
        };
        Timer::after_millis(2000).await;
    }
}

// #[embassy_executor::task]
// async fn static_g(
//     dmas: [DMAStream; 2],
//     mut driver_l: LedDriver<2, 1, 3>,
//     mut driver_center: LedDriver<5, 2, 5>,
//     mut driver_r: LedDriver<1, 1, 4>,
// ) {
//     // A buffer that can hold the biggest driver on GPIOA
//     let mut odr_gpio_a_buffer = [0u16; LedDriver::<2, 1, 0>::buffer_size()];
//     // A buffer that can hold the biggest driver on GPIOB
//     let mut odr_gpio_b_buffer = [0u16; LedDriver::<5, 2, 0>::buffer_size()];

//     driver_center.init(&mut odr_gpio_b_buffer);

//     dmas[0].set_source(&odr_gpio_a_buffer);
//     dmas[0].set_destination(pac::GPIOA.odr().as_ptr() as u32);
//     dmas[0].enable();

//     dmas[1].set_source(&odr_gpio_b_buffer);
//     dmas[1].set_destination(pac::GPIOB.odr().as_ptr() as u32);
//     dmas[1].enable();

//     let leds = driver_center.get_bank_leds(2);
//     for led in leds.iter_mut().skip(2).step_by(3) {
//         led.g.set(1);
//     }

//     // driver_l.calculate_output_buffer2(&mut odr_gpio_a_buffer);
//     // driver_r.calculate_output_buffer2(&mut odr_gpio_a_buffer);
//     // driver_center.calculate_output_buffer2(&mut odr_gpio_b_buffer);
//     driver_l.write_serial(&mut odr_gpio_a_buffer);
//     driver_r.write_serial(&mut odr_gpio_a_buffer);
//     driver_center.write_serial(&mut odr_gpio_b_buffer);

//     dmas[0].disable(); // stop current DMA
//     dmas[0].enable(); // start a new DMA
//     dmas[1].disable(); // stop current DMA
//     dmas[1].enable(); // start a new DMA

//     // for i in 0..odr_gpio_b_buffer.len() {
//     //     // set_bit(&mut odr_gpio_b_buffer[i], 8);
//     //     set_bit(&mut odr_gpio_b_buffer[i], 8);
//     // }
//     loop {
//         // for i in 0..odr_gpio_a_buffer.len() {
//         //     unset_bit(&mut odr_gpio_a_buffer[i], 3);
//         // }
//         // for i in 0..odr_gpio_b_buffer.len() {
//         //     set_bit(&mut odr_gpio_b_buffer[i], 8);
//         //     unset_bit(&mut odr_gpio_b_buffer[i], 9);
//         // }
//         // Timer::after_millis(2000).await;
//         // for i in 0..odr_gpio_a_buffer.len() {
//         //     set_bit(&mut odr_gpio_a_buffer[i], 3);
//         // }
//         // for i in 0..odr_gpio_b_buffer.len() {
//         //     unset_bit(&mut odr_gpio_b_buffer[i], 8);
//         //     set_bit(&mut odr_gpio_b_buffer[i], 9);
//         // }
//         Timer::after_millis(2000).await;
//     }
// }

// nice LED low_intensity
#[embassy_executor::task]
async fn travel(
    dmas: DMAStream,
    mut _driver_l: LedDriver<2, 1, 3>,
    mut driver_center: LedDriver<5, 2, 5>,
    mut _driver_r: LedDriver<1, 1, 4>,
) {
    // A buffer that can hold the biggest driver on GPIOB
    let mut odr_buffer = [0u16; LedDriver::<5, 2, 0>::buffer_size()];

    driver_center.init(&mut odr_buffer);

    dmas.set_source(&odr_buffer);
    dmas.set_destination(pac::GPIOB.odr().as_ptr() as u32);
    dmas.enable();

    loop {
        for c in [LedColour::RED, LedColour::GREEN, LedColour::BLUE] {
            for i in 0..48 {
                let bank = (i % 24) / 5;
                let pos = {
                    let mut x = (i % 24) % 5;
                    if i >= 24 {
                        x += 5;
                    }
                    x
                };

                let p_i = if i == 0 { 47 } else { i - 1 };
                let p_bank = ((p_i) % 24) / 5;
                let p_pos = {
                    let mut x = (p_i % 24) % 5;
                    if p_i >= 24 {
                        x += 5;
                    }
                    x
                };

                driver_center.get_bank_leds(p_bank)[p_pos].r.set(0);
                driver_center.get_bank_leds(p_bank)[p_pos].g.set(0);
                driver_center.get_bank_leds(p_bank)[p_pos].b.set(0);
                driver_center.get_bank_leds(bank)[pos].set(&c, 1);

                driver_center.write_serial(&mut odr_buffer);
                dmas.disable(); // stop current DMA
                dmas.enable(); // start a new DMA

                Timer::after_millis(50).await;
            }
        }
    }
}

// // nice LED low_intensity
// #[embassy_executor::task]
// async fn low_intensity(
//     dmas: [DMAStream; 2],
//     mut driver_l: LedDriver<2, 1, 3>,
//     mut driver_center: LedDriver<5, 2, 5>,
//     mut driver_r: LedDriver<1, 1, 4>,
// ) {
//     // A buffer that can hold the biggest driver on GPIOA
//     let mut odr_gpio_a_buffer = [0u16; LedDriver::<2, 1, 0>::buffer_size()];
//     // A buffer that can hold the biggest driver on GPIOB
//     let mut odr_gpio_b_buffer = [0u16; LedDriver::<5, 2, 0>::buffer_size()];

//     driver_center.init(&mut odr_gpio_b_buffer);

//     dmas[0].set_source(&odr_gpio_a_buffer);
//     dmas[0].set_destination(pac::GPIOA.odr().as_ptr() as u32);
//     dmas[0].enable();

//     dmas[1].set_source(&odr_gpio_b_buffer);
//     dmas[1].set_destination(pac::GPIOB.odr().as_ptr() as u32);
//     dmas[1].enable();

//     // let leds = driver_l.get_bank_leds(0);
//     // for led in leds {
//     //     led.g.set(1);
//     // }
//     let leds = driver_center.get_bank_leds(0);
//     for led in leds.iter_mut().step_by(3) {
//         led.r.set(1);
//     }
//     leds[0].r.set(100);
//     let leds = driver_center.get_bank_leds(1);
//     for led in leds.iter_mut().skip(1).step_by(3) {
//         led.g.set(100);
//     }
//     let leds = driver_center.get_bank_leds(2);
//     for led in leds.iter_mut().skip(2).step_by(3) {
//         led.b.set(1);
//     }
//     let leds = driver_center.get_bank_leds(3);
//     for led in leds.iter_mut().skip(0).step_by(3) {
//         led.r.set(1);
//     }
//     let leds = driver_center.get_bank_leds(4);
//     for led in leds.iter_mut().skip(1).step_by(3) {
//         led.g.set(100);
//     }
//     let leds = driver_r.get_bank_leds(0);
//     for led in leds {
//         led.g.set(1);
//     }

//     // driver_l.calculate_output_buffer2(&mut odr_gpio_a_buffer);
//     // driver_r.calculate_output_buffer2(&mut odr_gpio_a_buffer);
//     // driver_center.calculate_output_buffer2(&mut odr_gpio_b_buffer);
//     driver_l.write_serial(&mut odr_gpio_a_buffer);
//     driver_r.write_serial(&mut odr_gpio_a_buffer);
//     driver_center.write_serial(&mut odr_gpio_b_buffer);

//     dmas[0].disable(); // stop current DMA
//     dmas[0].enable(); // start a new DMA
//     dmas[1].disable(); // stop current DMA
//     dmas[1].enable(); // start a new DMA

//     // for i in 0..odr_gpio_b_buffer.len() {
//     //     // set_bit(&mut odr_gpio_b_buffer[i], 8);
//     //     set_bit(&mut odr_gpio_b_buffer[i], 8);
//     // }
//     loop {
//         // for i in 0..odr_gpio_a_buffer.len() {
//         //     unset_bit(&mut odr_gpio_a_buffer[i], 3);
//         // }
//         // for i in 0..odr_gpio_b_buffer.len() {
//         //     set_bit(&mut odr_gpio_b_buffer[i], 8);
//         //     unset_bit(&mut odr_gpio_b_buffer[i], 9);
//         // }
//         // Timer::after_millis(2000).await;
//         // for i in 0..odr_gpio_a_buffer.len() {
//         //     set_bit(&mut odr_gpio_a_buffer[i], 3);
//         // }
//         // for i in 0..odr_gpio_b_buffer.len() {
//         //     unset_bit(&mut odr_gpio_b_buffer[i], 8);
//         //     set_bit(&mut odr_gpio_b_buffer[i], 9);
//         // }
//         Timer::after_millis(2000).await;
//     }
// }

// // nice LED pulses
// #[embassy_executor::task]
// async fn pulses(dmas: [DMAStream; 2], mut driver_l: LedDriver<2, 1, 3>, mut driver_center: LedDriver<5, 2, 5>, mut driver_r: LedDriver<1, 1, 4>) {

//     // A buffer that can hold the biggest driver on GPIOA
//     let mut odr_gpio_a_buffer = [0u16; LedDriver::<2, 1, 0>::buffer_size()];
//     // A buffer that can hold the biggest driver on GPIOB
//     let mut odr_gpio_b_buffer = [0u16; LedDriver::<5, 2, 0>::buffer_size()];

//     dmas[0].set_source(&odr_gpio_a_buffer);
//     dmas[0].set_destination(pac::GPIOA.odr().as_ptr() as u32);
//     dmas[0].enable();

//     dmas[1].set_source(&odr_gpio_b_buffer);
//     dmas[1].set_destination(pac::GPIOB.odr().as_ptr() as u32);
//     dmas[1].enable();

//     loop {
//         for intensity in (0..LedIntensity::max_u8()).rev() {
//             let mindelay = Timer::after_millis(20);

//             let leds = driver_l.get_bank_leds(0);
//             for led in leds {
//                 led.g.set(intensity);
//             }
//             let leds = driver_center.get_bank_leds(0);
//             for led in leds {
//                 led.r.set(intensity);
//             }
//             let leds = driver_center.get_bank_leds(1);
//             for led in leds {
//                 led.g.set(intensity);
//             }
//             let leds = driver_center.get_bank_leds(2);
//             for led in leds {
//                 led.b.set(intensity);
//             }
//             let leds = driver_center.get_bank_leds(3);
//             for led in leds {
//                 led.r.set(intensity);
//             }
//             let leds = driver_center.get_bank_leds(4);
//             for led in leds {
//                 led.g.set(intensity);
//             }
//             let leds = driver_r.get_bank_leds(0);
//             for led in leds {
//                 led.g.set(intensity);
//             }

//             driver_l.calculate_output_buffer2(&mut odr_gpio_a_buffer);
//             driver_r.calculate_output_buffer2(&mut odr_gpio_a_buffer);
//             driver_center.calculate_output_buffer2(&mut odr_gpio_b_buffer);

//             dmas[0].disable(); // stop current DMA
//             dmas[0].enable(); // start a new DMA
//             dmas[1].disable(); // stop current DMA
//             dmas[1].enable(); // start a new DMA

//             mindelay.await;

//             // If this is removed, the code crashes.
//             if intensity == 0 {
//                 Timer::after_millis(100-20).await;
//             }
//         }
//     }
// }

// // simple full toggle
// #[embassy_executor::task]
// async fn full_toggle(dmas: [DMAStream; 2], mut driver_center: LedDriver<5, 2, 5>) {

//     // A buffer that can hold the biggest driver on GPIOA
//     let mut odr_gpio_a_buffer = [0u16; LedDriver::<5, 2, 5>::buffer_size()];
//     // A buffer that can hold the biggest driver on GPIOB
//     let mut odr_gpio_b_buffer = [0u16; LedDriver::<5, 2, 5>::buffer_size()];

//     dmas[0].set_source(&odr_gpio_a_buffer);
//     dmas[0].set_destination(pac::GPIOA.odr().as_ptr() as u32);
//     dmas[0].enable();

//     dmas[1].set_source(&odr_gpio_b_buffer);
//     dmas[1].set_destination(pac::GPIOB.odr().as_ptr() as u32);
//     dmas[1].enable();

//     loop {
//         for colour in [LedColour::BLUE, LedColour::GREEN, LedColour::RED] {
//             for intensity in (0..LedIntensity::max_u8()).chain((0..LedIntensity::max_u8()).rev()) {

//                 Timer::after_millis(10).await;
//                 // led.toggle();

//                 let i: LedIntensity = intensity.into();
//                 let leds = driver_center.get_bank_leds(0);
//                 for led in leds {
//                     led.set(&colour, i.into());
//                 }

//                 // driver_center.calculate_output_buffer(&mut odr_gpio_a_buffer);
//                 driver_center.calculate_output_buffer2(&mut odr_gpio_b_buffer);

//                 pac::DMA2.st(5).cr().modify(|w| w.set_en(false)); // stop current DMA
//                 pac::DMA2.st(5).cr().modify(|w| w.set_en(true)); // start a new DMA
//             }
//         }
//     }
// }

// // rainbow
// #[embassy_executor::task]
// async fn rainbow(
//     dmas: [DMAStream; 2],
//     mut driver_l: LedDriver<2, 1, 3>,
//     mut driver_center: LedDriver<5, 2, 5>,
//     mut driver_r: LedDriver<1, 1, 4>,
// ) {
//     // A buffer that can hold the biggest driver on GPIOA
//     let mut odr_gpio_a_buffer = [0u16; LedDriver::<5, 2, 5>::buffer_size()];
//     // A buffer that can hold the biggest driver on GPIOB
//     let mut odr_gpio_b_buffer = [0u16; LedDriver::<5, 2, 5>::buffer_size()];

//     driver_center.init(&mut odr_gpio_b_buffer);

//     dmas[0].set_source(&odr_gpio_a_buffer);
//     dmas[0].set_destination(pac::GPIOA.odr().as_ptr() as u32);
//     dmas[0].enable();

//     dmas[1].set_source(&odr_gpio_b_buffer);
//     dmas[1].set_destination(pac::GPIOB.odr().as_ptr() as u32);
//     dmas[1].enable();

//     let ledslen = driver_center.get_bank_leds(0).len();
//     let mut first_loop = true;
//     loop {
//         for led_i in 0usize..ledslen {
//             let list_of_i = {
//                 let mut r = [0; 6];
//                 for n in 0..r.len() {
//                     r[n] = if led_i.wrapping_sub(n) > led_i {
//                         led_i + (ledslen - n)
//                     } else {
//                         led_i - n
//                     };
//                 }
//                 r
//             };

//             for intensity in 0..LedIntensity::max_u8() + 1 {
//                 let leds = driver_center.get_bank_leds(0);
//                 leds[list_of_i[0]].b.set(intensity);
//                 leds[list_of_i[2]].b.set(LedIntensity::max_u8() - intensity);

//                 leds[list_of_i[1]].g.set(intensity);
//                 leds[list_of_i[4]].g.set(LedIntensity::max_u8() - intensity);

//                 leds[list_of_i[3]].r.set(intensity);
//                 leds[list_of_i[5]].r.set(LedIntensity::max_u8() - intensity);

//                 if first_loop {
//                     if led_i < list_of_i.len() - 1 {
//                         for i in 0..list_of_i.len() - 1 - led_i {
//                             leds[ledslen - 1 - i].r = 0.into();
//                             leds[ledslen - 1 - i].g = 0.into();
//                             leds[ledslen - 1 - i].b = 0.into();
//                         }
//                     } else {
//                         first_loop = false;
//                     }
//                 }

//                 let waitatleast = Timer::after_millis(10);
//                 // driver_center.calculate_output_buffer2(&mut odr_gpio_a_buffer);
//                 // driver_center.calculate_output_buffer2(&mut odr_gpio_b_buffer);
//                 driver_center.write_serial(&mut odr_gpio_a_buffer);
//                 driver_center.write_serial(&mut odr_gpio_b_buffer);
//                 dmas[1].disable(); // stop current DMA
//                 dmas[1].enable(); // start a new DMA
//                 waitatleast.await;
//             }
//         }
//     }
// }

#[embassy_executor::task]
async fn run_blink(mut led: Output<'static, PC13>) {
    // let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        info!("high");
        led.set_high();
        // ticker.next().await;
        Timer::after_millis(1000).await;

        info!("low");
        led.set_low();
        // ticker.next().await;
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
async fn run_btn(mut button: ExtiInput<'static, PA1>, mut led: Output<'static, PC13>) {
    // let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        button.wait_for_rising_edge().await;
        led.toggle();
        Timer::after_millis(200).await;
    }
}

// #[embassy_executor::task]
// async fn run_usb(fs: USB_OTG_FS, pa12: PA12, pa11: PA11) {
//     // Create the driver, from the HAL.
//     let mut ep_out_buffer = [0u8; 256];
//     let mut config = embassy_stm32::usb_otg::Config::default();
//     config.vbus_detection = true;
//     let driver = Driver::new_fs(fs, Irqs, pa12, pa11, &mut ep_out_buffer, config);

//     // Create embassy-usb Config
//     let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
//     config.manufacturer = Some("Embassy");
//     config.product = Some("USB-serial example");
//     config.serial_number = Some("12345678");

//     // Required for windows compatibility.
//     // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
//     config.device_class = 0xEF;
//     config.device_sub_class = 0x02;
//     config.device_protocol = 0x01;
//     config.composite_with_iads = true;

//     // Create embassy-usb DeviceBuilder using the driver and config.
//     // It needs some buffers for building the descriptors.
//     let mut device_descriptor = [0; 256];
//     let mut config_descriptor = [0; 256];
//     let mut bos_descriptor = [0; 256];
//     let mut control_buf = [0; 64];

//     let mut state = State::new();

//     let mut builder = Builder::new(
//         driver,
//         config,
//         &mut device_descriptor,
//         &mut config_descriptor,
//         &mut bos_descriptor,
//         &mut [], // no msos descriptors
//         &mut control_buf,
//     );

//     // Create classes on the builder.
//     let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

//     // Build the builder.
//     let mut usb = builder.build();

//     // Run the USB device.
//     let usb_fut = usb.run();

//     // Do stuff with the class!
//     let echo_fut = async {
//         loop {
//             class.wait_connection().await;
//             info!("Connected");
//             let _ = echo(&mut class).await;
//             info!("Disconnected");
//         }
//     };

//     // Run everything concurrently.
//     // If we had made everything `'static` above instead, we could do this using separate tasks instead.
//     join(usb_fut, echo_fut).await;
// }

// struct Disconnected {}

// impl From<EndpointError> for Disconnected {
//     fn from(val: EndpointError) -> Self {
//         match val {
//             EndpointError::BufferOverflow => Disconnected {},
//             EndpointError::Disabled => Disconnected {},
//         }
//     }
// }

// async fn echo<'d, T: Instance + 'd>(
//     class: &mut CdcAcmClass<'d, Driver<'d, T>>,
// ) -> Result<(), Disconnected> {
//     let mut buf = [0; 64];
//     loop {
//         let n = class.read_packet(&mut buf).await?;
//         let data = &buf[..n];
//         info!("data: {:x}", data);
//         class.write_packet(data).await?;
//     }
// }
