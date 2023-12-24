#![no_std]
#![no_main]
#![feature(type_alias_impl_trait, generic_const_exprs, iter_array_chunks, generic_arg_infer)]

use defmt::*;
use dma::DMAStream;
use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_stm32::gpio::{Input, Flex};
use embassy_stm32::gpio::low_level::Pin;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::pac::dma::regs::Ndtr;
use embassy_stm32::pac::dma::vals::Ct;
use embassy_stm32::pac::gpio::regs::{Bsrr, Odr};
use embassy_stm32::pac::timer::regs::Arr16;
use embassy_stm32::peripherals::{PB8, PB9, TIM1};
use embassy_stm32::rcc::low_level::RccPeripheral;
use embassy_stm32::rcc::{AHBPrescaler, APBPrescaler, Sysclk, PllSource, PllPreDiv, Hse, Pll};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::Basic16bitInstance;
use embassy_stm32::timer::low_level::CaptureCompare16bitInstance;
use embassy_stm32::timer::low_level::GeneralPurpose16bitInstance;
use embassy_stm32::timer::Channel;
use embassy_stm32::timer::{CountingMode, OutputCompareMode};
use embassy_stm32::{
    gpio::{Level, Output, Pull, Speed},
    peripherals::*,
};
use embassy_stm32::{interrupt, pac, Config as STM32Config};
use embassy_time::Timer;
use led::{LedIntensity, LedColour};
use led_driver::{LedDriver};
use shift_register::DMAShiftRegister;
use static_cell::StaticCell;

use crate::led::Led;
use {defmt_rtt as _, panic_probe as _};

mod led;
mod led_driver;
mod shift_register;
mod dma;

static EXECUTOR_BLINK: InterruptExecutor = InterruptExecutor::new();
static EXECUTOR_LS: StaticCell<Executor> = StaticCell::new();

#[interrupt]
unsafe fn USART2() {
    EXECUTOR_BLINK.on_interrupt()
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut p = {
        let mut config: STM32Config = Default::default();

        config.enable_debug_during_sleep = false;

        config.rcc.hsi = false;
        config.rcc.hse = Some(Hse{
            freq: Hertz::mhz(25),
            mode: embassy_stm32::rcc::HseMode::Oscillator,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll{
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

    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    // // Right-side daughterboard LED control
    // let sck_r = Output::new(p.PA0, Level::Low, Speed::VeryHigh);
    // let sin_r = Output::new(p.PA1, Level::Low, Speed::VeryHigh);
    // let lat_r = Output::new(p.PA2, Level::Low, Speed::VeryHigh);
    // let r_nen = Output::new(p.PA3, Level::Low, Speed::VeryHigh);
    // // Right-side daughterboard sensors
    // let sens_lock = Input::new(p.PA10, Pull::Down);



    // sck_ls.set_as_output(Speed::VeryHigh);


    // // SATA status LED control
    // let sck_rg = Output::new(p.PB5, Level::Low, Speed::VeryHigh);
    // let sin_rg = Output::new(p.PB4, Level::Low, Speed::VeryHigh);
    // let lat_rg = Output::new(p.PB3, Level::Low, Speed::VeryHigh);

    // // SATA sensors
    // let sck_sens = Output::new(p.PA15, Level::Low, Speed::VeryHigh);
    // let sin_sens = Output::new(p.PA12, Level::Low, Speed::VeryHigh);
    // let nload_sens = Output::new(p.PA11, Level::Low, Speed::VeryHigh);

    interrupt::USART2.set_priority(Priority::P6);
    let spawner = EXECUTOR_BLINK.start(interrupt::USART2);
    spawner.spawn(run_blink(led)).unwrap();

    // let executor = EXECUTOR_LS.init(Executor::new());
    // executor.run(|s| s.spawn(run_ls2(sck_ls, sin_ls, lat_ls, ls1_nen, ls2_nen)).unwrap());

    // send_bits(&mut sck_ls, &mut sin_ls, [Level::Low; 32]).await;
    // trigger_lat(&mut lat_ls).await;
    // Timer::after_secs(1).await;

    // Center daughterboard LED control
    let driver_center = {
        let sck = p.PB13;
        let sin = p.PB14;
        let lat = p.PB15;
        let nen1 = p.PB9;
        let nen2 = p.PB8;
        let nen3 = p.PB7;
        let nen4 = p.PB6;
        let nen5 = p.PB1;
        LedDriver::<5, 2, 5>::new(sck, sin, lat, [nen1.into(), nen2.into(), nen3.into(), nen4.into(), nen5.into()])
    };

    // Left-side daughterboard LED control
    let driver_l = {
        let sck = p.PA4;
        let sin = p.PA5;
        let lat = p.PA6;
        let nen1 = p.PA7;
        let nen2 = p.PB0;
        LedDriver::<2, 1, 3>::new(sck, sin, lat, [nen1.into(), nen2.into()])
    };
    // Left-side daughterboard sensors
    let btn_warn = Input::new(p.PB2, Pull::Down);
    let btn_volume = Input::new(p.PB10, Pull::Down);

    // Right-side daughterboard LED control
    let driver_r = {
        let sck = p.PA0;
        let sin = p.PA1;
        let lat = p.PA2;
        let nen = p.PA3;
        LedDriver::<1, 1, 4>::new(sck, sin, lat, [nen.into()])
    };
    // Right-side daughterboard sensors
    let sens_lock = Input::new(p.PA10, Pull::Down);

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

    let executor = EXECUTOR_LS.init(Executor::new());
    executor.run(|s| s.spawn(pulses(dmas, driver_l, driver_center, driver_r)).unwrap());
}

// nice LED pulses
#[embassy_executor::task]
async fn pulses(dmas: [DMAStream; 2], mut driver_l: LedDriver<2, 1, 3>, mut driver_center: LedDriver<5, 2, 5>, mut driver_r: LedDriver<1, 1, 4>) {

    // A buffer that can hold the biggest driver on GPIOA
    let mut odr_gpio_a_buffer = [0u16; LedDriver::<2, 1, 0>::buffer_size()];
    // A buffer that can hold the biggest driver on GPIOB
    let mut odr_gpio_b_buffer = [0u16; LedDriver::<5, 2, 0>::buffer_size()];

    dmas[0].set_source(&odr_gpio_a_buffer);
    dmas[0].set_destination(pac::GPIOA.odr().as_ptr() as u32);
    dmas[0].enable();

    dmas[1].set_source(&odr_gpio_b_buffer);
    dmas[1].set_destination(pac::GPIOB.odr().as_ptr() as u32);
    dmas[1].enable();

    loop {
        for intensity in (0..LedIntensity::max_u8()).rev() {
            let mindelay = Timer::after_millis(20);

            let leds = driver_l.get_bank_leds(0);
            for led in leds {
                led.g.set(intensity);
            }
            // leds[0].b.set(intensity);
            let leds = driver_center.get_bank_leds(1);
            for led in leds {
                led.g.set(intensity);
            }
            let leds = driver_r.get_bank_leds(0);
            for led in leds {
                led.g.set(intensity);
            }

            driver_l.calculate_output_buffer(&mut odr_gpio_a_buffer);
            driver_r.calculate_output_buffer(&mut odr_gpio_a_buffer);
            driver_center.calculate_output_buffer(&mut odr_gpio_b_buffer);

            dmas[0].disable(); // stop current DMA
            dmas[0].enable(); // start a new DMA
            dmas[1].disable(); // stop current DMA
            dmas[1].enable(); // start a new DMA

            mindelay.await;

            // If this is removed, the code crashes.
            if intensity == 0 {
                Timer::after_millis(100-20).await;
            }
        }
    }
}

// simple full toggle
#[embassy_executor::task]
async fn full_toggle(dmas: [DMAStream; 2], mut driver_center: LedDriver<5, 2, 5>) {

    // A buffer that can hold the biggest driver on GPIOA
    let mut odr_gpio_a_buffer = [0u16; LedDriver::<5, 2, 5>::buffer_size()];
    // A buffer that can hold the biggest driver on GPIOB
    let mut odr_gpio_b_buffer = [0u16; LedDriver::<5, 2, 5>::buffer_size()];

    dmas[0].set_source(&odr_gpio_a_buffer);
    dmas[0].set_destination(pac::GPIOA.odr().as_ptr() as u32);
    dmas[0].enable();

    dmas[1].set_source(&odr_gpio_b_buffer);
    dmas[1].set_destination(pac::GPIOB.odr().as_ptr() as u32);
    dmas[1].enable();

    loop {
        for colour in [LedColour::BLUE, LedColour::GREEN, LedColour::RED] {
            for intensity in (0..LedIntensity::max_u8()).chain((0..LedIntensity::max_u8()).rev()) {

                Timer::after_millis(10).await;
                // led.toggle();

                let i: LedIntensity = intensity.into();
                let leds = driver_center.get_bank_leds(0);
                for led in leds {
                    led.set(&colour, i.into());
                }

                // driver_center.calculate_output_buffer(&mut odr_gpio_a_buffer);
                driver_center.calculate_output_buffer(&mut odr_gpio_b_buffer);

                pac::DMA2.st(5).cr().modify(|w| w.set_en(false)); // stop current DMA
                pac::DMA2.st(5).cr().modify(|w| w.set_en(true)); // start a new DMA
            }
        }
    }
}

// rainbow
#[embassy_executor::task]
async fn rainbow(dmas: [DMAStream; 2], mut driver_center: LedDriver<5, 2, 5>) {

    // A buffer that can hold the biggest driver on GPIOA
    let mut odr_gpio_a_buffer = [0u16; LedDriver::<5, 2, 5>::buffer_size()];
    // A buffer that can hold the biggest driver on GPIOB
    let mut odr_gpio_b_buffer = [0u16; LedDriver::<5, 2, 5>::buffer_size()];

    dmas[0].set_source(&odr_gpio_a_buffer);
    dmas[0].set_destination(pac::GPIOA.odr().as_ptr() as u32);
    dmas[0].enable();

    dmas[1].set_source(&odr_gpio_b_buffer);
    dmas[1].set_destination(pac::GPIOB.odr().as_ptr() as u32);
    dmas[1].enable();

    let ledslen = driver_center.get_bank_leds(0).len();
    let mut first_loop = true;
    loop {
        for led_i in 0usize..ledslen {
            let list_of_i = {
                let mut r = [0; 6];
                for n in 0..r.len() {
                    r[n] = if led_i.wrapping_sub(n) > led_i {
                        led_i + (ledslen - n)
                    } else {
                        led_i - n
                    };
                }
                r
            };

            for intensity in 0..LedIntensity::max_u8()+1 {
                let leds = driver_center.get_bank_leds(0);
                leds[list_of_i[0]].b.set(intensity);
                leds[list_of_i[2]].b.set(LedIntensity::max_u8()-intensity);

                leds[list_of_i[1]].g.set(intensity);
                leds[list_of_i[4]].g.set(LedIntensity::max_u8()-intensity);

                leds[list_of_i[3]].r.set(intensity);
                leds[list_of_i[5]].r.set(LedIntensity::max_u8()-intensity);

                if first_loop {
                    if led_i < list_of_i.len()-1 {
                        for i in 0..list_of_i.len()-1-led_i {
                            leds[ledslen-1-i].r = 0.into();
                            leds[ledslen-1-i].g = 0.into();
                            leds[ledslen-1-i].b = 0.into();
                        }
                    } else {
                        first_loop = false;
                    }
                }

                let waitatleast = Timer::after_millis(10);
                driver_center.calculate_output_buffer(&mut odr_gpio_a_buffer);
                driver_center.calculate_output_buffer(&mut odr_gpio_b_buffer);
                dmas[1].disable(); // stop current DMA
                dmas[1].enable(); // start a new DMA
                waitatleast.await;
            }
        }
    }
}

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
