#![no_std]
#![no_main]
#![feature(type_alias_impl_trait, generic_const_exprs, iter_array_chunks)]

use defmt::*;
use embassy_executor::{Executor, InterruptExecutor, Spawner};
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_stm32::pac::dma::regs::Ndtr;
use embassy_stm32::pac::gpio::regs::Bsrr;
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
use led::LedIntensity;
use led_driver::LedDriver;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

mod led;
mod led_driver;
mod shift_register;

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

    // Right-side daughterboard LED control
    let sck_r = Output::new(p.PA0, Level::Low, Speed::VeryHigh);
    let sin_r = Output::new(p.PA1, Level::Low, Speed::VeryHigh);
    let lat_r = Output::new(p.PA2, Level::Low, Speed::VeryHigh);
    let r_nen = Output::new(p.PA3, Level::Low, Speed::VeryHigh);
    // Right-side daughterboard sensors
    let sens_lock = Input::new(p.PA10, Pull::Down);

    // Left-side daughterboard LED control
    let sck_l = Output::new(p.PA4, Level::Low, Speed::VeryHigh);
    let sin_l = Output::new(p.PA5, Level::Low, Speed::VeryHigh);
    let lat_l = Output::new(p.PA6, Level::Low, Speed::VeryHigh);
    let l_nen1 = Output::new(p.PA7, Level::Low, Speed::VeryHigh);
    let l_nen2 = Output::new(p.PB0, Level::Low, Speed::VeryHigh);
    // Left-side daughterboard sensors
    // let btn_warn = Input::new(p.PB2, Pull::Down);
    // let btn_volume = Input::new(p.PB10, Pull::Down);

    // Center daughterboard LED control
    let sck_ls = Output::new(p.PB15, Level::Low, Speed::VeryHigh);
    let sin_ls = Output::new(p.PA8, Level::Low, Speed::VeryHigh);
    let lat_ls = Output::new(p.PA9, Level::Low, Speed::VeryHigh);
    let sin_ls = Output::new(p.PB14, Level::Low, Speed::VeryHigh);
    let lat_ls = Output::new(p.PA13, Level::Low, Speed::VeryHigh);
    let ls1_nen = Output::new(p.PB9, Level::Low, Speed::VeryHigh);
    let ls2_nen = Output::new(p.PB8, Level::Low, Speed::VeryHigh);
    let ls3_nen = Output::new(p.PB7, Level::Low, Speed::VeryHigh);
    let ls4_nen = Output::new(p.PB6, Level::Low, Speed::VeryHigh);
    let ls5_nen = Output::new(p.PB1, Level::Low, Speed::VeryHigh);

    // SATA status LED control
    let sck_rg = Output::new(p.PB5, Level::Low, Speed::VeryHigh);
    let sin_rg = Output::new(p.PB4, Level::Low, Speed::VeryHigh);
    let lat_rg = Output::new(p.PB3, Level::Low, Speed::VeryHigh);

    // SATA sensors
    let sck_sens = Output::new(p.PA15, Level::Low, Speed::VeryHigh);
    let sin_sens = Output::new(p.PA12, Level::Low, Speed::VeryHigh);
    let nload_sens = Output::new(p.PA11, Level::Low, Speed::VeryHigh);

    // interrupt::USART2.set_priority(Priority::P6);
    // let spawner = EXECUTOR_BLINK.start(interrupt::USART2);
    // spawner.spawn(run_blink(led)).unwrap();

    // let executor = EXECUTOR_LS.init(Executor::new());
    // executor.run(|s| s.spawn(run_ls2(sck_ls, sin_ls, lat_ls, ls1_nen, ls2_nen)).unwrap());

    // send_bits(&mut sck_ls, &mut sin_ls, [Level::Low; 32]).await;
    // trigger_lat(&mut lat_ls).await;
    // Timer::after_secs(1).await;

    // let mut driver = LedDriver::<_, _, _, 2, 2, 5>::new(sck_ls, sin_ls, lat_ls, [ls1_nen.degrade(), ls2_nen.degrade()]);

    // let leds = driver.get_bank_leds(0);
    // for led in leds.into_iter() {
    //     led.r = 200.into();
    // }
    // leds[1].r = LedIntensity::MIN;

    // let leds = driver.get_bank_leds(1);
    // for led in leds.into_iter() {
    //     led.g = 200.into();
    // }
    // leds[0].g = LedIntensity::MIN;

    // loop {
    //     driver.display_cycle_test().await;
    // }

    let pb11 = Output::new(p.PB11, Level::Low, Speed::VeryHigh);
    let pb12 = Output::new(p.PB12, Level::Low, Speed::VeryHigh);
    let pb13 = Output::new(p.PB13, Level::Low, Speed::VeryHigh);
    // let pb14 = Output::new(p.PB14, Level::Low, Speed::VeryHigh);
    let pb2 = Output::new(p.PB2, Level::Low, Speed::VeryHigh);
    let pb10 = Output::new(p.PB10, Level::Low, Speed::VeryHigh);

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

    // References:
    // - https://embassy.dev/dev/layer_by_layer.html
    // - https://www.st.com/resource/en/datasheet/stm32f401cc.pdf (DS9716, block diagram, AF mappings)
    // - https://www.st.com/resource/en/reference_manual/rm0368-stm32f401xbc-and-stm32f401xde-advanced-armbased-32bit-mcus-stmicroelectronics.pdf (RM0368, registers)

    // // GPIO
    // // Alternate Function pins are configured in the Embassy HAL by declaring a PwmPin.
    // // It works, but there's more AFs than PWM. Embassy just exposes one AF so hence the naming I guess.
    // //
    // // Embassy HAL has an internal function to set any AF mode on a pin. It could basically consist of
    // // declaring an Input/Output::new() and running set_as_af(). Alas, the function is private.
    // //
    // // The commented HAL code and the uncommented registry writes are equivalent.

    // // let ch1 = PwmPin::new_ch1(p.PA8, OutputType::PushPull);
    // pac::RCC.ahb1enr().modify(|w| w.set_gpioaen(true));
    // pac::GPIOA.bsrr().write(|w| w.set_br(8, true));
    // pac::GPIOA.afr(8 / 8).modify(|w| {
    //     w.set_afr(8 % 8, 0x01);
    // });
    // pac::GPIOA.otyper().modify(|w| {
    //     w.set_ot(8, pac::gpio::vals::Ot::PUSHPULL);
    // });
    // pac::GPIOA.pupdr().modify(|w| {
    //     w.set_pupdr(8, Pull::None.into());
    //     // w.set_pupdr(8, pac::gpio::vals::Pupdr::FLOATING);
    // });
    // pac::GPIOA.moder().modify(|w| {
    //     w.set_moder(8, pac::gpio::vals::Moder::ALTERNATE);
    // });
    // pac::GPIOA.ospeedr().modify(|w| {
    //     w.set_ospeedr(8, pac::gpio::vals::Ospeedr::VERYHIGHSPEED);
    // });

    // Timer
    // The Embassy HAL can do almost all we need.
    //
    // The uncommented HAL code and the commented registry writes under it are equivalent.
    // Although the HAL code should be preferred where possible, the subtle errata for
    // enable_and_reset() should demonstrate why.

    TIM1::enable_and_reset();
    // pac::RCC.apb2enr().modify(|w| w.set_tim1en(true));
    // // // Errata: ES0005 - 2.1.11 Delay after an RCC peripheral clock enabling
    // // // only for stm32f2
    // // cortex_m::asm::dsb();
    // pac::RCC.apb2rstr().modify(|w| w.set_syscfgrst(true));
    // pac::RCC.apb2rstr().modify(|w| w.set_syscfgrst(false));

    p.TIM1.set_counting_mode(CountingMode::EdgeAlignedUp);
    // pac::TIM1.cr1().modify(|w| {
    //     // w.set_cen(false);
    //     let (cms, dir) = CountingMode::EdgeAlignedUp.into();
    //     w.set_dir(dir);
    //     w.set_cms(cms);
    // });

    // p.TIM1.set_frequency(Hertz(2));

    // 21Mhz (84Mhz / (2 clk signal) / 1 / 2)
    pac::TIM1.psc().write(|w| w.set_psc(2-1)); // prescaler // max 65535
    pac::TIM1.arr().write_value(Arr16(2-1)); // counter period // max 65535

    // // 2Hz (84Mhz / (2 clk signal) / 60_000 / 350)
    // pac::TIM1.psc().write(|w| w.set_psc(60_000 - 1)); // prescaler // max 65535
    // pac::TIM1.arr().write_value(Arr16(350-1)); // counter period // max 65535

    pac::TIM1.egr().write(|w| {
        w.set_ug(true);
    });
    pac::TIM1.cr1().modify(|w| {
        w.set_urs(pac::timer::vals::Urs::ANYEVENT);
    });

    p.TIM1.start();
    // pac::TIM1.cr1().modify(|r| r.set_cen(true));

    p.TIM1.enable_outputs();
    // pac::TIM1.bdtr().modify(|w| w.set_moe(true));

    p.TIM1.set_output_compare_mode(Channel::Ch1, OutputCompareMode::Toggle);
    // pac::TIM1.ccmr_output(0).write(|ccmr| {
    //     ccmr.set_ocm(0, OutputCompareMode::Toggle.into());
    //     // ccmr.set_ocm(0, pac::timer::vals::Ocm::TOGGLE);
    // });

    p.TIM1.enable_channel(Channel::Ch1, true);
    // pac::TIM1.ccer().modify(|w| {
    //     w.set_cce(Channel::Ch1.raw(), true);
    // });

    // Extra stuff from Cube HAL code not found in Embassy HAL.
    //
    // The only required registry write here is connecting the timer and DMA controller.
    // It is missing in the Embassy HAL.
    pac::TIM1.dier().modify(|dier| {
        dier.set_ccde(3, true);
        dier.set_ude(true); // enable DMA on TIM1_UP
    });
    // The others are default values, or not used in timing calculations.
    pac::TIM1.cr1().modify(|cr1| {
        cr1.set_ckd(pac::timer::vals::Ckd::DIV1); // internal clock division
        cr1.set_arpe(true); // auto-reload preload
        cr1.set_opm(pac::timer::vals::Opm::DISABLED); // one pulse mode
        cr1.set_udis(false); // update enable
    });
    pac::TIM1.cr2().modify(|cr2| {
        cr2.set_ccpc(false); // TRGO master/slave mode
        cr2.set_mms(pac::timer::vals::Mms::UPDATE); // TRGO trigger event selection
        // cr2.set_ccus(true); // capture/control update
        cr2.set_ccds(pac::timer::vals::Ccds::ONCOMPARE);
    });
    // pac::TIM1.egr().write(|w| {
    //     w.set_ug(true); // update generation
    // });
    pac::TIM1.ccmr_output(0).modify(|ccmr| {
        // ccmr.set_ccs(0, pac::timer::vals::CcmrOutputCcs::OUTPUT); // output compare is output
        ccmr.set_ocpe(0, pac::timer::vals::Ocpe::DISABLED); // output compare preload enable
    });
    // pac::TIM1.ccr(0).modify(|ccr| {
    //     ccr.set_ccr(0); // compare value
    // });
    // pac::TIM1.rcr().write_value(Rcr(0)); // repetition counter

    // // Demo value buffer to use for DMA.
    // // Uses the BSRR to set and reset outputs to step over and enable each GPIOB pin one by one.
    // let mut bsrr_values = [0x0000FFFFu32; 16];
    // for i in 0..bsrr_values.len() {
    //     let prev_i = if i == 0 { 15 } else { i - 1 };
    //     // bsrr_values[i] = (0x01 << (prev_i + 16)) | (0x01 << i);
    //     let mut b = Bsrr::default();
    //     b.set_br(prev_i, true);
    //     b.set_bs(i, true);
    //     bsrr_values[i] = b.0;
    // }
    // // for i in (0..bsrr_values.len()).step_by(2) {
    // //     bsrr_values[i] = 0xFFFF0000;
    // // }

    // let mut pb3 = Flex::new(p.PB3);
    // pb3.set_as_output(Speed::VeryHigh);
    // let mut bsrr_values2 = [0xFFFF0000u32; 16];
    // for i in (0..bsrr_values2.len()).step_by(2) {
    //     // let n = p.PB3._pin();
    //     let n = 3usize;
    //     // bsrr_values2[i] = 0x00000001;
    //     bsrr_values2[i] = 0u32 | (1u32 << n);
    // }

    // // 3 ODR values per serial value sent, 32 LEDs to steer, 20 brightness levels
    // let mut odr_values = [0u16; 3*32*20];
    // for i in (0..odr_values.len()).step_by(6) {
    //     odr_values[i] = 0x0001;
    //     odr_values[i+1] = 0x0003;
    //     odr_values[i+2] = 0x0001;
    //     odr_values[i+3] = 0x0000;
    //     odr_values[i+4] = 0x0002;
    //     odr_values[i+5] = 0x0000;
    // }
    let mut odr_values = [0xFFFFu16; 16];
    for i in (0..odr_values.len()).step_by(2) {
        odr_values[i] = 0;
    }
    // let mut odr_values2 = [0u16; 16];
    // for i in 0..odr_values2.len() {
    //     odr_values2[i] = 1u16 << i;
    // }


    // DMA
    // Embassy HAL is not used here. This code is purely made by setting up the project in
    // CubeIDE and seeing what the generated code does.

    // MX_DMA_Init
    // __HAL_RCC_DMA2_CLK_ENABLE();
    pac::RCC.ahb1enr().modify(|w| {
        // w.set_dma1en(true);
        w.set_dma2en(true);
    });
    // HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
    pac::Interrupt::DMA2_STREAM5.set_priority(Priority::P0);
    pac::Interrupt::DMA2_STREAM4.set_priority(Priority::P0);
    pac::DMA2.st(4).cr().modify(|w| {
        // w.set_en(false);
        w.set_chsel(6);
        w.set_circ(pac::dma::vals::Circ::ENABLED); // circular mode
        // w.set_circ(pac::dma::vals::Circ::DISABLED); // circular mode
        // w.set_ct(pac::dma::vals::Ct::MEMORY0); // double buffer bank select
        w.set_dbm(pac::dma::vals::Dbm::DISABLED); // double buffer mode
        w.set_dir(pac::dma::vals::Dir::MEMORYTOPERIPHERAL);
        w.set_minc(pac::dma::vals::Inc::INCREMENTED); // memory increment
        w.set_pinc(pac::dma::vals::Inc::FIXED); // peripheral not incrumented
        w.set_msize(pac::dma::vals::Size::BITS16); // memory size = word
        w.set_psize(pac::dma::vals::Size::BITS16);
        w.set_mburst(pac::dma::vals::Burst::SINGLE); // no bursting
        w.set_pburst(pac::dma::vals::Burst::SINGLE);
        w.set_pfctrl(pac::dma::vals::Pfctrl::DMA); // DMA sets flow
        w.set_pl(pac::dma::vals::Pl::HIGH); // priority
        w.set_tcie(true); // done interrupt enable
        w.set_teie(true); // error interrupt enable
    });
    pac::DMA2.st(5).cr().modify(|w| {
        // w.set_en(false);
        w.set_chsel(6);
        w.set_circ(pac::dma::vals::Circ::ENABLED); // circular mode
        // w.set_circ(pac::dma::vals::Circ::DISABLED); // circular mode
        // w.set_ct(pac::dma::vals::Ct::MEMORY0); // double buffer bank select
        w.set_dbm(pac::dma::vals::Dbm::DISABLED); // double buffer mode
        w.set_dir(pac::dma::vals::Dir::MEMORYTOPERIPHERAL);
        w.set_minc(pac::dma::vals::Inc::INCREMENTED); // memory increment
        w.set_pinc(pac::dma::vals::Inc::FIXED); // peripheral not incrumented
        w.set_msize(pac::dma::vals::Size::BITS16); // memory size = word
        w.set_psize(pac::dma::vals::Size::BITS16);
        w.set_mburst(pac::dma::vals::Burst::SINGLE); // no bursting
        w.set_pburst(pac::dma::vals::Burst::SINGLE);
        w.set_pfctrl(pac::dma::vals::Pfctrl::DMA); // DMA sets flow
        w.set_pl(pac::dma::vals::Pl::HIGH); // priority
        w.set_tcie(true); // done interrupt enable
        w.set_teie(true); // error interrupt enable
    });

    // Done in CubeIDE, breaks everything here.
    // pac::DMA2.ifcr(5).write_value(Ixr(0)); // clear all interrupt flags

    pac::DMA2.st(4).ndtr().write_value(Ndtr(odr_values.len() as u32)); // stream this many values
    pac::DMA2.st(4).m0ar().write_value(odr_values.as_ptr() as u32); // source address
    pac::DMA2.st(4).par().write_value(pac::GPIOA.odr().as_ptr() as u32); // destination address
    pac::DMA2.st(4).cr().modify(|w| w.set_en(true)); // enable

    pac::DMA2.st(5).ndtr().write_value(Ndtr(odr_values.len() as u32)); // stream this many values
    pac::DMA2.st(5).m0ar().write_value(odr_values.as_ptr() as u32); // source address
    pac::DMA2.st(5).par().write_value(pac::GPIOB.odr().as_ptr() as u32); // destination address
    pac::DMA2.st(5).cr().modify(|w| w.set_en(true)); // enable

    // Busy loop required
    loop {
        Timer::after_millis(1000).await;
        led.toggle();

        // Timer::after_millis(10_000).await;
        // led.toggle();
        // pac::DMA2.st(5).cr().modify(|w| w.set_en(true)); // re-enable DMA
    }

    // let executor = EXECUTOR_LS.init(Executor::new());
    // executor.run(|s| s.spawn(run_ls(sck_ls, sin_ls, lat_ls)).unwrap());
    // let executor = EXECUTOR_BLINK.init(Executor::new());
    // executor.run(|s| s.spawn(run_blink(led)).unwrap());
}

#[embassy_executor::task]
async fn run_ls2(
    mut sck: Output<'static, PB15>,
    mut sin: Output<'static, PA8>,
    mut lat: Output<'static, PA9>,
    ls1_nen: Output<'static, PB9>,
    ls2_nen: Output<'static, PB8>,
) {
    send_bits(&mut sck, &mut sin, [Level::Low; 32]).await;
    trigger_lat(&mut lat).await;
    Timer::after_secs(1).await;

    let mut driver = LedDriver::<_, _, _, 2, 2, 5>::new(sck, sin, lat, [ls1_nen.degrade(), ls2_nen.degrade()]);

    let leds = driver.get_bank_leds(0);
    for led in leds.into_iter() {
        led.r = 200.into();
    }
    leds[1].r = LedIntensity::MIN;

    let leds = driver.get_bank_leds(1);
    for led in leds.into_iter() {
        led.g = 200.into();
    }
    leds[0].g = LedIntensity::MIN;

    loop {
        driver.display_cycle_test().await;
        // Timer::after_secs(1).await;
    }
}

// // nice LED pulses
// #[embassy_executor::task]
// async fn run_ls(
//     mut sck: Output<'static, PB15>,
//     mut sin: Output<'static, PA8>,
//     mut lat: Output<'static, PA9>
// ) {
//     let mut m = 0;
//     loop {
//         send_bits(&mut sck, &mut sin, [Level::High; 32]).await;
//         trigger_lat(&mut lat).await;
//         Timer::after_micros(1).await;
//         send_bits(&mut sck, &mut sin, [Level::Low; 32]).await;
//         trigger_lat(&mut lat).await;
//         Timer::after_micros(m).await;

//         m = match m {
//             n if n >= 20_000 => 1,
//             _ => m + 100,
//         }
//     }
// }

// // ticker example
// #[embassy_executor::task]
// async fn run_ls(
//     mut sck: Output<'static, PB15>,
//     mut sin: Output<'static, PA8>,
//     mut lat: Output<'static, PA9>
// ) {
//     let mut ticker = Ticker::every(Duration::from_millis(500));
//     loop {
//         for m in [0, 1, 10, 100, 1000, 10_000] {
//             // let t = pin!(ticker.next());
//             loop {
//                 send_bits(&mut sck, &mut sin, [Level::High; 32]).await;
//                 trigger_lat(&mut lat).await;
//                 Timer::after_micros(1).await;
//                 send_bits(&mut sck, &mut sin, [Level::Low; 32]).await;
//                 trigger_lat(&mut lat).await;
//                 Timer::after_micros(m).await;

//                 if let core::task::Poll::Ready(_) = poll!(ticker.next()) {
//                     break;
//                 }
//                 // if TICK.load(core::sync::atomic::Ordering::Relaxed) {
//                 //     TICK.store(false, core::sync::atomic::Ordering::Relaxed);
//                 //     break;
//                 // }
//             }
//         }
//     }
// }

#[embassy_executor::task]
async fn run_ls(
    mut sck: Output<'static, PB15>,
    mut sin: Output<'static, PA8>,
    mut lat: Output<'static, PA9>,
) {
    // init with all LEDs off
    send_bits(&mut sck, &mut sin, [Level::Low; 32]).await;
    trigger_lat(&mut lat).await;

    loop {
        for ledlevels in [
            [Level::Low, Level::Low, Level::High], // red
            [Level::Low, Level::High, Level::Low], // green
            [Level::High, Level::Low, Level::Low], // blue
        ] {
            // for each (chained) chip
            for _ in 0..2 {
                // 16th output is not used, fill with 0
                send_bits(&mut sck, &mut sin, [Level::Low; 1]).await;
                // for each LED
                for _ in 0..5 {
                    send_bits(&mut sck, &mut sin, ledlevels).await;
                }
            }
            trigger_lat(&mut lat).await;
            Timer::after_millis(1000).await;
        }
    }
}

async fn send_bits<const N: usize>(
    sck: &mut Output<'static, PB15>,
    sin: &mut Output<'static, PA8>,
    levels: [Level; N],
) {
    for l in levels {
        sin.set_level(l);
        sck.set_high();
        // Timer::after_millis(10).await;
        sck.set_low();
        // Timer::after_millis(10).await;
    }
    sin.set_low();
}

async fn trigger_lat(lat: &mut Output<'static, PA9>) {
    lat.set_high();
    // Timer::after_millis(10).await;
    lat.set_low();
    // Timer::after_millis(10).await;
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
