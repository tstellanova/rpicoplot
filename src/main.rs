#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;
use rp2040_hal as p_hal;
use fugit::RateExtU32;
use embedded_hal::adc::OneShot;
use rand_core::RngCore;
use embedded_graphics_sparklines::*;

use ssd1351::{
    self,
    properties::{DisplaySize, DisplayRotation},
    interface::SpiInterface,
    mode::{GraphicsMode, displaymode::DisplayModeTrait},

};


use embedded_graphics::{
    pixelcolor::Rgb565, prelude::*, primitives::{Polyline, PrimitiveStyle},
};

use embedded_graphics::{
    mono_font::{ascii::FONT_5X7, ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Circle, Line, PrimitiveStyleBuilder, Rectangle, StrokeAlignment},
    text::{Alignment, Text},
};


// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};


const DISPLAY_BUF_SIZE: usize = 32768;
static mut FAST_IMG0: [u8; DISPLAY_BUF_SIZE] = [0u8; DISPLAY_BUF_SIZE];

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    let _spi0_sck_pin = pins.gpio6.into_mode::<p_hal::gpio::FunctionSpi>();
    let _spi0_do_pin = pins.gpio7.into_mode::<p_hal::gpio::FunctionSpi>();
    //let mut spi0_di_pin  = pins.gpio4.into_mode::<p_hal::gpio::FunctionSpi>();

    let dc_pin  = pins.gpio8.into_push_pull_output();
    let _cs_pin = pins.gpio9.into_push_pull_output();
    let spi0 = p_hal::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let spi0 = spi0.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16u32.MHz(),
        &embedded_hal::spi::MODE_0,
    );

    let spii = SpiInterface::new(spi0, dc_pin );
    let mut display_base = ssd1351::display::Display::new(
        spii, DisplaySize::Display128x128, DisplayRotation::Rotate0);
    let _ = display_base.init();


    let mut display = unsafe {
	GraphicsMode::new(display_base, &mut FAST_IMG0)
    };

    let mut adc = p_hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temp_sensor = adc.enable_temp_sensor();

    let mut raw_rng = p_hal::rosc::RingOscillator::new(pac.ROSC).initialize();

    let bbox = Rectangle::new(Point::new(4, 4), Size::new(124, 124));
    let draw_fn = |lastp, p| Line::new(lastp, p);

    const NUM_BUF_SAMPLES:usize = 64;
    const NUM_DRAW_SAMPLES:usize = 64;

    // create sparkline object
    let mut sparkline = Sparkline::<_,_,_,NUM_BUF_SAMPLES>::new(
      bbox, // position and size of the sparkline
      NUM_DRAW_SAMPLES,   //  max samples to display on graph 
      Rgb565::GREEN,
      1, // stroke size
      draw_fn,
    );


    let border_stroke = PrimitiveStyleBuilder::new()
	.stroke_color(Rgb565::BLUE)
	.stroke_width(3)
	.stroke_alignment(StrokeAlignment::Inside)
	.build();

    loop {
        //info!("on!");
        led_pin.set_high().unwrap();

	display.clear(false);
	let _ = display.bounding_box().into_styled(border_stroke).draw(&mut display);
	
	let traw:u16 = adc.read(&mut temp_sensor).unwrap();
	let tvolt:f32 = (traw as f32) * (3.30f32/4095f32);
	//info!("traw: {} tvolt: {}", traw, tvolt);
        // T = 27 - (ADC_voltage - 0.706)/0.001721
	let temp_c = 27.0f32 - ((tvolt - 0.706)/0.001721);
        info!("temp_c: {}", temp_c);
        let scaled_val= (temp_c * 100f32) as i32;

        
        let _rand_val = raw_rng.next_u32();
        /*
        let scaled_val= (rand_val/2) as i32; 
        info!("rand_val: {} scaled: {}", rand_val, scaled_val);
        */

        sparkline.add(scaled_val);
        let _ = sparkline.draw(&mut display);
        display.flush();
        //delay.delay_ms(500);
        //info!("off!");
        led_pin.set_low().unwrap();
        //delay.delay_ms(300);
    }

/*
    delay.delay_ms(3000);
    loop {
        cortex_m::asm::bkpt();
    }
*/
}

