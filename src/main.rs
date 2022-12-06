//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
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
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::primitives::Line;

use ssd1351::{
    self,
    properties::{DisplaySize, DisplayRotation},
    interface::SpiInterface,
    mode::{GraphicsMode, displaymode::DisplayModeTrait},

};


use embedded_graphics::{
    pixelcolor::Rgb565, prelude::*, primitives::{Polyline, PrimitiveStyle},
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

    // A "heartbeat" shaped polyline
    let points: [Point; 13] = [
	Point::new(5, 32),
	Point::new(25, 64),
	Point::new(30, 44),
	Point::new(35, 64),
	Point::new(40, 64),
	Point::new(45, 74),
	Point::new(50, 10),
	Point::new(55, 84),
	Point::new(60, 64),
        Point::new(65, 0),
	Point::new(70, 64),
	Point::new(75, 128),
	Point::new(128, 64),
    ];

    let line_style = PrimitiveStyle::with_stroke(Rgb565::GREEN, 3);

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
    let mut display = GraphicsMode::new(display_base);

    Polyline::new(&points)
      .into_styled(line_style)
      .draw(&mut display).unwrap();


    let mut adc = p_hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temp_sensor = adc.enable_temp_sensor();

    let mut raw_rng = p_hal::rosc::RingOscillator::new(pac.ROSC).initialize();

	let bbox = Rectangle::new(Point::new(4, 4), Size::new(120, 120));
	let draw_fn = |lastp, p| Line::new(lastp, p);

	// create sparkline object
	let mut sparkline = Sparkline::new(
	bbox, // position and size of the sparkline
	32,   // max samples to store in memory (and display on graph)
        Rgb565::GREEN,
	//BinaryColor::On,
	1, // stroke size
	draw_fn,
	);

    display.clear();

    loop {
        //info!("on!");
        led_pin.set_high().unwrap();
        let traw:u16 = adc.read(&mut temp_sensor).unwrap();
        let scaled_high = (128*traw)/4095;
	info!("scaled Y: {}", scaled_high);
	let tvolt:f32 = (traw as f32) * (3.30f32/4095f32);
	info!("traw: {} tvolt: {}", traw, tvolt);
        // T = 27 - (ADC_voltage - 0.706)/0.001721
	let temp_c = 27.0f32 - ((tvolt - 0.706)/0.001721);
        info!("temp_c: {}", temp_c);
        let rand_val = raw_rng.next_u32();
        let scaled_high = (rand_val % 128);
        info!("rand_val: {} scaled Y: {}", rand_val, scaled_high);
        sparkline.add(rand_val as i32);
        let _ = sparkline.draw(&mut display);

        //delay.delay_ms(500);
        //info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

