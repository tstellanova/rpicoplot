#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;
use rp2040_hal as p_hal;
use fugit::RateExtU32;
use micromath::F32Ext;
use embedded_hal::adc::OneShot;
use rand_core::RngCore;
use embedded_graphics_sparklines::*;
use numtoa::NumToA;
use arraystring::ArrayString;
use typenum::{U40,U80};

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
    prelude::*,
    primitives::{ Line, PrimitiveStyleBuilder, Rectangle, StrokeAlignment},
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

    let mut rst_pin = pins.gpio10.into_push_pull_output();
    rst_pin.set_low().unwrap();

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
    rst_pin.set_high().unwrap(); //re-enable OLED

    let mut display_base = ssd1351::display::Display::new(
        spii, DisplaySize::Display128x128, DisplayRotation::Rotate0);
    let _ = display_base.init();


    let mut display = unsafe {
	GraphicsMode::new(display_base, &mut FAST_IMG0)
    };

    let mut adc = p_hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut temp_sensor = adc.enable_temp_sensor();
    let mut raw_rng = p_hal::rosc::RingOscillator::new(pac.ROSC).initialize();

    const NUM_DRAW_SAMPLES:usize = 64;
    const NUM_BUF_SAMPLES:usize = 64;
    const SUBPLOT_W: u32 = 64;
    const SUBPLOT_H: u32 = 64;
 
    //create plots
    let bbox00 = Rectangle::new(Point::new(0, 0), Size::new(SUBPLOT_W, SUBPLOT_H));
    let mut plot00 = Sparkline::<_,_,_,NUM_BUF_SAMPLES>::new(
      bbox00,
      NUM_DRAW_SAMPLES, // nsamples to display
      Rgb565::GREEN,
      1, // stroke size
      |lastp, p| Line::new(lastp, p),
    );

    let bbox10 = Rectangle::new(Point::new(SUBPLOT_W as i32, 0), Size::new(SUBPLOT_W, SUBPLOT_H));
    let mut plot10 = Sparkline::<_,_,_,NUM_BUF_SAMPLES>::new(
      bbox10,
      NUM_DRAW_SAMPLES, // nsamples to display
      Rgb565::YELLOW,
      1, // stroke size
      |lastp, p| Line::new(lastp, p),
    );

    let bbox01 = Rectangle::new(Point::new(0, SUBPLOT_H as i32), Size::new(SUBPLOT_W, SUBPLOT_H));
    let mut plot01 = Sparkline::<_,_,_,NUM_BUF_SAMPLES>::new(
      bbox01,
      NUM_DRAW_SAMPLES, //samples to display
      Rgb565::WHITE,
      1, // stroke size
      |lastp, p| Line::new(lastp, p),
    );

    let bbox11 = Rectangle::new(Point::new(SUBPLOT_W as i32, SUBPLOT_H as i32), Size::new(SUBPLOT_W, SUBPLOT_H));
    let mut plot11 = Sparkline::<_,_,_,NUM_BUF_SAMPLES>::new(
      bbox11,
      NUM_DRAW_SAMPLES, // nsamples to display
      Rgb565::RED,
      1, // stroke size
      |lastp, p| Line::new(lastp, p),
    );

    let cyan_frame_style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::CYAN)
        .stroke_width(1)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();
    let magenta_frame_style =  PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::MAGENTA)
        .stroke_width(1)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();

    let frame_border_stroke = PrimitiveStyleBuilder::new()
	.stroke_color(Rgb565::BLUE)
	.stroke_width(1)
	.stroke_alignment(StrokeAlignment::Inside)
	.build();
    let frame_styled =  display.bounding_box().into_styled(frame_border_stroke);
 
    let mut sawtooth_count:i32 = 0;
    let mut sine_count:f32 = 0f32;
    let mut loop_count:i32 = 0;
    let subplot_frame_strokes: [_; 4] = [ cyan_frame_style, magenta_frame_style, magenta_frame_style, cyan_frame_style ];
    let subplot_boxes: [_; 4] = [ bbox00, bbox10, bbox01, bbox11 ];
	

    loop {
        //info!("on!");
        led_pin.set_high().unwrap();

	display.clear(false);
	let _ = frame_styled.draw(&mut display);
	for i in 0..4 {
          let _ = subplot_boxes[i].into_styled(subplot_frame_strokes[i]).draw(&mut display);
        }

	let traw:u16 = adc.read(&mut temp_sensor).unwrap();
	let tvolt:f32 = (traw as f32) * (3.30f32/4095f32);
	//info!("traw: {} tvolt: {}", traw, tvolt);
        // T = 27 - (ADC_voltage - 0.706)/0.001721
	let temp_c = 27.0f32 - ((tvolt - 0.706)/0.001721);
        //info!("temp_c: {}", temp_c);
        let scaled_val = (temp_c * 2f32) as i32;
        plot00.add(scaled_val);
        let _ = plot00.draw(&mut display);

        plot10.add(sawtooth_count);
        let _ = plot10.draw(&mut display);

        let sinval = (100f32*f32::sin(sine_count)) as i32;
        plot01.add(sinval);
        let _ = plot01.draw(&mut display);

        let rand_val = raw_rng.next_u32();
        let scaled_val= (rand_val/2) as i32; 
        //info!("rand_val: {} scaled: {}", rand_val, scaled_val);
        plot11.add(scaled_val);
	let _ = plot11.draw(&mut display);

	let mut num_buffer = [0u8; 20];
	let mut text_buf =  ArrayString::<U40>::new();
	text_buf.push_str("count ");
        text_buf.push_str(loop_count.numtoa_str(10, &mut num_buffer));

        // subplot labels
        let _ = Text::with_alignment(
            &text_buf,
            display.bounding_box().top_left + Point::new(4, 8),
            MonoTextStyle::new(&FONT_6X10, Rgb565::RED),
            Alignment::Left,
        )
        .draw(&mut display);

        sawtooth_count = (sawtooth_count+ 1) % ((scaled_val % 128) + 1);
        sine_count += 0.20f32;
        loop_count += 1;

        led_pin.set_low().unwrap();
        display.flush();
        //info!("off!");
        delay.delay_ms(5);
    }

/*
    delay.delay_ms(3000);
    loop {
        cortex_m::asm::bkpt();
    }
*/
}

