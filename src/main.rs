#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;
use rp2040_hal as p_hal;
use fugit::{ RateExtU32};

use embedded_hal::adc::OneShot;
use rand_core::RngCore;
//use emplot::*;
use numtoa::NumToA;
use arraystring::ArrayString;
use typenum::{U40};

// image file format supported
use tinyqoi::Qoi;

//use embedded_graphics_core::draw_target::DrawTarget;

use embedded_graphics::{
    prelude::*,
    draw_target::DrawTarget,
    mono_font::{
	//ascii::FONT_5X7, 
	ascii::FONT_6X10, MonoTextStyle},
    image::{Image },
    pixelcolor::{ Rgb565 },
    primitives::{ Sector, Ellipse, PrimitiveStyleBuilder, Rectangle, StrokeAlignment},
    text::{Alignment, Text},
};

use  display_interface_spi::{SPIInterface };

use mipidsi::{
  ColorInversion,
};

type DisplayColor = Rgb565; //ST7789::ColorFormat; 


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


const DISPLAY_WIDTH: i32 = 320;
const DISPLAY_HEIGHT: i32 = 240;


// framebuffer for faster rendering to OLED display; TODO move to different memory section?
//static mut FAST_IMG0: [u8; DISPLAY_BUF_SIZE] = [0u8; DISPLAY_BUF_SIZE];

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

    let mut delay_source = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let timer = p_hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    //let mut count_down = timer.count_down();

    //let mut clocko =  Rp2040Monotonic ::new(pac.TIMER);
    let mut led_pin = pins.led.into_push_pull_output();

    let _spi0_sck_pin = pins.gpio6.into_mode::<p_hal::gpio::FunctionSpi>();
    let _spi0_do_pin = pins.gpio7.into_mode::<p_hal::gpio::FunctionSpi>();

    // rst_pin is used to reset the OLED display during power-on sequence
    let mut rst_pin = pins.gpio10.into_push_pull_output();
    rst_pin.set_high().unwrap();
    delay_source.delay_us(200);
    rst_pin.set_low().unwrap();

    let mut backlight_pin = pins.gpio11.into_push_pull_output();
    backlight_pin.set_high().unwrap();

    let dc_pin  = pins.gpio8.into_push_pull_output();
    let cs_pin = pins.gpio9.into_push_pull_output();
    let spi0 = p_hal::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialized SPI driver for an initialized one
    let spi0 = spi0.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16u32.MHz(),
        //&embedded_hal::spi::MODE_0,
        &embedded_hal::spi::MODE_3,
    );

    let spii = SPIInterface::new(spi0, dc_pin, cs_pin );
 
    // ensure that we keep rst_pin low for at least this long
    delay_source.delay_us(100);
    rst_pin.set_high().unwrap(); //re-enable OLED

    //let mut display_base = ssd1351::display::Display::new(
    //    spii, DISPLAY_SIZE, DisplayRotation::Rotate0);
    let mut display =  mipidsi::Builder::st7789(spii)
      .with_invert_colors(ColorInversion::Inverted)
      //.with_color_order(ColorOrder::Bgr)
      .with_orientation(mipidsi::options::Orientation::LandscapeInverted(true))
      .with_display_size(DISPLAY_WIDTH as u16, DISPLAY_HEIGHT as u16)
      .with_framebuffer_size(DISPLAY_WIDTH as u16, DISPLAY_HEIGHT as u16)
      .init(&mut delay_source,  Some(rst_pin)).unwrap();
    //let _ = display_base.init();

    //let mut display = Display::with_model(di, Some(rst_display), DisplayModel::new());
    let _ = display.clear(Rgb565::BLACK);
 
   /*
    let mut display = unsafe {
	GraphicsMode::new(display_base, &mut FAST_IMG0)
    };
    */

    let mut adc = p_hal::Adc::new(pac.ADC, &mut pac.RESETS);
    // analog input read from GPIO26 / A0;
    let mut adc_pin_0 = pins.gpio26.into_floating_input();
    let mut adc_pin_1 = pins.gpio27.into_floating_input();
    //let mut gpio28_in = pins.gpio28.into_floating_input();
    let mut adc_pin_2 = pins.gpio28.into_floating_input();
    let mut temp_sensor = adc.enable_temp_sensor();
    let mut raw_rng = p_hal::rosc::RingOscillator::new(pac.ROSC).initialize();

    let mut num_buffer = [0u8; 20];
    let mut text_buf =  ArrayString::<U40>::new();

    // Parse QOI image.
    let img_data = include_bytes!("../img/240_dim_fleur.qoi"); //gold_shaded_fleur_de_lis.qoi");
    //info!("img_data.len(): {} ", img_data.len());
    let qoi = Qoi::new(img_data).unwrap();
    let img_size = qoi.size();
    let img_inset_point = Point::new(
      (DISPLAY_WIDTH- img_size.width as i32)/2,
      (DISPLAY_HEIGHT - img_size.height as i32)/2 );
    let bg_img = Image::new(&qoi, img_inset_point);

    //const NUM_DRAW_SAMPLES:usize = (DISPLAY_BUF_SIZE/ 2) as usize;
    //const NUM_BUF_SAMPLES:usize =  (DISPLAY_BUF_SIZE/ 2) as usize; 
    const SUBPLOT_W: u32 = (DISPLAY_WIDTH / 2) as u32;
    const SUBPLOT_H: u32 = (DISPLAY_HEIGHT / 2) as u32;
    
    const PURPLE_DARK:DisplayColor = DisplayColor::new(14, 0, 13);// #6D006A
    const PURPLE_MID:DisplayColor = DisplayColor::new(17, 0, 11);// #880085 
    const PURPLE_LIGHT:DisplayColor = DisplayColor::new(21, 0, 21);// #AA00A6

    const GOLD_DARK:DisplayColor = DisplayColor::new(29, 43, 2); 
    const GOLD_MID:DisplayColor = DisplayColor::new(31, 48, 1); 
    const GOLD_LIGHT:DisplayColor = DisplayColor::new(31, 63, 0); 

    const GREEN_DARK:DisplayColor = DisplayColor::new(0, 42, 1); 
    const GREEN_MID:DisplayColor = DisplayColor::GREEN;
    const GREEN_LIGHT:DisplayColor = DisplayColor::new(0, 63, 0);  


    let ctr_point = Point::new(SUBPLOT_W as i32, SUBPLOT_H as i32);
    //create plots
    let bbox00 = Rectangle::new(Point::new(0, 0), Size::new(SUBPLOT_W, SUBPLOT_H));
    /*
    let mut plot00 = Emplot::<_,NUM_BUF_SAMPLES>::new(
      bbox00,
      NUM_DRAW_SAMPLES,  
      GOLD_DARK,
      2, // stroke size
    );
    */
    let bbox10 = Rectangle::new(Point::new(SUBPLOT_W as i32, 0), Size::new(SUBPLOT_W, SUBPLOT_H));
    /*
    let mut plot10 = Emplot::<_,NUM_BUF_SAMPLES>::new(
      bbox10,
      NUM_DRAW_SAMPLES,  
      GREEN_DARK,
      2, // stroke size
    );
    */
    let bbox01 = Rectangle::new(Point::new(0, SUBPLOT_H as i32), Size::new(SUBPLOT_W, SUBPLOT_H));
    /*
    let mut plot01 = Emplot::<_,NUM_BUF_SAMPLES>::new(
      bbox01,
      NUM_DRAW_SAMPLES, 
      PURPLE_DARK,
      2, // stroke size
    );
i   */
    let bbox11 = Rectangle::new(Point::new(SUBPLOT_W as i32, SUBPLOT_H as i32), Size::new(SUBPLOT_W, SUBPLOT_H));
    /*
    let mut plot11 = Emplot::<_,NUM_BUF_SAMPLES>::new(
      bbox11,
      NUM_DRAW_SAMPLES, 
      GOLD_DARK, 
      2, // stroke size
    );
    */

    let cyan_frame_style = PrimitiveStyleBuilder::new()
        .stroke_color(DisplayColor::CYAN)
        .stroke_width(1)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();
    let magenta_frame_style =  PrimitiveStyleBuilder::new()
        .stroke_color(DisplayColor::MAGENTA)
        .stroke_width(1)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();

    let frame_border_stroke = PrimitiveStyleBuilder::new()
	.stroke_color(DisplayColor::BLUE)
	.stroke_width(1)
	.stroke_alignment(StrokeAlignment::Inside)
	.build();
    //let frame_styled =  display.bounding_box().into_styled(frame_border_stroke);
 
    let purple_mid_fill =  PrimitiveStyleBuilder::new().fill_color(PURPLE_MID).build();
    let purple_light_fill =  PrimitiveStyleBuilder::new().fill_color(PURPLE_LIGHT).build();

    let gold_mid_fill =  PrimitiveStyleBuilder::new().fill_color(GOLD_MID).build();
    let gold_light_fill =  PrimitiveStyleBuilder::new().fill_color(GOLD_LIGHT).build();

    let green_mid_fill =  PrimitiveStyleBuilder::new().fill_color(GREEN_MID).build();
    let green_light_fill =  PrimitiveStyleBuilder::new().fill_color(GREEN_LIGHT).build();


    let red_fill_style =  PrimitiveStyleBuilder::new()
    .fill_color(DisplayColor::RED)
    .build();
    let cyan_fill_style =  PrimitiveStyleBuilder::new()
    .fill_color(DisplayColor::CYAN)
    .build();
    let magenta_fill_style =  PrimitiveStyleBuilder::new()
    .fill_color(DisplayColor::MAGENTA)
    .build();
    let blue_fill_style =  PrimitiveStyleBuilder::new()
    .fill_color(DisplayColor::BLUE)
    .build();


    let mut loop_count:i32 = 0;
    let subplot_frame_strokes: [_; 4] = [ cyan_frame_style, magenta_frame_style, magenta_frame_style, cyan_frame_style ];
    let subplot_boxes: [_; 4] = [ bbox00, bbox10, bbox01, bbox11 ];

    let sect_radius: i32 = SUBPLOT_H as i32;
    // this is the top-left point of the bounding box around the circle from which sectors are cut
    let sect_pt = Point::new(ctr_point.x - sect_radius, ctr_point.y - sect_radius);
    let sect_dia = (sect_radius * 2) as u32;
    let sect_tr = Sector::new(sect_pt, sect_dia, Angle::from_degrees(45.0), Angle::from_degrees(10.0));
    let sect_tl = Sector::new(sect_pt, sect_dia, Angle::from_degrees(135.0), Angle::from_degrees(10.0));
    let sect_bl = Sector::new(sect_pt, sect_dia, Angle::from_degrees(225.0), Angle::from_degrees(10.0));
    let sect_br = Sector::new(sect_pt, sect_dia, Angle::from_degrees(315.0), Angle::from_degrees(10.0));

	
    let ellipse_minor:u32 = SUBPLOT_W / 6;
    let ellipse_major:u32 = SUBPLOT_H ;
    let ellipse_h_ctr:i32 = (SUBPLOT_W - ellipse_minor/2) as i32;
    let ellipse_v_ctr:i32 = (SUBPLOT_H - ellipse_minor/2) as i32;
    let v_ell_size = Size::new(ellipse_minor, ellipse_major);
    let v_ellipse_t = Ellipse::new(
      Point::new(ellipse_h_ctr, 0), v_ell_size);
    let v_ellipse_b = Ellipse::new(
       Point::new(ellipse_h_ctr,  SUBPLOT_H as i32), v_ell_size);


    let h_ell_size =  Size::new(ellipse_major, ellipse_minor);
    let h_ellipse_l = Ellipse::new(
      Point::new((ellipse_h_ctr - ellipse_major as i32) as i32, ellipse_v_ctr ), h_ell_size);
    let h_ellipse_r = Ellipse::new(
      Point::new(ellipse_h_ctr, ellipse_v_ctr), h_ell_size);

    let mut start_time;
    loop {
        //info!("on!");
        led_pin.set_high().unwrap();
        start_time = timer.get_counter();
        // draw frames
        //let _ = display.clear(Rgb565::BLACK);

        // Draw image to display.
        let _ = bg_img.draw(&mut display.color_converted()).unwrap(); 

        // draw some colored ellipses
        let _ = display.draw_iter(h_ellipse_l.into_styled(red_fill_style).pixels());
        let _ = display.draw_iter(v_ellipse_t.into_styled(green_mid_fill).pixels());
        let _ = display.draw_iter(h_ellipse_r.into_styled(purple_light_fill).pixels());
        let _ = display.draw_iter(v_ellipse_b.into_styled(gold_mid_fill).pixels());

        // draw some rays from center
        let _ = display.draw_iter(sect_tl.into_styled(green_light_fill).pixels()); //.draw(&mut display);
        let _ = display.draw_iter(sect_tr.into_styled(green_mid_fill).pixels()); //draw(&mut display);
        let _ = display.draw_iter(sect_br.into_styled(purple_light_fill).pixels()); //draw(&mut display);
        let _ = display.draw_iter(sect_bl.into_styled(purple_mid_fill).pixels()); //draw(&mut display);


        for i in 0..4 {
          //let _ = subplot_boxes[i].into_styled(subplot_frame_strokes[i]).draw(&mut display);
          let _ = display.draw_iter(subplot_boxes[i].into_styled(subplot_frame_strokes[i]).pixels());
        }

        let adc0_raw_val : u16 = adc.read(&mut adc_pin_0).unwrap();
        //plot00.push(adc0_raw_val  as f32);

        let adc1_raw_val : u16 = adc.read(&mut adc_pin_1).unwrap();
        //plot10.push(adc1_raw_val as f32);

        let rand_val = raw_rng.next_u32();

        let adc2_raw_val: u16 = adc.read(&mut adc_pin_2).unwrap();
        //plot01.push(adc2_raw_val as f32);

        // read the temperature of the rp2040
        let traw:u16 = adc.read(&mut temp_sensor).unwrap();
        //plot11.push(traw as f32); //temp_c);


        // draw all the sub-plots
        /*
        let _ = plot00.draw(&mut display);
        let _ = plot10.draw(&mut display);
        let _ = plot01.draw(&mut display);
        let _ = plot11.draw(&mut display);
        */

        // draw a text label
        text_buf.clear();
	text_buf.push_str("2023");
        text_buf.push_str(loop_count.numtoa_str(10, &mut num_buffer));

        // labels
        /*
        let _ = display.draw_iter(Text::with_alignment(
            &text_buf,
            display.bounding_box().top_left + Point::new(4, 8),
            MonoTextStyle::new(&FONT_6X10, GREEN_MID),
            Alignment::Left,
        )
        .pixels());
        */

        loop_count += 1;

        led_pin.set_low().unwrap();

/*
	let rot = match (rand_val % 4) {
		1 => DisplayRotation::Rotate90,
		2 => DisplayRotation::Rotate180,
		3 => DisplayRotation::Rotate270,
		_=> DisplayRotation::Rotate0,
        };
	display.set_rotation(rot);
i*/
        //display.flush();
        //info!("off!");
	let end_time =  timer.get_counter();
        let delta = end_time - start_time;
        info!("micros: {:?}", delta);
    }

/*
    delay.delay_ms(3000);
    loop {
        cortex_m::asm::bkpt();
    }
*/
}

