use display_interface_spi::SPIInterface;
use embedded_hal::digital::v2::OutputPin;
use esp_idf_hal::gpio::Gpio21;
use esp_idf_hal::gpio::Pin;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi;
use esp_idf_hal::spi::SPI2;
use esp_idf_hal::units::FromValueType;
use esp_idf_sys as _;
use smart_leds::hsv::{hsv2rgb, Hsv};
use smart_leds::SmartLedsWrite;
use std::thread;
use std::time::Duration;
use ws2812_esp32_rmt_driver::Ws2812Esp32Rmt; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

fn main() {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    println!("Hello, world!");

    let peripherals = Peripherals::take().unwrap();

    let pins = peripherals.pins;

    let mut neopixel_power = pins.gpio2.into_output_od().unwrap();
    neopixel_power.set_high().unwrap();
    let neopixel_data = pins.gpio0.into_output_od().unwrap();

    let mut ws2812 = Ws2812Esp32Rmt::new(0, neopixel_data.pin() as u32).unwrap();

    //

    let config = <spi::config::Config as Default>::default().baudrate(24.MHz().into());

    let spi = spi::Master::<SPI2, _, _, _, Gpio21<esp_idf_hal::gpio::Unknown>>::new(
        peripherals.spi2,
        spi::Pins {
            sclk: pins.gpio5,
            sdo: pins.gpio19,
            sdi: Some(pins.gpio21),
            cs: None,
        },
        config,
    )
    .unwrap();
    let lcd_dc = pins.gpio12.into_output().unwrap();
    let lcd_rst = pins.gpio27.into_output().unwrap();
    let lcd_cs = pins.gpio33.into_output().unwrap();
    let di = SPIInterface::new(spi, lcd_dc, lcd_cs);

    #[cfg(feature = "mipidsi")]
    use mipidsi_b::{lcd_test_pattern, lcd_test_pattern_2, make_display, rgb565};
    #[cfg(feature = "st7789")]
    use st7789b::{lcd_test_pattern, lcd_test_pattern_2, make_display, rgb565};
    let mut lcd = make_display(di, lcd_rst).unwrap();

    let c1 = rgb565(0x1f, 0x3f, 0x1f);
    let c2 = rgb565(0, 0, 0);
    let c3 = rgb565(0, 0x3f, 0x1f);
    let c4 = rgb565(0, 0, 0x1f);

    lcd_test_pattern(&mut lcd, c1, c2).unwrap();

    println!("beep");

    let mut hue = 0;
    loop {
        let color = hsv2rgb(Hsv {
            hue,
            sat: 255,
            val: 8,
        });
        let pixels = std::iter::once(color);
        ws2812.write(pixels).unwrap();

        let bamboo = hue & 0x10 != 0;
        lcd_test_pattern_2(
            &mut lcd,
            if bamboo { c1 } else { c3 },
            if bamboo { c2 } else { c4 },
        )
        .unwrap();
        println!("boop");

        thread::sleep(Duration::from_millis(25)); // must let the RTOS pacify the watchdog

        hue = hue.wrapping_add(5);
    }
}

#[cfg(feature = "st7789")]
mod st7789b {
    use display_interface::WriteOnlyDataCommand;
    use embedded_graphics_core::geometry::{OriginDimensions, Size};
    use embedded_hal::digital::v2::OutputPin;
    use st7789::{Orientation, ST7789};

    pub fn make_display<DI, RST, PinE>(
        di: DI,
        lcd_rst: RST,
    ) -> Result<ST7789<DI, RST>, st7789::Error<PinE>>
    where
        DI: WriteOnlyDataCommand,
        RST: OutputPin<Error = PinE>,
    {
        let mut lcd = ST7789::new(di, lcd_rst, 240, 320);
        lcd.set_orientation(Orientation::Portrait)?;

        let mut delay = esp_idf_hal::delay::Ets;
        lcd.init(&mut delay)?;

        Ok(lcd)
    }

    pub fn rgb565(red: u8, green: u8, blue: u8) -> u16 {
        ((red as u16) << 11) | ((green as u16) << 5) | (blue as u16)
    }

    pub fn lcd_test_pattern<DI, RST, PinE>(
        lcd: &mut ST7789<DI, RST>,
        c1: u16,
        c2: u16,
    ) -> Result<(), st7789::Error<PinE>>
    where
        DI: WriteOnlyDataCommand,
        RST: OutputPin<Error = PinE>,
    {
        let Size { width, height } = lcd.size();
        let pattern = (0..height as i32).flat_map(|y| {
            (0..width as i32).map(move |x| {
                let dx = x - width as i32 / 2;
                let dy = y - height as i32 / 2;
                if dx * dx * 2 + dy * dy < 40 * 40 {
                    c1
                } else {
                    c2
                }
            })
        });
        lcd.set_pixels(0, 0, width as u16, height as u16, pattern)
    }

    pub fn lcd_test_pattern_2<DI, RST, PinE>(
        lcd: &mut ST7789<DI, RST>,
        c1: u16,
        c2: u16,
    ) -> Result<(), st7789::Error<PinE>>
    where
        DI: WriteOnlyDataCommand,
        RST: OutputPin<Error = PinE>,
    {
        let Size { width, height } = lcd.size();
        let pattern = (0..height as i32).flat_map(|y| {
            (0..width as i32).map(move |x| if (x / 8) % 2 == (y / 8) % 2 { c1 } else { c2 })
        });
        lcd.set_pixels(0, 0, width as u16, height as u16, pattern)
    }
}

#[cfg(feature = "mipidsi")]
mod mipidsi_b {
    use display_interface::WriteOnlyDataCommand;
    use embedded_graphics_core::geometry::{OriginDimensions, Size};
    use embedded_graphics_core::pixelcolor::Rgb565;
    use embedded_hal::digital::v2::OutputPin;
    use mipidsi::models::{Model, ST7789};
    use mipidsi::{Display, DisplayOptions, Orientation};

    pub fn make_display<DI, RST, PinE>(
        di: DI,
        lcd_rst: RST,
    ) -> Result<Display<DI, RST, ST7789>, mipidsi::Error<PinE>>
    where
        DI: WriteOnlyDataCommand,
        RST: OutputPin<Error = PinE>,
    {
        let mut lcd = Display::st7789(di, lcd_rst);
        let mut delay = esp_idf_hal::delay::Ets;
        lcd.init(
            &mut delay,
            DisplayOptions {
                orientation: Orientation::PortraitInverted(false),
                invert_vertical_refresh: false,
                color_order: Default::default(),
                invert_horizontal_refresh: false,
            },
        )?;

        Ok(lcd)
    }

    pub fn rgb565(red: u8, green: u8, blue: u8) -> Rgb565 {
        Rgb565::new(red, green, blue)
    }

    pub fn lcd_test_pattern<DI, RST, M, C: Copy, PinE>(
        lcd: &mut Display<DI, RST, M>,
        c1: C,
        c2: C,
    ) -> Result<(), mipidsi::Error<PinE>>
    where
        DI: WriteOnlyDataCommand,
        RST: OutputPin<Error = PinE>,
        M: Model<ColorFormat = C>,
    {
        let Size { width, height } = lcd.size();
        let pattern = (0..height as i32).flat_map(|y| {
            (0..width as i32).map(move |x| {
                let dx = x - width as i32 / 2;
                let dy = y - height as i32 / 2;
                if dx * dx * 2 + dy * dy < 40 * 40 {
                    c1
                } else {
                    c2
                }
            })
        });
        lcd.set_pixels(0, 0, width as u16, height as u16, pattern)
    }

    pub fn lcd_test_pattern_2<DI, RST, M, C: Copy, PinE>(
        lcd: &mut Display<DI, RST, M>,
        c1: C,
        c2: C,
    ) -> Result<(), mipidsi::Error<PinE>>
    where
        DI: WriteOnlyDataCommand,
        RST: OutputPin<Error = PinE>,
        M: Model<ColorFormat = C>,
    {
        let Size { width, height } = lcd.size();
        let pattern = (0..height as i32).flat_map(|y| {
            (0..width as i32).map(move |x| if (x / 8) % 2 == (y / 8) % 2 { c1 } else { c2 })
        });
        lcd.set_pixels(0, 0, width as u16, height as u16, pattern)
    }
}
