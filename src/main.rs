use display_interface::WriteOnlyDataCommand;
use display_interface_spi::SPIInterface;
use embedded_graphics_core::geometry::OriginDimensions;
use embedded_graphics_core::geometry::Size;
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
use st7789::{Orientation, ST7789};
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

    let mut lcd = ST7789::new(di, lcd_rst, 240, 320);
    lcd.set_orientation(Orientation::Portrait);

    // let mut delay = Delay::new();
    let mut delay = esp_idf_hal::delay::Ets;
    lcd.init(&mut delay);

    lcd_test_pattern(&mut lcd);

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
            if bamboo { 0xffff } else { 0x07ff },
            if bamboo { 0x0000 } else { 0x001f },
        );

        thread::sleep(Duration::from_millis(25)); // must let the RTOS pacify the watchdog

        hue = hue.wrapping_add(5);
    }
}

fn lcd_test_pattern<DI, RST, PinE>(lcd: &mut ST7789<DI, RST>)
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
                0xffff
            } else {
                0x0000
            }
        })
    });
    lcd.set_pixels(0, 0, width as u16, height as u16, pattern);
}

fn lcd_test_pattern_2<DI, RST, PinE>(lcd: &mut ST7789<DI, RST>, c1: u16, c2: u16)
where
    DI: WriteOnlyDataCommand,
    RST: OutputPin<Error = PinE>,
{
    let Size { width, height } = lcd.size();
    let pattern = (0..height as i32).flat_map(|y| {
        (0..width as i32).map(move |x| if (x / 8) % 2 == (y / 8) % 2 { c1 } else { c2 })
    });
    lcd.set_pixels(0, 0, width as u16, height as u16, pattern);
}
