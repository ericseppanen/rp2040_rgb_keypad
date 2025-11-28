#![no_std]
#![no_main]

use embedded_hal::{delay::DelayNs, digital::StatefulOutputPin};
use embedded_hal_02::digital::v2::OutputPin;
use hal::pac;
use panic_halt as _;
use rp2040_hal as hal;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[unsafe(link_section = ".boot2")]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    // loop {
    //     led_pin.set_high().unwrap();
    //     timer.delay_ms(500);
    //     led_pin.set_low().unwrap();
    //     timer.delay_ms(500);
    // }

    // use fugit::RateExtU32;
    // use rp2040_hal::{i2c::I2C, pac};

    // let mut i2c = I2C::i2c1(
    //     pac.I2C1,
    //     pins.gpio18.reconfigure(), // sda
    //     pins.gpio19.reconfigure(), // scl
    //     400.kHz(),
    //     &mut pac.RESETS,
    //     125_000_000.Hz(),
    // );

    // // Scan for devices on the bus by attempting to read from them
    // use embedded_hal::i2c::I2c;
    // //use embedded_hal_02::prelude::_embedded_hal_blocking_i2c_Read;

    // for i in 0..=127u8 {
    //     let mut readbuf: [u8; 1] = [0; 1];
    //     let result = i2c.read(i, &mut readbuf);
    //     if let Ok(d) = result {
    //         // Do whatever work you want to do with found devices
    //         // writeln!(uart, "Device found at address{:?}", i).unwrap();
    //     }
    // }

    // // Write some data to a device at 0x2c
    // //use embedded_hal_02::prelude::_embedded_hal_blocking_i2c_Write;

    // i2c.write(0x2Cu8, &[1, 2, 3]).unwrap();

    // // Write and then read from a device at 0x3a
    // //use embedded_hal_02::prelude::_embedded_hal_blocking_i2c_WriteRead;

    // let mut readbuf: [u8; 1] = [0; 1];
    // i2c.write_read(0x2Cu8, &[1, 2, 3], &mut readbuf).unwrap();
    //
    //

    // Set up a SPI bus to write data to the LEDs.
    let spi;
    {
        use embedded_hal::spi::MODE_0;
        use fugit::RateExtU32;
        use rp2040_hal::{gpio::FunctionSpi, spi::Spi};

        // The only pins needed
        let sclk = pins.gpio18.into_function::<FunctionSpi>();
        let mosi = pins.gpio19.into_function::<FunctionSpi>();

        let spi_device = pac.SPI0;
        let spi_pin_layout = (mosi, sclk);

        spi = Spi::<_, _, _, 8>::new(spi_device, spi_pin_layout).init(
            &mut pac.RESETS,
            125_000_000u32.Hz(),
            16_000_000u32.Hz(),
            MODE_0,
        );
    }

    use apa102_spi::{Apa102Pixel, Apa102Writer, PixelOrder, RGB8, SmartLedsWrite, u5};

    let mut led_strip = Apa102Writer::new(spi, 1, PixelOrder::default());

    // Use GPIO17 as a manual chip-select
    // FIXME: can we make this automatically assert when SPI is active?
    let mut led_cs = pins.gpio17.into_push_pull_output();
    led_cs.set_low().unwrap();

    // // Specify pixel values as 8 bit RGB + 5 bit brightness
    // let led_buffer = [Apa102Pixel {
    //     red: 255,
    //     green: 0,
    //     blue: 0,
    //     brightness: u5::new(1),
    // }];
    // led_strip.write(led_buffer);

    let mut rgb_buffer = [Apa102Pixel {
        red: 0,
        green: 0,
        blue: 0,
        brightness: u5::new(4),
    }; 16];

    for (index, pixel) in rgb_buffer.iter_mut().enumerate() {
        pixel.red = 255 - (8 * index as u8);
        pixel.green = 8 * index as u8;
    }

    loop {
        for pixel in &mut rgb_buffer {
            if pixel.blue == 0 && pixel.green < 255 {
                pixel.red -= 1;
                pixel.green += 1;
            } else if pixel.red == 0 && pixel.blue < 255 {
                pixel.green -= 1;
                pixel.blue += 1;
            } else {
                pixel.blue -= 1;
                pixel.red += 1;
            }
        }

        // Brightness is set to maximum value (31) in `impl From<RGB8> for Apa102Pixel`
        led_strip.write(rgb_buffer).unwrap();

        timer.delay_ms(10);
    }
}
