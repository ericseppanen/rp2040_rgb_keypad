#![no_std]
#![no_main]

use embedded_hal::{delay::DelayNs, i2c::I2c};
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

/// The I2C address of the TCA9555 io register chip.
const TCA9555_I2C_ADDR: u8 = 0x20;

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
    // let mut led_pin = pins.gpio25.into_push_pull_output();

    // Set up an I2C bus to read the keyboard state.
    let mut i2c;
    {
        use fugit::RateExtU32;
        use rp2040_hal::i2c::I2C;

        i2c = I2C::i2c0(
            pac.I2C0,
            pins.gpio4.reconfigure(), // sda
            pins.gpio5.reconfigure(), // scl
            400.kHz(),
            &mut pac.RESETS,
            125_000_000.Hz(),
        );
    }

    // Make sure SPI interrupt pin is set as an input. It's currently unused.
    let _spi_interrupt = pins.gpio3.into_floating_disabled();

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

    use apa102_spi::{Apa102Pixel, Apa102Writer, PixelOrder, SmartLedsWrite, u5};

    let mut led_strip = Apa102Writer::new(spi, 1, PixelOrder::default());

    // Use GPIO17 as a manual chip-select
    // FIXME: can we make this automatically assert when SPI is active?
    let mut led_cs = pins.gpio17.into_push_pull_output();
    led_cs.set_low().unwrap();

    let mut rgb_buffer = [Apa102Pixel {
        red: 0,
        green: 0,
        blue: 0,
        // Note: brightness is 0-31.
        brightness: u5::new(4),
    }; 16];

    for (index, pixel) in rgb_buffer.iter_mut().enumerate() {
        pixel.red = 255 - (8 * index as u8);
        pixel.green = 8 * index as u8;
    }

    loop {
        let input_port_register = [0u8];
        let mut keyboard_buf: [u8; 2] = [0, 0];
        // Note: keyboard bits are active low.
        let mut keyboard_state = u16::MAX;
        match i2c.write_read(TCA9555_I2C_ADDR, &input_port_register, &mut keyboard_buf) {
            Ok(()) => {
                keyboard_state = u16::from_le_bytes(keyboard_buf);
            }
            Err(_) => {}
        }

        for (index, pixel) in &mut rgb_buffer.iter_mut().enumerate() {
            let this_key_hit: bool = ((keyboard_state >> index) & 0x01) == 0;

            if this_key_hit {
                pixel.red = 255;
                pixel.green = 0;
                pixel.blue = 0;
            } else if pixel.blue == 0 && pixel.green < 255 {
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

        led_strip.write(rgb_buffer).unwrap();

        timer.delay_ms(10);
    }
}
