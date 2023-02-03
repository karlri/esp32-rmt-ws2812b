//! Enables writes to ws8212 smart leds reliably and efficiently using specialized ESP32 RMT hardware.
//!
//! Notice: ws8212 needs higher logical high voltage than the 3.3V that ESP32 can provide.
//! Without a logic level shifter, expect color/position gliteches! Don't try to run the ESP32 on 5V!
//!
//! You may use almost any output pin(s), contrary to for example an SPI approach and you may also drive up
//! to 8 leds strips with 300 leds each concurrently with 140+ FPS!
//!
//! You also don't need to precompute the data. The data can be computed on the fly from an iterator,
//! keeping the CPU available for other concurrent tasks. The interrupt pressure is high though!
//!
//! If you have precomputed data no problem, but an SPI approach would perhaps be a better fit in that case!
//! If you have 8 led strips, it may be a better idea to use a hardware timer and 8bit port writes.
//!
//! # Example
//!
//! Turn on the swedish flag:
//! ```
//! use esp32_rmt_ws8212::*;
//!
//! let data_output_pin = pins.gpio24;
//! let ch = peripherals.rmt.channel2;
//!
//! let driver = Esp32RmtWs8212Driver::new(data_output_pin, ch);
//!
//! // half yellow and half blue
//! let iterator = (0..300).map(|e| {
//!     if e < 150 {
//!         Ws8212Color { r: 255, g: 255, b: 0 }
//!     } else {
//!         Ws8212Color { r: 0, g: 0, b: 255 }
//!     }
//! });
//!
//! // Safety: iterator does not block/allocate and has no side effects.
//! // Reliability: no heavy math so iterator returns next element very fast.
//! unsafe { writer.write(iterator); }
//! ```
use std::time::Duration;

use esp_idf_hal::{
    gpio::OutputPin,
    peripheral::Peripheral,
    rmt::{
        config::TransmitConfig, FixedLengthSignal, PinState, Pulse, RmtChannel, Signal, TxRmtDriver,
    },
    units::Hertz,
};
use esp_idf_sys::EspError;
use std::thread::sleep;

pub struct Esp32RmtWs8212Driver<'a> {
    tx: TxRmtDriver<'a>,
    send_zero_bit: esp_idf_sys::rmt_item32_t,
    send_one_bit: esp_idf_sys::rmt_item32_t,
}

impl<'a> Esp32RmtWs8212Driver<'a> {
    pub fn new<P: OutputPin, C: RmtChannel>(
        pin: P,
        channel: impl Peripheral<P = C> + 'a,
    ) -> Result<Esp32RmtWs8212Driver<'a>, EspError> {
        // Prepare the config.
        let config = TransmitConfig::new().clock_divider(1).carrier(None);

        // Create an RMT transmitter.
        let tx = TxRmtDriver::new(channel, pin, &config)?;
        let clk = tx.counter_clock()?;

        /// convert ergonomic high level API to low level c structs for use in interrupt handlers
        fn high_then_low_ns(
            ticks_hz: Hertz,
            high_ns: u64,
            low_ns: u64,
        ) -> std::result::Result<esp_idf_sys::rmt_item32_t, EspError> {
            let high =
                Pulse::new_with_duration(ticks_hz, PinState::High, &Duration::from_nanos(high_ns))?;
            let low =
                Pulse::new_with_duration(ticks_hz, PinState::Low, &Duration::from_nanos(low_ns))?;
            let mut signal: FixedLengthSignal<1> = FixedLengthSignal::new();
            signal.set(0, &(high, low))?;
            Ok(signal.as_slice()[0])
        }

        // Correct timing values for led chip WS2812b
        // We ensure that sending 1 and sending 0 takes the same amount of time, so the FPS will be consistent.
        let t1 = 250;
        let t2 = 625;
        let t3 = 375;
        let send_zero_bit = high_then_low_ns(clk, t1, t2 + t3)?;
        let send_one_bit = high_then_low_ns(clk, t1 + t2, t3)?;

        Ok(Esp32RmtWs8212Driver {
            tx,
            send_zero_bit,
            send_one_bit,
        })
    }

    /// Writes an iterator
    ///
    /// Safety:
    /// The next function of the iterator you provide will be called from an ISR (Interrupt Service Routine).
    /// As such, certain functions that are normally safe are NOT safe to call from within the iterator.
    /// Don't allocate memory and don't block among many other things!
    pub unsafe fn write<T, I>(&mut self, iterator: T) -> std::result::Result<(), EspError>
    where
        T: Iterator<Item = Ws8212Color> + Send, /* + ISR safe */
    {
        let send_zero_bit = self.send_zero_bit;
        let send_one_bit = self.send_one_bit;

        // turn RGB iterator into high/low timed pulses iterator
        let iter_rmt_item32_t = iterator
            .map(move |color| {
                // iter color components
                IntoIterator::into_iter([color.g, color.r, color.b])
                    .map(move |component| {
                        // iter bits in each component
                        (0..8).into_iter().map(move |bitn| {
                            // MSB first. yes. send 0 if bit is 1.
                            if component & (1 << (7 - bitn)) == 0 {
                                send_zero_bit
                            } else {
                                send_one_bit
                            }
                        })
                    })
                    .flatten()
            })
            .flatten();

        self.tx.start_iter_blocking(iter_rmt_item32_t)?;
        sleep(Duration::from_millis(1));
        Ok(())
    }
}

/// Represents a color and intensity in the ws8212b color space.
#[repr(packed)]
pub struct Ws8212Color {
    // do not reorder fields, it leads to slower machine code.
    g: u8,
    r: u8,
    b: u8,
}
