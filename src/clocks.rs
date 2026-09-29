//! # clocks
//!
//! Embedded clocks designed to work with for millisecond-precision timers present
//! on many microcontrollers, and sensors with sub-millisecond output rates.
//! These clocks return floating-point ticks without losing precision over time.

use crate::env::NS_PER_MS;

pub(crate) trait ToSeconds {
    fn to_seconds(self) -> f32;
}

pub(crate) type MsTick = f32;

impl ToSeconds for MsTick {
    fn to_seconds(self) -> f32 {
        let k = self / 1000.0;
        if k.is_normal() { k } else { 0.0 }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AbnormalRate;

pub(crate) struct Clock {
    tick_ns: u64,
    rate_ns: u64,
    rate_ms: f32,
}

impl Clock {
    pub(crate) fn new(start: u32, rate: f32) -> Result<Self, AbnormalRate> {
        if !rate.is_normal() || rate <= 0.0 {
            return Err(AbnormalRate);
        }

        let ns = (rate * NS_PER_MS as f32 + 0.5) as u64;

        if ns == 0 {
            return Err(AbnormalRate);
        }

        let st_ns = start as u64 * NS_PER_MS;

        Ok(Self {
            tick_ns: st_ns.checked_add(ns).ok_or(AbnormalRate)?,
            rate_ns: ns,
            rate_ms: rate,
        })
    }

    pub(crate) fn next_tick(&mut self, tim: u32) -> Option<(u32, MsTick)> {
        let deadline = tim as u64 * NS_PER_MS;

        if self.tick_ns > deadline {
            return None;
        }

        let now_ms = (self.tick_ns / NS_PER_MS) as u32;
        self.tick_ns = self.tick_ns.checked_add(self.rate_ns)?;

        Some((now_ms, self.rate_ms))
    }

    pub(crate) fn fast_forward<F>(&mut self, tim: u32, mut step: F)
    where
        F: FnMut(u32, MsTick),
    {
        while let Some((now, tick)) = self.next_tick(tim) {
            step(now, tick);
        }
    }
}
