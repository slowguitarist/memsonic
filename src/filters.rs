//! # filters
//!
//! Generic digital filters used by sensors. Emulated Windowed Sinc FIR is used
//! for oversampling and a biquad variant -- as a general ODR-bound IIR.

use crate::env::{FIR_MAX_TAPS, NORMAL_POSITIVE};
use core::f32::consts::PI;
use libm::{cosf, sinf};

/// Structs implementing this trait are trivial constructs that spin a recursive
/// formula; all sensor-specific config is stored and passed by the sensor. This
/// reduces memory usage because often every axis needs its own Filter instance.
pub(crate) trait Filter<T: Copy, U: Copy> {
    type SensorParams;

    fn new() -> Self
    where
        Self: Default,
    {
        Default::default()
    }

    fn _reset(&mut self) -> &mut Self
    where
        Self: Default,
    {
        *self = Self::new();
        self
    }

    /// Execute one iteration of an underlying filter.
    fn filter(&mut self, conf: &Self::SensorParams, sample: T) -> U;
}

#[allow(dead_code)]
pub(crate) enum BiquadType {
    LowPass(f32, f32),
    HighPass(f32, f32),
    Notch(f32, f32),
}

pub(crate) struct BiquadCoef {
    pub b0: f32,
    pub b1: f32,
    pub b2: f32,
    pub a1: f32,
    pub a2: f32,
}

impl BiquadCoef {
    pub(crate) fn derive(conf: BiquadType, odr: f32) -> Self {
        let raw_cutoff = match conf {
            BiquadType::LowPass(cutoff, ..)
            | BiquadType::HighPass(cutoff, ..)
            | BiquadType::Notch(cutoff, ..) => cutoff,
        };
        let cutoff = raw_cutoff.min(odr * 0.49);
        let omega = 2.0 * PI * cutoff / odr;

        let cos_w = cosf(omega);
        let sin_w = sinf(omega);

        let alpha = 0.5 * sin_w
            / match conf {
                BiquadType::LowPass(.., q)
                | BiquadType::HighPass(.., q)
                | BiquadType::Notch(.., q) => q,
            };

        let (b0, b1, b2, a0, a1, a2) = match conf {
            BiquadType::LowPass(..) => (
                (1.0 - cos_w) * 0.5,
                1.0 - cos_w,
                (1.0 - cos_w) * 0.5,
                1.0 + alpha,
                -2.0 * cos_w,
                1.0 - alpha,
            ),
            BiquadType::HighPass(..) => (
                (1.0 + cos_w) * 0.5,
                -(1.0 + cos_w),
                (1.0 + cos_w) * 0.5,
                1.0 + alpha,
                -2.0 * cos_w,
                1.0 - alpha,
            ),
            BiquadType::Notch(..) => (
                1.0,
                -2.0 * cos_w,
                1.0,
                1.0 + alpha,
                -2.0 * cos_w,
                1.0 - alpha,
            ),
        };

        Self {
            b0: b0 / a0,
            b1: b1 / a0,
            b2: b2 / a0,
            a1: a1 / a0,
            a2: a2 / a0,
        }
    }
}

#[derive(Clone, Copy, Default)]
pub(crate) struct Biquad {
    s1: f32,
    s2: f32,
}

impl Filter<f32, f32> for Biquad {
    type SensorParams = BiquadCoef;

    fn filter(&mut self, conf: &BiquadCoef, sample: f32) -> f32 {
        let out = self.s1 + conf.b0 * sample;

        self.s1 = conf.b1 * sample - conf.a1 * out + self.s2;
        self.s2 = conf.b2 * sample - conf.a2 * out;

        out
    }
}

pub(crate) struct FIRDecim(u32);

impl FIRDecim {
    pub(crate) fn new(decim_factor: u32) -> Self {
        assert!(
            decim_factor > 7,
            "Windowed sinc FIR is suboptimal for D < 8, please adjust your ODR."
        );
        Self(decim_factor)
    }
}

#[derive(Clone, Copy)]
pub(crate) struct WindowedSinc {
    buf: [f32; FIR_MAX_TAPS],
    len: usize,
    ctr: u32,
}

impl Default for WindowedSinc {
    fn default() -> Self {
        Self {
            buf: [0.0; FIR_MAX_TAPS],
            len: 0,
            ctr: 0,
        }
    }
}

impl Filter<f32, Option<f32>> for WindowedSinc {
    type SensorParams = FIRDecim;

    fn filter(&mut self, conf: &FIRDecim, sample: f32) -> Option<f32> {
        self.ctr += 1;

        if self.len < FIR_MAX_TAPS {
            self.buf[self.len] = sample;
            self.len += 1;
        } else {
            self.buf.copy_within(1..FIR_MAX_TAPS, 0);
            self.buf[FIR_MAX_TAPS - 1] = sample;
        }

        if self.ctr < conf.0 {
            return None;
        }

        self.ctr = 0;
        let m = self.len.min(conf.0 as usize);

        let out = if m <= 1 {
            self.buf[self.len - 1]
        } else {
            let mid = (m - 1) as f32 / 2.0;
            let start = self.len - m;

            let mut values = 0.0;
            let mut weights = 0.0;

            for k in 0..m {
                let x = k as f32 - mid;

                // Approximate Hamming window
                let win = 0.54 + 0.46 * cosf(PI * x / mid);
                let sinc = if x.abs() < 1e-6 {
                    1.0
                } else {
                    sinf(PI * x / mid) / (PI * x / mid)
                };

                let w = (win * sinc).max(0.0);
                values += w * self.buf[start + k];
                weights += w;
            }

            if weights > NORMAL_POSITIVE {
                values / weights
            } else {
                self.buf[self.len - 1]
            }
        };

        self.len = 0;
        Some(out)
    }
}

/////////////////////////////////////////////////////////////////////////////
// Tests
/////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fir_precision_loss() {
        let conf = FIRDecim::new(10);
        let mut fir = WindowedSinc::new();

        let mut result = None;
        for _ in 0..10 {
            result = fir.filter(&conf, 0.0015);
        }

        let out = result.expect("Wrong counter?");

        assert!(
            (out - 0.0015).abs() < f32::EPSILON,
            "FIR precision loss: expected 0.0015, but got: {}",
            out
        );
    }

    #[test]
    fn test_notch_q_factor_scaling() {
        let odr = 1000.0;
        let q = 0.707;
        let fc = 50.0;

        let notch = BiquadCoef::derive(BiquadType::Notch(fc, q), odr);

        let w_0 = 2.0 * PI * fc / odr;
        let expected_alpha = 0.5 * sinf(w_0) / q;

        let expected_a1 = (-2.0 * cosf(w_0)) / (1.0 + expected_alpha);

        assert!(
            (notch.a1 - expected_a1).abs() < 1e-4,
            "Expected a1: {}, Got a1: {}",
            expected_a1,
            notch.a1
        );
    }

    #[test]
    fn test_biquad_lowpass_dc_gain() {
        let conf = BiquadCoef::derive(BiquadType::LowPass(10.0, 0.707), 100.0);
        let mut biquad = Biquad::new();

        let mut out = 0.0;

        // Constant DC value of 1.0 to let it settle
        for _ in 0..100 {
            out = biquad.filter(&conf, 1.0);
        }

        assert!(
            (out - 1.0).abs() < 1e-6,
            "Bad DC gain. Expected ~1.0, got: {}",
            out
        );
    }

    #[test]
    fn test_biquad_highpass_dc_rejection() {
        let conf = BiquadCoef::derive(BiquadType::HighPass(10.0, 0.707), 100.0);
        let mut biquad = Biquad::new();

        let mut out = 0.0;

        // Constant DC value of 1.0
        for _ in 0..100 {
            out = biquad.filter(&conf, 1.0);
        }

        assert!(
            out.abs() < 1e-6,
            "Bad DC rejection. Expected ~0.0, got: {}",
            out
        );
    }
}
