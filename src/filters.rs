//! # filters
//!
//! Generic digital filters used by sensors. Emulated decimator CIC is used
//! for oversampling and a biquad variant -- as a general ODR-bound IIR.

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

pub(crate) struct CICConf {
    decim_fac: u32,
    _sens: f32,
}

impl CICConf {
    pub(crate) fn new(decim_fac: u32, sens: f32) -> Self {
        Self {
            decim_fac,
            _sens: sens,
        }
    }
}

#[derive(Clone, Copy, Default)]
pub(crate) struct EmuCIC {
    acc: f32,
    ctr: u32,
}

impl Filter<f32, Option<f32>> for EmuCIC {
    type SensorParams = CICConf;

    /// Temporary replacement of CIC with a block accumulator.
    fn filter(&mut self, conf: &CICConf, sample: f32) -> Option<f32> {
        self.acc += sample;
        self.ctr += 1;

        if self.ctr >= conf.decim_fac {
            self.ctr = 0;
            let mean = self.acc / conf.decim_fac as f32;
            self.acc = 0.0;
            return Some(mean);
        }

        None
    }
}

/////////////////////////////////////////////////////////////////////////////
// Tests
/////////////////////////////////////////////////////////////////////////////

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cic_precision_loss() {
        let conf = CICConf::new(10, 1000.0);
        let mut cic = EmuCIC::new();

        let mut result = None;
        for _ in 0..10 {
            result = cic.filter(&conf, 0.0015);
        }

        let out = result.expect("Wrong counter?");

        assert!(
            (out - 0.0015).abs() < f32::EPSILON,
            "CIC precision loss: expected 0.0015, but got: {}",
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
