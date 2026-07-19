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
		where Self: Default
	{
		Default::default()
	}

	fn _reset(&mut self) -> &mut Self
		where Self: Default
	{
		*self = Self::new();
		self
	}

	/// Execute one iteration of an underlying filter.
	fn filter(&mut self, conf: &Self::SensorParams, sample: T) -> U;
}

// TODO implement variant selection in builder
#[allow(dead_code)]
pub(crate) enum BiquadType {
	LowPass(f32, f32),
	HighPass(f32, f32),
	Notch(f32, f32),
}

pub(crate) struct BiquadCoef {
	pub b0: f32, pub b1: f32, pub b2: f32,
	pub a1: f32, pub a2: f32
}

impl BiquadCoef {
	pub(crate) fn derive(conf: BiquadType, odr: f32) -> Self {
		let omega = 2.0 * PI * match conf {
			BiquadType::LowPass(cutoff, ..)
			| BiquadType::HighPass(cutoff, ..)
			| BiquadType::Notch(cutoff, ..) => cutoff,
		} / odr;

		let cos_w = cosf(omega);
		let sin_w = sinf(omega);

		let alpha = 0.5 * sin_w / match conf {
			BiquadType::LowPass(.., q) | BiquadType::HighPass(.., q) => q,
			BiquadType::Notch(c, q) => c / q,
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
	s2: f32
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
	sens: f32,
}

impl CICConf {
	pub(crate) fn new(decim_fac: u32, sens: f32) -> Self {
		Self { decim_fac, sens }
	}
}

#[derive(Clone, Copy, Default)]
pub(crate) struct EmuCIC {
	integrator: i32,
	comb: i32,
	ctr: u32,
}

impl Filter<f32, Option<f32>> for EmuCIC {
	type SensorParams = CICConf;

	fn filter(&mut self, conf: &CICConf, sample: f32) -> Option<f32> {
		let raw = (sample * conf.sens) as i32;

		self.integrator = self.integrator.wrapping_add(raw);
		self.ctr += 1;

		if self.ctr >= conf.decim_fac {
			self.ctr = 0;

			let out = self.integrator.wrapping_sub(self.comb);
			self.comb = self.integrator;

			return Some((out / conf.decim_fac as i32) as f32 / conf.sens)
		}

		None
	}
}